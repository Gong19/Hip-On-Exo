"""Software reference poses. Never writes calibration registers to an IMU."""
import json
import math
import threading
import time
from datetime import datetime
from pathlib import Path
import numpy as np


def rotation(roll, pitch, yaw):
    r,p,y = np.radians([roll,pitch,yaw])
    cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def euler(matrix):
    p=math.asin(float(np.clip(-matrix[2,0],-1,1)))
    if abs(math.cos(p)) > 1e-7:
        r,y=math.atan2(matrix[2,1],matrix[2,2]),math.atan2(matrix[1,0],matrix[0,0])
    else:
        r,y=0.0,math.atan2(-matrix[0,1],matrix[1,1])
    return np.degrees([r,p,y]).tolist()


class ImuReference:
    duration_s=3.0
    min_samples=100
    max_gap_s=0.15
    max_gyro_dps=3.0
    max_pose_spread_deg=2.0

    def __init__(self, count):
        self.lock=threading.RLock()
        self.count=count
        self.session_id=None
        self.serial=0
        self.pending=None
        self.refs={}
        self.states=['未设置']*count
        self.message='保持标准姿势静止，再设置参考姿态（3 秒）。'

    def _session(self, session):
        if self.session_id != session.session_id:
            self.session_id=session.session_id
            self.pending=None
            self.refs.clear()
            self.states=['未设置']*self.count
            self.message='新会话：请重新设置参考姿态。'

    def request(self, online, session, now=None):
        with self.lock:
            self._session(session)
            if not all(online):
                self.message='需要四路 IMU 均在线，未开始设置。'
                return False
            self.refs.clear()
            self.states=['采集中']*self.count
            self.pending={'start':time.perf_counter() if now is None else now,
                          'samples':{i:[] for i in range(self.count)}}
            self.message='保持静止，正在采集 3 秒参考姿态…'
            return True

    def invalidate(self, idx=None, reason='已失效，请重新设置'):
        with self.lock:
            self.message = reason + '。重新设置时请保持标准姿势静止。'
            if self.pending is not None:
                self.pending=None
                self.states=[reason]*self.count
                self.refs.clear()
                self.message='参考采集取消：'+reason
            for i in (range(self.count) if idx is None else [idx]):
                self.refs.pop(i,None)
                self.states[i]=reason

    def status(self, session, now=None):
        with self.lock:
            self._session(session)
            now=time.perf_counter() if now is None else now
            if self.pending and now-self.pending['start']>self.duration_s+1.0:
                self.invalidate(reason='数据不足，请重试')
            return list(self.states),self.message,self.pending is not None

    def process(self, idx, data, session, devices, now=None):
        now=time.perf_counter() if now is None else now
        with self.lock:
            self._session(session)
            result=dict(data)
            values=[data[k] for k in ('roll_deg','pitch_deg','yaw_deg','gx_dps','gy_dps','gz_dps','ax_g','ay_g','az_g')]
            finite=all(math.isfinite(v) for v in values)
            mat=rotation(*values[:3]) if finite else None
            if self.pending:
                samples=self.pending['samples'][idx]
                accel=float(np.linalg.norm(values[6:]))
                if (not finite or max(abs(v) for v in values[3:6])>self.max_gyro_dps
                        or not 0.85 <= accel <= 1.15
                        or (samples and now-samples[-1][0]>self.max_gap_s)):
                    self.invalidate(reason='运动或数据异常，请静止后重试')
                else:
                    if samples:
                        delta=samples[0][1].T @ mat
                        angle=math.degrees(math.acos(float(np.clip((np.trace(delta)-1)/2,-1,1))))
                        if angle>self.max_pose_spread_deg:
                            self.invalidate(reason='姿态不稳定，请重试')
                    if self.pending:
                        samples.append((now,mat,values))
                        groups=self.pending['samples']
                        ready=all(len(s)>=self.min_samples and s[-1][0]-s[0][0]>=self.duration_s and now-s[-1][0]<=self.max_gap_s for s in groups.values())
                        if ready:
                            self._commit(session,devices)
            ref=self.refs.get(idx)
            result.update(reference_valid=int(ref is not None and finite),reference_id=ref['id'] if ref else 0)
            angles=euler(ref['matrix'].T @ mat) if ref and finite else [float('nan')]*3
            result.update(zip(('rel_roll_deg','rel_pitch_deg','rel_yaw_deg'),angles))
            return result

    def _commit(self, session, devices):
        self.serial+=1
        refs={}
        sensors=[]
        for idx,samples in self.pending['samples'].items():
            u,_,vt=np.linalg.svd(np.mean([s[1] for s in samples],axis=0))
            mat=u @ np.diag([1,1,np.linalg.det(u@vt)]) @ vt
            refs[idx]={'id':self.serial,'matrix':mat}
            sensors.append(dict(index=idx,i2c_bus=devices[idx][0],i2c_address=devices[idx][1],
                samples=len(samples),first_mono_s=samples[0][0],last_mono_s=samples[-1][0],
                reference_rotation_matrix=mat.tolist(),reference_euler_deg=euler(mat),
                mean_gyro_dps=np.mean([s[2][3:6] for s in samples],axis=0).tolist()))
        info=dict(schema='hipexo-imu-reference/1',reference_id=self.serial,session_id=session.session_id,
            created_at=datetime.now().astimezone().isoformat(),host_wall_ns=time.time_ns(),host_mono_ns=time.perf_counter_ns(),
            metadata=session.metadata,sensors=sensors,
            convention='R = Rz(yaw) Ry(pitch) Rx(roll); relative = R_reference.T @ R_current. Not anatomical joint angles.',
            thresholds=dict(duration_s=self.duration_s,min_samples=self.min_samples,max_gap_s=self.max_gap_s,
                            max_gyro_dps=self.max_gyro_dps,max_pose_spread_deg=self.max_pose_spread_deg,
                            acceleration_norm_g=[0.85,1.15]))
        path=Path(session.directory)/(session.file_stem('IMU_reference')+f'__{self.serial}.json')
        try:
            path.write_text(json.dumps(info,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            session.artifact('imu_reference',path,reference_id=self.serial)
        except Exception as exc:
            self.invalidate(reason='参考文件保存失败')
            self.message=f'参考文件保存失败，未应用：{exc}'
            return
        self.refs=refs
        self.pending=None
        self.states=[f'已设置 #{self.serial}']*self.count
        self.message='四路参考姿态已保存。相对角度采用三维旋转计算；原始数据保留。'
