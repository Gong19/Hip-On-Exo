"""HiPExo EMG bridge v1 client. No Delsys/.NET dependency on this computer."""
from collections import deque
import json
import math
import os
from pathlib import Path
import socket
import statistics
import time
import uuid

PROTOCOL = 'hipexo-emg/1'
MAX_LINE = 4 * 1024 * 1024


class ClockSyncError(RuntimeError):
    """The run has ended; its timing needs review and a new sync is required."""


def estimate_clock(t1, t2, t3, t4, elapsed_ns):
    """Offset = Windows UTC - workstation UTC; retain all raw clock evidence."""
    if t3 < t2 or elapsed_ns < 0 or abs((t4-t1)-elapsed_ns) > 5_000_000:
        raise ValueError('Clock jumped during synchronization; retry')
    delay = elapsed_ns - (t3-t2)
    if delay < -100_000:
        raise ValueError('Invalid remote clock response')
    return {'offset_ns': ((t2-t1)+(t3-t4))//2, 'rtt_ns': max(0,delay),
            't1_ns':t1, 't2_ns':t2, 't3_ns':t3, 't4_ns':t4}


class RemoteWindowsSource:
    def __init__(self, config, cancel, export_dir, session=None):
        self.config, self.cancel = config, cancel
        self.export_dir = Path(export_dir)
        self.sock = None
        self.buffer = bytearray()
        self.pending = deque()
        self.journal = None
        self.sync = None
        self.anchor = None
        self.run_id = None
        self.expected_seq = 0
        self.started = False
        self.last_health = 0.
        self.last_flush = 0.
        self.session = session
        self.sync_invalid = False
        self._drift_streak = 0
        self._drift_sign = 0
        self._last_good_clock = 0.
        self.timing_report = None
        self.status = {'connected':False,'ready':False,'synced':False,'streaming':False}

    def _log(self, kind, **fields):
        if self.journal:
            event={'kind':kind,'local_wall_ns':time.time_ns(),
                   'local_mono_ns':time.perf_counter_ns(), **fields}
            self.journal.write(json.dumps(event,allow_nan=False,separators=(',',':'))+'\n')
            if time.monotonic()-self.last_flush > 1:
                self.journal.flush(); self.last_flush=time.monotonic()

    def _send(self, message):
        self.sock.sendall((json.dumps(message,allow_nan=False)+'\n').encode())

    def _receive(self, deadline, cancellable=True):
        while True:
            if cancellable and self.cancel.is_set():
                raise InterruptedError('EMG operation cancelled')
            end=self.buffer.find(b'\n')
            if end >= 0:
                if end > MAX_LINE:
                    raise ValueError('Oversized bridge message')
                raw=bytes(self.buffer[:end]); del self.buffer[:end+1]
                message=json.loads(raw,parse_constant=lambda x: (_ for _ in ()).throw(ValueError('Nonfinite JSON')))
                if not isinstance(message,dict):
                    raise ValueError('Bridge message must be an object')
                return message
            if len(self.buffer)>MAX_LINE:
                raise ValueError('Oversized bridge message')
            if time.monotonic() >= deadline:
                raise TimeoutError('Windows bridge response timed out')
            try:
                chunk=self.sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError('Windows bridge disconnected')
            self.buffer.extend(chunk)

    def _queue_data(self, message):
        if not self.started or message.get('run_id') != self.run_id:
            raise ValueError('Unexpected EMG run ID')
        seq=message.get('seq')
        if seq != self.expected_seq:
            raise ValueError(f'EMG packet sequence gap: expected {self.expected_seq}, got {seq}')
        if len(self.pending)>=256:
            raise BufferError('EMG receiver backlog; stop instead of silently dropping data')
        self.expected_seq += 1
        received_wall,received_mono=time.time_ns(),time.perf_counter_ns()
        delivery=self._delivery_timing(message,received_wall,received_mono)
        self._log('data',message=message,local_wall_ns=received_wall,
                  local_mono_ns=received_mono,delivery_timing=delivery)
        self.pending.append(message)
        if self.timing_report is not None and self.timing_report['first_data'] is None:
            self.timing_report['first_data']=delivery
            self._write_timing_report()
            self._log('initial_delivery_timing',run_id=self.run_id,**delivery)
            delay=delivery['windows_send_to_ubuntu_receive_est_ns']
            if delay is not None:
                self.status['initial_delivery_ms']=delay/1e6

    def _rpc(self, command, timeout=3., cancellable=True, **payload):
        request_id=uuid.uuid4().hex
        t1=time.time_ns(); m1=time.perf_counter_ns()
        self._send({'type':command,'id':request_id,**payload})
        deadline=time.monotonic()+timeout
        while True:
            message=self._receive(deadline,cancellable)
            t4=time.time_ns(); m4=time.perf_counter_ns()
            if message.get('type')=='data':
                self._queue_data(message)
                continue
            if message.get('id') != request_id:
                raise ValueError('Unexpected bridge response ID')
            if message.get('type')=='error':
                raise RuntimeError('Windows: '+str(message.get('message','unknown error')))
            if message.get('type') != command+'_ack':
                raise ValueError('Unexpected bridge response type')
            return message,(t1,m1,t4,m4)

    def connect(self):
        host=self.config.get('remote_host','').strip()
        if not host:
            raise ValueError('Set Windows IP / hostname in EMG Settings')
        if host not in ('127.0.0.1','localhost','::1') and not self.config.get('remote_token'):
            raise ValueError('Set the same connection token on both computers')
        self.sock=socket.create_connection((host,int(self.config.get('remote_port',8765))),timeout=2.)
        self.sock.settimeout(.1)
        self.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        reply,_=self._rpc('hello',protocol=PROTOCOL,token=self.config.get('remote_token',''))
        if reply.get('protocol') != PROTOCOL:
            raise ValueError('Incompatible Windows bridge protocol')
        channels=reply.get('channels',[])
        if len(channels)!=7:
            raise ValueError('Windows must describe seven fixed logical channels')
        for i,ch in enumerate(channels):
            if ch.get('sid') != self.config['sensor_ids'][i]:
                raise ValueError('Windows sensor order/IDs do not match workstation settings')
            if not isinstance(ch.get('present'),bool) or not isinstance(ch.get('is_rms'),bool):
                raise ValueError('Invalid channel availability/type')
            if not isinstance(ch.get('mode'),str) or not math.isfinite(float(ch.get('sample_rate',0))) or float(ch.get('sample_rate',0))<=0:
                raise ValueError('Invalid channel mode/sample rate')
        self.channels=channels
        self.export_dir.mkdir(parents=True,exist_ok=True)
        path=self.export_dir/f'emg_remote_{uuid.uuid4().hex}.jsonl'
        self.journal=path.open('x',encoding='utf-8')
        self.status.update(connected=True,ready=reply.get('ready') is True,journal=str(path),
                           simulated=reply.get('simulated') is True)
        self._log('hello',reply=reply,protocol=PROTOCOL)
        if self.session:
            self.session.artifact('emg_remote_raw', path, simulated=self.status['simulated'])
        self.last_health=time.monotonic()
        return channels

    def _probe(self):
        reply,stamps=self._rpc('ping')
        t1,m1,t4,m4=stamps
        result=estimate_clock(t1,int(reply['t2_ns']),int(reply['t3_ns']),t4,m4-m1)
        result.update(local_mono_ns=m4, local_wall_ns=t4)
        self.status['ready']=reply.get('ready') is True
        self._log('clock_probe',**result)
        self.last_health=time.monotonic()
        return result

    def idle(self):
        if time.monotonic()-self.last_health>=1.:
            self._probe()

    def synchronize(self):
        if self.started:
            raise ValueError('Stop EMG before resynchronizing')
        probes=[self._probe() for _ in range(9)]
        if not self.status['ready']:
            raise RuntimeError('Windows bridge is connected but not armed / ready')
        best=min(probes,key=lambda x:x['rtt_ns'])
        self.sync_probes=probes
        self.sync={**best,'sync_id':uuid.uuid4().hex,'created_mono':time.monotonic()}
        self.sync_invalid = False
        self._drift_streak = self._drift_sign = 0
        self._last_good_clock = time.monotonic()
        self.status.pop('clock_fault', None)
        self.status.update(synced=True,clock_quality='good',drift_ms=0.,
                           offset_ms=best['offset_ns']/1e6,rtt_ms=best['rtt_ns']/1e6,
                           one_way_est_ms=best['rtt_ns']/2e6,
                           sync_uncertainty_ms=best['rtt_ns']/2e6)
        self._log('sync_selected',sync=self.sync)

    def start(self):
        if self.sync_invalid or not self.sync or time.monotonic()-self.sync['created_mono']>300:
            raise RuntimeError('Press Sync & Start to synchronize both computers first')
        self._check_local_clock()
        self.run_id=uuid.uuid4().hex
        reply,stamps=self._rpc('start',timeout=10.,cancellable=False,run_id=self.run_id,sync_id=self.sync['sync_id'],
                          workstation_offset_ns=self.sync['offset_ns'])
        self.started=True  # ensure close sends stop if validation below rejects the ACK
        if reply.get('run_id')!=self.run_id or reply.get('recording') is not True:
            raise ValueError('Windows did not confirm local raw recording')
        self.anchor=reply.get('clock_anchor',{})
        for name in ('device_time_s','windows_wall_ns','windows_mono_ns'):
            if name not in self.anchor or not math.isfinite(float(self.anchor[name])):
                raise ValueError('Windows START response lacks a valid clock anchor')
        if not isinstance(self.anchor.get('method'),str):
            raise ValueError('Windows must describe device-to-host timestamp mapping')
        self.expected_seq=0; self.pending.clear()
        self.status.update(streaming=True,run_id=self.run_id,windows_file=reply.get('record_path',''))
        self.status.pop('initial_delivery_ms',None)
        self.timing_report={
            'schema':'hipexo-run-timing/1','run_id':self.run_id,
            'session_id':self.session.session_id if self.session else None,
            'simulated':self.status.get('simulated',False),'sync':self.sync,
            'sync_probes':self.sync_probes,'clock_anchor':self.anchor,
            'windows_minus_ubuntu_offset_ns':self.sync['offset_ns'],
            'network_rtt_min_ns':self.sync['rtt_ns'],
            'network_rtt_median_ns':statistics.median(p['rtt_ns'] for p in self.sync_probes),
            'network_rtt_max_ns':max(p['rtt_ns'] for p in self.sync_probes),
            'one_way_network_est_ns':self.sync['rtt_ns']/2,
            'one_way_method':'RTT/2 assuming symmetric paths; not a measured one-way delay',
            'offset_network_uncertainty_ns':self.sync['rtt_ns']/2,
            'device_acquisition_latency_ns':None,
            'device_latency_note':'Unknown to this client. Retain bridge clock_anchor; requires bridge evidence or physical calibration. Not equal to network delay.',
            'start_command_wall_ns':stamps[0],'start_ack_wall_ns':stamps[2],
            'start_command_to_ack_ns':stamps[3]-stamps[1],
            'first_data':None,'timing_valid':True,
            'csv_time_formula':'anchor.windows_wall_ns + (device_time_s-anchor.device_time_s)*1e9 - windows_minus_ubuntu_offset_ns',
            'manual_alignment_note':'CSV already removes clock offset and uses source timestamps: do not subtract network transport delay again. If an uncorrected device-to-host latency D is measured, subtract D from EMG CSV time (or add D to the reference stream). Unknown D is not zero.',
        }
        self._timing_path=self.export_dir/f'emg_timing_{self.run_id}.json'
        self._write_timing_report()
        self.status['timing_report']=str(self._timing_path)
        self._log('start',reply=reply,sync=self.sync)
        if self.session:
            self.session.artifact('emg_run_timing',self._timing_path,run_id=self.run_id,
                                  readable_summary=str(self._timing_path.with_suffix('.txt')))
            self.session.event('emg_start',run_id=self.run_id,sync=self.sync,
                               ubuntu_raw=self.status['journal'],windows_raw=reply.get('record_path'),
                               simulated=self.status.get('simulated',False),clock_anchor=self.anchor)

    def _write_timing_report(self):
        temporary=self._timing_path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(self.timing_report,ensure_ascii=False,allow_nan=False,indent=2),encoding='utf-8')
        os.replace(temporary,self._timing_path)
        report=self.timing_report
        initial=report['first_data']
        delay=initial['windows_send_to_ubuntu_receive_est_ns'] if initial else None
        text=(f"EMG 起始时间与延迟记录\nrun_id: {report['run_id']}\n"
              f"模拟数据: {report['simulated']}\n时间映射有效: {report['timing_valid']}\n"
              f"Windows − Ubuntu 时钟偏差: {report['windows_minus_ubuntu_offset_ns']/1e6:+.6f} ms\n"
              f"网络 RTT（最小/中位/最大）: {report['network_rtt_min_ns']/1e6:.6f} / "
              f"{report['network_rtt_median_ns']/1e6:.6f} / {report['network_rtt_max_ns']/1e6:.6f} ms\n"
              f"单向网络延迟估计（RTT/2，假设来回对称）: {report['one_way_network_est_ns']/1e6:.6f} ms\n"
              f"时钟偏差的网络不确定范围: ±{report['offset_network_uncertainty_ns']/1e6:.6f} ms\n"
              f"START 命令到确认耗时: {report['start_command_to_ack_ns']/1e6:.6f} ms（不是采样延迟）\n"
              f"首包发送→接收延迟估计: {f'{delay/1e6:.6f} ms' if delay is not None else '等待首包或桥接端未提供发送时间'}\n"
              "Delsys 采样→Windows 取数的内部延迟: 未知，待桥接端证据/物理标定；不是 0\n"
              f"设备时间映射方法: {self.anchor['method']}\n"
              "CSV 已按源时间映射并扣除两机时钟偏差，不要再扣一次网络传输延迟。\n"
              "若实测存在尚未补偿的设备取数延迟 D，应将 EMG 的 CSV 时间减 D，或将参考信号时间加 D。\n"
              "首次延迟不是全程固定保证；每包到达延迟估计保存在原始 JSONL 的 delivery_timing。\n"
              f"时钟异常: {report.get('clock_fault','无')}\n完整证据见同名 JSON。\n")
        self._timing_path.with_suffix('.txt').write_text(text,encoding='utf-8')

    def _delivery_timing(self,message,wall,mono):
        sent=message.get('windows_send_wall_ns')
        delay=wall+self.sync['offset_ns']-sent if type(sent) is int else None
        ranges=[]
        for channel in message.get('channels',[]):
            times=channel.get('device_time_s',[])
            if times:
                first,last=float(times[0]),float(times[-1])
                if math.isfinite(first) and math.isfinite(last):
                    ranges.append({'slot':channel.get('slot'),'first_device_time_s':first,
                                   'last_device_time_s':last,
                                   'first_sample_age_est_ns':wall-self.mapped_time(first)[0],
                                   'last_sample_age_est_ns':wall-self.mapped_time(last)[0]})
        return {'seq':message['seq'],'ubuntu_received_wall_ns':wall,'ubuntu_received_mono_ns':mono,
                'windows_send_wall_ns':sent,'windows_send_to_ubuntu_receive_est_ns':delay,
                'estimate_uncertainty_ns':self.sync['rtt_ns']/2,
                'method':'application send-to-receive estimate after clock-offset correction; includes serialization, queues and network',
                'channel_sample_age_estimates':ranges}

    def mapped_time(self, source_s):
        win_ns=int(self.anchor['windows_wall_ns'])+round((source_s-self.anchor['device_time_s'])*1e9)
        wall_ns=win_ns-self.sync['offset_ns']
        mono_ns=self.sync['local_mono_ns']+wall_ns-self.sync['local_wall_ns']
        return wall_ns,mono_ns,win_ns

    def sample_metadata(self, source_s):
        _,_,win_ns=self.mapped_time(source_s)
        return {'windows_wall_ns_est':win_ns,'sync_offset_ns':self.sync['offset_ns'],
                'sync_rtt_ns':self.sync['rtt_ns'],'sync_id':self.sync['sync_id'],
                'remote_run_id':self.run_id,'remote_simulated':int(self.status.get('simulated',False)),
                'clock_quality':self.status.get('clock_quality','unknown')}

    def _abort_clock(self, reason, **evidence):
        # Retain the original mapping and all raw STOP tail packets for audit.
        # No more derived samples are published after a timing fault.
        self.sync_invalid = True
        self.status.update(synced=False,clock_quality='invalid',clock_fault=reason)
        if self.timing_report is not None:
            self.timing_report.update(timing_valid=False,clock_fault=reason)
            self._write_timing_report()
        fields=dict(run_id=self.run_id,reason=reason,sync=self.sync,
                    last_good_probe_mono=self._last_good_clock,**evidence)
        self._log('clock_fault',**fields)
        stop_error = None
        try:
            self.stop()
        except Exception as exc:
            stop_error = str(exc)
            self._log('stop_failed',run_id=self.run_id,error=stop_error)
            # Closing TCP also tells a conforming bridge to end its old run.
            if self.sock:
                self.sock.close(); self.sock=None
            self.started=False
            self.status.update(connected=False,streaming=False)
        if self.session:
            self.session.event('emg_clock_fault',**fields,stop_error=stop_error)
        raise ClockSyncError(reason+'; EMG stopped. Press Sync & Start again'
                             + (f' (STOP acknowledgement failed: {stop_error})' if stop_error else ''))

    def _check_local_clock(self):
        before=time.perf_counter_ns()
        wall=time.time_ns()
        after=time.perf_counter_ns()
        if after-before>2_000_000:
            return  # a preempted clock read is not evidence of a system time step
        mono=(before+after)//2
        step=(wall-self.sync['local_wall_ns'])-(mono-self.sync['local_mono_ns'])
        if abs(step)>10_000_000:
            self._abort_clock('Ubuntu wall/monotonic clock changed by more than 10 ms',
                              local_clock_change_ns=step)

    def _check_clock_probe(self, probe):
        drift=probe['offset_ns']-self.sync['offset_ns']
        self.status['drift_ms']=drift/1e6
        # Offset uncertainty includes both probes' RTT / 2. A clock change
        # outside that entire interval cannot be explained by queueing alone.
        # Never update the run offset; three same-direction breaches end it.
        gate=max(20_000_000,3*self.sync['rtt_ns'])
        uncertainty=(probe['rtt_ns']+self.sync['rtt_ns'])/2
        breach=abs(drift)-uncertainty>10_000_000
        reliable=probe['rtt_ns']<=gate or breach
        sign=1 if drift>=0 else -1
        if reliable and breach:
            self._drift_streak=self._drift_streak+1 if sign==self._drift_sign else 1
            self._drift_sign=sign
            self.status['clock_quality']='suspect'
        elif reliable:
            self._drift_streak=self._drift_sign=0
            self._last_good_clock=time.monotonic()
            self.status['clock_quality']='good'
        else:
            self._drift_streak=self._drift_sign=0
            self.status['clock_quality']='network_uncertain'
        self._log('clock_health',run_id=self.run_id,drift_ns=drift,
                  uncertainty_ns=uncertainty,rtt_gate_ns=gate,reliable=reliable,
                  streak=self._drift_streak,quality=self.status['clock_quality'])
        if self._drift_streak>=3:
            self._abort_clock('Windows clock drift exceeds 10 ms on three reliable probes',probe=probe)

    def _decode(self,message):
        result={}
        for ch in message.get('channels',[]):
            i=ch.get('slot'); times=ch.get('device_time_s',[]); values=ch.get('values_v',[])
            if type(i) is not int or not 0<=i<7 or i in result or len(times)!=len(values) or len(times)>20000:
                raise ValueError('Malformed EMG channel batch')
            if any(not math.isfinite(float(t)) for t in times) or any(not math.isfinite(float(v)) for v in values):
                raise ValueError('Nonfinite EMG samples')
            if any(b<=a for a,b in zip(times,times[1:])):
                raise ValueError('EMG source times must increase')
            result[i]=(times,values)
        return result

    def poll(self):
        if self.sync_invalid:
            raise ClockSyncError('Clock alignment invalid; press Sync & Start again')
        self._check_local_clock()
        interval=5. if self.status.get('clock_quality')=='good' else 1.
        if time.monotonic()-self.last_health>=interval:
            try:
                probe=self._probe()
            except ValueError as exc:
                self._abort_clock('Invalid clock probe: '+str(exc))
            self._check_clock_probe(probe)
        if self.pending:
            return self._decode(self.pending.popleft())
        try:
            message=self._receive(time.monotonic()+.1)
        except TimeoutError:
            return {}
        if message.get('type')=='error':
            raise RuntimeError('Windows: '+str(message.get('message')))
        if message.get('type')!='data':
            raise ValueError('Unexpected streaming message')
        self._queue_data(message)
        return self._decode(self.pending.popleft())

    def stop(self):
        if self.started and self.sock:
            timeout=float(self.config.get('remote_stop_timeout_s',20.))
            if not math.isfinite(timeout) or not 0.1<=timeout<=120:
                raise ValueError('remote_stop_timeout_s must be between 0.1 and 120 seconds')
            self._log('stop_requested',run_id=self.run_id,timeout_s=timeout,
                      expected_next_seq=self.expected_seq)
            if self.journal:self.journal.flush()
            try:
                reply,_=self._rpc('stop',timeout=timeout,cancellable=False,run_id=self.run_id)
                if reply.get('run_id')!=self.run_id or reply.get('last_seq')!=self.expected_seq-1:
                    raise ValueError('Windows STOP count does not match received packets')
            except Exception as exc:
                # Do not issue a second STOP on a stream with a possibly late ACK.
                self._log('stop_failed',run_id=self.run_id,error=str(exc),
                          received_last_seq=self.expected_seq-1)
                if self.journal:self.journal.flush()
                self.sock.close();self.sock=None;self.started=False
                self.status.update(connected=False,streaming=False,stop_ack_received=False)
                raise
            self.status['stop_ack_received']=True
            self._log('stop',reply=reply)
            if self.session:
                self.session.event('emg_stop',run_id=self.run_id,reply=reply,
                                   timing_valid=not self.sync_invalid)
        self.started=False
        self.status['streaming']=False
        if self.journal:
            self.journal.flush()

    def drain_pending(self):
        if self.sync_invalid:
            return  # raw packets remain in the JSONL; mapping needs offline review
        while self.pending:
            yield self._decode(self.pending.popleft())

    def close(self):
        try:
            if self.started:
                self.stop()
        finally:
            if self.sock:
                self.sock.close();self.sock=None
            if self.journal:
                self.journal.close();self.journal=None
            self.status.update(connected=False,ready=False,synced=False,streaming=False)
