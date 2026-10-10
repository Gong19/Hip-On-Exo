"""Version 2: bounded native capture + 1 kHz grid, 5 x 60 ms per cycle.
No motor commands and no network transport. Monotonic acquisition time is authoritative.
"""
import csv
import os
import json
import math
import queue
import threading
import time
from collections import defaultdict, deque
from itertools import takewhile
from pathlib import Path
import numpy as np
try:
    import orjson
except ImportError:
    orjson=None

VERSION='hipexo-cycle/2'
FIELDS={
 'imu': [('ax_g','acceleration_including_gravity_x_g'),('ay_g','acceleration_including_gravity_y_g'),('az_g','acceleration_including_gravity_z_g'),
         ('gx_dps','angular_velocity_x_deg_s'),('gy_dps','angular_velocity_y_deg_s'),('gz_dps','angular_velocity_z_deg_s'),
         ('roll_deg','orientation_roll_deg'),('pitch_deg','orientation_pitch_deg'),('yaw_deg','orientation_yaw_deg'),
         ('rel_roll_deg','reference_relative_roll_deg'),('rel_pitch_deg','reference_relative_pitch_deg'),('rel_yaw_deg','reference_relative_yaw_deg'),('reference_id','reference_id'),('reference_valid','reference_valid'),('read_duration_ms','host_read_duration_ms'),('repeated_register_block','repeated_register_block')],
 'motor':[('q_rotor','rotor_position_rad'),('dq','rotor_angular_velocity_rad_s'),('temp','temperature_degC'),('merror','motor_error_code'),('read_duration_ms','host_round_trip_ms')],
 'force':[('V','sensor_voltage_V'),('kg','load_equivalent_kgf'),('adc_raw_count','ADC_raw_count')],
 'emg':[('raw_v','resampled_input_V'),('envelope_v','baseline_corrected_envelope_V'),('mvc_ratio','MVC_normalized_ratio'),('valid','source_valid')],
 'lidar':[('n_scan_points','scan_point_count'),('nonzero_points','nonzero_point_count'),('bad_packets','bad_packet_count'),('reported_hz','reported_scan_rate_Hz')]
}
STREAMS=[f'imu_{i}' for i in range(4)]+[f'motor_{i}' for i in range(2)]+[f'force_{i}' for i in range(2)]+[f'emg_{i}' for i in range(7)]+['lidar_L','lidar_R']
DISCRETE={'adc_raw_count','repeated_register_block','read_duration_ms','reference_id','reference_valid','valid','merror','temp'}
META_KEYS={'timestamp_basis','motor_transport','adc_reference_v','adc_range_code','conversion_version','sensor_id','session_id','is_rms','simulated','remote_simulated','baseline_v','mvc_v','sdk_unit','scale_to_v','value_units','i2c_bus','i2c_address','reference_id','source_sample_rate_hz','filter_description','sync_offset_ns','sync_rtt_ns','sync_id','remote_run_id','clock_quality'}
MAX_GAP_MS={'imu':40,'motor':40,'force':40,'emg':4,'lidar':250}

def family(stream):return stream.split('_')[0]
def compact(value):
    if orjson is not None:return orjson.dumps(value).decode('utf-8')
    return json.dumps(value,ensure_ascii=False,allow_nan=False,separators=(',',':'))
def write_numeric_rows(handle,writer,rows):
    """Numeric-only table fast path. JSON numbers are valid unquoted CSV cells."""
    if orjson is None:
        writer.writerows(rows);return
    if not isinstance(rows,list):rows=list(rows)
    if rows:
        payload=orjson.dumps(rows)
        handle.write(payload[2:-2].replace(b'],[',b'\n').replace(b'null',b'').decode('ascii')+'\n')

def clean(value):
    if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    if isinstance(value,(float,np.floating)) and not math.isfinite(value):return None
    if isinstance(value,np.generic):return value.item()
    return value

def resample(stream, samples, grid):
    """No extrapolation across gaps. Quality 0=missing,1=exact,2=interpolated,3=held discrete scan."""
    fields=FIELDS[family(stream)];out=np.full((len(grid),len(fields)),np.nan);quality=np.zeros(len(grid),dtype=int);age=np.full(len(grid),np.nan)
    if not samples:return out,quality,age
    # Only materialize this cycle's bracket, not the entire network-latency buffer.
    limit=int(grid[-1])+MAX_GAP_MS[family(stream)]*1_000_000
    samples=list(takewhile(lambda sample:sample[0]<=limit,samples))
    if not samples:return out,quality,age
    times=np.asarray([s[0] for s in samples],dtype=np.int64)
    vals=np.asarray([s[1] if not isinstance(s[1],dict) else [s[1].get(k,float('nan')) for k,_ in fields] for s in samples],dtype=float)
    right=np.searchsorted(times,grid,side='left');left=np.maximum(0,right-1);r=np.minimum(right,len(times)-1)
    exact=(right<len(times)) & (times[r]==grid)
    gap=(times[r]-times[left])/1e6
    supported=(right>0)&(right<len(times))&(gap<=MAX_GAP_MS[family(stream)])&(gap>0)
    if family(stream)=='emg':
        v=[k for k,_ in fields].index('valid');supported &= (vals[left,v]==1)&(vals[r,v]==1);exact &= vals[r,v]==1
    if family(stream)=='imu':
        ref=[k for k,_ in fields].index('reference_id')
        # No interpolation over a new reference pose; raw samples are still retained.
        supported &= vals[left,ref]==vals[r,ref]
    fraction=np.divide(grid-times[left],times[r]-times[left],out=np.zeros(len(grid),dtype=float),where=times[r]!=times[left])
    for j,(key,_) in enumerate(fields):
        delta=vals[r,j]-vals[left,j]
        if key.endswith('_deg'):
            finite=np.isfinite(delta);delta[finite]=(delta[finite]+180)%360-180
        estimate=vals[left,j]+fraction*delta
        if key in DISCRETE or family(stream)=='lidar':estimate=vals[left,j]
        if key.endswith('_deg'):
            finite=np.isfinite(estimate);estimate[finite]=(estimate[finite]+180)%360-180
        out[supported,j]=estimate[supported];out[exact,j]=vals[r[exact],j]
    quality[supported]=2;quality[exact]=1
    age[supported]=(grid[supported]-times[left[supported]])/1e6;age[exact]=0
    if family(stream)=='lidar':
        prior=np.searchsorted(times,grid,side='right')-1;idx=np.maximum(prior,0)
        hold=(prior>=0)&((grid-times[idx])/1e6<=MAX_GAP_MS['lidar'])
        out[hold]=vals[idx[hold]];quality[hold]=3;age[hold]=(grid[hold]-times[idx[hold]])/1e6
    return out,quality,age

class CycleRecorder:
    def __init__(self,path,on_error=None,on_cycle=None,latency_s=None,capacity=2048,start_mono_ns=None,start_wall_ns=None,block_on_full=False,inline=False):
        self.path=str(path);self.directory=Path(path).parent;self.on_error=on_error;self.on_cycle=on_cycle
        self.error=None;self.stats={};self._block_on_full=block_on_full;self._inline=inline;self._closed=False
        latency_s=float(os.environ.get("HIPEXO_SYNC_WAIT_MS","3000"))/1000 if latency_s is None else latency_s
        if not math.isfinite(latency_s) or not 0<=latency_s<=10:raise ValueError("Alignment wait must be between 0 and 10 seconds")
        self.latency_ns=int(latency_s*1e9)
        self.start_mono_ns=time.perf_counter_ns() if start_mono_ns is None else start_mono_ns
        self.start_wall_ns=time.time_ns() if start_wall_ns is None else start_wall_ns
        self.stop_ns=None;self._stop=threading.Event();self._queue=queue.Queue(capacity)
        self._budget_lock=threading.Lock();self._queued_bytes=0;self._images=deque()
        self._image_jobs=queue.Queue(64)
        self._image_thread=threading.Thread(target=self._write_images,name='image-writer',daemon=True)
        self._samples=defaultdict(deque);self._native={};self._writeback_offsets={};self._metadata={};self._last_time={};self._cycle=0
        self.metrics=dict(accepted_frames=0,late_frames=0,out_of_order_frames=0,queue_overflow=0,queue_backpressure_waits=0,max_queue_depth=0,cycles=0,callback_errors=0,buffer_overflow=0,camera_images=0,camera_images_written=0,late_images=0,camera_device_frame_gaps=0)
        self.columns=['cycle_id','segment_id_0_to_4','sample_in_segment_0_to_59','sample_index','time_from_record_start_ms','mapped_utc_ns','within_recording']
        for stream in STREAMS:self.columns += [f'{stream}__{label}' for _,label in FIELDS[family(stream)]]+[f'{stream}__quality',f'{stream}__source_age_ms']
        self._fh=open(path,'w',buffering=256*1024,newline='',encoding='utf-8');self._csv=csv.writer(self._fh);self._csv.writerow(self.columns)
        self._packets=(self.directory/'cycles.jsonl').open('w',encoding='utf-8')
        self._events=(self.directory/'source_metadata.jsonl').open('w',encoding='utf-8')
        self._image_index=(self.directory/'camera_images.jsonl').open('w',encoding='utf-8');self._last_camera={}
        schema=dict(schema=VERSION,grid_hz=1000,segment_ms=60,segments_per_cycle=5,cycle_ms=300,alignment_wait_ms=latency_s*1000,
            columns=self.columns,start_mono_ns=self.start_mono_ns,start_wall_ns=self.start_wall_ns,
            quality_codes={'0':'missing','1':'exact timestamp match, not proof of native 1kHz','2':'linear interpolation, angles use shortest arc','3':'held LiDAR scan summary'},
            max_gap_ms=MAX_GAP_MS,field_mapping=FIELDS,grid_decimal_places=9,
            notes=['cycles.jsonl indexes complete combined CSV byte ranges; live callbacks contain full rows.',
                   'Image paths may be indexed before asynchronous encoding finishes; Stop drains image jobs.',
                   'IMU acceleration includes gravity. Camera images are indexed separately, never interpolated.',
                   'EMG input is already processed onto 1kHz. Vendor RMS and raw EMG are distinct; see source metadata.',
                   'Local sensor timestamps are host read times, not hardware trigger synchronization.',
                   'Latency buffer is finite. Late native data retained; already emitted cycles not silently revised.',
                   'Final cycle padded to 300 rows with within_recording=0 beyond Stop. Null means unavailable.',
                   'Sensor native CSVs preserve host/mapped time and selected essential measurements; static metadata logged on change.'])
        (self.directory/'pipeline_schema.json').write_text(json.dumps(schema,ensure_ascii=False,indent=2),encoding='utf-8')
        descriptions={
            'imu':'IMU 加速度含重力；角速度为传感器轴；姿态为设备解算欧拉角；relative 为参考姿态旋转',
            'motor':'电机转子反馈；输出轴角度=转子角度/减速比；host_round_trip_ms 为主机请求到有效回复的往返耗时，不能直接当作单向延迟扣除',
            'force':'力传感器电压与标定载荷（kgf 等效）；牛顿值=kgf×9.80665',
            'emg':'EMG 处理网格；原始/RMS 模式、单位和滤波版本见 source_metadata.jsonl；原始设备包在会话 EMG 日志',
            'lidar':'雷达扫描摘要；完整点云使用 Raw scans 独立保存，非 1000Hz 扫描'}
        with (self.directory/'COLUMNS_字段说明.csv').open('w',newline='',encoding='utf-8-sig') as f:
            writer=csv.writer(f);writer.writerow(['CSV列名','数据含义与来源'])
            for column in self.columns:
                prefix=column.split('__')[0]
                if column.endswith('__quality'):description='0缺失；1时间戳恰好匹配（不代表原始1000Hz）；2插值；3保持上一帧雷达摘要'
                elif column.endswith('__source_age_ms'):description='目标时刻距离前一个源样本的毫秒数'
                elif '__' in column:description=descriptions[family(prefix)]
                else:description={'cycle_id':'从0开始的300ms周期编号','segment_id_0_to_4':'周期内60ms片段编号0到4','sample_in_segment_0_to_59':'片段内1ms采样点编号0到59','sample_index':'从本次Record开始的1ms网格索引','time_from_record_start_ms':'相对本次Record起点的毫秒数','mapped_utc_ns':'按Record起点UTC与单调时钟映射的纳秒数','within_recording':'1属于本次记录；0为末周期补齐区域'}[column]
                writer.writerow([column,description])
        self._image_thread.start()
        self._thread=None
        if not inline:
            self._thread=threading.Thread(target=self._run,name='cycle-writer',daemon=True);self._thread.start()

    def _fail(self,message):
        if self.error is None:
            self.error=message
            if self.stop_ns is None:self.stop_ns=time.perf_counter_ns()
            self._stop.set()
            if self.on_error:
                try:self.on_error(message)
                except Exception:pass  # Keep failure state and cleanup even if reporting fails.

    def _enqueue(self,item,size):
        if self._inline:return self.accept_owned(item)
        if self.error or self._stop.is_set():return False
        with self._budget_lock:
            if self._queued_bytes+size>64*1024*1024:
                self.metrics['queue_overflow']+=1;self._fail('Recording queue exceeds 64 MiB; incomplete');return False
            self._queued_bytes+=size
        queued=(item,size,time.perf_counter_ns())
        try:
            if self._block_on_full:
                # The private IPC reader waits; socket backpressure then reaches
                # the existing bounded parent queue. Capture threads never wait
                # here. No budget or queue limit is increased.
                while True:
                    if self.error or self._stop.is_set():
                        with self._budget_lock:self._queued_bytes-=size
                        return False
                    try:
                        self._queue.put(queued,timeout=.05);break
                    except queue.Full:self.metrics['queue_backpressure_waits']+=1
            else:self._queue.put_nowait(queued)
            self.metrics['max_queue_depth']=max(self.metrics['max_queue_depth'],self._queue.qsize())
            return True
        except queue.Full:
            with self._budget_lock:self._queued_bytes-=size
            self.metrics['queue_overflow']+=1;self._fail('Recording queue full; incomplete');return False

    def enqueue_frames(self,stream,times,frames,monos,ids):
        if not frames:return True
        if not len(times)==len(frames)==len(monos)==len(ids):raise ValueError('Frame/timestamp lengths differ')
        item=(stream,list(times),[dict(f) for f in frames],list(monos),list(ids))
        return self._enqueue(item,len(frames)*(len(frames[0])*48+128))

    def enqueue_image(self,depth,timing):
        return self._enqueue(('__image',depth.copy(),dict(timing)),depth.nbytes+2048)

    def _write_images(self):
        from hipexo_image_writer import ImageWriter
        encoder=None;encoder_broken=False
        try:
            while True:
                job=self._image_jobs.get()
                try:
                    if job is None:return
                    if encoder_broken:continue
                    if encoder is None:encoder=ImageWriter()
                    path,depth=job;encoder.write(path,depth)
                    self.metrics['camera_images_written']+=1
                except Exception as exc:
                    encoder_broken=True;self._fail('Camera image write failed: '+str(exc))
                finally:self._image_jobs.task_done()
        finally:
            if encoder:encoder.close(force=encoder_broken)

    def _accept_image(self,item):
        _,depth,timing=item
        mono=timing['host_frame_received_mono_ns'];idx=self.metrics['camera_images']
        folder=self.directory/'images';folder.mkdir(exist_ok=True)
        relative='images/'+self.directory.name.removesuffix('__Recording')+f'__Camera_depth_{idx:08d}.png'
        try:self._image_jobs.put_nowait((str(self.directory/relative),depth))
        except queue.Full:raise BufferError('Image writer queue full; recording incomplete')
        event=dict(timing,path=relative,image_index=idx)
        self._image_index.write(compact(clean(event))+'\n');self.metrics['camera_images']+=1
        sid=timing.get('camera_stream_id');number=timing.get('device_frame_number')
        if number is not None:
            if sid in self._last_camera:self.metrics['camera_device_frame_gaps']+=max(0,number-self._last_camera[sid]-1)
            self._last_camera[sid]=number
        if mono<self.start_mono_ns+self._cycle*300_000_000:self.metrics['late_images']+=1
        else:
            if len(self._images)>=256:raise BufferError('Camera index buffer full; check timestamps')
            self._images.append(event)
        t=timing['host_frame_received_wall_ns']/1e6
        stats=self.stats.setdefault('vision',dict(samples=0,valid_samples=0,first_t_ms=t,last_t_ms=t,fields=['image_path'],simulated=False))
        stats['samples']+=1;stats['valid_samples']+=1;stats['last_t_ms']=t;stats['simulated'] |= bool(timing.get('simulated',False))

    def _accept(self,item):
        if item[0]=='__image':return self._accept_image(item)
        stream,times,frames,monos,ids=item
        kind=family(stream)
        if stream not in STREAMS:
            for t,m,f,sid in zip(times,monos,frames,ids):self._events.write(compact(clean(dict(kind='ungridded',stream=stream,t_wall_ms=t,t_mono_ns=m,sample_id=sid,values=f)))+'\n')
            return
        if stream not in self._native:
            f=(self.directory/(self.directory.name.removesuffix('__Recording')+'__'+stream+'__native.csv')).open('w',buffering=256*1024,newline='',encoding='utf-8')
            writer=csv.writer(f);writer.writerow(['mapped_utc_ms','source_or_host_mono_ns','source_sample_id']+[label for _,label in FIELDS[kind]])
            self._native[stream]=(f,writer)
        stats=self.stats.setdefault(stream,dict(samples=0,valid_samples=0,first_t_ms=times[0],last_t_ms=times[0],fields=[k for k,_ in FIELDS[kind]],simulated=False))
        keys=[k for k,_ in FIELDS[kind]]
        missing=float('nan')
        values=[[frame.get(k,missing) for k in keys] for frame in frames]
        write_numeric_rows(*self._native[stream],([t,mono,sid]+v for t,mono,sid,v in zip(times,monos,ids,values)))
        previous=stats.get('last_mono_ns',monos[0])
        stats['max_interval_ms']=max(stats.get('max_interval_ms',0),max((b-a)/1e6 for a,b in zip([previous]+monos[:-1],monos)))
        stats.setdefault('first_mono_ns',monos[0]);stats['last_mono_ns']=monos[-1]
        stats['identical_register_reads']=stats.get('identical_register_reads',0)+sum(int(f.get('repeated_register_block',0)) for f in frames)
        stats['samples']+=len(frames)
        stats['valid_samples']+=sum(f.get('valid')==1 for f in frames) if kind=='emg' else len(frames)
        stats['first_t_ms']=min(stats['first_t_ms'],min(times));stats['last_t_ms']=max(stats['last_t_ms'],max(times))
        stats['simulated'] |= any(f.get('simulated',0) or f.get('remote_simulated',0) for f in frames)
        self.metrics['accepted_frames']+=len(frames)
        threshold=self.start_mono_ns+self._cycle*300_000_000
        samples=self._samples[stream];last=self._last_time.get(stream,-1)
        for mono,frame,value in zip(monos,frames,values):
            meta={k:v for k,v in frame.items() if k in META_KEYS}
            if meta!=self._metadata.get(stream):
                self._events.write(compact(dict(kind='metadata',stream=stream,source_mono_ns=mono,values=clean(meta)))+'\n');self._metadata[stream]=meta
            if mono<threshold:
                self.metrics['late_frames']+=1
                continue
            if mono<=last:
                self.metrics['out_of_order_frames']+=1
                continue
            last=mono;samples.append((mono,value))
        self._last_time[stream]=last
        if len(samples)>16000:
            self.metrics['buffer_overflow']+=1
            raise BufferError('Alignment buffer limit exceeded; native data retained, no silent eviction')

    def _emit(self):
        first=self._cycle*300
        indices=np.arange(first,first+300,dtype=np.int64);grid=self.start_mono_ns+indices*1_000_000
        within=np.ones(300,dtype=int) if self.stop_ns is None else (grid<self.stop_ns).astype(int)
        parts=[indices//300,(indices%300)//60,indices%60,indices,self.start_wall_ns+indices*1_000_000]
        # Keep integer ns as Python ints; avoid float coercion of epoch ns.
        arrays=[];coverage={}
        for stream in STREAMS:
            values,q,age=resample(stream,self._samples[stream],grid)
            values[within==0]=np.nan;q[within==0]=0;age[within==0]=np.nan
            arrays.extend([values,q[:,None],age[:,None]])
            coverage[stream]=int(np.count_nonzero(q))
            samples=self._samples[stream]
            while len(samples)>1 and samples[1][0]<=grid[-1]:samples.popleft()
        data=np.concatenate(arrays,axis=1)
        # Grid values use 9 decimal places; native files retain full input precision.
        objects=np.round(data,9).astype(object);objects[~np.isfinite(data)]=None
        rows=[]
        for i,values in enumerate(objects.tolist()):
            rows.append([self._cycle,i//60,i%60,int(indices[i]),int(indices[i]),int(parts[4][i]),int(within[i])]+values)
        csv_start=self._fh.tell()
        write_numeric_rows(self._fh,self._csv,rows)
        csv_end=self._fh.tell()
        images=[v for v in self._images if grid[0]<=v['host_frame_received_mono_ns']<grid[-1]+1_000_000]
        while self._images and self._images[0]['host_frame_received_mono_ns']<grid[-1]+1_000_000:self._images.popleft()
        packet=dict(camera_images=images,schema=VERSION,cycle_id=self._cycle,start_mono_ns=int(grid[0]),start_utc_ns=int(parts[4][0]),
                    grid_hz=1000,segment_ms=60,segments=5,rows=rows,coverage_samples=coverage,
                    emitted_mono_ns=time.perf_counter_ns())
        # One authoritative payload on disk. Index each complete CSV block;
        # the live callback still receives the full 300-row packet.
        manifest={k:v for k,v in packet.items() if k!='rows'}
        manifest.update(payload_storage='combined_csv',csv_byte_start=csv_start,csv_byte_end=csv_end,row_count=len(rows))
        self._packets.write(compact(manifest)+'\n')
        self._fh.flush();self._packets.flush();self._events.flush();self._image_index.flush()
        for f,_ in self._native.values():f.flush()
        from hipexo_writeback import submit
        # Keep kernel writeback paced with each cycle instead of allowing
        # minutes of dirty page cache to trigger a large global flush burst.
        submitted=True
        for handle in [self._fh,self._packets,self._events,self._image_index]+[f for f,_ in self._native.values()]:
            fd=handle.fileno();end=os.lseek(fd,0,os.SEEK_CUR);begin=self._writeback_offsets.get(fd,0)
            if end>begin:submitted=submit(fd,begin,end-begin) and submitted
            self._writeback_offsets[fd]=end
        self.metrics['async_writeback_enabled']=submitted
        if self.on_cycle:
            try:self.on_cycle(packet)
            except Exception as exc:
                self.metrics['callback_errors']+=1;self._fail('Cycle consumer failed: '+str(exc))
        self._cycle+=1;self.metrics['cycles']=self._cycle

    def _run(self):
        profile=None
        if os.environ.get('HIPEXO_PROFILE_PATH'):
            import cProfile
            profile=cProfile.Profile();profile.enable()
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    item,size,accepted_at=self._queue.get(timeout=.02)
                    try:self._accept(item)
                    finally:
                        with self._budget_lock:self._queued_bytes-=size
                except queue.Empty:accepted_at=time.perf_counter_ns()
                now=time.perf_counter_ns()
                # Do not emit past data still waiting in our ordered input queue.
                if not self._queue.empty():now=min(now,accepted_at)
                while now>=self.start_mono_ns+(self._cycle+1)*300_000_000+self.latency_ns:
                    if self._stop.is_set():break
                    self._emit()
            while self.start_mono_ns+self._cycle*300_000_000 < self.stop_ns:self._emit()
        except Exception as exc:self._fail('Cycle recording failed: '+str(exc))
        finally:
            self._finish()
            if profile:
                profile.disable();profile.dump_stats(os.environ['HIPEXO_PROFILE_PATH'])

    def accept_owned(self,item):
        """Consume a private-IPC-owned packet without cloning/queuing it again."""
        if not self._inline:raise RuntimeError('Owned packets require inline writer mode')
        if self.error or self._stop.is_set():return False
        try:
            self._accept(item);return True
        except Exception as exc:
            self._fail('Cycle recording failed: '+str(exc));return False

    def emit_ready(self,watermark_ns):
        try:
            while not self._stop.is_set() and watermark_ns>=self.start_mono_ns+(self._cycle+1)*300_000_000+self.latency_ns:
                self._emit()
        except Exception as exc:self._fail('Cycle recording failed: '+str(exc))

    def _finish(self):
        if self._closed:return
        self._closed=True
        self._image_jobs.put(None)
        self._image_thread.join()
        for f in [self._fh,self._packets,self._events,self._image_index]+[v[0] for v in self._native.values()]:
            try:f.close()
            except Exception as exc:self._fail('Recording close failed: '+str(exc))
        try:
            for stats in self.stats.values():
                span=stats.get('last_mono_ns',0)-stats.get('first_mono_ns',0)
                stats['accepted_rate_hz']=(stats['samples']-1)*1e9/span if span>0 else None
            (self.directory/'pipeline_quality.json').write_text(json.dumps(dict(self.metrics,error=self.error,stop_mono_ns=self.stop_ns,streams=self.stats),indent=2),encoding='utf-8')
        except Exception as exc:self._fail('Quality report failed: '+str(exc))

    def stop(self,timeout=15):
        if self.stop_ns is None:self.stop_ns=time.perf_counter_ns()
        self._stop.set()
        if self._inline:
            if not self._closed:
                try:
                    while self.start_mono_ns+self._cycle*300_000_000 < self.stop_ns:self._emit()
                except Exception as exc:self._fail('Cycle recording failed: '+str(exc))
                finally:self._finish()
        else:
            self._thread.join(timeout)
            if self._thread.is_alive():self._fail('Cycle writer still draining; keep application open')
        return self.error is None
