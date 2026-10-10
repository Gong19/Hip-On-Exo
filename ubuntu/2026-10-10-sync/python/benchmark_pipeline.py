"""Synthetic only: no hardware or networking. Retains a performance report."""
import argparse,json,time,tempfile,resource
from pathlib import Path
import numpy as np
from hipexo_pipeline import CycleRecorder,STREAMS,FIELDS,family
p=argparse.ArgumentParser();p.add_argument('--seconds',type=int,default=10);p.add_argument('--isolated',action='store_true');p.add_argument('--images',action='store_true');p.add_argument('--image-noise',action='store_true');p.add_argument('--cycles-callback',action='store_true');p.add_argument('--output',default='evidence/synthetic_benchmark.json');a=p.parse_args()
with tempfile.TemporaryDirectory() as folder:
    start=time.perf_counter_ns();cpu=time.process_time();rss=[]
    callback=(lambda packet:None) if a.cycles_callback else None
    def memory(pid):
        try:
            for line in Path(f'/proc/{pid}/status').read_text().splitlines():
                if line.startswith('VmRSS:'):return int(line.split()[1])/1024
        except OSError:return None
    if a.isolated:
        from hipexo_recording_process import ProcessCycleRecorder
        r=ProcessCycleRecorder(Path(folder)/'SIMULATED.csv',on_cycle=callback);start=r.start_mono_ns
    else:r=CycleRecorder(Path(folder)/'SIMULATED.csv',start_mono_ns=start)
    child_cpu_before=resource.getrusage(resource.RUSAGE_CHILDREN)
    image_count=0;rng=np.random.default_rng(1000)
    import os
    for tick in range(a.seconds*50):
        if tick%250==0:rss.append(dict(seconds=tick/50,parent_MiB=memory(os.getpid()),writer_MiB=memory(r._process.pid) if a.isolated else None))
        for stream in STREAMS:
            kind=family(stream)
            hz=1000 if kind in ('emg','force','motor') else 200 if kind=='imu' else 50
            n=hz//50;first=tick*n
            frames=[]
            for j in range(n):
                f={k:float(np.sin((first+j)/hz)) for k,_ in FIELDS[kind]}
                f.update(valid=1,reference_valid=0,reference_id=0,simulated=1)
                frames.append(f)
            ns=[start+round((first+j)/hz*1e9) for j in range(n)]
            r.enqueue_frames(stream,[v/1e6 for v in ns],frames,ns,list(range(first,first+n)))
        if a.images and tick%3==0:
            depth=np.broadcast_to(np.arange(640,dtype=np.uint16)*4,(480,640)).copy()
            if a.image_noise:depth+=rng.integers(0,32,size=depth.shape,dtype=np.uint16)
            r.enqueue_image(depth,dict(host_frame_received_mono_ns=start+tick*20_000_000,host_frame_received_wall_ns=r.start_wall_ns+tick*20_000_000,device_frame_number=image_count,camera_stream_id='SIMULATED',simulated=True))
            image_count+=1
        delay=(start+(tick+1)*20_000_000-time.perf_counter_ns())/1e9
        if delay>0:time.sleep(delay)
    stop_start=time.perf_counter();ok=r.stop()
    result=dict(image_noise=a.image_noise,error=r.error,memory_samples=rss,expected_frames=a.seconds*11900,expected_images=image_count,simulated=True,seconds=a.seconds,ok=ok,cpu_seconds_parent=time.process_time()-cpu,cpu_seconds_children=resource.getrusage(resource.RUSAGE_CHILDREN).ru_utime+resource.getrusage(resource.RUSAGE_CHILDREN).ru_stime-child_cpu_before.ru_utime-child_cpu_before.ru_stime,
        elapsed_seconds=(time.perf_counter_ns()-start)/1e9,stop_drain_seconds=time.perf_counter()-stop_start,
        peak_rss_MiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
        output_bytes=sum(x.stat().st_size for x in Path(folder).rglob('*') if x.is_file()),metrics=r.metrics)
    Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    if not ok:raise SystemExit(1)
