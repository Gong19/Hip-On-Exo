"""Manual acquisition benchmark; --motors-zero sends only zero-output commands."""
import os,time,json,tempfile,sys
from pathlib import Path
from PyQt5 import QtWidgets
import hipexo_monitor as m
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
app=QtWidgets.QApplication([])
out=Path('evidence/live-acquisition');out.mkdir(exist_ok=True)
m.EXPORT_DIR=str(out.resolve());dm=m.DataManager();dm.set_export_dir(str(out.resolve()),subject_id='BENCH_NO_SUBJECT',location='Workstation')
imu=m.ImuWorker(dm);force=m.ForceSensorWorker(dm);camera=m.VisionWorker(dm)
motor=m.MotorWorker(m.MOTOR_DEVICES,dm) if "--motors-zero" in sys.argv else None
messages=[];memory_samples=[];last_memory=0
def rss(pid):
    try:
        for line in Path(f'/proc/{pid}/status').read_text().splitlines():
            if line.startswith('VmRSS:'):return int(line.split()[1])/1024
    except OSError:pass
    return 0.
def child_pids():
    result=set()
    for path in Path(f'/proc/{os.getpid()}/task').glob('*/children'):
        try:result.update(int(x) for x in path.read_text().split())
        except OSError:pass
    return result
for w in [imu,force,camera]:w.sig_status.connect(messages.append)
if motor:motor.sig_error.connect(messages.append)
duration=float(sys.argv[sys.argv.index("--seconds")+1]) if "--seconds" in sys.argv else 12
start=time.perf_counter();dm.start_recording();imu.start();force.start();camera.start()
if motor:motor.start_monitoring()
while time.perf_counter()-start<duration:
    app.processEvents();time.sleep(.01)
    if time.perf_counter()-last_memory>=5:
        last_memory=time.perf_counter()
        memory_samples.append(dict(seconds=last_memory-start,parent_MiB=rss(os.getpid()),
            children_MiB=(sum(rss(pid) for pid in child_pids()) or None)))
imu.shutdown();force.shutdown();camera.shutdown()
if motor:motor.shutdown()
ok=dm.stop_recording()
result={'motor_constructed':motor is not None,'motor_zero_output_only':motor is not None,'motor_sdk_releases_gil':getattr(m,'_MOTOR_GIL_RELEASED',False),'seconds':time.perf_counter()-start,'csv_complete':ok,'path':dm._record_path,'rates':{},'messages':messages[-40:],'memory_samples':memory_samples}
for name in [f'imu_{i}' for i in range(4)]+['force_0','force_1']+(['motor_0','motor_1'] if motor else []):
    t,v=dm.snapshot(name+'_t_mono_ns')
    if len(v)>1:
        import numpy as np
        delta=np.diff(v)/1e6
        result['rates'][name]={'buffer_samples':len(v),'achieved_hz':(len(v)-1)*1e9/(v[-1]-v[0]),'interval_ms_p50':float(np.median(delta)),'interval_ms_p99':float(np.percentile(delta,99)),'max_gap_ms':float(delta.max())}
    else:result['rates'][name]={'samples':len(v)}
result['quality']=json.loads((Path(dm._record_path).parent/'pipeline_quality.json').read_text())
Path('evidence/live_benchmark.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False,indent=2))
dm._flush_timer.stop();dm._mem_timer.stop()
