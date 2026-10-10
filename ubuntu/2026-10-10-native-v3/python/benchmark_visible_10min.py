"""Full GUI bench recording, zero motor output, 600 s after sensor warm-up."""
import os,sys,time,json,datetime,subprocess,traceback
from pathlib import Path
from unittest.mock import patch
LIVE=Path('/home/gong/Desktop/exo-control (copy)/python');sys.path.insert(0,str(LIVE))
os.environ['QT_QPA_PLATFORM']='xcb'
os.environ['HIPEXO_MOTOR_NATIVE_HZ']='1000'
os.environ.setdefault('DISPLAY',':0')
os.environ.setdefault('HIPEXO_SDK_LIB_DIR',str(LIVE.parent/'lib'))
from PyQt5 import QtWidgets,QtCore
import numpy as np
import hipexo_monitor as m
stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
root=Path('/home/gong/Desktop')/f'HiPExo_Visible_10min_{stamp}';root.mkdir();data_root=root/'data';data_root.mkdir()
work=Path(__file__).parent/'evidence';(work/'latest_run_root.txt').write_text(str(root))
m.EXPORT_DIR=str(data_root);m.SESSION_STATE_PATH=str(work/'endurance_session_state.json')
app=QtWidgets.QApplication([])
assert app.platformName()=='xcb','Visible desktop X11 backend required'
app.setApplicationName('HiPExo Visible 10-minute Test');m._S=m._compute_scale();m.apply_theme(app,'light')
redraws={name:0 for name in ['Motor','IMU','Force']};curve_updates={name:0 for name in redraws};phase=[None]
# Instrument existing refresh/setData calls; do not change refresh rate or buffers.
for name,cls in [('Motor',m.MotorPanel),('IMU',m.ImuPanel),('Force',m.ForceSensorPanel)]:
 original=cls._refresh
 def refresh(self,_name=name,_original=original):
  if not self.isVisible():return _original(self)
  phase[0]=_name
  try:
   result=_original(self);redraws[_name]+=1;return result
  finally:phase[0]=None
 cls._refresh=refresh
import pyqtgraph as pg
original_set_data=pg.PlotDataItem.setData
def set_data(self,*args,**kwargs):
 result=original_set_data(self,*args,**kwargs)
 if phase[0] in curve_updates:curve_updates[phase[0]]+=1
 return result
pg.PlotDataItem.setData=set_data
class PaintCounter(QtCore.QObject):
 def __init__(self):super().__init__();self.count=0
 def eventFilter(self,obj,event):
  if event.type()==QtCore.QEvent.Paint:self.count+=1
  return False
paint_counter=PaintCounter();app.installEventFilter(paint_counter)
with patch.object(m,'_ensure_motor_mode_for_ports',lambda *a:None):window=m.MainWindow()
window.setWindowTitle('HiPExo — Visible 10-minute zero-output recording')
window.showFullScreen();window.raise_();window.activateWindow();app.processEvents()
expose_deadline=time.monotonic()+10
while not window.windowHandle().isExposed() and time.monotonic()<expose_deadline:
 app.processEvents();time.sleep(.02)
assert window.isVisible() and window.windowHandle().isExposed(),'Window must be visible on desktop'
window._panels['Motor'].btn_start.setEnabled(False)
window._current_subject='Bench_NoParticipant';window._current_location='UbuntuWorkstation_Visible';window._update_session_label()
(root/'screenshots').mkdir()
window._dm.set_export_dir(str(data_root),subject_id='Bench_NoParticipant',location='UbuntuWorkstation_Visible')
messages=[];events=(root/'events.jsonl').open('w')
def event(kind,**value):
 row=dict(kind=kind,mono_ns=time.perf_counter_ns(),utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),**value);events.write(json.dumps(row,ensure_ascii=False)+'\n');events.flush()
def status(message):event('status',message=str(message))
for w in [window._imu_worker,window._force_worker,window._vision_worker]:w.sig_status.connect(status)
window._motor_worker.sig_error.connect(status);window._dm.sig_record_error.connect(status)
window._toggle_recording();assert window._dm._recording
window._imu_worker.start();window._force_worker.start();window._motor_worker.start_monitoring()
# No camera enumerating retry load: preflight found no camera; EMG and LiDAR absent.
streams=[f'imu_{i}' for i in range(4)]+['force_0','force_1','motor_0','motor_1']
names=['Motor','IMU','Force'];samples=[];last_page=None;screenshots_taken=set()
for name in names:window._set_panel_state(name,'State: MONITORING / recording')
window._lbl_status.setText('10-minute visible test · live waveforms · zero motor output')

def snapshot(elapsed):
 rates={}
 for name in streams:
  _,values=window._dm.snapshot(name+'_t_mono_ns');a=np.asarray(values[-2000:],dtype=np.int64)
  if len(a)>1:rates[name]=dict(rolling_hz=float((len(a)-1)*1e9/(a[-1]-a[0])),last_age_ms=(time.perf_counter_ns()-int(a[-1]))/1e6)
 process_rows=[]
 raw=subprocess.check_output(['ps','-eo','pid=,ppid=,rss=,comm='],text=True)
 all_rows={}
 for line in raw.splitlines():
  fields=line.split(None,3)
  if len(fields)==4:all_rows[int(fields[0])]=(int(fields[1]),int(fields[2]),fields[3])
 selected={os.getpid()}
 for _ in range(5):selected|={pid for pid,(parent,_,_) in all_rows.items() if parent in selected}
 for pid in selected:
  if pid in all_rows:
   parent,rss,name=all_rows[pid]
   if name!='ps':process_rows.append(dict(pid=pid,process=name,rss_mib=rss/1024))
 meminfo={line.split(':')[0]:int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines() if len(line.split())>=2}
 vmstat=dict(line.split() for line in Path('/proc/vmstat').read_text().splitlines())
 gui=dict(visible=window.isVisible(),exposed=window.windowHandle().isExposed(),minimized=window.isMinimized(),platform=app.platformName(),page=names[last_page] if last_page is not None else None,redraws=dict(redraws),curve_updates=dict(curve_updates),paint_events=paint_counter.count,timers={n:window._panels[n]._timer.isActive() for n in names})
 stat=dict(gui=gui,system_memory=dict(available_mib=meminfo['MemAvailable']/1024,swap_used_mib=(meminfo['SwapTotal']-meminfo['SwapFree'])/1024,pswpin=int(vmstat.get('pswpin',0)),pswpout=int(vmstat.get('pswpout',0))),desktop_process_rss_mib={str(pid)+':'+row[2]:row[1]/1024 for pid,row in all_rows.items() if row[2] in ['Xorg','gnome-shell']},elapsed_s=elapsed,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rates=rates,rss_total_mib=sum(x['rss_mib'] for x in process_rows),processes=process_rows,motor_transport=window._motor_worker.transport_stats,record_path=window._dm._record_path)
 samples.append(stat)
 temp=root/'status.json.tmp';temp.write_text(json.dumps(stat,indent=2));temp.replace(root/'status.json')
 with (root/'health.jsonl').open('a') as f:f.write(json.dumps(stat)+'\n')
 return stat

error=None;core_start=None;core_end=None;record_begin=time.perf_counter();iterations=0;max_loop_ms=0;last_loop=time.perf_counter()
try:
 deadline=time.perf_counter()+20;ready_at=None
 while time.perf_counter()<deadline:
  app.processEvents();time.sleep(.01)
  ready=all(window._imu_worker.is_online(i) for i in range(4)) and all(window._force_worker.is_online(i) for i in range(2)) and all(window._motor_worker.is_online(i) for i in range(2))
  if ready and ready_at is None:ready_at=time.perf_counter()
  if ready_at is not None and time.perf_counter()-ready_at>=3:break
 else:raise RuntimeError('Not all eight required channels became ready')
 core_start=time.perf_counter_ns();last_loop=time.perf_counter();deadline=core_start+600_000_000_000;last_health=-10
 event('core_start',duration_s=600,mode='zero-output bench',streams=streams,motor_target_hz=1000)
 print('STARTED',root,flush=True)
 while time.perf_counter_ns()<deadline:
  elapsed=(time.perf_counter_ns()-core_start)/1e9;page=int(elapsed//30)%len(names)
  if page!=last_page:window._switch_panel(names[page]);last_page=page;event('page',name=names[page])
  now=time.perf_counter();max_loop_ms=max(max_loop_ms,(now-last_loop)*1000);last_loop=now
  app.processEvents();iterations+=1;time.sleep(.005)
  if elapsed>5 and (names[page] not in screenshots_taken or (elapsed>=595 and 'final' not in screenshots_taken)):
   tag=names[page] if names[page] not in screenshots_taken else 'final'
   app.primaryScreen().grabWindow(int(window.winId())).save(str(root/'screenshots'/f'{int(elapsed):03d}s_{tag}.png'));screenshots_taken.add(tag)
  assert window.isVisible() and window.windowHandle().isExposed() and not window.isMinimized(),'Visible waveform test window hidden/minimized'
  assert not window._motor_worker.running.is_set(),'Unexpected active motor control'
  window._lbl_status.setText(f'LIVE TEST {elapsed/60:.1f}/10 min · {names[page]} waveforms · zero motor output')
  if elapsed-last_health>=10:
   stat=snapshot(elapsed);last_health=elapsed
   print(json.dumps(dict(elapsed_s=round(elapsed,1),rss_total_mib=round(stat['rss_total_mib'],1),rates={k:round(v['rolling_hz'],2) for k,v in stat['rates'].items()})),flush=True)
  if not window._dm._recording:raise RuntimeError('Recorder stopped unexpectedly')
 core_end=deadline;event('core_end',actual_elapsed_s=(time.perf_counter_ns()-core_start)/1e9)
except BaseException as exc:
 error=f'{type(exc).__name__}: {exc}';event('error',message=error);traceback.print_exc()
finally:
 window.close();app.processEvents();events.close()
 directory=Path(window._dm._record_path).parent
 quality=json.loads((directory/'pipeline_quality.json').read_text()) if (directory/'pipeline_quality.json').exists() else None
 result=dict(root=str(root),record_directory=str(directory),core_start_mono_ns=core_start,core_end_mono_ns=core_end,requested_core_seconds=600,record_and_shutdown_seconds=time.perf_counter()-record_begin,error=error,recording_still_active=window._dm._recording,quality=quality,qt_offscreen=False,qt_platform=app.platformName(),gui_redraws=redraws,gui_curve_updates=curve_updates,gui_paint_events=paint_counter.count,screen_geometry=[app.primaryScreen().geometry().width(),app.primaryScreen().geometry().height()],qt_loop_iterations=iterations,max_qt_loop_interval_ms=max_loop_ms,motor_zero_output_only=True,streams=streams,excluded={'camera':'not detected','lidar':'no configured ports','emg':'not enabled for this local sensor/display test'})
 (root/'run_result.json').write_text(json.dumps(result,indent=2));(work/'endurance_result.json').write_text(json.dumps(result,indent=2));print('FINISHED',root,'error',error,flush=True)
 if error:sys.exit(1)
