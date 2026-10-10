"""Real Qt plotting + physical IMU/force and explicitly requested zero motor feedback.
No EMG/LiDAR start, no motor mode setup, no active motor control.
"""
import argparse,os,time,json
from pathlib import Path
from unittest.mock import patch
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=60);p.add_argument('--motors-zero',action='store_true');a=p.parse_args()
if not a.motors_zero:raise SystemExit('Use --motors-zero to explicitly enable zero-output motor measurement')
from PyQt5 import QtWidgets
import hipexo_monitor as m
root=Path('evidence/gui-acquisition').resolve();root.mkdir(parents=True,exist_ok=True)
m.EXPORT_DIR=str(root);m.SESSION_STATE_PATH=str(root/'BENCH_GUI_session_state.json')
app=QtWidgets.QApplication([])
with patch.object(m,'_ensure_motor_mode_for_ports',lambda *a:None):window=m.MainWindow()
window.resize(1280,800);window.show();app.processEvents()
window._dm.set_export_dir(str(root),subject_id='BENCH_GUI_NO_SUBJECT',location='Workstation')
assert window._dm.start_recording()
window._imu_worker.start();window._force_worker.start();window._motor_worker.start_monitoring();window._vision_worker.start()
names=['Motor','IMU','Force','EMG','Vision','Lidar'];start=time.perf_counter();last_page=None;frames=0;loop_stamps=[]
try:
 while time.perf_counter()-start<a.seconds:
  index=min(5,int((time.perf_counter()-start)/a.seconds*6))
  if index!=last_page:window._switch_panel(names[index]);last_page=index
  loop_stamps.append(time.perf_counter());app.processEvents();frames+=1;time.sleep(.005)
  assert not window._motor_worker.running.is_set(),'Unexpected active motor control'
finally:window.close();app.processEvents()
q=json.loads((Path(window._dm._record_path).parent/'pipeline_quality.json').read_text())
result=dict(seconds=time.perf_counter()-start,qt_pages_rendered=names,event_loop_iterations=frames,motor_zero_output_only=True,skipped_vendor_mode_setup=True,quality=q)
import numpy as np
dt=np.diff(loop_stamps)*1000
result['event_loop_interval_ms']=dict(p50=float(np.median(dt)),p99=float(np.percentile(dt,99)),maximum=float(dt.max()))
Path('evidence/gui_live_benchmark.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
