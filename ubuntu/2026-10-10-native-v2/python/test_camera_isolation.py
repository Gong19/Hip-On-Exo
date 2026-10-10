"""Fake camera subprocess hangs; it must not freeze other acquisition."""
import os,sys,tempfile,time,unittest,subprocess
from pathlib import Path
from unittest.mock import patch
os.environ['QT_QPA_PLATFORM']='offscreen'
sys.argv.append('--preview')
from PyQt5 import QtWidgets
import hipexo_monitor as m
APP=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
class CameraIsolation(unittest.TestCase):
    def test_unresponsive_camera_child_shutdown_is_bounded(self):
        with tempfile.TemporaryDirectory() as d,patch.object(m,'EXPORT_DIR',d):
            dm=m.DataManager();w=m.VisionWorker(dm)
            real_popen=subprocess.Popen
            def sleeping_child(*a,**kw):return real_popen([sys.executable,'-c','import time;time.sleep(60)'],**kw)
            try:
                with patch('subprocess.Popen',side_effect=sleeping_child),patch.object(m,'VISION_IMAGE_ONLY',True),patch.object(m,'_VISION_MODULE_OK',True):
                    w.start();time.sleep(.15)
                    began=time.perf_counter()
                    for i in range(100):dm.append_frame('force_0',i,dict(V=1,kg=1))
                    self.assertLess(time.perf_counter()-began,.5)
                    began=time.perf_counter();w.shutdown()
                    self.assertLess(time.perf_counter()-began,1.5)
                    self.assertFalse(w._thread.is_alive())
            finally:w.shutdown();dm._flush_timer.stop();dm._mem_timer.stop()
if __name__=='__main__':
    sys.argv.remove('--preview');unittest.main()
