import os,sys,tempfile,time,json,csv,unittest
from pathlib import Path
from unittest.mock import patch
os.environ['QT_QPA_PLATFORM']='offscreen'
sys.argv.append('--preview')
from PyQt5 import QtWidgets
import hipexo_monitor as m
from hipexo_recording_process import ProcessCycleRecorder as CycleRecorder
APP=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
class PipelineIntegration(unittest.TestCase):
    def test_default_record_and_finalize(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(m,'EXPORT_DIR',tmp),patch.object(m,'PIPELINE_ENABLED',True):
            dm=m.DataManager();dm.set_export_dir(tmp,subject_id='SIMULATED_S01',location='Test_Lab')
            try:
                self.assertTrue(dm.start_recording());self.assertIsInstance(dm._recorder,CycleRecorder)
                start=dm._recorder.start_mono_ns
                for i in range(21):dm.append_frame('force_0',time.time_ns()/1e6,dict(V=1,kg=5),start+i*1_000_000)
                dm._recorder.stop_ns=start+60_000_000
                self.assertTrue(dm.stop_recording())
                directory=Path(dm._record_path).parent
                self.assertIn('Force',directory.name)
                self.assertTrue((directory/'acquisition_config.json').exists())
                info=json.loads((directory/'recording_info.json').read_text())
                self.assertIn('1000Hz',info['recording_format'])
                self.assertEqual(json.loads((directory/'pipeline_quality.json').read_text())['accepted_frames'],21)
                with open(dm._record_path) as f:rows=list(csv.DictReader(f))
                self.assertEqual(len(rows),300);self.assertEqual(rows[10]['force_0__sensor_voltage_V'],'1.0')
                self.assertEqual(rows[100]['within_recording'],'0')
            finally:dm.stop_recording();dm._flush_timer.stop();dm._mem_timer.stop()
if __name__=='__main__':
    sys.argv.remove('--preview');unittest.main()
