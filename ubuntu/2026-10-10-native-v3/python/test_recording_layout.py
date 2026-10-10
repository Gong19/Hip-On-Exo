"""Record-folder, validity, late-channel and no-overwrite regressions."""
import csv,json,os,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
if '--preview' not in sys.argv:sys.argv.append('--preview')
from PyQt5 import QtWidgets
import hipexo_monitor as hm
from hipexo_recording_layout import component
APP=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class LayoutTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        with patch.object(hm,'EXPORT_DIR',str(self.root)):
            self.dm=hm.DataManager()
        self.dm.set_export_dir(str(self.root/'session'),subject_id='S007 张三',location='Lab/A')

    def tearDown(self):
        self.dm.stop_recording();self.dm._flush_timer.stop();self.dm._mem_timer.stop();self.tmp.cleanup()

    def info(self):
        path=Path(self.dm._record_path)
        return path,json.loads((path.parent/'recording_info.json').read_text())

    def test_each_record_is_new_folder_and_late_stream_is_named(self):
        self.assertTrue(self.dm.start_recording())
        pending=Path(self.dm._record_path)
        self.assertIn('Recording',pending.parent.name)
        self.dm.append_frame('imu_0',1000,{'ax_g':1.})
        self.dm.append_batch('emg_0',[1000,1001],[{'valid':0,'raw_v':float('nan')}]*2,[1,2])
        self.dm.append_frame('force_0',1010,{'force':3.})
        self.assertTrue(self.dm.stop_recording())
        first,info=self.info()
        self.assertTrue(first.is_file());self.assertFalse(pending.exists())
        self.assertIn('S007_张三__Lab_A',first.name)
        self.assertEqual(info['sensors_with_valid_data'],['IMU','Force'])
        self.assertNotIn('EMG',first.name)
        self.assertEqual(info['streams']['emg_0']['valid_samples'],0)
        self.assertTrue(self.dm.start_recording())
        self.dm.append_frame('motor_0',2000,{'q':.1})
        self.dm.stop_recording();second,info=self.info()
        self.assertNotEqual(first.parent,second.parent)
        self.assertTrue(first.is_file())
        self.assertEqual(info['sensors_with_valid_data'],['Motor'])
        self.assertEqual(set(info['streams']),{'motor_0'})

    def test_valid_emg_and_raw_reference_with_exact_subject_location(self):
        raw=self.root/'session'/(self.dm.session.file_stem('EMG_raw')+'.jsonl')
        raw.write_text('test raw file\n')
        self.dm.session.artifact('emg_remote_raw',raw)
        self.dm.start_recording()
        self.dm.append_batch('emg_1',[1000,1001,1002],
                             [{'valid':1,'raw_v':.001,'simulated':0},
                              {'valid':0,'raw_v':float('nan'),'simulated':0},
                              {'valid':1,'raw_v':.002,'simulated':0}],[1,2,3])
        self.dm.stop_recording();path,info=self.info()
        self.assertIn('__EMG__combined.csv',path.name)
        self.assertEqual(info['subject_id'],'S007 张三')
        self.assertEqual(info['location'],'Lab/A')
        self.assertEqual(info['streams']['emg_1']['valid_samples'],2)
        self.assertEqual(info['session_raw_references'][0]['path'],str(raw.resolve()))
        self.assertTrue(raw.is_file())
        with path.open() as f:self.assertEqual(len(list(csv.DictReader(f))),9)

    def test_empty_record_and_explicit_path(self):
        self.dm.start_recording();self.dm.stop_recording()
        path,info=self.info()
        self.assertIn('NoValidData',path.name);self.assertEqual(info['streams'],{})
        explicit=self.root/'custom.csv'
        self.dm.start_recording(str(explicit));self.dm.append_frame('vision',1,{'confidence':.8})
        self.dm.stop_recording()
        self.assertEqual(self.dm._record_path,str(explicit));self.assertTrue(explicit.exists())

    def test_writer_error_marks_incomplete(self):
        self.dm.start_recording();self.dm.append_frame('imu_3',1,{'roll':.3})
        self.dm._recorder._fail('injected write integrity failure')
        self.assertFalse(self.dm.stop_recording())
        path,info=self.info()
        self.assertTrue(path.exists());self.assertIn('INCOMPLETE',path.name)
        self.assertFalse(info['csv_write_complete'])

    def test_filename_components_are_bounded_and_snapshot_names_contents(self):
        self.assertLessEqual(len(component('张'*100).encode()),48)
        self.assertNotIn('/',component('../Lab/room'))
        self.dm.append_frame('vision',10,{'confidence':.8})
        self.dm.append_frame('emg_0',10,{'valid':0,'raw_v':float('nan')})
        name=Path(self.dm.suggested_export_path()).name
        self.assertIn('S007_张三__Lab_A__Camera__snapshot',name)
        self.assertNotIn('EMG',name)


if __name__=='__main__':
    sys.argv.remove('--preview');unittest.main()
