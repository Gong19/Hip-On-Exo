"""Regression checks for the six-page merge; no physical device access."""
import csv
import json
import time
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch, Mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.argv.append('--preview')
from PyQt5 import QtWidgets
import numpy as np
import hipexo_monitor as hm

APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class CombinedTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.paths = patch.multiple(hm, EXPORT_DIR=self.tmp.name,
            SESSION_STATE_PATH=str(Path(self.tmp.name)/'session.json'))
        self.paths.start()
        self.win = hm.MainWindow()

    def tearDown(self):
        self.win.close()
        APP.processEvents()
        self.paths.stop()
        self.tmp.cleanup()

    def test_all_pages_visible_offline(self):
        self.assertEqual(set(self.win._panels), {'Motor','IMU','Force','EMG','Vision','Lidar'})
        for name, panel in self.win._panels.items():
            self.win._switch_panel(name)
            APP.processEvents()
            self.assertIs(self.win._stack.currentWidget(), panel)
        self.assertFalse(self.win._terrain_switcher.enabled)
        self.assertFalse(self.win._panels['Vision'].chk_auto_mode.isChecked())

    def test_preview_blocks_physical_connections_and_config_writes(self):
        self.assertFalse(any((hm._SDK_OK,hm._SMBUS_OK,hm._SPIDEV_OK,hm._VISION_MODULE_OK)))
        self.win._panels['Vision']._on_start()
        self.assertIn('preview',self.win._panels['Vision'].lbl_state.text().lower())
        self.assertIsNone(self.win._vision_worker._thread)
        worker=self.win._lidar_worker_l
        worker.port='/dev/should-not-open'
        with patch.object(worker, '_open_serial', side_effect=AssertionError('hardware access')):
            worker.start()
            self.assertIsNone(worker._thread)
        worker=self.win._emg_worker
        self.assertTrue(worker.simulation_only)
        with self.assertRaisesRegex(ValueError, 'preview'):
            worker.configure(dict(worker.config, source='delsys'))
        import hipexo_emg
        with patch.object(hipexo_emg, 'CONFIG_PATH') as config_path:
            worker.configure(dict(worker.config, source='simulation'))
            config_path.write_text.assert_not_called()

    def test_subject_switch_closes_previous_recording(self):
        w=self.win
        w._toggle_recording()
        first=Path(w._dm._record_path)
        w._dm.append_dict('vision',1,{'confidence':.8})
        w._start_new_session('S999','Lab')
        self.assertFalse(w._recording)
        self.assertIsNone(w._dm._recorder)
        first=next(first.parent.parent.glob('*/*__combined.csv'))
        with first.open() as f:
            self.assertEqual(len(list(csv.DictReader(f))),1)
        w._toggle_recording()
        second=Path(w._dm._record_path)
        self.assertNotEqual(first.parent,second.parent)
        w._dm.append_dict('lidar_L',2,{'n_scan_points':400})
        w._toggle_recording()
        second=Path(w._dm._record_path)
        with second.open() as f:
            self.assertEqual({r['stream'] for r in csv.DictReader(f)},{'lidar_L'})

    def test_scalar_temperature_is_recorded(self):
        dm=self.win._dm
        path=Path(self.tmp.name)/'temperature.csv'
        dm.start_recording(str(path))
        dm.append('imu_0_temp_c',10,27.5)
        dm.stop_recording()
        with path.open() as f:
            rows=list(csv.DictReader(f))
        self.assertEqual((rows[0]['stream'],rows[0]['field']),('imu_0','temp_c'))

    def test_failed_record_start_keeps_ui_idle(self):
        with patch.object(hm,'FrameRecorder',side_effect=OSError('test disk denied')):
            self.win._toggle_recording()
        self.assertFalse(self.win._recording)
        self.assertIn('test disk denied', self.win._lbl_status.text())

    def test_collect_all_uses_feedback_only_and_stops_every_source(self):
        w=self.win
        workers=[w._emg_worker,w._imu_worker,w._force_worker,w._motor_worker,w._vision_worker,
                 w._lidar_worker_l,w._lidar_worker_r]
        from contextlib import ExitStack
        with ExitStack() as stack:
            calls={}
            for worker in workers:
                for method in ('start','stop'):
                    calls[(worker,method)] = stack.enter_context(patch.object(worker,method))
            feedback=stack.enter_context(patch.object(w._motor_worker,'start_monitoring'))
            end_feedback=stack.enter_context(patch.object(w._motor_worker,'stop_monitoring'))
            for side in ('L','R'):
                w._panels['Lidar']._side_widgets[side]['enabled'].setChecked(True)
            w._toggle_collection()
            feedback.assert_called_once()
            calls[(w._motor_worker,'start')].assert_not_called()
            for worker in workers:
                if worker is not w._motor_worker:
                    calls[(worker,'start')].assert_called_once()
            w._toggle_collection()
            end_feedback.assert_called_once()
            for worker in workers:
                if worker is not w._motor_worker:
                    calls[(worker,'stop')].assert_called_once()

    def test_camera_capture_routes_to_current_session_without_device(self):
        w=self.win
        w._on_capture_dataset_sample()
        self.assertIn('no camera frame',w._panels['Vision'].lbl_capture_status.text())
        worker=w._vision_worker
        worker._latest_depth_raw=np.ones((48,64),dtype=np.uint16)*1000
        worker._latest_projection=np.zeros((100,100),dtype=np.uint8)
        worker._latest_depth_scale=.001
        worker._latest_intrinsics={'fx':60.,'fy':60.,'ppx':32.,'ppy':24.}
        worker._latest_timing={'device_frame_number':73,'device_timestamp_ms':1234.5,
                              'device_timestamp_domain':'hardware_clock',
                              'host_frame_received_wall_ns':time.time_ns()-2_000_000_000,
                              'host_frame_received_mono_ns':time.perf_counter_ns()-2_000_000_000,
                              'session_id':w._dm.session.session_id}
        w._panels['Vision'].cb_capture_label.setCurrentText('flat')
        w._on_capture_dataset_sample()
        folder=Path(w._current_session_dir)/'vision_dataset'/'flat'
        self.assertTrue(list(folder.glob('depth_raw_*')))
        self.assertNotIn('failed',w._panels['Vision'].lbl_capture_status.text())
        meta=json.loads(next(folder.glob('meta_*.json')).read_text())
        self.assertEqual(meta['frame_timing']['device_frame_number'],73)
        self.assertGreater(meta['saved_wall_ns']-meta['frame_timing']['host_frame_received_wall_ns'],1_000_000_000)
        events=[json.loads(s) for s in w._dm.session.path.read_text().splitlines()]
        artifact=next(e for e in events if e.get('modality')=='camera_sample')
        self.assertTrue(Path(artifact['metadata_path']).is_file())
        self.assertEqual(artifact['frame_timing'],meta['frame_timing'])
        w._start_new_session('S995','Lab')
        self.assertIsNone(worker.capture_dataset_sample('flat',str(Path(w._current_session_dir)/'vision_dataset')))

    def test_raw_lidar_must_stop_before_session_change(self):
        w=self.win
        previous=w._dm.session
        w._lidar_worker_l.set_recording(True)
        with patch.object(w,'_toast') as toast:
            w._start_new_session('S996','Lab')
            self.assertIn('Raw scans',toast.call_args.args[0])
        self.assertIs(w._dm.session,previous)
        w._lidar_worker_l.set_recording(False)
        w._start_new_session('S996','Lab')
        self.assertIsNot(w._dm.session,previous)

    def test_same_read_timestamp_does_not_break_quality_or_export(self):
        dm=self.win._dm
        dm.append_frame('vision',100,{'device_timestamp_ms':None,'confidence':.7})
        dm.append_frame('vision',100,{'device_timestamp_ms':5.5,'confidence':.8})
        report=dm.quality_report()['streams']['vision']
        self.assertEqual(report['duplicate_timestamps'],1)
        self.assertEqual(report['observed_hz'],0)
        self.assertEqual(dm.export_snapshot_csv(str(Path(self.tmp.name)/'duplicates.csv')),1)

    def test_camera_worker_logs_frame_receipt_before_inference_completion(self):
        from types import SimpleNamespace
        from contextlib import ExitStack
        w=self.win._vision_worker
        class Frame:
            def __init__(self,n):self.n=n
            def get_frame_number(self):return self.n
            def get_timestamp(self):return self.n*33.
            def get_frame_timestamp_domain(self):return 'hardware_clock'
            def get_data(self):return np.ones((48,64),dtype=np.uint16)*1000
        class Pipeline:
            def __init__(self):self.n=0
            def wait_for_frames(self,**kwargs):
                self.n+=1
                if self.n==3:w._alive=False
                return SimpleNamespace(get_depth_frame=lambda:Frame(self.n))
            def stop(self):pass
        def classify(*args):
            time.sleep(.01)
            return 'flat',.9,{'flat':.9}
        fake_torch=SimpleNamespace(device=lambda s:s,cuda=SimpleNamespace(is_available=lambda:False))
        with ExitStack() as stack:
            stack.enter_context(patch.dict(sys.modules,{'pyrealsense2':SimpleNamespace()}))
            stack.enter_context(patch.object(w,'_open_pipeline',return_value=(Pipeline(),.001)))
            stack.enter_context(patch.multiple(hm,VISION_CAPTURE_FPS=100,VISION_INFER_HZ=100,
                                              VISION_LOG_HZ=100,VISION_DISPLAY_HZ=100))
            stack.enter_context(patch.object(hm._vt,'torch',fake_torch,create=True))
            stack.enter_context(patch.object(hm._vt,'load_model',return_value=(object(),['flat'],{})))
            stack.enter_context(patch.object(hm._vt,'classify_projection',side_effect=classify))
            stack.enter_context(patch.object(hm._vt,'intrinsics_from_realsense',return_value={'fx':60.,'fy':60.,'ppx':32.,'ppy':24.}))
            w._running.set();w._loop()
        stamps,received=self.win._dm.snapshot('vision_host_frame_received_wall_ns')
        _,done=self.win._dm.snapshot('vision_inference_done_wall_ns')
        self.assertEqual(len(stamps),3)
        self.assertTrue(all(t==r/1e6 for t,r in zip(stamps,received)))
        self.assertTrue(all(d-r>10_000_000 for d,r in zip(done,received)))
        self.assertEqual(w._latest_timing['device_frame_number'],3)

    def test_new_subject_id_works_on_python310(self):
        dialog=hm.SessionDialog(self.win,self.win._session_mgr)
        dialog._cb_subject.addItem('S001')
        with patch.object(dialog._mgr,'next_subject_id',return_value='S001'):
            dialog._assign_new_subject()
        self.assertNotEqual(dialog._cb_subject.currentText(),'S001')
        dialog.close()


if __name__ == '__main__':
    sys.argv.remove('--preview')
    unittest.main()
