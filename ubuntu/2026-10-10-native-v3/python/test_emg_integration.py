"""Hardware-free tests: QT_QPA_PLATFORM=offscreen python -m unittest -v test_emg_integration"""
import csv, os, sys, tempfile, time, threading, unittest
from pathlib import Path
import numpy as np
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5 import QtWidgets
from hipexo_emg_core import EmgProcessor
from hipexo_emg import SimulatedSource, EmgWorker, DEFAULT_CONFIG, read_credentials
APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
def channels():
    return SimulatedSource(DEFAULT_CONFIG, threading.Event()).connect()
def emit(p, start, end, value=.001, now=0., absent=()):
    t = np.arange(start, end) / 1000
    return p.ingest({i: (t, np.full(len(t), value)) for i in range(7) if i not in absent}, now)

class ProcessingTests(unittest.TestCase):
    def test_continuous_grid_and_units(self):
        p=EmgProcessor(channels()); rows=emit(p,0,25)+emit(p,25,80)
        self.assertEqual([r['label_ms'] for r in rows],list(range(80)))
        np.testing.assert_allclose(rows[-1]['envelope_v'],.001)
        self.assertTrue(np.isnan(rows[-1]['mvc_ratio']).all())
    def test_single_sample_batches(self):
        p=EmgProcessor(channels())
        self.assertEqual(emit(p,0,1),[])
        self.assertEqual(len(emit(p,1,2)),2)
        self.assertEqual(len(emit(p,2,3)),1)
    def test_missing_dropout_and_recovery(self):
        ch=channels(); ch[2]['present']=False; p=EmgProcessor(ch)
        rows=emit(p,0,10,absent=(2,)); self.assertEqual(len(rows),10)
        self.assertFalse(rows[-1]['valid'][2]); self.assertEqual(rows[-1]['envelope_v'][2],0)
        self.assertTrue(rows[-1]['valid'][3])
        self.assertEqual(emit(p,10,20,now=.02,absent=(2,4)),[])
        rows=p.drain(now=.3); self.assertEqual(len(rows),10); self.assertFalse(rows[-1]['valid'][4])
        rows=emit(p,20,30,now=.31,absent=(2,))
        self.assertTrue(rows[-1]['valid'][4]); self.assertEqual(rows[0]['label_ms'],20)
    def test_gaps_and_clock_reset(self):
        p=EmgProcessor(channels()); emit(p,0,10)
        rows=emit(p,500,510,now=.5)
        self.assertEqual([r['label_ms'] for r in rows],list(range(500,510)))
        self.assertTrue(all(n==1 for n in p.discontinuities))
        with self.assertRaises(ValueError): emit(p,0,10,now=.6)
    def test_invalid_values_filtered(self):
        p=EmgProcessor(channels())
        p.ingest({i:([0,.001,.002,.003],[.001,float('nan'),.001,float('inf')]) for i in range(7)},0)
        self.assertTrue(all(n==2 for n in p.rejected))
    def test_baseline_repeat_with_mvc_enabled(self):
        p=EmgProcessor(channels()); emit(p,0,20,value=.0001)
        p.baseline[:]=.0001; p.mvc[:]=.002
        for start,now in [(20,0),(120,.2)]:
            p.begin_calibration('baseline',duration=.1,now=now)
            emit(p,start,start+100,value=.0001,now=now+.09)
            p.finish_calibration(now+.11)
            np.testing.assert_allclose(p.baseline,.0001)
            np.testing.assert_allclose(p.mvc,0)
    def test_stale_calibration_rejected(self):
        p=EmgProcessor(channels()); p.baseline[:]=.0003
        p.begin_calibration('baseline',duration=1,now=0)
        emit(p,0,10,now=.1); p.finish_calibration(1.1)
        np.testing.assert_allclose(p.baseline,.0003)
        self.assertIn('rejected',p.last_calibration_message)
    def test_calibration_identity_and_pair(self):
        p=EmgProcessor(channels(),'simulation'); p.baseline[:]=.0002; p.mvc[:,0]=.001
        record=p.calibration_dict(); q=EmgProcessor(channels(),'simulation'); q.load_calibration(record)
        np.testing.assert_allclose(q.baseline,p.baseline); np.testing.assert_allclose(q.mvc,p.mvc)
        ch=channels(); ch[0]['mode']='raw'
        with self.assertRaises(ValueError): EmgProcessor(ch,'simulation').load_calibration(record)
        with self.assertRaises(ValueError): EmgProcessor(channels(),'delsys').load_calibration(record)
    def test_raw_rectification(self):
        ch=channels()
        for c in ch: c['is_rms']=False
        p=EmgProcessor(ch); rows=emit(p,0,20,value=-.001)
        np.testing.assert_allclose(rows[-1]['raw_v'],-.001)
        np.testing.assert_allclose(rows[-1]['uncalibrated_v'],.001)

class IntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.argv.append('--preview')
        import hipexo_monitor19 as monitor
        cls.monitor=monitor; cls.tmp=tempfile.TemporaryDirectory()
        monitor.EXPORT_DIR=cls.tmp.name
        monitor.SESSION_STATE_PATH=str(Path(cls.tmp.name)/'session.json')
    @classmethod
    def tearDownClass(cls): cls.tmp.cleanup()
    def test_recording_ring_eviction_stop_and_late_stream(self):
        dm=self.monitor.DataManager(); path=str(Path(self.tmp.name)/'full.csv')
        dm.append_frame('emg_0',-1,{'envelope_v':10})
        self.assertTrue(dm.start_recording(path))
        for start in range(0,7000,100):
            dm.append_batch('emg_0',list(range(start,start+100)),
                [{'envelope_v':i,'valid':1} for i in range(start,start+100)], list(range(start,start+100)))
        dm._trim_buffers(); dm.append_frame('imu_0',0,{'roll_deg':3},123)
        self.assertTrue(dm.stop_recording())
        with open(path) as f: rows=list(csv.DictReader(f))
        self.assertEqual([int(r['value']) for r in rows if r['field']=='envelope_v'],list(range(7000)))
        self.assertEqual(len([r for r in rows if r['stream']=='imu_0']),1)
        self.assertEqual(len(dm.snapshot('emg_0_envelope_v')[0]),self.monitor.DISPLAY_KEEP_PTS)
        dm._flush_timer.stop(); dm._mem_timer.stop()
    def test_panel_worker_restart_and_shutdown(self):
        w=self.monitor.MainWindow(); worker=w._emg_worker
        try:
            for _ in range(2):
                worker.start(); deadline=time.monotonic()+4
                while time.monotonic()<deadline:
                    APP.processEvents()
                    if len(worker.snapshot()[0][0])>=50: break
                    time.sleep(.01)
                self.assertEqual(worker.state,'RUNNING')
                self.assertGreaterEqual(len(worker.snapshot()[0][0]),50)
                self.assertIn('emg_6_envelope_v',w._dm.keys())
                self.assertTrue(all(v==1 for v in w._dm.snapshot('emg_0_simulated')[1]))
                worker.stop(); deadline=time.monotonic()+3
                while worker.state!='READY' and time.monotonic()<deadline:
                    APP.processEvents(); time.sleep(.01)
                self.assertEqual(worker.state,'READY')
            w._switch_panel('EMG'); w._panels['EMG']._refresh(); w._panels['EMG'].set_theme('dark')
        finally:
            w.close(); APP.processEvents()
        self.assertFalse(worker._thread.is_alive())
    def test_credentials_not_executed(self):
        path=Path(self.tmp.name)/'creds.py'
        path.write_text("key='test-key'\nlicense='test-license'\nraise RuntimeError('must not execute')\n")
        self.assertEqual(read_credentials(path),('test-key','test-license'))
    def test_missing_sdk_keeps_ui_alive(self):
        dm=self.monitor.DataManager(); worker=EmgWorker(dm)
        worker.config=dict(DEFAULT_CONFIG,source='delsys',sdk_dll='/nonexistent/DelsysAPI.dll'); worker.start()
        deadline=time.monotonic()+3
        while worker.state!='ERROR' and time.monotonic()<deadline:
            APP.processEvents(); time.sleep(.01)
        self.assertEqual(worker.state,'ERROR'); self.assertTrue(worker.shutdown())
        dm._flush_timer.stop(); dm._mem_timer.stop()

class DelsysAdapterTests(unittest.TestCase):
    def test_guid_mapping_ignores_unknown_and_disabled_channels(self):
        from types import SimpleNamespace as NS
        from unittest.mock import patch
        from hipexo_emg import DelsysSource
        selected=[]
        sensors=[]
        for sid, enabled in [(99999,True),(DEFAULT_CONFIG['sensor_ids'][5],True),(DEFAULT_CONFIG['sensor_ids'][1],False)]:
            sensors.append(NS(Properties=NS(Sid=sid,BatteryPercent=.8),Configuration=NS(ModeString='RMS mode'),
                TrignoChannels=[NS(IsEnabled=enabled,Type='EMG',Name='EMG RMS',Id=f'guid{sid}',SampleRate=148.148)]))
        guid=f'guid{DEFAULT_CONFIG["sensor_ids"][5]}'
        api=NS(ValidateBase=lambda *a:None,ScanSensors=lambda:NS(Wait=lambda timeout:True),
               GetScannedSensorsFound=lambda:sensors,SelectSensor=lambda i:selected.append(i),
               Configure=lambda *a:None,IsPipelineConfigured=lambda:True,GetSensorObject=lambda i:sensors[i],
               CheckYTDataQueue=lambda:True,PollYTData=lambda:{guid:[NS(Item1=1.,Item2=.001)]})
        with tempfile.TemporaryDirectory() as folder:
            dll=Path(folder)/'DelsysAPI.dll';dll.touch()
            cred=Path(folder)/'license.json';cred.write_text('{"key":"test","license":"test"}')
            with patch.dict(sys.modules,{'clr':NS(AddReference=lambda x:None),'Aero':NS(AeroPy=lambda:api)}):
                source=DelsysSource(dict(DEFAULT_CONFIG,sdk_dll=str(dll),credentials_file=str(cred)),threading.Event())
                info=source.connect(); frames=source.poll()
        self.assertEqual(selected,[1,2])
        self.assertTrue(info[5]['present']);self.assertFalse(info[1]['present'])
        self.assertEqual(set(frames),{5});self.assertEqual(frames[5],([1.],[.001]))

class RecorderFailureTests(unittest.TestCase):
    def test_disk_error_reported(self):
        from hipexo_recording import FrameRecorder
        from types import SimpleNamespace as NS
        errors=[]
        with tempfile.TemporaryDirectory() as folder:
            rec=FrameRecorder(str(Path(folder)/'bad.csv'),errors.append)
            def fail(rows): raise OSError('test disk failure')
            rec._writer=NS(writerows=fail)
            rec.enqueue([[0,'emg_0','valid',0,0,1]])
            deadline=time.monotonic()+2
            while not errors and time.monotonic()<deadline: time.sleep(.01)
            self.assertTrue(errors);self.assertIn('disk failure',errors[0])
            self.assertFalse(rec.stop())

if __name__=='__main__': unittest.main(verbosity=2)
