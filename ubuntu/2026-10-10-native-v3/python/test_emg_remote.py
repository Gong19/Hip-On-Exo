"""Loopback protocol tests: no physical EMG device or external computer."""
import csv,json,os,sys,tempfile,threading,time,unittest
from pathlib import Path
from unittest.mock import patch
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
sys.argv.append('--preview')
from PyQt5 import QtWidgets
from hipexo_emg import EmgWorker,EmgPanel,DEFAULT_CONFIG
from hipexo_emg_remote import RemoteWindowsSource,estimate_clock,ClockSyncError
from hipexo_session import SessionManifest
from windows_bridge_simulator import BridgeSimulator,SIDS
import hipexo_monitor as hm
APP=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class RemoteTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.server=BridgeSimulator(record_dir=self.root/'windows',offset_ns=2_000_000_000).start()
        self.config=dict(DEFAULT_CONFIG,source='windows',remote_host='127.0.0.1',remote_port=self.server.port)
        self.source=RemoteWindowsSource(self.config,threading.Event(),self.root/'workstation')

    def tearDown(self):
        try:self.source.close()
        except (OSError,ValueError):pass
        self.server.close();self.tmp.cleanup()

    def test_clock_equations_and_clock_step_rejection(self):
        p=estimate_clock(1000,3010,3012,1022,22)
        self.assertEqual(p['offset_ns'],2000);self.assertEqual(p['rtt_ns'],20)
        with self.assertRaises(ValueError):estimate_clock(0,1,2,100_000_000,2)

    def test_connect_sync_start_stop_and_raw_evidence(self):
        self.assertEqual(len(self.source.connect()),7)
        with self.assertRaisesRegex(RuntimeError,'Sync'):self.source.start()
        self.source.synchronize()
        self.assertLess(abs(self.source.sync['offset_ns']-2_000_000_000),5_000_000)
        self.source.start();frames=self.source.poll()
        self.assertEqual(set(frames),set(range(7)))
        wall,mono,win=self.source.mapped_time(frames[0][0][-1])
        self.assertEqual(win-wall,self.source.sync['offset_ns'])
        self.assertLess(abs(wall-time.time_ns()),500_000_000)
        self.source.stop();list(self.source.drain_pending())
        path=Path(self.source.status['journal']);self.source.close()
        events=[json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual(sum(e['kind']=='clock_probe' for e in events),9)
        self.assertTrue({'start','data','stop','sync_selected'}<={e['kind'] for e in events})
        self.assertTrue(list((self.root/'windows').glob('*.jsonl')))

    def test_windows_not_armed_refuses_sync(self):
        self.server.ready=False;self.source.connect()
        self.assertTrue(self.source.status['connected']);self.assertFalse(self.source.status['ready'])
        with self.assertRaisesRegex(RuntimeError,'not armed'):self.source.synchronize()

    def test_between_probe_clock_steps_stop_run_and_preserve_tail(self):
        self.source.session=SessionManifest(self.root/'workstation')
        self.source.connect()
        for step in (2_000_000_000,-2_000_000_000):
            self.source.synchronize();self.source.start()
            run=self.source.run_id
            windows_path=Path(self.source.status['windows_file'])
            original_mapping=dict(self.source.sync)
            self.source.poll()
            self.server.offset_ns+=step
            for _ in range(2):
                self.source.last_health=0
                self.source.poll()
                self.assertTrue(self.source.started)
            self.source.last_health=0
            with self.assertRaises(ClockSyncError):self.source.poll()
            self.assertFalse(self.source.started)
            self.assertFalse(self.source.status['streaming'])
            self.assertFalse(self.source.status['synced'])
            self.assertEqual(self.source.sync,original_mapping)
            self.assertEqual(list(self.source.drain_pending()),[])
            with self.assertRaisesRegex(RuntimeError,'Sync'):self.source.start()
            ubuntu=[json.loads(s) for s in Path(self.source.status['journal']).read_text().splitlines()]
            remote=[json.loads(s) for s in windows_path.read_text().splitlines()]
            udata=[e['message'] for e in ubuntu if e['kind']=='data' and e['message']['run_id']==run]
            rdata=[e['message'] for e in remote if e['kind']=='data']
            self.assertEqual(udata,rdata)
            self.assertEqual([p['seq'] for p in udata],list(range(len(udata))))
            self.assertTrue(any(e['kind']=='clock_fault' and e['run_id']==run for e in ubuntu))
        events=[json.loads(s) for s in self.source.session.path.read_text().splitlines()]
        self.assertEqual(sum(e['kind']=='emg_clock_fault' for e in events),2)
        self.assertEqual(sum(e['kind']=='emg_start' for e in events),2)

    def test_queue_spike_and_one_offset_outlier_do_not_abort(self):
        self.source.connect();self.source.synchronize();self.source.start()
        baseline=self.source.sync['offset_ns']
        self.source._check_clock_probe({'offset_ns':baseline+50_000_000,'rtt_ns':200_000_000})
        self.assertEqual(self.source.status['clock_quality'],'network_uncertain')
        self.assertTrue(self.source.started)
        self.source._check_clock_probe({'offset_ns':baseline+30_000_000,'rtt_ns':1_000_000})
        self.assertEqual(self.source.status['clock_quality'],'suspect')
        self.source._check_clock_probe({'offset_ns':baseline,'rtt_ns':1_000_000})
        self.assertEqual(self.source.status['clock_quality'],'good')
        self.assertTrue(self.source.started)

    def test_large_initial_rtt_is_allowed_and_start_delay_evidence_is_saved(self):
        self.source.connect()
        original=self.source._probe
        def slow_network_probe():
            result=original()
            result['rtt_ns']=350_000_000
            return result
        with patch.object(self.source,'_probe',side_effect=slow_network_probe):
            self.source.synchronize()
        self.source.start();self.source.poll()
        report=json.loads(Path(self.source.status['timing_report']).read_text())
        self.assertEqual(report['network_rtt_min_ns'],350_000_000)
        self.assertEqual(report['one_way_network_est_ns'],175_000_000)
        self.assertEqual(report['offset_network_uncertainty_ns'],175_000_000)
        self.assertIsNone(report['device_acquisition_latency_ns'])
        self.assertEqual(len(report['sync_probes']),9)
        initial=report['first_data']
        self.assertEqual(initial['windows_send_to_ubuntu_receive_est_ns'],
                         initial['ubuntu_received_wall_ns']+report['windows_minus_ubuntu_offset_ns']-initial['windows_send_wall_ns'])
        self.assertEqual(len(initial['channel_sample_age_estimates']),7)
        self.assertTrue(self.source.started)

    def test_prolonged_network_delay_does_not_end_run(self):
        self.source.connect();self.source.synchronize();self.source.start()
        self.source._last_good_clock=time.monotonic()-31
        self.source._check_clock_probe({'offset_ns':self.source.sync['offset_ns'],'rtt_ns':200_000_000})
        self.assertTrue(self.source.started)
        self.assertEqual(self.source.status['clock_quality'],'network_uncertain')

    def test_workstation_wall_step_ends_run(self):
        self.source.connect();self.source.synchronize();self.source.start()
        import hipexo_emg_remote as remote
        from unittest.mock import Mock
        fake=Mock(wraps=time)
        fake.time_ns.side_effect=lambda:time.time_ns()+2_000_000_000
        with patch.object(remote,'time',fake):
            with self.assertRaisesRegex(ClockSyncError,'Ubuntu'):self.source.poll()
        self.assertFalse(self.source.started)

    def test_clock_fault_stop_timeout_is_explicit_and_closes_tcp(self):
        self.source.connect();self.source.synchronize();self.source.start()
        with patch.object(self.source,'_rpc',side_effect=TimeoutError('injected timeout')):
            with self.assertRaisesRegex(ClockSyncError,'acknowledgement failed'):
                self.source._abort_clock('injected fault')
        self.assertIsNone(self.source.sock)
        self.assertFalse(self.source.started)

    def test_bad_token_and_sequence_loss_are_errors(self):
        self.server.token='expected'
        with self.assertRaisesRegex(RuntimeError,'token'):self.source.connect()
        self.source.started=True;self.source.run_id='r'
        with self.assertRaisesRegex(ValueError,'sequence gap'):
            self.source._queue_data({'type':'data','run_id':'r','seq':5})
        self.source.started=False

    def test_disconnected_peer_is_not_reported_as_empty_data(self):
        self.source.connect();self.server.close()
        with self.assertRaises((ConnectionError,OSError)):self.source._probe()

    def test_worker_ui_sync_and_csv_timestamps(self):
        with patch.object(hm,'EXPORT_DIR',str(self.root/'csv')):
            dm=hm.DataManager()
        worker=EmgWorker(dm);worker.config=self.config
        panel=EmgPanel(worker);panel.show()
        path=self.root/'remote.csv';dm.start_recording(str(path))
        def pump_until(condition,seconds=5):
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:
                APP.processEvents();time.sleep(.01)
                if condition():return True
            return False
        try:
            self.assertTrue(panel.sync_btn.isEnabled())
            panel.sync_btn.click()
            self.assertTrue(pump_until(lambda:worker.state=='RUNNING' and bool(dm.snapshot('emg_6_envelope_v')[0])))
            self.assertIn('STREAMING',panel.remote_label.text())
            self.assertIn('SIMULATED',panel.remote_label.text())
            worker.stop()
            self.assertTrue(pump_until(lambda:worker.state=='READY' and panel.sync_btn.isEnabled()))
            self.assertTrue(dm.stop_recording())
            with path.open() as f:rows=list(csv.DictReader(f))
            fields={r['field'] for r in rows}
            self.assertTrue({'source_time_s','windows_wall_ns_est','sync_offset_ns','sync_id','remote_run_id'}<=fields)
            self.assertTrue(all(r['value']=='1' for r in rows if r['field']=='simulated'))
            # Clock mapping removes the deliberately injected 2-second Windows offset.
            source_s=float([r['value'] for r in rows if r['field']=='source_time_s'][-1])
            expected_ms=worker.source.mapped_time(source_s)[0]/1e6
            self.assertLess(abs(float(rows[-1]['t_ms'])-expected_ms),.001)
            panel.sync_btn.click()
            self.assertTrue(pump_until(lambda:worker.state=='RUNNING'))
            old_run=worker.source.run_id
            self.server.offset_ns+=2_000_000_000
            worker.source.last_health=0
            self.assertTrue(pump_until(lambda:worker.state=='ERROR',seconds=9),
                            f'state={worker.state}, status={getattr(worker.source,"status",None)}, sync={getattr(worker.source,"sync",None)}, offset={self.server.offset_ns}')
            APP.processEvents()
            self.assertIn('同步失效',panel.remote_label.text())
            self.assertTrue(panel.sync_btn.isEnabled())
            panel.sync_btn.click()
            self.assertTrue(pump_until(lambda:worker.state=='RUNNING'))
            self.assertNotEqual(worker.source.run_id,old_run)
            worker.stop()
            self.assertTrue(pump_until(lambda:worker.state=='READY'))
            self.server.close()
            self.assertTrue(pump_until(lambda:worker.state=='ERROR'))
            APP.processEvents()
            self.assertIn('DISCONNECTED',panel.remote_label.text())
        finally:
            worker.shutdown();dm.stop_recording();panel.close()
            dm._flush_timer.stop();dm._mem_timer.stop()


if __name__=='__main__':
    sys.argv.remove('--preview');unittest.main()
