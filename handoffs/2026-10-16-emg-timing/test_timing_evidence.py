import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from timing_evidence import milliseconds, summarize, install_probe
from apply_ubuntu_timing_update import install


class TimingEvidenceTests(unittest.TestCase):
    def test_cross_host_offset_direction_and_local_monotonic(self):
        # Independent monotonic clock origins must never be subtracted across hosts.
        run = dict(clock_sync=dict(offset_ns=600_000_000, rtt_ns=4_000_000),
                   ubuntu_request=dict(wall_ns=10_000_000_000, mono_ns=5_000_000_000),
                   gui_sync_pulse=dict(wall_ns=10_710_000_000, mono_ns=90_000_000_000),
                   start_client_receipt=dict(received=dict(wall_ns=10_140_000_000, mono_ns=5_140_000_000)),
                   ubuntu_stop_request=dict(wall_ns=20_000_000_000),
                   stop_clock_sync=dict(offset_ns=605_000_000),
                   stop_returned=dict(wall_ns=20_805_000_000))
        result = summarize(run)
        self.assertEqual(result['request_to_gui_pulse_ms'], 110)
        self.assertEqual(result['gui_pulse_to_ubuntu_ack_receive_ms'], 30)
        self.assertEqual(result['request_to_ubuntu_ack_receive_ms'], 140)
        self.assertEqual(result['stop_click_to_save_complete_ms'], 200)
        self.assertIsNone(result['request_to_first_frame_seen_ms'])
        self.assertIsNone(result['collect_all_to_gui_pulse_ms'])

    def test_missing_evidence_is_not_zero(self):
        self.assertIsNone(milliseconds({}, {}))
        self.assertNotIn('request_to_gui_pulse_ms', summarize({}))

    def test_empty_write_does_not_claim_first_data_saved(self):
        class Plot:
            def release_sync_transition(self): return None
            def stop_streaming(self): pass
        class Device:
            def Stop(self): pass
        class Saver:
            def stop(self): pass
            def _save_matrix_batches_to_csv(self, path, batches): return True
        class Processor:
            def process_data_frame(self, *args): pass
        probe=install_probe(dict(PlottingManager=Plot,TrignoBase=Device,FileSaveManager=Saver,
                                 EMGPreProcessor=Processor,ENABLE_FILE_SAVING=True))
        probe.begin()
        Saver()._save_matrix_batches_to_csv('unused',[])
        self.assertNotIn('first_csv_write_returned',probe.snapshot())
        Saver()._save_matrix_batches_to_csv('unused',[[1]])
        self.assertIn('first_csv_write_returned',probe.snapshot())

    def test_upgrade_backup_and_changed_base_rejection(self):
        root = Path(__file__).resolve().parents[2]
        upstream = root/'work'/'timing-upstream'
        if not upstream.is_dir():
            self.skipTest('Reviewed upstream fixture is local; not included in public handoff')
        names = ['hipexo_monitor.py', 'hipexo_emg_control.py', 'control_protocol.py']
        with tempfile.TemporaryDirectory(dir=root/'work') as tmp:
            target=Path(tmp)/'python';target.mkdir()
            original={name:(upstream/name).read_bytes() for name in names}
            for name,data in original.items():(target/name).write_bytes(data)
            self.assertTrue(install(target, dry_run=True)['dry_run'])
            self.assertEqual((target/names[0]).read_bytes(), original[names[0]])
            result=install(target)
            for name,data in original.items():
                self.assertEqual((Path(result['backup'])/name).read_bytes(),data)
                compile((target/name).read_bytes(),name,'exec')
            monitor=(target/names[0]).read_text(encoding='utf-8')
            self.assertIn('record_start(self._dm._record_path, clicked=emg_click)',monitor)
            self.assertIn('self._emg_worker.collect_all_requested(emg_click)',monitor)
            before={name:(target/name).read_bytes() for name in names}
            with self.assertRaises(ValueError):install(target)
            for name,data in before.items():self.assertEqual((target/name).read_bytes(),data)


if __name__ == '__main__': unittest.main()
