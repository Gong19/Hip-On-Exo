import json
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from pathlib import Path
import shutil
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
import uuid
import importlib.util
import sys
from PyQt5 import QtCore, QtWidgets
from control_protocol import ControlClient, stamp
from windows_control import GuiControl
from hipexo_emg_control import EmgWorker
import apply_ubuntu_control_only as installer

APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class FakeWindow(QtWidgets.QMainWindow):
    sync_pulse_signal = QtCore.pyqtSignal()
    def __init__(self, root):
        super().__init__()
        self.root = root
        self.is_streaming = False
        self.start_btn = QtWidgets.QPushButton()
        self.plotting_manager = SimpleNamespace(streaming_mode='display_only', preprocessor=None)
        self.file_save_manager = SimpleNamespace(envelope_file_path=str(root/'envelope_data.csv'),
                                                envelope_row_counter=0, last_final_save_success=True)
        self.starts = 0

    def on_start(self):
        self.starts += 1
        self.is_streaming = True
        self.plotting_manager.streaming_mode = 'saving'
        self.start_btn.setEnabled(False)
        (self.root/'envelope_data.csv').write_text('time_ms,EMG\n0,0.001\n', encoding='utf-8')
        self.sync_pulse_signal.emit()

    def on_stop(self):
        self.is_streaming = False
        self.plotting_manager.streaming_mode = 'stopped'
        self.start_btn.setEnabled(True)
        self.control.after_local_stop(True)
        return True


def pump_until(predicate, timeout=5):
    deadline = time.monotonic()+timeout
    while not predicate() and time.monotonic()<deadline:
        APP.processEvents()
        time.sleep(.005)
    if not predicate():
        raise AssertionError('Timed out waiting for state')


class Tests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.win = FakeWindow(self.root)
        ns = dict(QT_BINDING='PyQt5', QTimer=QtCore.QTimer, QApplication=QtWidgets.QApplication,
                  ENABLE_FILE_SAVING=True, EMG_SIMULATION_MODE=True)
        self.gui = GuiControl(self.win, ns, 'test-only', self.root, port=0)
        self.win.control = self.gui
        self.client = ControlClient('127.0.0.1', self.gui.server.port, 'test-only')
        self.client.connect()

    def tearDown(self):
        self.client.close()
        self.gui.close()
        self.win.deleteLater()
        APP.processEvents()
        self.temp.cleanup()

    def async_call(self, fn):
        result = {}
        def run():
            try: result['value'] = fn()
            except Exception as exc: result['error'] = exc
        thread = threading.Thread(target=run)
        thread.start()
        pump_until(lambda:not thread.is_alive())
        thread.join()
        if 'error' in result: raise result['error']
        return result['value']

    def start(self):
        run_id = str(uuid.uuid4())
        clock = self.client.synchronize()
        reply = self.async_call(lambda:self.client.request('start', run_id=run_id,
            ubuntu_request=stamp(), clock_sync=clock))
        return run_id, reply

    def test_zero_sample_transport_local_save_idempotent_start_stop(self):
        run_id, reply = self.start()
        self.assertTrue(reply['status']['recording'])
        self.assertFalse(reply['status']['data_transport'])
        self.assertGreater((self.root/'envelope_data.csv').stat().st_size, 0)
        self.async_call(lambda:self.client.request('start', run_id=run_id))
        self.assertEqual(self.win.starts, 1)
        # No unsolicited sample packets may precede any status ACK.
        for _ in range(5):
            response = self.client.request('status')
            self.assertNotIn('channels', response)
            self.assertNotIn('values', response)
        self.async_call(lambda:self.client.request('stop', run_id=run_id))
        self.async_call(lambda:self.client.request('stop', run_id=run_id))
        sidecar = json.loads((self.root/('sync_'+run_id+'.json')).read_text())
        self.assertEqual(sidecar['state'], 'STOPPED')
        self.assertTrue(sidecar['save_ok'])
        self.assertNotIn('test-only', json.dumps(sidecar))

    def test_disconnect_keeps_local_acquisition_reconnect_can_stop(self):
        run_id, _ = self.start()
        self.client.close()
        pump_until(lambda:not self.gui.server.connected)
        self.assertTrue(self.win.is_streaming)
        status = self.client.connect()
        self.assertEqual(status['run_id'], run_id)
        self.async_call(lambda:self.client.request('stop', run_id=run_id))
        self.assertFalse(self.win.is_streaming)

    def test_remote_opt_out_rejects_start(self):
        self.gui.allow.setChecked(False)
        with self.assertRaisesRegex(RuntimeError, 'not ready'):
            self.start()
        self.assertEqual(self.win.starts, 0)

    def test_ubuntu_worker_records_only_control_metadata(self):
        self.client.close()
        pump_until(lambda:not self.gui.server.connected)
        worker = EmgWorker(SimpleNamespace(export_dir=str(self.root/'ubuntu')))
        worker.config.update(remote_host='127.0.0.1', remote_port=self.gui.server.port, remote_token='test-only')
        try:
            worker.record_start('disabled.csv')
            self.assertIsNone(worker._thread)
            worker.enabled = True
            worker.record_start('jetson.csv')
            pump_until(lambda:worker.state == 'RUNNING')
            self.assertIsNone(worker.processor)
            worker.stop()
            pump_until(lambda:worker.state == 'READY')
            files = list((self.root/'ubuntu').iterdir())
            self.assertEqual(len(files), 1)
            events = [json.loads(line) for line in files[0].read_text(encoding='utf-8').splitlines()]
            self.assertEqual([e['kind'] for e in events], ['start_request','start_ack','stop_request','stop_ack'])
            self.assertNotIn('test-only', files[0].read_text())
        finally:
            self.assertTrue(worker.shutdown())


class InstallerTests(unittest.TestCase):
    def test_backups_and_removes_emg_pipeline_columns(self):
        source = Path(__file__).resolve().parents[2]/'work'/'control-only-upstream'
        sys.path.insert(0, str(source))
        with tempfile.TemporaryDirectory() as temp:
            target = Path(temp)/'python'; target.mkdir()
            for name in installer.CHANGES:
                shutil.copy2(source/name, target/name)
            result = installer.install(target)
            for name in installer.CHANGES:
                self.assertEqual((Path(result['backup'])/name).read_bytes(), (source/name).read_bytes())
            self.assertNotIn("+[f'emg_{i}' for i in range(7)]", (target/'hipexo_pipeline.py').read_text())
            spec = importlib.util.spec_from_file_location('patched_pipeline_test', target/'hipexo_pipeline.py')
            module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
            self.assertFalse(any(s.startswith('emg_') for s in module.STREAMS))
            records = target/'test-record'; records.mkdir()
            recorder = module.CycleRecorder(records/'combined.csv', latency_s=0, inline=True)
            self.assertFalse(any(c.startswith('emg_') for c in recorder.columns))
            self.assertTrue(recorder.stop(), recorder.error)
            with self.assertRaisesRegex(RuntimeError,'SHA256'):
                installer.install(target)


if __name__ == '__main__':
    unittest.main()
