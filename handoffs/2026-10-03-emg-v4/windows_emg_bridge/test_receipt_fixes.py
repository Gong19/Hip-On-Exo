"""Tests motivated by the real cross-PC STOP timeout and unit ambiguity."""
import importlib.util
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest

from bridge_service import Bridge
from bridge_sources import Clock, DelsysSource, SyntheticSource, SIDS, voltage_scale_to_v
import tempfile

patch_path=Path(__file__).resolve().parent.parent/'EMG_实测核查_20261003/ubuntu_patch/hipexo_emg_remote.py'
spec=importlib.util.spec_from_file_location('patched_ubuntu',patch_path)
patched=importlib.util.module_from_spec(spec);spec.loader.exec_module(patched)
clock=Clock()
patched.time=SimpleNamespace(time_ns=clock.wall_ns,perf_counter_ns=time.perf_counter_ns,monotonic=time.monotonic)


class ReceiptTests(unittest.TestCase):
    def test_sdk_unit_conversion_preserves_original_samples(self):
        class Poll(dict):
            def ContainsKey(self,key):return key in self
        source=DelsysSource(Clock());source.guids[1]='guid'
        source.channels[1].update(sdk_unit='Millivolts',scale_to_v=voltage_scale_to_v('Millivolts'))
        source.api=SimpleNamespace(CheckYTDataQueue=lambda:True,
            PollYTData=lambda:Poll(guid=[SimpleNamespace(Item1=0.,Item2=.21159871793246918)]))
        ch=source.poll()['channels'][0]
        self.assertEqual(ch['sdk_values'],[.21159871793246918])
        self.assertAlmostEqual(ch['values_v'][0],.00021159871793246918)
        self.assertEqual(ch['sdk_unit'],'Millivolts')
        self.assertEqual(voltage_scale_to_v('Volts'),1.)
        self.assertEqual(voltage_scale_to_v('Microvolts'),1e-6)
        with self.assertRaises(ValueError):voltage_scale_to_v('Unknown')

    def exercise_stop(self,delay,timeout=None):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source=SyntheticSource(Clock())
            original_stop=source.stop
            def slow_stop():
                if source.started:time.sleep(delay)
                original_stop()
            source.stop=slow_stop
            bridge=Bridge(source,port=0,record_dir=root/'windows').start()
            config={'remote_host':'127.0.0.1','remote_port':bridge.port,'sensor_ids':SIDS}
            if timeout is not None:config['remote_stop_timeout_s']=timeout
            client=patched.RemoteWindowsSource(config,threading.Event(),root/'ubuntu')
            try:
                bridge.prepare().result(3);bridge.arm().result(3)
                client.connect();client.synchronize();client.start();client.poll()
                if timeout is None:
                    client.stop();list(client.drain_pending())
                    self.assertTrue(client.status['stop_ack_received'])
                else:
                    with self.assertRaises(TimeoutError):client.stop()
                    self.assertIsNone(client.sock);self.assertFalse(client.started)
                    self.assertFalse(client.status['stop_ack_received'])
                deadline=time.monotonic()+delay+3
                while time.monotonic()<deadline:
                    receipts=list((root/'windows').glob('*.stop_receipt.json'))
                    if receipts and receipts[0].stat().st_size:break
                    time.sleep(.02)
                bridge.io.q.join()
                receipt=json.loads(receipts[0].read_text())
                self.assertGreaterEqual(receipt['stop_timing']['sdk_stop_duration_ns'],delay*1e9)
                self.assertLessEqual(receipt['raw_file_closed']['mono_ns'],receipt['ack_send_begin']['mono_ns'])
                self.assertIsNotNone(receipt['stop_timing']['request_received'])
                client.close()
                events=[json.loads(l) for l in next((root/'ubuntu').glob('emg_remote_*.jsonl')).read_text().splitlines()]
                self.assertEqual(sum(e['kind']=='stop_requested' for e in events),1)
                if timeout is None:
                    from validate_bridge import compare
                    self.assertEqual(compare(next((root/'windows').glob('*.jsonl')),
                                             next((root/'ubuntu').glob('*.jsonl')))['stop_tail'],'PASS')
                else:self.assertTrue(any(e['kind']=='stop_failed' for e in events))
            finally:
                client.close();bridge.shutdown()

    def test_sdk_stop_longer_than_old_three_second_budget(self):
        self.exercise_stop(3.3)

    def test_stop_timeout_closes_stream_without_second_stop(self):
        self.exercise_stop(.35,.1)


if __name__=='__main__':unittest.main()
