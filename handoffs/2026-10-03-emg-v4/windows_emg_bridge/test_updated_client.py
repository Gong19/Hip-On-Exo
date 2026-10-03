"""October 3 Ubuntu compatibility and credential-free handshake diagnostics."""
import json
from pathlib import Path
import socket
import threading
import time
from types import SimpleNamespace
import unittest

import test_bridge
import ubuntu_client_20261003 as updated
from bridge_service import BUILD, PROTOCOL
from bridge_sources import Clock, SIDS

client_clock=Clock()
updated.time=SimpleNamespace(time_ns=client_clock.wall_ns,
                            perf_counter_ns=time.perf_counter_ns,monotonic=time.monotonic)


class UpdatedClientTests(test_bridge.Tests):
    def setUp(self):
        super().setUp()
        self.client=updated.RemoteWindowsSource(
            {'remote_host':'127.0.0.1','remote_port':self.bridge.port,
             'remote_token':'test-only','sensor_ids':SIDS},
            threading.Event(),self.root/'ubuntu')

    def test_new_timing_report_matches_wire_timestamps(self):
        self.ready();self.client.start();self.pump(.15)
        self.client.stop();list(self.client.drain_pending())
        timing=json.loads(Path(self.client.status['timing_report']).read_text(encoding='utf-8'))
        self.assertTrue(timing['simulated'])
        self.assertEqual(len(timing['sync_probes']),9)
        self.assertIsNone(timing['device_acquisition_latency_ns'])
        self.assertTrue(timing['timing_valid'])
        delivery=timing['first_data']
        self.assertEqual(delivery['windows_send_to_ubuntu_receive_est_ns'],
                         delivery['ubuntu_received_wall_ns']+timing['windows_minus_ubuntu_offset_ns']-
                         delivery['windows_send_wall_ns'])
        journal=Path(self.client.status['journal'])
        self.client.close()
        events=[json.loads(line) for line in journal.read_text(encoding='utf-8').splitlines()]
        hello=next(e['reply'] for e in events if e['kind']=='hello')
        self.assertEqual(hello['bridge_build'],BUILD)
        self.assertEqual(hello['bridge_instance'],self.bridge.instance_id)
        self.assertTrue(all('delivery_timing' in e for e in events if e['kind']=='data'))

    def test_handshake_reasons_and_exact_token_match(self):
        cases=[('wrong-version','test-only','PROTOCOL_MISMATCH'),
               (PROTOCOL,None,'TOKEN_FORMAT_INVALID'),
               (PROTOCOL,' test-only','TOKEN_MISMATCH'),
               (PROTOCOL,'test-only\\_','TOKEN_MISMATCH'),
               (PROTOCOL,'test-only','OK')]
        for protocol,token,expected in cases:
            with self.subTest(expected=expected):
                deadline=time.monotonic()+2
                while self.bridge.peer is not None and time.monotonic()<deadline:time.sleep(.01)
                self.assertIsNone(self.bridge.peer)
                with socket.create_connection(('127.0.0.1',self.bridge.port),timeout=2) as sock:
                    with sock.makefile('rb') as stream:
                        request={'type':'hello','id':'test-hello','protocol':protocol,'token':token}
                        sock.sendall(json.dumps(request).encode()+b'\n')
                        reply=json.loads(stream.readline())
                        self.assertEqual(reply['type'],'hello_ack' if expected=='OK' else 'error')
                        if expected!='OK':self.assertIn(expected,reply['message'])
                        snapshot=self.bridge.snapshot()
                        self.assertEqual(snapshot['last_hello']['code'],expected)
                        self.assertNotIn('test-only',json.dumps(snapshot))
                        self.assertNotIn('test-only',json.dumps(reply))
                        self.assertFalse(self.source.started)


if __name__=='__main__':unittest.main()
