"""Protocol/lifecycle tests against the user's unmodified Ubuntu client."""
import json
from pathlib import Path
import socket
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
import uuid

import ubuntu_client_reference as remote
from bridge_service import Bridge, PROTOCOL
from bridge_sources import Clock, SyntheticSource, SIDS

# Ubuntu has high-resolution realtime. Emulate it on Windows without modifying
# the source client or the machine's clock. The server has an independent clock.
client_clock=Clock()
remote.time=SimpleNamespace(time_ns=client_clock.wall_ns,perf_counter_ns=time.perf_counter_ns,monotonic=time.monotonic)


class Tests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.clock=Clock();self.clock.offset_ns=2_000_000_000
        self.source=SyntheticSource(self.clock,missing=(3,))
        self.bridge=Bridge(self.source,port=0,token='test-only',record_dir=self.root/'windows').start()
        self.client=remote.RemoteWindowsSource({'remote_host':'127.0.0.1','remote_port':self.bridge.port,
                                              'remote_token':'test-only','sensor_ids':SIDS},threading.Event(),self.root/'ubuntu')

    def tearDown(self):
        try:self.client.close()
        except Exception:pass
        self.bridge.shutdown()
        self.temp.cleanup()

    def ready(self):
        self.bridge.prepare().result(3);self.bridge.arm().result(3)
        self.client.connect();self.client.synchronize()

    def pump(self,duration=.12):
        end=time.monotonic()+duration
        while time.monotonic()<end:self.client.poll()

    def test_real_client_restart_missing_slot_and_tail(self):
        self.ready()
        self.assertLess(abs(self.client.sync['offset_ns']-2_000_000_000),5_000_000)
        self.assertFalse(self.client.channels[3]['present'])
        self.assertEqual(self.client.channels[4]['sid'],SIDS[4])
        ids=[]
        for _ in range(2):
            self.client.start();ids.append(self.client.run_id);self.pump()
            self.client.stop();list(self.client.drain_pending())
        self.assertNotEqual(*ids)
        journal=Path(self.client.status['journal']);self.client.close()
        received=[x['message'] for x in map(json.loads,journal.read_text().splitlines()) if x['kind']=='data']
        for path in (self.root/'windows').glob('*.jsonl'):
            events=list(map(json.loads,path.read_text().splitlines()))
            self.assertTrue(events[-1]['recording_complete'])
            sent=[e['message'] for e in events if e['kind']=='data']
            counterpart=[x for x in received if x['run_id']==events[0]['run_id']]
            self.assertEqual(sent,counterpart)
            self.assertEqual([m['seq'] for m in sent],list(range(events[-1]['last_seq']+1)))
            self.assertNotIn(3,[c['slot'] for m in sent for c in m['channels']])
            polls=[e['poll'] for e in events if e['kind']=='raw_poll']
            self.assertEqual(sum(len(c['values_v']) for p in polls for c in p['channels']),sum(events[-1]['channel_sample_counts']))
        self.assertFalse(self.source.started)

    def test_unarmed_and_bad_token(self):
        self.client.connect()
        with self.assertRaisesRegex(RuntimeError,'not armed'):self.client.synchronize()
        self.client.close();time.sleep(.08)
        self.client.config['remote_token']='wrong'
        with self.assertRaisesRegex(RuntimeError,'token'):self.client.connect()
        self.assertFalse(self.source.started)

    def test_disconnect_stops_and_preserves_raw(self):
        self.ready();self.client.start();self.pump()
        self.client.sock.close();self.client.sock=None;self.client.started=False
        end=time.monotonic()+3
        while self.bridge.run and time.monotonic()<end:time.sleep(.02)
        self.assertIsNone(self.bridge.run);self.assertFalse(self.source.started)
        events=list(map(json.loads,next((self.root/'windows').glob('*.jsonl')).read_text().splitlines()))
        self.assertEqual(events[-1]['reason'],'client_disconnected')
        self.assertFalse(events[-1]['recording_complete'])
        self.assertTrue(any(e['kind']=='raw_poll' for e in events))

    def test_wall_clock_step_ends_run(self):
        self.ready();self.client.start();self.pump(.06)
        self.clock.offset_ns+=2_000_000_000
        end=time.monotonic()+3
        while self.bridge.run and time.monotonic()<end:time.sleep(.02)
        self.assertIsNone(self.bridge.run);self.assertFalse(self.source.started)
        self.assertIn('UTC changed',self.bridge.snapshot()['error'])
        self.client.started=False

    def test_heartbeat_timeout_ends_run(self):
        self.ready();self.client.start()
        self.bridge.heartbeat_seconds=.1
        end=time.monotonic()+2
        while self.bridge.run and time.monotonic()<end:time.sleep(.02)
        self.assertIsNone(self.bridge.run)
        self.assertIn('heartbeat_timeout',self.bridge.snapshot()['error'])
        self.client.started=False

    def test_fragmented_commands_and_wrong_run(self):
        self.bridge.prepare().result(3);self.bridge.arm().result(3)
        sock=socket.create_connection(('127.0.0.1',self.bridge.port),timeout=2)
        stream=sock.makefile('rb')
        try:
            hello=json.dumps({'type':'hello','id':'a','protocol':PROTOCOL,'token':'test-only'}).encode()+b'\n'
            for chunk in (hello[:3],hello[3:19],hello[19:]):sock.sendall(chunk)
            self.assertEqual(json.loads(stream.readline())['type'],'hello_ack')
            sock.sendall(b'{"type":"ping","id":"b"}\n{"type":"stop","id":"c","run_id":"wrong"}\n')
            self.assertEqual(json.loads(stream.readline())['type'],'ping_ack')
            self.assertEqual(json.loads(stream.readline())['type'],'error')
        finally:stream.close();sock.close()

    def test_record_open_failure_never_starts_sdk(self):
        self.ready()
        bad=self.root/'is-a-file';bad.write_text('x');self.bridge.record_dir=str(bad)
        with self.assertRaises((OSError,ConnectionError,RuntimeError,ValueError,TimeoutError)):
            self.client.start()
        self.assertFalse(self.source.started)
        self.client.started=False

    def test_source_clock_rewind_is_recorded_and_stops(self):
        self.ready();self.client.start();self.pump(.06)
        original=self.source.poll
        injected=[False]
        def rewind():
            poll=original()
            if poll and not injected[0]:
                injected[0]=True
                for ch in poll['channels']:ch['device_time_s']=[t-10 for t in ch['device_time_s']]
            return poll
        self.source.poll=rewind
        end=time.monotonic()+3
        while self.bridge.run and time.monotonic()<end:time.sleep(.02)
        self.assertIsNone(self.bridge.run)
        self.assertIn('backwards',self.bridge.snapshot()['error'])
        events=list(map(json.loads,next((self.root/'windows').glob('*.jsonl')).read_text().splitlines()))
        self.assertTrue(any(e['kind']=='raw_poll' and any(t<0 for c in e['poll']['channels'] for t in c['device_time_s']) for e in events))
        self.client.started=False

    def test_slow_writer_stops_with_explicit_backlog_error(self):
        self.ready();self.client.start()
        self.bridge.io.q.maxsize=4
        original=self.bridge.io._handle
        def slow(kind,fields):
            if kind=='raw_poll':time.sleep(.025)
            return original(kind,fields)
        self.bridge.io._handle=slow
        end=time.monotonic()+4
        while self.bridge.run and time.monotonic()<end:
            try:self.client.poll()
            except RuntimeError:break
        while self.bridge.run and time.monotonic()<end:time.sleep(.02)
        self.assertIsNone(self.bridge.run)
        self.assertEqual(self.bridge.snapshot()['state'],'ERROR')
        self.assertFalse(self.source.started)
        self.client.started=False


if __name__=='__main__':unittest.main()
