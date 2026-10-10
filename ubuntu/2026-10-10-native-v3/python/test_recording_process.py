import tempfile,time,unittest,json
from pathlib import Path
from hipexo_recording_process import ProcessCycleRecorder
class ProcessTests(unittest.TestCase):
    def test_process_exit_is_visible_not_success(self):
        with tempfile.TemporaryDirectory() as d:
            errors=[];r=ProcessCycleRecorder(Path(d)/'x.csv',on_error=errors.append)
            r._process.kill()
            self.assertTrue(r._done.wait(3))
            self.assertFalse(r.stop());self.assertTrue(errors)
    def test_overflow_automatically_stops_writer(self):
        with tempfile.TemporaryDirectory() as d:
            r=ProcessCycleRecorder(Path(d)/'x.csv')
            self.assertFalse(r._enqueue(('bad',),65*1024*1024))
            self.assertTrue(r._done.wait(5))
            self.assertFalse(r.stop())
            self.assertFalse(r._process.poll() is None)
            quality=json.loads((Path(d)/'pipeline_quality.json').read_text())
            self.assertIn('Parent recorder',quality['error'])

    def test_unexpected_ipc_close_flushes_accepted_data_and_marks_incomplete(self):
        import socket
        with tempfile.TemporaryDirectory() as d:
            r=ProcessCycleRecorder(Path(d)/'x.csv')
            r.enqueue_frames('force_0',[0],[dict(V=1,kg=2)],[r.start_mono_ns],[0])
            time.sleep(.15)
            r._sock.shutdown(socket.SHUT_RDWR)
            self.assertTrue(r._done.wait(3));self.assertFalse(r.stop())
            r._process.wait(timeout=3)
            quality=json.loads((Path(d)/'pipeline_quality.json').read_text())
            self.assertIsNotNone(quality['error']);self.assertEqual(quality['accepted_frames'],1)

    def test_order_and_nan_survive_ipc(self):
        with tempfile.TemporaryDirectory() as d:
            packets=[];r=ProcessCycleRecorder(Path(d)/'x.csv',on_cycle=packets.append)
            start=r.start_mono_ns
            for i in range(10):r.enqueue_frames('force_0',[0],[dict(V=i,kg=float('nan'))],[start+i*1_000_000],[i])
            r.stop_ns=start+10_000_000
            self.assertTrue(r.stop());self.assertEqual(r.stats['force_0']['samples'],10)
            self.assertEqual(r.metrics['out_of_order_frames'],0);self.assertEqual(len(packets),1)
if __name__=='__main__':unittest.main()
