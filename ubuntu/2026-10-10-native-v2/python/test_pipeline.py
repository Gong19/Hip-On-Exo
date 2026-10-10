import csv,json,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from hipexo_pipeline import CycleRecorder,resample,FIELDS,STREAMS

class PipelineTests(unittest.TestCase):
    def test_interpolation_gap_angle_and_reference(self):
        frame=lambda angle,ref:dict(roll_deg=angle,pitch_deg=0,yaw_deg=0,reference_id=ref,reference_valid=1)
        samples=[(0,frame(179,1)),(20_000_000,frame(-179,1)),(100_000_000,frame(10,2))]
        values,q,age=resample('imu_0',samples,np.array([0,10_000_000,60_000_000],dtype=np.int64))
        j=[k for k,_ in FIELDS['imu']].index('roll_deg')
        self.assertEqual(q.tolist(),[1,2,0]);self.assertAlmostEqual(abs(values[1,j]),180)
        self.assertTrue(np.isnan(values[2,j]))
        _,q,_=resample('imu_0',[(0,frame(0,1)),(10_000_000,frame(1,2))],np.array([5_000_000]))
        self.assertEqual(q[0],0)
    def test_emg_invalid_is_missing_and_no_extrapolation(self):
        samples=[(0,dict(raw_v=1,valid=1)),(1_000_000,dict(raw_v=0,valid=0)),(2_000_000,dict(raw_v=2,valid=1))]
        _,q,_=resample('emg_0',samples,np.array([0,500_000,1_000_000,3_000_000]))
        self.assertEqual(q.tolist(),[1,0,0,0])
    def test_exact_cycles_native_units_camera_and_tail(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'test.csv';start=time.perf_counter_ns();packets=[]
            r=CycleRecorder(path,on_cycle=packets.append,start_mono_ns=start,start_wall_ns=1_000_000_000)
            frames=[dict(V=i,kg=i*2) for i in range(7)]
            r.enqueue_frames('force_0',list(range(7)),frames,[start+i*5_000_000 for i in range(7)],list(range(7)))
            r.enqueue_image(np.ones((4,4),dtype=np.uint16),dict(host_frame_received_mono_ns=start+10_000_000,host_frame_received_wall_ns=1_010_000_000,device_frame_number=1,camera_stream_id='fake'))
            r.stop_ns=start+310_000_000
            self.assertTrue(r.stop());self.assertEqual(len(packets),2)
            for p in packets:
                self.assertEqual(len(p['rows']),300)
                self.assertEqual([row[1] for row in p['rows']],sum(([i]*60 for i in range(5)),[]))
            self.assertEqual(sum(row[6] for row in packets[1]['rows']),10)
            self.assertEqual(len(packets[0]['camera_images']),1)
            with path.open() as f:rows=list(csv.DictReader(f))
            self.assertEqual(len(rows),600);self.assertIn('force_0__load_equivalent_kgf',rows[0])
            manifests=[json.loads(line) for line in (Path(tmp)/'cycles.jsonl').read_text().splitlines()]
            self.assertNotIn('rows',manifests[0])
            with path.open('rb') as payload:
                payload.seek(manifests[0]['csv_byte_start'])
                block=payload.read(manifests[0]['csv_byte_end']-manifests[0]['csv_byte_start'])
            self.assertEqual(len(list(csv.reader(block.decode().splitlines()))),300)
            self.assertEqual(len(list(Path(tmp).glob('*native.csv'))),1)
            self.assertEqual(r.stats['force_0']['samples'],7)
            self.assertEqual(r.metrics['queue_overflow'],0)
    def test_late_data_retained_native_not_rewritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            start=time.perf_counter_ns();r=CycleRecorder(Path(tmp)/'x.csv',start_mono_ns=start)
            # Prevent worker time boundary; direct acceptance only while idle.
            r._cycle=1
            r.enqueue_frames('force_0',[0],[dict(V=1,kg=2)],[start],[0]);r.stop_ns=start+300_000_000
            self.assertTrue(r.stop());self.assertEqual(r.metrics['late_frames'],1)
            self.assertEqual(r.stats['force_0']['samples'],1)
    def test_disk_failure_surfaces(self):
        with tempfile.TemporaryDirectory() as tmp:
            errors=[];r=CycleRecorder(Path(tmp)/'x.csv',on_error=errors.append)
            with patch.object(r,'_emit',side_effect=OSError('disk full')):self.assertFalse(r.stop())
            self.assertTrue(errors)
    def test_overflow_visible(self):
        with tempfile.TemporaryDirectory() as tmp:
            r=CycleRecorder(Path(tmp)/'x.csv')
            self.assertFalse(r._enqueue(('fake',),65*1024*1024));self.assertFalse(r.stop())
            self.assertEqual(r.metrics['queue_overflow'],1)

class BackpressureTests(unittest.TestCase):
    def test_transient_writer_pause_preserves_bounded_queue_and_all_frames(self):
        import threading
        entered=threading.Event();release=threading.Event();outcome=[]
        original=CycleRecorder._accept
        def paused(recorder,item):
            if not entered.is_set():entered.set();release.wait(2)
            return original(recorder,item)
        with tempfile.TemporaryDirectory() as tmp,patch.object(CycleRecorder,'_accept',paused):
            start=time.perf_counter_ns();r=CycleRecorder(Path(tmp)/'x.csv',capacity=1,block_on_full=True,start_mono_ns=start)
            def put(i):return r.enqueue_frames('force_0',[i],[dict(V=i,kg=i)],[start+i*1_000_000],[i])
            self.assertTrue(put(0));self.assertTrue(entered.wait(1));self.assertTrue(put(1))
            thread=threading.Thread(target=lambda:outcome.append(put(2)));thread.start()
            try:
                time.sleep(.12);self.assertTrue(thread.is_alive());self.assertEqual(r._queue.qsize(),1)
                self.assertIsNone(r.error)
            finally:release.set();thread.join(2)
            r.stop_ns=start+3_000_000;self.assertTrue(r.stop());self.assertEqual(outcome,[True])
            self.assertEqual(r.stats['force_0']['samples'],3);self.assertEqual(r.metrics['queue_overflow'],0)
            self.assertGreater(r.metrics['queue_backpressure_waits'],0)

class AntiAliasTests(unittest.TestCase):
    def test_above_nyquist_rejected_and_chunks_continuous(self):
        from hipexo_emg_core import EmgProcessor
        channels=[dict(present=i==0,is_rms=False,sample_rate=2000,sid=i,mode='raw') for i in range(7)]
        t=np.arange(4000)/2000;v=.001*np.sin(2*np.pi*700*t)
        a=EmgProcessor(channels);rows=a.ingest({0:(t,v)});a.drain(force=True)
        rms=np.sqrt(np.mean([r['raw_v'][0]**2 for r in rows[500:]]))
        self.assertLess(rms,.00005)
        b=EmgProcessor(channels);split=[]
        for offset in range(0,len(t),40):split+=b.ingest({0:(t[offset:offset+40],v[offset:offset+40])})
        np.testing.assert_allclose([r['raw_v'][0] for r in split],[r['raw_v'][0] for r in rows],atol=1e-12)

if __name__=='__main__':unittest.main()
