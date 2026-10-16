import unittest
import numpy as np
from hipexo_emg_core import EmgProcessor
from hipexo_pipeline import resample

def channels(fs):return [dict(sid=i,present=True,mode='raw',sample_rate=fs,is_rms=False) for i in range(7)]
class RateTests(unittest.TestCase):
 def test_rates_batches_and_window_bound(self):
  for fs in (1000,2000,4000/27):
   p=EmgProcessor(channels(fs));rows=[];t=np.arange(int(fs*4)+1)/fs
   for start in range(0,len(t),73):
    ts=t[start:start+73];rows+=p.ingest({i:(ts,np.sin(ts*10)*.001) for i in range(7)},now=start/fs)
   self.assertEqual([r['label_ms'] for r in rows],list(range(rows[-1]['label_ms']+1)))
   self.assertLessEqual(len(p.output_window),2001)
   for c in p.rate_status():
    self.assertAlmostEqual(c['native_measured_hz'],fs,places=5);self.assertEqual(c['valid_output_hz'],1000)
 def test_short_native_gap_not_fabricated(self):
  p=EmgProcessor(channels(2000));t=np.r_[np.arange(0,20)/2000,np.arange(60,100)/2000]
  rows=p.ingest({i:(t,np.ones(len(t))*.001) for i in range(7)},now=.1)
  labels=[r['label_ms'] for r in rows]
  self.assertNotIn(20,labels);self.assertLess(p.rate_status()[0]['coverage'],1)
  samples=[(r['label_ms']*1000000,dict(raw_v=r['raw_v'][0],envelope_v=r['envelope_v'][0],mvc_ratio=0,valid=1)) for r in rows]
  _,q,_=resample('emg_0',samples,np.arange(10,30,dtype=np.int64)*1000000)
  self.assertTrue((q==0).all())
 def test_combined_does_not_repair_missing_output_sample(self):
  sample=dict(raw_v=.001,envelope_v=.001,mvc_ratio=0,valid=1)
  _,q,_=resample('emg_0',[(0,sample),(2000000,sample)],np.array([0,1000000,2000000]))
  self.assertEqual(list(q),[1,0,1])
 def test_filter_independent_of_chunk_size(self):
  t=np.arange(4000)/2000;v=np.sin(2*np.pi*100*t)*.001
  results=[]
  for batch in (37,4000):
   p=EmgProcessor(channels(2000));rows=[]
   for start in range(0,len(t),batch):rows+=p.ingest({i:(t[start:start+batch],v[start:start+batch]) for i in range(7)},now=start/2000)
   results.append(np.array([r['raw_v'] for r in rows]))
  np.testing.assert_allclose(*results,atol=1e-12)
if __name__=='__main__':unittest.main()
