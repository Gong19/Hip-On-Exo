import json,os,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from hipexo_i2c_clock import I2cClockLease
class ClockTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.path=Path(self.tmp.name)/'clock.lock';self.state={'rate':136000000,'mrq_rate_locked':0,'max_rate':204000000}
  self.patches=[patch.dict(os.environ,{'HIPEXO_DISABLE_TUNING':'0','HIPEXO_I2C_CLOCK_TUNING':'1'}),patch.object(I2cClockLease,'_eligible',return_value=True),patch.object(I2cClockLease,'_read',lambda _,k:self.state[k]),patch.object(I2cClockLease,'_write',lambda _,k,v:self.state.__setitem__(k,v))]
  for p in self.patches:p.start()
 def tearDown(self):
  for p in reversed(self.patches):p.stop()
  self.tmp.cleanup()
 def lease(self):
  x=I2cClockLease(1);x.lock_path=self.path;return x
 def test_restore_and_lock(self):
  with self.lease() as first:
   self.assertEqual(self.state['rate'],204000000)
   with self.lease() as second:self.assertIn('clock_error',second.report)
   self.assertEqual(self.state['rate'],204000000)
  self.assertEqual(self.state['rate'],136000000);self.assertEqual(self.state['mrq_rate_locked'],0)
 def test_recover_saved_policy_after_crash(self):
  self.path.write_text(json.dumps({'rate':136000000,'mrq_rate_locked':0}));self.state.update(rate=204000000,mrq_rate_locked=1)
  with self.lease():pass
  self.assertEqual(self.state['rate'],136000000);self.assertEqual(self.path.read_text(),'')
 def test_existing_override_refused(self):
  self.state['mrq_rate_locked']=1
  with self.lease() as x:self.assertIn('clock_error',x.report)
  self.assertEqual(self.state['rate'],136000000);self.assertEqual(self.state['mrq_rate_locked'],1)
 def test_partial_failure_restores(self):
  x=self.lease();base=x._write
  def write(k,v):
   if k=='rate' and v==204000000:raise OSError('denied')
   base(k,v)
  with patch.object(x,'_write',side_effect=write):
   with x:self.assertIn('clock_error',x.report)
  self.assertEqual(self.state['mrq_rate_locked'],0)
 def test_disabled(self):
  with patch.dict(os.environ,{'HIPEXO_I2C_CLOCK_TUNING':'0'}):
   with self.lease():self.assertEqual(self.state['rate'],136000000)
if __name__=='__main__':unittest.main()
