import os,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from hipexo_realtime import ControllerPowerLease,tune_current_process

class PowerTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
  self.controller=self.root/('test-'+self.root.name+'.spi');(self.controller/'power').mkdir(parents=True)
  self.control=self.controller/'power/control';self.control.write_text('auto\n')
  self.device=self.controller/'spi_master/spi0/spi0.0';self.device.mkdir(parents=True)
  self.env=patch.dict(os.environ,{'HIPEXO_DISABLE_TUNING':'0'});self.env.start()
  self.write=patch.object(ControllerPowerLease,'_write',lambda lease,v:lease.control.write_text(v+'\n'));self.write.start()
 def tearDown(self):self.write.stop();self.env.stop();self.tmp.cleanup()
 def test_exception_restores(self):
  lease=ControllerPowerLease(self.device)
  with self.assertRaises(RuntimeError):
   with lease:
    self.assertEqual(self.control.read_text().strip(),'on');raise RuntimeError('child failed')
  self.assertEqual(self.control.read_text().strip(),'auto')
 def test_second_owner_cannot_restore_first(self):
  with ControllerPowerLease(self.device):
   with ControllerPowerLease(self.device) as second:self.assertIn('power_error',second.report)
   self.assertEqual(self.control.read_text().strip(),'on')
  self.assertEqual(self.control.read_text().strip(),'auto')
 def test_disable_does_not_tune(self):
  with patch.dict(os.environ,{'HIPEXO_DISABLE_TUNING':'1'}),patch('subprocess.run') as run:
   self.assertTrue(tune_current_process()['disabled'])
   with ControllerPowerLease(self.device):pass
   run.assert_not_called();self.assertEqual(self.control.read_text().strip(),'auto')
 def test_failed_restore_retains_recovery_state(self):
  lease=ControllerPowerLease(self.device);lease.__enter__()
  with patch.object(lease,'_write',side_effect=OSError('denied')):lease.__exit__(None,None,None)
  self.assertIn('restore_error',lease.report)
  with ControllerPowerLease(self.device) as retry:self.assertEqual(retry.previous,'auto')
  self.assertEqual(self.control.read_text().strip(),'auto')
if __name__=='__main__':unittest.main()
