import unittest,tempfile,sys,os,json
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PyQt5 import QtWidgets
from hipexo_emg_control import EmgWorker,EmgPanel
from control_protocol import ControlClient
APP=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
class UbuntuTests(unittest.TestCase):
 def test_explicit_start_is_independent_of_global_linkage(self):
  w=EmgWorker(SimpleNamespace(export_dir='/tmp/unused'))
  with patch.object(w,'_submit') as submit:
   w.record_start('off.csv');submit.assert_not_called()
   w.sync_start();self.assertEqual(submit.call_args.args[0],'start')
   self.assertIsNone(submit.call_args.kwargs['ubuntu_record_path'])
   w.enabled=True;w.record_start('on.csv');self.assertEqual(submit.call_args.kwargs['ubuntu_record_path'],'on.csv')
  self.assertFalse(hasattr(w,'history'));self.assertIsNone(w.processor)
 def test_status_only_panel_has_no_plot(self):
  w=EmgWorker(SimpleNamespace(export_dir='/tmp/unused'));p=EmgPanel(w)
  try:
   p._remote(dict(connected=True,recording=True,ready=False,record_path='Windows.csv',saved_rows=1200,save_ok=True))
   self.assertIn('1200',p.file_status.text());self.assertFalse(hasattr(p,'plots'))
  finally:p.close()
 def test_clock_step_rejected(self):
  c=ControlClient('localhost',8766,'test')
  with patch('control_protocol.stamp',side_effect=[dict(wall_ns=0,mono_ns=0),dict(wall_ns=100000000,mono_ns=1000)]),patch.object(c,'request',return_value=dict(t2_ns=1,t3_ns=2)):
   with self.assertRaisesRegex(RuntimeError,'clock stepped'):c.synchronize(1)
 def test_negative_windows_processing_time_rejected(self):
  c=ControlClient('localhost',8766,'test')
  with patch('control_protocol.stamp',side_effect=[dict(wall_ns=0,mono_ns=0),dict(wall_ns=1000,mono_ns=1000)]),patch.object(c,'request',return_value=dict(t2_ns=200,t3_ns=100)):
   with self.assertRaisesRegex(RuntimeError,'Clock changed'):c.synchronize(1)
if __name__=='__main__':unittest.main()

