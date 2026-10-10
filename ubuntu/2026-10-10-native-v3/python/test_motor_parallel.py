"""Fake links only: independent ports, no command interleaving, checked feedback."""
import os,time,threading,unittest
from collections import defaultdict
from types import SimpleNamespace
from unittest.mock import patch
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
os.environ['HIPEXO_DISABLE_TUNING']='1'
from PyQt5 import QtWidgets
import hipexo_monitor as m
APP=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
class Cmd:pass
class Data:
 q=0.;dq=0.;temp=25.;merror=0;correct=True
class Sink:
 def __init__(self):self.rows=defaultdict(list)
 def append_batch(self,prefix,wall,frames,mono):self.rows[prefix].extend(zip(mono,frames))
class Serial:
 def __init__(self,path):self.delay=.002 if path=='fast' else .012;self.lock=threading.Lock();self.calls=[];self.bad=False;self.overlap=False
 def sendRecv(self,cmd,data):
  if not self.lock.acquire(False):self.overlap=True;raise RuntimeError('interleaved')
  try:
   self.calls.append((cmd.q,cmd.dq,cmd.kp,cmd.kd,cmd.tau));time.sleep(self.delay)
   data.q+=1;data.correct=not self.bad;return not self.bad
  finally:self.lock.release()
class MotorTests(unittest.TestCase):
 def test_independent_links_and_zero_stop(self):
  with patch.multiple(m,create=True,_SDK_OK=True,MotorCmd=Cmd,MotorData=Data,SerialPort=Serial,MotorType=SimpleNamespace(GO_M8010_6=0),MotorMode=SimpleNamespace(FOC=1),queryMotorMode=lambda *a:1):
   dm=Sink();worker=m.MotorWorker([('fast',0),('slow',1)],dm)
   errors=[];worker.sig_error.connect(errors.append)
   worker.start_monitoring();time.sleep(.28);worker.shutdown()
   self.assertGreater(len(dm.rows['motor_0']),3*len(dm.rows['motor_1']))
   for serial in worker.serials.values():
    self.assertFalse(serial.overlap);self.assertTrue(serial.calls)
    self.assertTrue(all(c==(0,0,0,0,0) for c in serial.calls))
   for rows in dm.rows.values():
    stamps=[r[0] for r in rows];self.assertEqual(stamps,sorted(set(stamps)))
   self.assertIsNot(worker._motor_state[('fast',0)][1],worker._motor_state[('slow',1)][1])
   self.assertFalse(worker._thread.is_alive())
   APP.processEvents();self.assertFalse(errors)
 def test_invalid_feedback_never_becomes_fresh_sample(self):
  with patch.multiple(m,create=True,_SDK_OK=True,MotorCmd=Cmd,MotorData=Data,SerialPort=Serial,MotorType=SimpleNamespace(GO_M8010_6=0),MotorMode=SimpleNamespace(FOC=1),queryMotorMode=lambda *a:1):
   dm=Sink();worker=m.MotorWorker([('fast',0)],dm);worker.init_links();worker.serials['fast'].bad=True
   worker.start_monitoring();time.sleep(.06);worker.shutdown()
   self.assertFalse(dm.rows['motor_0'])
if __name__=='__main__':unittest.main()
