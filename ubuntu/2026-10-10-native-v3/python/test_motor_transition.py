"""Exercise native-monitor / SDK-control ownership with fake links only."""
import os,time,threading,unittest
from types import SimpleNamespace
from unittest.mock import patch
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PyQt5 import QtWidgets
import hipexo_monitor as m
from test_motor_parallel import Cmd,Data,Sink,Serial
APP=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
class FakeCapture:
 active=False;started=threading.Event()
 def __init__(self,*a):
  self.stopping=False;self.sock=SimpleNamespace(_closed=False);self.stats={};self.tuning_error=None;self.cpu_affinity=4;FakeCapture.active=True;FakeCapture.started.set()
 def request_stop(self):self.stopping=True
 def stop(self):self.stopping=True;FakeCapture.active=False
 def receive(self):
  time.sleep(.002)
  if self.stopping:return None
  return [(time.perf_counter_ns(),time.time_ns(),0.,0.,25.,0)]
 def close(self):self.stop();self.sock._closed=True
class CheckedSerial(Serial):
 def sendRecv(self,*a):
  if FakeCapture.active:raise AssertionError('SDK transmission overlapped native capture')
  return super().sendRecv(*a)
class TransitionTests(unittest.TestCase):
 def test_native_control_stop_ownership(self):
  FakeCapture.started.clear()
  with patch.dict(os.environ,{'HIPEXO_DISABLE_TUNING':'1','HIPEXO_MOTOR_NATIVE':'1'}),patch('hipexo_motor_process.MotorCapture',FakeCapture),patch('os.path.isfile',return_value=True),patch.multiple(m,create=True,_SDK_OK=True,MOTOR_PARAMS={0:{"MODE":"ZERO"}},MotorCmd=Cmd,MotorData=Data,SerialPort=CheckedSerial,MotorType=SimpleNamespace(GO_M8010_6=0),MotorMode=SimpleNamespace(FOC=1),queryMotorMode=lambda *a:1):
   worker=m.MotorWorker([('/dev/FAKE_v3',0)],Sink());errors=[];worker.sig_error.connect(errors.append)
   try:
    worker.start_monitoring();self.assertTrue(FakeCapture.started.wait(2));time.sleep(.02)
    worker.start();time.sleep(.08);self.assertTrue(worker.serials['/dev/FAKE_v3'].calls);self.assertFalse(FakeCapture.active)
    FakeCapture.started.clear();worker.start_monitoring();self.assertTrue(FakeCapture.started.wait(2))
    worker.stop_monitoring();self.assertFalse(FakeCapture.active)
   finally:worker.shutdown()
   APP.processEvents();self.assertFalse(errors);self.assertFalse(worker._thread.is_alive())
if __name__=='__main__':unittest.main()
