"""Pseudo-terminal protocol tests only; no physical motors are opened."""
import os,pty,socket,struct,time,unittest,select
from pathlib import Path
from unittest.mock import patch
from hipexo_motor_process import MotorCapture,ROW,STATS,configured_target_hz

def crc(data):
 v=0
 for b in data:
  v^=b
  for _ in range(8):v=(v>>1)^0x8408 if v&1 else v>>1
 return v

def feedback(mid=0):
 b=b'\xfd\xee'+bytes([0x10|mid])+struct.pack('<hhibH',0,0,0,25,0)
 return b+struct.pack('<H',crc(b))

class PacketTests(unittest.TestCase):
 def test_target_configuration(self):
  with patch.dict(os.environ,{'HIPEXO_MOTOR_NATIVE_HZ':'950'}):self.assertEqual(configured_target_hz(),950)
  with patch.dict(os.environ,{'HIPEXO_MOTOR_NATIVE_HZ':'995'}):self.assertEqual(configured_target_hz(),995)
  with patch.dict(os.environ,{'HIPEXO_MOTOR_NATIVE_HZ':'990'}):self.assertEqual(configured_target_hz(),990)
  for value in ['0','1001','nan']:
   with patch.dict(os.environ,{'HIPEXO_MOTOR_NATIVE_HZ':value}):
    with self.assertRaises(ValueError):configured_target_hz()

 def test_packet_split_and_invalid_length(self):
  c=MotorCapture.__new__(MotorCapture);c.sock,other=socket.socketpair();c.pending=bytearray();c.stats={}
  payload=STATS.pack(3,2,0,0,0,64)+ROW.pack(100,200,1,2,25,0);packet=struct.pack('<I',len(payload))+payload
  try:
   other.sendall(packet[:6]);self.assertEqual(c.receive(),[]);other.sendall(packet[6:]);self.assertEqual(c.receive(),[(100,200,1.,2.,25.,0)])
   self.assertEqual(c.stats['valid'],2);other.sendall(struct.pack('<I',1000000))
   with self.assertRaises(ValueError):c.receive()
  finally:c.sock.close();other.close()

@unittest.skipUnless(Path(__file__).with_name('hipexo_motor_capture').exists(),'Build helper first')
class NativeTests(unittest.TestCase):
 def test_zero_commands_crc_filter_and_stop(self):
  master,slave=pty.openpty();path=os.ttyname(slave);os.set_blocking(master,False)
  with patch.dict(os.environ,{'HIPEXO_DISABLE_TUNING':'1'}):c=MotorCapture(path,0)
  commands=bytearray();rows=[];count=0;start=time.monotonic();last_tx=None
  try:
   while time.monotonic()-start<.35:
    if select.select([master],[],[],.003)[0]:
     commands.extend(os.read(master,4096))
     while len(commands)>=17:
      command=bytes(commands[:17]);del commands[:17];self.assertEqual(command[:3],b'\xfe\xee\x10');self.assertEqual(command[3:15],bytes(12));self.assertEqual(int.from_bytes(command[15:],'little'),crc(command[:15]));count+=1
      if count==2:os.write(master,b'\xfd\xee'+bytes(14)) # invalid CRC / ID
      os.write(master,feedback())
    ready=select.select([c.sock],[],[],0)[0]
    if ready:
     batch=c.receive()
     if batch:rows.extend(batch)
   c.request_stop()
   while True:
    # Finish requests already queued before the stop was observed.
    if select.select([master],[],[],0)[0]:
     extra=os.read(master,4096);commands.extend(extra)
     while len(commands)>=17:del commands[:17];os.write(master,feedback())
    batch=c.receive()
    if batch is None:break
    rows.extend(batch)
   c.close();self.assertGreater(count,50);self.assertGreater(len(rows),50);self.assertGreater(c.stats['bad_bytes'],0)
   self.assertTrue(all(r[4]==25 for r in rows));self.assertEqual([r[0] for r in rows],sorted(set(r[0] for r in rows)))
   self.assertEqual(c.stats['valid'],len(rows))
  finally:
   if c.process.poll() is None:c.stop()
   if not c.sock._closed:c.sock.close()
   os.close(master);os.close(slave)
 def test_parent_disconnect_exits_without_further_commands(self):
  master,slave=pty.openpty()
  with patch.dict(os.environ,{'HIPEXO_DISABLE_TUNING':'1'}):c=MotorCapture(os.ttyname(slave),0)
  c.sock.close();c.process.wait(timeout=2);c.process.stderr.close();os.close(master);os.close(slave)
  self.assertEqual(c.process.returncode,0)
if __name__=='__main__':unittest.main()
