import json,socket,struct,threading,unittest,time
from unittest.mock import patch
import hipexo_imu_capture as capture

class Bus:
 def __init__(self,bus):self.closed=False
 def read_i2c_block_data(self,addr,reg,n):
  if addr==0x53:raise OSError('disconnected')
  return [1,0]*(n//2)
 def close(self):self.closed=True

def packet(sock):
 def read(n):
  out=b''
  while len(out)<n:
   b=sock.recv(n-len(out))
   if not b:raise EOFError
   out+=b
  return out
 return json.loads(read(struct.unpack('!I',read(4))[0]))

class CaptureTest(unittest.TestCase):
 def test_bounded_batches_disconnect_and_stop_drain(self):
  parent,child=socket.socketpair();parent.settimeout(2)
  with patch.object(capture,'SMBus',Bus),patch.object(capture,'tune_current_process',return_value={'test':True}):
   thread=threading.Thread(target=capture.child,args=(child.detach(),dict(bus=98,devices=[(0,0x50),(1,0x53)],retry_s=.01)))
   thread.start()
   try:
    self.assertIn('tuning',packet(parent));rows=packet(parent)['samples']
    self.assertLessEqual(len(rows),16)
    good=[r for r in rows if 'raw' in r];bad=[r for r in rows if 'error' in r]
    self.assertTrue(good);self.assertTrue(bad)
    self.assertTrue(all(len(r['raw'])==18 and r['duration_ms']>=0 for r in good))
    stamps=[r['mono'] for r in good];self.assertEqual(stamps,sorted(set(stamps)))
    parent.sendall(b'stop')
    while True:
     try:packet(parent)
     except EOFError:break
   finally:parent.close();thread.join(3)
   self.assertFalse(thread.is_alive())
 def test_slow_parent_does_not_pause_sampling(self):
  parent,child=socket.socketpair();child.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,1024);parent.settimeout(2)
  reads=[]
  class CountingBus(Bus):
   def read_i2c_block_data(self,addr,reg,n):
    if reg==0x34:reads.append(time.monotonic())
    return super().read_i2c_block_data(addr,reg,n)
  with patch.object(capture,'SMBus',CountingBus),patch.object(capture,'tune_current_process',return_value={'test':True}):
   thread=threading.Thread(target=capture.child,args=(child.detach(),dict(bus=99,devices=[(0,0x50),(1,0x51)],retry_s=.01)))
   thread.start()
   try:
    packet(parent);time.sleep(.15);before=len(reads);time.sleep(.15);continued=len(reads)-before
    parent.sendall(b'stop')
    while True:
     try:packet(parent)
     except EOFError:break
    self.assertGreaterEqual(continued,30,'A stalled GUI must not block native I2C sampling')
   finally:parent.close();thread.join(3)
   self.assertFalse(thread.is_alive())
if __name__=='__main__':unittest.main()
