import socket,struct,json,unittest
from hipexo_capture_ipc import PacketSender
class IPCTests(unittest.TestCase):
 def test_nonblocking_backlog_then_exact_drain(self):
  a,b=socket.socketpair();a.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,4096);b.setblocking(False);sender=PacketSender(a,limit=65536);expected=[];raw=bytearray()
  try:
   for i in range(40):
    value={'i':i,'padding':'x'*700};expected.append(value);sender.send(value)
   self.assertGreater(sender.size,0);self.assertLessEqual(sender.peak,65536)
   for _ in range(100):
    try:raw.extend(b.recv(65536))
    except BlockingIOError:pass
    sender.pump()
   actual=[]
   while raw:
    n=struct.unpack('!I',raw[:4])[0];actual.append(json.loads(raw[4:4+n]));del raw[:4+n]
   self.assertEqual(actual,expected);self.assertEqual(sender.size,0)
  finally:a.close();b.close()
 def test_capacity_error_and_bounded_stop(self):
  a,b=socket.socketpair();a.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,4096);sender=PacketSender(a,limit=8192)
  try:
   with self.assertRaises(BufferError):
    for i in range(100):sender.send({'i':i,'data':'x'*1000})
   self.assertLessEqual(sender.size,8192)
   with self.assertRaises(TimeoutError):sender.drain(.01)
  finally:a.close();b.close()
if __name__=='__main__':unittest.main()
