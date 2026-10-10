"""Bounded nonblocking sensor IPC: host GUI stalls must not pause sampling."""
from collections import deque
import json,select,struct,time
class PacketSender:
    def __init__(self,sock,limit=256*1024):
        self.sock=sock;self.sock.setblocking(False);self.limit=limit;self.queue=deque();self.size=0;self.peak=0
    def send(self,value):
        raw=json.dumps(value,separators=(',',':')).encode();packet=struct.pack('!I',len(raw))+raw
        if self.size+len(packet)>self.limit:raise BufferError('Sensor IPC bounded queue overflow')
        self.queue.append(memoryview(packet));self.size+=len(packet);self.peak=max(self.peak,self.size);self.pump()
    def pump(self):
        while self.queue:
            try:n=self.sock.send(self.queue[0])
            except BlockingIOError:return
            if n<=0:raise BrokenPipeError('Sensor IPC disconnected')
            self.size-=n
            if n==len(self.queue[0]):self.queue.popleft()
            else:self.queue[0]=self.queue[0][n:]
    def drain(self,timeout=1):
        end=time.monotonic()+timeout
        while self.queue:
            self.pump()
            if not self.queue:return
            remaining=end-time.monotonic()
            if remaining<=0:raise TimeoutError('Sensor IPC final drain timed out')
            select.select([],[self.sock],[],min(.02,remaining))
