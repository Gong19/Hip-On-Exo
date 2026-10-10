"""Private, bounded 16-bit PNG encoder process. No camera or network access."""
import json,os,socket,struct,subprocess,sys
from pathlib import Path
MAX_IMAGE_BYTES=2*1024*1024

def read_exact(sock,n):
    out=bytearray()
    while len(out)<n:
        data=sock.recv(n-len(out))
        if not data:raise EOFError('PNG encoder disconnected')
        out.extend(data)
    return out

class ImageWriter:
    def __init__(self):
        self.sock,other=socket.socketpair();self.sock.settimeout(5)
        try:
            self.process=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),str(other.fileno())],
                pass_fds=(other.fileno(),),stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        except Exception:
            self.sock.close();raise
        finally:other.close()
    def write(self,path,depth):
        if depth.dtype.name!='uint16' or len(depth.shape)!=2 or depth.nbytes>MAX_IMAGE_BYTES:
            raise ValueError('Expected bounded uint16 depth image')
        header=json.dumps(dict(path=path,shape=list(depth.shape))).encode()
        raw=depth.tobytes(order='C')
        self.sock.sendall(struct.pack('!II',len(header),len(raw))+header+raw)
        n=struct.unpack('!I',read_exact(self.sock,4))[0]
        if n>65536:raise ValueError('Invalid PNG status size')
        reply=json.loads(read_exact(self.sock,n))
        if reply.get('error'):raise IOError(reply['error'])
    def close(self,force=False):
        try:
            if self.process.poll() is None:
                if force:self.process.terminate()
                else:self.sock.sendall(struct.pack('!II',0,0))
                self.process.wait(timeout=2)
        except Exception:
            self.process.terminate()
            try:self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        finally:self.sock.close()

def child(fd):
    import numpy as np
    import cv2
    sock=socket.socket(fileno=fd)
    try:
        while True:
            nh,nd=struct.unpack('!II',read_exact(sock,8))
            if nh==0 and nd==0:break
            if not 0<nh<=65536 or not 0<nd<=MAX_IMAGE_BYTES:raise ValueError('Invalid image IPC size')
            header=json.loads(read_exact(sock,nh));data=read_exact(sock,nd)
            try:
                depth=np.frombuffer(data,dtype=np.uint16).reshape(header['shape'])
                if not cv2.imwrite(header['path'],depth,[cv2.IMWRITE_PNG_COMPRESSION,3]):raise IOError('PNG write failed')
                from hipexo_writeback import submit
                fd=os.open(header['path'],os.O_RDWR)
                try:scheduled=submit(fd)
                finally:os.close(fd)
                response={'async_writeback':scheduled}
            except Exception as exc:response={'error':str(exc)}
            payload=json.dumps(response).encode();sock.sendall(struct.pack('!I',len(payload))+payload)
    finally:sock.close()

if __name__=='__main__':child(int(sys.argv[1]))
