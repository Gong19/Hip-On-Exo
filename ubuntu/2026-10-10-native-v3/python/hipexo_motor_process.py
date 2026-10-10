"""Bounded IPC client for the zero-output native motor monitor."""
import os,socket,struct,subprocess,threading
from pathlib import Path
ROW=struct.Struct('<QQfffi');STATS=struct.Struct('<6Q')
KEYS=('sent','valid','bad_bytes','skipped_deadlines','tx_errors','queue_peak_bytes')
class MotorCapture:
    def __init__(self,port,motor_id):
        self.sock,other=socket.socketpair();self.sock.settimeout(.05);self.pending=bytearray();self.stats={};self._stop_lock=threading.Lock();self.stopping=False;self.tuning_error=None;self.cpu_affinity=None
        binary=Path(__file__).with_name('hipexo_motor_capture')
        try:
            self.process=subprocess.Popen([str(binary),str(other.fileno()),port,str(motor_id)],pass_fds=(other.fileno(),),stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
            if os.environ.get('HIPEXO_DISABLE_TUNING')!='1':
                try:
                    cpus=sorted(os.sched_getaffinity(0))
                    if len(cpus)>=4:
                        self.cpu_affinity=cpus[-2+(motor_id%2)]
                        os.sched_setaffinity(self.process.pid,{self.cpu_affinity})
                except OSError as exc:self.tuning_error=str(exc)
                try:subprocess.run(['sudo','-n','chrt','-f','-p','10',str(self.process.pid)],check=True,capture_output=True,timeout=3)
                except Exception as exc:self.tuning_error=str(exc)
            self.sock.sendall(b'G')
        except Exception:
            self.sock.close()
            if hasattr(self,'process'):self.process.terminate();self.process.wait()
            raise
        finally:other.close()
    def request_stop(self):
        with self._stop_lock:
            if self.stopping:return
            self.stopping=True
            try:self.sock.sendall(b'S')
            except OSError:pass
    def stop(self):
        self.request_stop()
        try:self.process.wait(timeout=1.5)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:self.process.wait(timeout=.5)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
    def receive(self):
        try:data=self.sock.recv(65536)
        except socket.timeout:return []
        if not data:return None
        self.pending.extend(data);rows=[]
        while len(self.pending)>=4:
            n=struct.unpack_from('<I',self.pending)[0]
            if n<STATS.size or n>65536 or (n-STATS.size)%ROW.size:raise ValueError('Invalid motor IPC packet')
            if len(self.pending)<4+n:break
            self.stats=dict(zip(KEYS,STATS.unpack_from(self.pending,4)))
            rows.extend(ROW.iter_unpack(self.pending[4+STATS.size:4+n]));del self.pending[:4+n]
        return rows
    def close(self):
        self.stop();self.sock.close();error=self.process.stderr.read(4096).decode(errors='replace');self.process.stderr.close()
        if self.process.returncode:raise RuntimeError(error or f'Motor helper exited {self.process.returncode}')
        if self.pending:raise RuntimeError('Truncated final motor IPC packet')
