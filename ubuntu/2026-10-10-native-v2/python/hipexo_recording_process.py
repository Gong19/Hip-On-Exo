"""Cycle encoding/writing in an isolated process so CSV formatting cannot stall I/O."""
import json,os,pickle,queue,socket,struct,subprocess,sys,threading,time,select
from pathlib import Path
from hipexo_pipeline import CycleRecorder

MAX_IPC=64*1024*1024

def send(sock,item):
    data=pickle.dumps(item,protocol=5)
    if len(data)>MAX_IPC:raise ValueError('IPC packet exceeds limit')
    sock.sendall(struct.pack('!I',len(data))+data)

def receive(sock):
    def read(n):
        result=bytearray()
        while len(result)<n:
            data=sock.recv(n-len(result))
            if not data:raise EOFError('Recording process disconnected')
            result.extend(data)
        return result
    n=struct.unpack('!I',read(4))[0]
    if n>MAX_IPC:raise ValueError('IPC packet exceeds limit')
    # Only an inherited private socketpair is accepted, never a network socket.
    return pickle.loads(read(n))

class ProcessCycleRecorder:
    def __init__(self,path,on_error=None,on_cycle=None,capacity=2048):
        self.path=str(path);self.on_error=on_error;self.on_cycle=on_cycle;self.error=None;self.stats={};self.metrics={}
        self.start_mono_ns=time.perf_counter_ns();self.start_wall_ns=time.time_ns();self.stop_ns=None
        self._stop=threading.Event();self._done=threading.Event();self._queue=queue.Queue(capacity)
        self._budget=0;self._lock=threading.Lock();self._parent_metrics=dict(parent_max_queue_depth=0,parent_max_queued_bytes=0)
        self._sock,other=socket.socketpair()
        self._process=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--child',str(other.fileno())],
            pass_fds=(other.fileno(),),stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        other.close()
        try:
            self._sock.settimeout(10)
            send(self._sock,dict(path=self.path,start_mono_ns=self.start_mono_ns,start_wall_ns=self.start_wall_ns,publish_cycles=on_cycle is not None))
            result=receive(self._sock)
            if result[0]!='ready':raise RuntimeError(str(result))
            self._sock.settimeout(None)
        except Exception:
            self._process.kill();self._process.wait();self._sock.close();raise
        self._reader=threading.Thread(target=self._read,name='cycle-ipc-reader',daemon=True);self._reader.start()
        self._thread=threading.Thread(target=self._write,name='cycle-ipc-writer',daemon=True);self._thread.start()

    def _fail(self,message):
        if self.error is None:
            self.error=message
            if self.stop_ns is None:self.stop_ns=time.perf_counter_ns()
            self._stop.set()
            if self.on_error:
                try:self.on_error(message)
                except Exception:pass  # Keep failure state and cleanup even if reporting fails.

    def _enqueue(self,item,estimate):
        if self.error or self._stop.is_set():return False
        with self._lock:
            if self._budget+estimate>MAX_IPC:self._fail('Parent recording queue exceeds 64 MiB');return False
            self._budget+=estimate
            self._parent_metrics["parent_max_queued_bytes"]=max(self._parent_metrics["parent_max_queued_bytes"],self._budget)
        try:
            self._queue.put_nowait((item,estimate,time.perf_counter_ns()))
            self._parent_metrics['parent_max_queue_depth']=max(self._parent_metrics['parent_max_queue_depth'],self._queue.qsize())
            return True
        except queue.Full:
            with self._lock:self._budget-=estimate
            self._fail('Parent recording queue full; incomplete');return False

    def enqueue_frames(self,stream,times,frames,monos,ids):
        if not frames:return True
        if not len(times)==len(frames)==len(monos)==len(ids):raise ValueError('Frame/timestamp lengths differ')
        return self._enqueue(('frames',stream,list(times),[dict(f) for f in frames],list(monos),list(ids)),len(frames)*(len(frames[0])*48+128))

    def enqueue_image(self,depth,timing):return self._enqueue(('image',depth.copy(),dict(timing)),depth.nbytes+2048)

    def _read(self):
        try:
            while True:
                kind,payload=receive(self._sock)
                if kind=='cycle':
                    if self.on_cycle:self.on_cycle(payload)
                elif kind=='error':self._fail(payload)
                elif kind=='done':
                    self.stats=payload['stats'];self.metrics=payload['metrics']
                    if payload['error']:self._fail(payload['error'])
                    break
        except Exception as exc:self._fail(str(exc))
        finally:self._done.set()

    def _write(self):
        try:
            while not self._stop.is_set() or not self._queue.empty():
                if self._done.is_set():return
                try:item,size,accepted_at=self._queue.get(timeout=.02)
                except queue.Empty:continue
                try:send(self._sock,('input',accepted_at,item))
                finally:
                    with self._lock:self._budget-=size
            send(self._sock,('stop',self.stop_ns,self.error,dict(self._parent_metrics)))
            self._done.wait()
        except Exception as exc:self._fail(str(exc));self._done.set()
        finally:
            if self._done.is_set():
                try:self._sock.shutdown(socket.SHUT_RDWR)
                except OSError:pass
                self._sock.close()
                try:self._process.wait(timeout=1)
                except subprocess.TimeoutExpired:self._process.terminate()

    def stop(self,timeout=15):
        if self.stop_ns is None:self.stop_ns=time.perf_counter_ns()
        self._stop.set();self._thread.join(timeout)
        if self._thread.is_alive():self._fail('Cycle process still draining; keep interface open')
        return self.error is None

def child(fd):
    sock=socket.socket(fileno=fd);lock=threading.Lock();recorder=None;profile=None
    if os.environ.get('HIPEXO_PROFILE_PATH'):
        import cProfile
        profile=cProfile.Profile();profile.enable()
    def output(kind,value):
        with lock:send(sock,(kind,value))
    try:
        config=receive(sock)
        publish=config.pop('publish_cycles',True)
        recorder=CycleRecorder(**config,inline=True,on_error=lambda e:output('error',e),on_cycle=(lambda p:output('cycle',p)) if publish else None)
        recorder.metrics['writer_mode']='single-consumer-ipc'
        output('ready',None)
        # One Python consumer owns decoding, numeric acceptance and cycle
        # writing. Removing a second GIL-contending queue/clone stage leaves
        # the original bounded parent queue and socket as backpressure.
        while True:
            if not select.select([sock],[],[],.02)[0]:
                recorder.emit_ready(time.perf_counter_ns());continue
            item=receive(sock);accepted_at=time.perf_counter_ns()
            if item[0]=='input':_,accepted_at,item=item
            if item[0]=='frames':recorder.accept_owned(tuple(item[1:]))
            elif item[0]=='image':recorder.accept_owned(('__image',item[1],item[2]))
            elif item[0]=='stop':
                recorder.stop_ns=item[1]
                if len(item)>2 and item[2]:recorder._fail('Parent recorder: '+item[2])
                if len(item)>3:recorder.metrics.update(item[3])
                recorder.stop()
                output('done',dict(stats=recorder.stats,metrics=recorder.metrics,error=recorder.error));break
            # Use original parent enqueue time, not delayed receipt, so a
            # temporary backlog does not prematurely finalize grid cycles.
            recorder.emit_ready(accepted_at)
    except Exception as exc:
        try:output('error',str(exc))
        except Exception:pass
    finally:
        if recorder is not None and not recorder._closed:
            recorder._fail('Recording IPC ended before orderly Stop')
            recorder.stop()
        sock.close()
        if profile:
            profile.disable();profile.dump_stats(os.environ['HIPEXO_PROFILE_PATH'])

if __name__=='__main__':
    if len(sys.argv)==3 and sys.argv[1]=='--child':child(int(sys.argv[2]))
