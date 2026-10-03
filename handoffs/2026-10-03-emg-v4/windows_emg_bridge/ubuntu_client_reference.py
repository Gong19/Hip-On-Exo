"""HiPExo EMG bridge v1 client. No Delsys/.NET dependency on this computer."""
from collections import deque
import json
import math
from pathlib import Path
import socket
import time
import uuid

PROTOCOL = 'hipexo-emg/1'
MAX_LINE = 4 * 1024 * 1024


def estimate_clock(t1, t2, t3, t4, elapsed_ns):
    """Offset = Windows UTC - workstation UTC; retain all raw clock evidence."""
    if t3 < t2 or elapsed_ns < 0 or abs((t4-t1)-elapsed_ns) > 5_000_000:
        raise ValueError('Clock jumped during synchronization; retry')
    delay = elapsed_ns - (t3-t2)
    if delay < -100_000:
        raise ValueError('Invalid remote clock response')
    return {'offset_ns': ((t2-t1)+(t3-t4))//2, 'rtt_ns': max(0,delay),
            't1_ns':t1, 't2_ns':t2, 't3_ns':t3, 't4_ns':t4}


class RemoteWindowsSource:
    def __init__(self, config, cancel, export_dir):
        self.config, self.cancel = config, cancel
        self.export_dir = Path(export_dir)
        self.sock = None
        self.buffer = bytearray()
        self.pending = deque()
        self.journal = None
        self.sync = None
        self.anchor = None
        self.run_id = None
        self.expected_seq = 0
        self.started = False
        self.last_health = 0.
        self.last_flush = 0.
        self.status = {'connected':False,'ready':False,'synced':False,'streaming':False}

    def _log(self, kind, **fields):
        if self.journal:
            event={'kind':kind,'local_wall_ns':time.time_ns(),
                   'local_mono_ns':time.perf_counter_ns(), **fields}
            self.journal.write(json.dumps(event,allow_nan=False,separators=(',',':'))+'\n')
            if time.monotonic()-self.last_flush > 1:
                self.journal.flush(); self.last_flush=time.monotonic()

    def _send(self, message):
        self.sock.sendall((json.dumps(message,allow_nan=False)+'\n').encode())

    def _receive(self, deadline, cancellable=True):
        while True:
            if cancellable and self.cancel.is_set():
                raise InterruptedError('EMG operation cancelled')
            end=self.buffer.find(b'\n')
            if end >= 0:
                if end > MAX_LINE:
                    raise ValueError('Oversized bridge message')
                raw=bytes(self.buffer[:end]); del self.buffer[:end+1]
                message=json.loads(raw,parse_constant=lambda x: (_ for _ in ()).throw(ValueError('Nonfinite JSON')))
                if not isinstance(message,dict):
                    raise ValueError('Bridge message must be an object')
                return message
            if len(self.buffer)>MAX_LINE:
                raise ValueError('Oversized bridge message')
            if time.monotonic() >= deadline:
                raise TimeoutError('Windows bridge response timed out')
            try:
                chunk=self.sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError('Windows bridge disconnected')
            self.buffer.extend(chunk)

    def _queue_data(self, message):
        if not self.started or message.get('run_id') != self.run_id:
            raise ValueError('Unexpected EMG run ID')
        seq=message.get('seq')
        if seq != self.expected_seq:
            raise ValueError(f'EMG packet sequence gap: expected {self.expected_seq}, got {seq}')
        if len(self.pending)>=256:
            raise BufferError('EMG receiver backlog; stop instead of silently dropping data')
        self.expected_seq += 1
        self._log('data',message=message)
        self.pending.append(message)

    def _rpc(self, command, timeout=3., cancellable=True, **payload):
        request_id=uuid.uuid4().hex
        t1=time.time_ns(); m1=time.perf_counter_ns()
        self._send({'type':command,'id':request_id,**payload})
        deadline=time.monotonic()+timeout
        while True:
            message=self._receive(deadline,cancellable)
            t4=time.time_ns(); m4=time.perf_counter_ns()
            if message.get('type')=='data':
                self._queue_data(message)
                continue
            if message.get('id') != request_id:
                raise ValueError('Unexpected bridge response ID')
            if message.get('type')=='error':
                raise RuntimeError('Windows: '+str(message.get('message','unknown error')))
            if message.get('type') != command+'_ack':
                raise ValueError('Unexpected bridge response type')
            return message,(t1,m1,t4,m4)

    def connect(self):
        host=self.config.get('remote_host','').strip()
        if not host:
            raise ValueError('Set Windows IP / hostname in EMG Settings')
        if host not in ('127.0.0.1','localhost','::1') and not self.config.get('remote_token'):
            raise ValueError('Set the same connection token on both computers')
        self.sock=socket.create_connection((host,int(self.config.get('remote_port',8765))),timeout=2.)
        self.sock.settimeout(.1)
        self.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        reply,_=self._rpc('hello',protocol=PROTOCOL,token=self.config.get('remote_token',''))
        if reply.get('protocol') != PROTOCOL:
            raise ValueError('Incompatible Windows bridge protocol')
        channels=reply.get('channels',[])
        if len(channels)!=7:
            raise ValueError('Windows must describe seven fixed logical channels')
        for i,ch in enumerate(channels):
            if ch.get('sid') != self.config['sensor_ids'][i]:
                raise ValueError('Windows sensor order/IDs do not match workstation settings')
            if not isinstance(ch.get('present'),bool) or not isinstance(ch.get('is_rms'),bool):
                raise ValueError('Invalid channel availability/type')
            if not isinstance(ch.get('mode'),str) or not math.isfinite(float(ch.get('sample_rate',0))) or float(ch.get('sample_rate',0))<=0:
                raise ValueError('Invalid channel mode/sample rate')
        self.channels=channels
        self.export_dir.mkdir(parents=True,exist_ok=True)
        path=self.export_dir/f'emg_remote_{uuid.uuid4().hex}.jsonl'
        self.journal=path.open('x',encoding='utf-8')
        self.status.update(connected=True,ready=reply.get('ready') is True,journal=str(path),
                           simulated=reply.get('simulated') is True)
        self._log('hello',reply=reply,protocol=PROTOCOL)
        self.last_health=time.monotonic()
        return channels

    def _probe(self):
        reply,stamps=self._rpc('ping')
        t1,m1,t4,m4=stamps
        result=estimate_clock(t1,int(reply['t2_ns']),int(reply['t3_ns']),t4,m4-m1)
        self.status['ready']=reply.get('ready') is True
        self._log('clock_probe',**result)
        self.last_health=time.monotonic()
        return result

    def idle(self):
        if time.monotonic()-self.last_health>=1.:
            self._probe()

    def synchronize(self):
        if self.started:
            raise ValueError('Stop EMG before resynchronizing')
        probes=[self._probe() for _ in range(9)]
        if not self.status['ready']:
            raise RuntimeError('Windows bridge is connected but not armed / ready')
        best=min(probes,key=lambda x:x['rtt_ns'])
        if best['rtt_ns']>100_000_000:
            raise RuntimeError('Synchronization RTT exceeds 100 ms; check network and retry')
        self.sync={**best,'sync_id':uuid.uuid4().hex,'local_mono_ns':time.perf_counter_ns(),
                   'local_wall_ns':time.time_ns(),'created_mono':time.monotonic()}
        self.status.update(synced=True,offset_ms=best['offset_ns']/1e6,rtt_ms=best['rtt_ns']/1e6)
        self._log('sync_selected',sync=self.sync)

    def start(self):
        if not self.sync or time.monotonic()-self.sync['created_mono']>300:
            raise RuntimeError('Press Sync & Start to synchronize both computers first')
        self.run_id=uuid.uuid4().hex
        reply,_=self._rpc('start',timeout=10.,cancellable=False,run_id=self.run_id,sync_id=self.sync['sync_id'],
                          workstation_offset_ns=self.sync['offset_ns'])
        self.started=True  # ensure close sends stop if validation below rejects the ACK
        if reply.get('run_id')!=self.run_id or reply.get('recording') is not True:
            raise ValueError('Windows did not confirm local raw recording')
        self.anchor=reply.get('clock_anchor',{})
        for name in ('device_time_s','windows_wall_ns','windows_mono_ns'):
            if name not in self.anchor or not math.isfinite(float(self.anchor[name])):
                raise ValueError('Windows START response lacks a valid clock anchor')
        if not isinstance(self.anchor.get('method'),str):
            raise ValueError('Windows must describe device-to-host timestamp mapping')
        self.expected_seq=0; self.pending.clear()
        self.status.update(streaming=True,run_id=self.run_id,windows_file=reply.get('record_path',''))
        self._log('start',reply=reply,sync=self.sync)

    def mapped_time(self, source_s):
        win_ns=int(self.anchor['windows_wall_ns'])+round((source_s-self.anchor['device_time_s'])*1e9)
        wall_ns=win_ns-self.sync['offset_ns']
        mono_ns=self.sync['local_mono_ns']+wall_ns-self.sync['local_wall_ns']
        return wall_ns,mono_ns,win_ns

    def sample_metadata(self, source_s):
        _,_,win_ns=self.mapped_time(source_s)
        return {'windows_wall_ns_est':win_ns,'sync_offset_ns':self.sync['offset_ns'],
                'sync_rtt_ns':self.sync['rtt_ns'],'sync_id':self.sync['sync_id'],
                'remote_run_id':self.run_id,'remote_simulated':int(self.status.get('simulated',False))}

    def _decode(self,message):
        result={}
        for ch in message.get('channels',[]):
            i=ch.get('slot'); times=ch.get('device_time_s',[]); values=ch.get('values_v',[])
            if type(i) is not int or not 0<=i<7 or i in result or len(times)!=len(values) or len(times)>20000:
                raise ValueError('Malformed EMG channel batch')
            if any(not math.isfinite(float(t)) for t in times) or any(not math.isfinite(float(v)) for v in values):
                raise ValueError('Nonfinite EMG samples')
            if any(b<=a for a,b in zip(times,times[1:])):
                raise ValueError('EMG source times must increase')
            result[i]=(times,values)
        return result

    def poll(self):
        if time.monotonic()-self.last_health>=5.:
            probe=self._probe()
            # Observe drift; never change an active run's time mapping discontinuously.
            self.status['drift_ms']=(probe['offset_ns']-self.sync['offset_ns'])/1e6
        if self.pending:
            return self._decode(self.pending.popleft())
        try:
            message=self._receive(time.monotonic()+.1)
        except TimeoutError:
            return {}
        if message.get('type')=='error':
            raise RuntimeError('Windows: '+str(message.get('message')))
        if message.get('type')!='data':
            raise ValueError('Unexpected streaming message')
        self._queue_data(message)
        return self._decode(self.pending.popleft())

    def stop(self):
        if self.started and self.sock:
            reply,_=self._rpc('stop',cancellable=False,run_id=self.run_id)
            if reply.get('run_id')!=self.run_id or reply.get('last_seq')!=self.expected_seq-1:
                raise ValueError('Windows STOP count does not match received packets')
            self._log('stop',reply=reply)
        self.started=False
        self.status['streaming']=False
        if self.journal:
            self.journal.flush()

    def drain_pending(self):
        while self.pending:
            yield self._decode(self.pending.popleft())

    def close(self):
        try:
            if self.started:
                self.stop()
        finally:
            if self.sock:
                self.sock.close();self.sock=None
            if self.journal:
                self.journal.close();self.journal=None
            self.status.update(connected=False,ready=False,synced=False,streaming=False)
