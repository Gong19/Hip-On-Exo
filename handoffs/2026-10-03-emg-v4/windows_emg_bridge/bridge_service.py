"""hipexo-emg/1 service: one SDK actor, bounded I/O, ordered JSONL transport."""
from concurrent.futures import Future
from dataclasses import dataclass
import copy
import hmac
import ipaddress
import json
import math
import os
from pathlib import Path
import queue
import socket
import threading
import time
import uuid

from bridge_sources import Clock, default_channels

PROTOCOL = 'hipexo-emg/1'
BUILD = '20261003.4'
MAX_LINE = 4*1024*1024


def safe_json(value):
    """Retain invalid source values as tagged evidence; never emit invalid JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return {'invalid_numeric_value': repr(value)}
    if isinstance(value, dict): return {k:safe_json(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)): return [safe_json(v) for v in value]
    return value


class Peer:
    def __init__(self, sock, address):
        self.sock, self.address = sock, address
        self.dead = threading.Event()
        self.authorized = False
        self.heartbeat = time.monotonic()
        self.generation = None

    def close(self):
        self.dead.set()
        try: self.sock.shutdown(socket.SHUT_RDWR)
        except OSError: pass
        try: self.sock.close()
        except OSError: pass


class OrderedIO:
    """Only this thread writes sockets or run files. It never invokes the SDK."""
    def __init__(self, bridge, capacity):
        self.bridge = bridge
        self.q = queue.Queue(capacity)
        self.file = None
        self.path = None
        self.failure = None
        self.last_flush = 0.
        self.peak = 0
        self.closed_stamps = None
        self.thread = threading.Thread(target=self._loop, name='EMG-file-network', daemon=True)

    def put(self, kind, wait=False, **fields):
        if self.failure: raise RuntimeError(self.failure)
        future = Future()
        self.q.put((kind,fields,future), timeout=2 if wait else 0)
        self.peak = max(self.peak,self.q.qsize())
        return future

    def log(self, kind, **fields):
        if self.file:
            record={'kind':kind, 'windows_wall_ns':self.bridge.clock.wall_ns(),
                    'windows_mono_ns':time.perf_counter_ns(), **fields}
            self.file.write(json.dumps(safe_json(record),allow_nan=False,separators=(',',':'))+'\n')
            if time.monotonic()-self.last_flush >= 1:
                self.file.flush(); self.last_flush=time.monotonic()

    def send(self, peer, message):
        if peer is None or peer.dead.is_set(): return False
        if message['type'] == 'ping_ack':
            message['t3_ns'] = self.bridge.clock.wall_ns()
        if message['type'] == 'data':
            message['windows_send_wall_ns'] = self.bridge.clock.wall_ns()
            message['windows_send_mono_ns'] = time.perf_counter_ns()
        wire = json.dumps(message,allow_nan=False,separators=(',',':')).encode('utf-8')
        if len(wire) > MAX_LINE: raise ValueError('Outgoing protocol message exceeds 4 MiB')
        try:
            peer.sock.sendall(wire+b'\n')
            return True
        except OSError:
            peer.close()
            return False

    def _handle(self, kind, f):
        if kind == 'open':
            if self.file: raise RuntimeError('Previous recording still open')
            directory=Path(f['directory']); directory.mkdir(parents=True,exist_ok=True)
            self.path=directory/('emg_'+uuid.uuid4().hex+'.jsonl')
            self.file=self.path.open('x',encoding='utf-8')
            self.log('metadata',**f['metadata'])
            self.file.flush()
            return str(self.path)
        if kind == 'event':
            self.log(f.pop('event'),**f)
        elif kind == 'raw_poll':
            self.log('raw_poll',run_id=f['run_id'],poll=f['poll'])
            # Flush raw evidence before sending data derived from it.
            self.file.flush()
        elif kind == 'start_ack':
            self.log('start',reply=f['message'])
            self.file.flush(); os.fsync(self.file.fileno())
            return self.send(f['peer'],f['message'])
        elif kind == 'data':
            message=f['message']
            sent=self.send(f['peer'],message)
            self.log('data',message=message,transmitted=sent)
            return sent
        elif kind == 'send':
            sent=self.send(f['peer'],f['message'])
            if f['message']['type'] == 'ping_ack':
                self.log('ping_reply',reply=f['message'],transmitted=sent)
            return sent
        elif kind == 'end':
            self.log('stop',**f)
            if self.file:
                self.file.flush(); os.fsync(self.file.fileno())
                self.file.close(); self.file=None
                self.closed_stamps=self.bridge.clock.stamps()
        elif kind == 'stop_ack':
            before=self.bridge.clock.stamps()
            sent=self.send(f['peer'],f['message'])
            after=self.bridge.clock.stamps()
            receipt={'run_id':f['message']['run_id'],'last_seq':f['message']['last_seq'],
                     'raw_file_closed':self.closed_stamps,'ack_send_begin':before,
                     'ack_send_returned':after,'socket_send_success':sent,
                     'remote_application_received_ack':'unknown; socket send is not remote acknowledgement',
                     'stop_timing':f['stop_timing']}
            with self.path.with_suffix('.stop_receipt.json').open('x',encoding='utf-8') as stream:
                json.dump(receipt,stream,indent=2);stream.flush();os.fsync(stream.fileno())
            return sent
        else:
            raise ValueError('Unknown I/O operation')

    def _loop(self):
        while True:
            item=self.q.get()
            if item is None:
                self.q.task_done();break
            kind,fields,future=item
            try:
                if self.failure: raise RuntimeError(self.failure)
                future.set_result(self._handle(kind,fields))
            except Exception as exc:
                self.failure='Recording/transport failed: '+type(exc).__name__+': '+str(exc)
                future.set_exception(RuntimeError(self.failure))
            finally:
                self.q.task_done()
        if self.file:
            try: self.file.flush();self.file.close()
            except OSError: pass
            self.file=None


class Bridge:
    def __init__(self, source, host='127.0.0.1', port=8765, token='', record_dir='records',
                 clock=None, io_capacity=128, heartbeat_seconds=15., wall_step_ns=20_000_000):
        self.source=source
        self.clock=clock or source.clock
        self.host,self.port,self.token=host,int(port),token
        self.record_dir=str(Path(record_dir).resolve())
        self.heartbeat_seconds=heartbeat_seconds
        self.wall_step_ns=wall_step_ns
        self.commands=queue.Queue(32)
        self.io=OrderedIO(self,io_capacity)
        self.lock=threading.RLock()
        self.done=threading.Event()
        self.peer=None
        self.listener=None
        self.generation=0
        self.instance_id=uuid.uuid4().hex[:12]
        self.prepared=False
        self.armed=False
        self.run=None
        self.failed=False
        self.channels=default_channels()
        self.pending={}
        self.last_times={}
        self.actor=threading.Thread(target=self._actor,name='EMG-SDK-owner',daemon=True)
        self.acceptor=threading.Thread(target=self._accept,name='EMG-accept',daemon=True)
        self._snapshot={'state':'DISARMED','error':'','run_id':None,'packets':0,
                        'samples':[0]*7,'record_path':'','simulated':bool(source.simulated),
                        'last_hello':None,'build':BUILD,'instance_id':self.instance_id}

    def _set(self, **fields):
        with self.lock:self._snapshot.update(fields)

    def snapshot(self):
        with self.lock:
            return {**copy.deepcopy(self._snapshot), 'ready':self.armed and self.prepared and not self.failed,
                    'connected':bool(self.peer and self.peer.authorized and not self.peer.dead.is_set()),
                    'peer':self.peer.address[0] if self.peer else '',
                    'channels':copy.deepcopy(self.channels),'io_queue':self.io.q.qsize(),
                    'io_peak':self.io.peak,'host':self.host,'port':self.port}

    def start(self):
        if not ipaddress.ip_address(self.host).is_loopback and not self.token:
            raise ValueError('A nonempty connection token is required on the LAN')
        self.listener=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        if os.name == 'nt':
            self.listener.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
        else:self.listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        try:
            self.listener.bind((self.host,self.port));self.listener.listen(4);self.listener.settimeout(.2)
        except Exception:
            self.listener.close();self.listener=None;raise
        self.port=self.listener.getsockname()[1]
        self.io.thread.start();self.actor.start();self.acceptor.start()
        return self

    def command(self, kind, peer=None, request=None):
        future=Future()
        self.commands.put_nowait((kind,peer,request or {},future))
        return future

    def prepare(self): return self.command('prepare')
    def arm(self): return self.command('arm')
    def disarm(self): return self.command('disarm')

    def _reply(self, peer, request, **fields):
        self.io.put('send',peer=peer,message={'type':request['type']+'_ack','id':request['id'],**fields})

    def _error(self, peer, request, message):
        try:self.io.put('send',wait=True,peer=peer,message={'type':'error','id':request.get('id',''),'message':message})
        except Exception: peer.close()

    def _accept(self):
        while not self.done.is_set():
            try:sock,address=self.listener.accept()
            except socket.timeout:continue
            except OSError:break
            sock.settimeout(.5);sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            with self.lock:
                if self.peer is not None:
                    sock.close();continue
                peer=Peer(sock,address);self.peer=peer
            threading.Thread(target=self._reader,args=(peer,),name='EMG-control-reader',daemon=True).start()

    def _reader(self, peer):
        buffer=bytearray()
        opened=time.monotonic()
        try:
            while not self.done.is_set() and not peer.dead.is_set():
                if not peer.authorized and time.monotonic()-opened>5:break
                try:chunk=peer.sock.recv(65536)
                except socket.timeout:continue
                if not chunk:break
                buffer.extend(chunk)
                while b'\n' in buffer:
                    raw,_,rest=buffer.partition(b'\n');buffer=bytearray(rest)
                    if len(raw)>MAX_LINE:raise ValueError('Command too large')
                    msg=json.loads(raw,parse_constant=lambda x: (_ for _ in ()).throw(ValueError('Nonfinite JSON')))
                    t2=self.clock.wall_ns()
                    if not isinstance(msg,dict) or not isinstance(msg.get('id'),str) or not 0<len(msg['id'])<=128:
                        raise ValueError('Invalid command envelope')
                    kind=msg.get('type')
                    if kind=='hello':
                        token=msg.get('token','')
                        failure=None
                        if peer.authorized:failure=('ALREADY_AUTHENTICATED','HELLO already completed')
                        elif msg.get('protocol')!=PROTOCOL:failure=('PROTOCOL_MISMATCH','Protocol mismatch; expected '+PROTOCOL)
                        elif not isinstance(token,str) or len(token)>1024:failure=('TOKEN_FORMAT_INVALID','Connection token format invalid')
                        elif not hmac.compare_digest(token.encode(),self.token.encode()):failure=('TOKEN_MISMATCH','Connection token mismatch; copy current Windows token exactly')
                        # Never retain the submitted token, its hash, length or fragments.
                        self._set(last_hello={'code':failure[0] if failure else 'OK',
                                             'peer':peer.address[0],'wall_ns':t2})
                        if failure:
                            self._error(peer,msg,failure[0]+': '+failure[1]);break
                        with self.lock:
                            peer.authorized=True;peer.generation=self.generation
                            snap=self.snapshot()
                        self._reply(peer,msg,protocol=PROTOCOL,ready=snap['ready'],
                                    simulated=bool(self.source.simulated),channels=snap['channels'],
                                    bridge_build=BUILD,bridge_instance=self.instance_id)
                    elif not peer.authorized:
                        self._error(peer,msg,'HELLO required');break
                    elif kind=='ping':
                        peer.heartbeat=time.monotonic()
                        self._reply(peer,msg,t2_ns=t2,ready=self.snapshot()['ready'])
                    elif kind in ('start','stop'):
                        msg['_bridge_received']=self.clock.stamps()
                        self.command(kind,peer,msg)
                    else:
                        self._error(peer,msg,'Unknown command')
                if len(buffer)>MAX_LINE:raise ValueError('Command too large')
        except (OSError,ValueError,TypeError,queue.Full):
            pass
        finally:
            peer.close()

    def _emit(self, wait=False):
        if not self.pending:return
        while any(c['device_time_s'] for c in self.pending.values()):
            channels=[]
            for slot,ch in self.pending.items():
                if ch['device_time_s']:
                    channels.append({'slot':slot,'device_time_s':ch['device_time_s'][:1000],
                                     'values_v':ch['values_v'][:1000]})
            message={'type':'data','run_id':self.run['id'],'seq':self.run['seq'],'channels':channels}
            self.io.put('data',wait=wait,peer=self.run['peer'],message=message)
            for ch in channels:
                count=len(ch['device_time_s']);slot=ch['slot']
                del self.pending[slot]['device_time_s'][:count];del self.pending[slot]['values_v'][:count]
                self.run['counts'][slot]+=count
            self.run['seq']+=1
        self.pending.clear()
        self.run['last_emit']=time.monotonic()
        self._set(packets=self.run['seq'],samples=self.run['counts'][:])

    def _ingest(self, poll, wait=False):
        if poll is None:return
        self.io.put('raw_poll',wait=wait,run_id=self.run['id'],poll=poll)
        seen=set()
        for ch in poll['channels']:
            slot=ch['slot'];t=ch['device_time_s'];v=ch['values_v']
            if type(slot) is not int or not 0<=slot<7 or slot in seen or not self.channels[slot]['present']:
                raise ValueError('Invalid source channel')
            seen.add(slot)
            if len(t)!=len(v) or not all(math.isfinite(x) for x in t+v):
                raise ValueError('Invalid raw SDK values; inspect raw_poll evidence')
            if any(b<=a for a,b in zip(t,t[1:])) or (t and slot in self.last_times and t[0]<=self.last_times[slot]):
                raise ValueError('Device time repeated/moved backwards; reconnect and start new run')
            if not t:continue
            self.last_times[slot]=t[-1]
            target=self.pending.setdefault(slot,{'device_time_s':[],'values_v':[]})
            target['device_time_s'].extend(t);target['values_v'].extend(v)
            if len(target['device_time_s'])>100000:
                raise BufferError('Raw sample accumulation limit reached')

    def _begin(self, peer, request):
        if not self.armed or not self.prepared or self.failed:raise ValueError('Windows not armed / ready')
        if self.run:raise ValueError('Acquisition already active')
        if peer is None or peer.dead.is_set() or peer is not self.peer or peer.generation!=self.generation:
            raise ValueError('Reconnect and HELLO after sensor configuration changes')
        for field in ('run_id','sync_id'):
            value=request.get(field)
            if not isinstance(value,str) or not value or len(value)>128:raise ValueError('Invalid '+field)
            uuid.UUID(value)
        if type(request.get('workstation_offset_ns')) is not int:raise ValueError('Invalid clock offset')
        self.pending={};self.last_times={}
        self.run={'id':request['run_id'],'peer':peer,'seq':0,'counts':[0]*7,'last_emit':time.monotonic()}
        self._set(state='STARTING',run_id=request['run_id'],packets=0,samples=[0]*7,error='')
        metadata={'protocol':PROTOCOL,'run_id':request['run_id'],'sync_id':request['sync_id'],
                  'workstation_offset_ns':request['workstation_offset_ns'],'channels':self.channels,
                  'simulated':bool(self.source.simulated),'wall_clock_method':self.clock.method,
                  'timestamp_units':'SDK Item1 assumed seconds; hardware-domain verification pending',
                  'value_units':'V after per-channel SDK Unit conversion; original SDK values retained in raw_poll',
                  'bridge_build':BUILD}
        path=self.io.put('open',directory=self.record_dir,metadata=metadata).result(3)
        self._set(record_path=path)
        self.source.start()
        deadline=time.monotonic()+6
        first=None
        while time.monotonic()<deadline:
            if peer.dead.is_set():raise ConnectionError('Client disconnected during start')
            first=self.source.poll()
            if first and any(c['device_time_s'] for c in first['channels']):break
            if first:self.io.put('raw_poll',run_id=self.run['id'],poll=first)
            time.sleep(.002)
        else:raise TimeoutError('Delsys started but no raw samples arrived within 6 seconds')
        latest=max(max(c['device_time_s']) for c in first['channels'] if c['device_time_s'])
        anchor={'device_time_s':latest,'windows_wall_ns':first['poll_received']['wall_ns'],
                'windows_mono_ns':first['poll_received']['mono_ns'],
                'method':'first_poll_latest_sample','uncertainty_ns':None,
                'clock_read_bracket_ns':first['poll_received']['read_bracket_ns'],
                'device_buffer_delay':'unknown; included in arrival-based anchor',
                'time_domain':'SDK YT common-domain assumption; not hardware-trigger calibrated'}
        if not math.isfinite(latest):raise ValueError('Invalid initial device timestamp')
        self.run['wall_minus_mono']=anchor['windows_wall_ns']-anchor['windows_mono_ns']
        ack={'type':'start_ack','id':request['id'],'run_id':request['run_id'],
             'recording':True,'record_path':path,'clock_anchor':anchor}
        # The ordered writer guarantees ACK before any DATA.
        self.io.put('start_ack',peer=peer,message=ack)
        self._ingest(first)
        self._emit()
        self._set(state='STREAMING')

    def _end(self, reason='normal_stop', request=None, error=None):
        if not self.run:return
        run=self.run
        self._set(state='STOPPING')
        fault=(str(error) or 'Unspecified acquisition failure') if error is not None else None
        stop_timing={'request_received':(request or {}).get('_bridge_received'),
                     'actor_stop_begin':self.clock.stamps()}
        try:
            self.io.put('event',wait=True,event='stop_begin',run_id=run['id'],
                        stop_timing=copy.deepcopy(stop_timing))
            stop_timing['sdk_stop_begin']=self.clock.stamps()
            self.source.stop()
            stop_timing['sdk_stop_returned']=self.clock.stamps()
            stop_timing['sdk_stop_duration_ns']=(stop_timing['sdk_stop_returned']['mono_ns']-
                                                 stop_timing['sdk_stop_begin']['mono_ns'])
            quiet=time.monotonic();deadline=quiet+1.2
            while time.monotonic()<deadline:
                poll=self.source.poll()
                if poll and poll['channels']:
                    self._ingest(poll,wait=True);quiet=time.monotonic()
                elif time.monotonic()-quiet>=.06:break
                time.sleep(.002)
            else:raise TimeoutError('SDK tail did not become empty; recording incomplete')
            self._emit(wait=True)
            stop_timing['tail_drain_returned']=self.clock.stamps()
        except Exception as exc:
            fault=fault or str(exc)
            # Best effort stop is still serialized on the SDK owner thread.
            try:self.source.stop()
            except Exception as stop_exc:fault+='; '+str(stop_exc)
        try:
            self.io.put('end',wait=True,run_id=run['id'],last_seq=run['seq']-1,
                        channel_sample_counts=run['counts'],recording_complete=fault is None,
                        reason=reason,error=fault,io_queue_peak=self.io.peak,
                        stop_timing=stop_timing).result(3)
        except Exception as exc:fault=fault or str(exc)
        self.run=None;self.pending={}
        if fault or reason!='normal_stop':
            self.armed=False
            self.failed=bool(fault)
            self._set(state='ERROR' if fault else 'DISARMED',error=fault or reason)
            if run['peer'] and not run['peer'].dead.is_set():
                self._error(run['peer'],request or {},fault or reason)
        else:
            self._set(state='ARMED',error='')
            if request:
                self.io.put('stop_ack',peer=run['peer'],stop_timing=stop_timing,
                            message={'type':'stop_ack','id':request['id'],
                                     'run_id':run['id'],'last_seq':run['seq']-1})

    def _execute(self, kind, peer, request):
        if kind=='prepare':
            if self.run:raise ValueError('Stop acquisition before configuring sensors')
            self.armed=False;self.prepared=False;self.failed=False
            if self.peer:self.peer.close()
            self._set(state='SCANNING',error='')
            channels=self.source.prepare()
            with self.lock:self.channels=copy.deepcopy(channels);self.generation+=1
            self.prepared=True
            self._set(state='DISARMED',error='')
            return channels
        if kind=='arm':
            if not self.prepared or self.failed or self.io.failure:
                raise ValueError('Connect/scan successfully before arming; restart on a disk error')
            if self.run:raise ValueError('Already streaming')
            self.armed=True;self._set(state='ARMED',error='');return True
        if kind=='disarm':
            self.armed=False
            if self.run:self._end('local_stop')
            if self.peer:self.peer.close()
            self._set(state='DISARMED');return True
        if kind=='start':return self._begin(peer,request)
        if kind=='stop':
            if not self.run or peer is not self.run['peer'] or request.get('run_id')!=self.run['id']:
                raise ValueError('Unknown run ID')
            return self._end(request=request)
        if kind=='shutdown':
            self.armed=False
            if self.run:self._end('service_shutdown')
            self.source.close();self.done.set();return True
        raise ValueError('Unknown local command')

    def _actor(self):
        try:
            while not self.done.is_set():
                try:item=self.commands.get(timeout=.002 if self.run else .05)
                except queue.Empty:item=None
                if item:
                    kind,peer,request,future=item
                    try:future.set_result(self._execute(kind,peer,request))
                    except Exception as exc:
                        if self.run and kind=='start':self._end('start_failure',error=str(exc))
                        if kind in ('prepare','arm'):
                            self.armed=False;self._set(state='ERROR',error=str(exc))
                        if peer:self._error(peer,request,str(exc))
                        future.set_exception(exc)
                    finally:self.commands.task_done()
                peer=self.peer
                if peer and (peer.dead.is_set() or time.monotonic()-peer.heartbeat>self.heartbeat_seconds):
                    reason='client_disconnected' if peer.dead.is_set() else 'heartbeat_timeout'
                    peer.close()
                    if self.run:self._end(reason,error=reason)
                    with self.lock:
                        if self.peer is peer:self.peer=None
                if self.run:
                    try:
                        if self.io.failure:raise RuntimeError(self.io.failure)
                        stamps=self.clock.stamps()
                        if abs(stamps['wall_ns']-stamps['mono_ns']-self.run['wall_minus_mono'])>self.wall_step_ns:
                            raise RuntimeError('Windows UTC changed relative to monotonic clock; resynchronize')
                        poll=self.source.poll()
                        if poll is not None:
                            try:self._ingest(poll)
                            except queue.Full:
                                # Preserve the one poll already removed from the SDK, then end.
                                self._ingest(poll,wait=True)
                                raise BufferError('Bounded I/O backlog reached; run stopped')
                        if time.monotonic()-self.run['last_emit']>=.02:self._emit()
                    except Exception as exc:
                        self._end('acquisition_fault',error=type(exc).__name__+': '+(str(exc) or 'bounded queue cannot accept more data'))
        finally:
            self.done.set()
            if self.peer:self.peer.close()
            if self.listener:
                try:self.listener.close()
                except OSError:pass

    def shutdown(self, timeout=12):
        if not self.done.is_set():self.command('shutdown').result(timeout)
        self.actor.join(timeout)
        if self.actor.is_alive():raise TimeoutError('SDK owner did not exit; do not start another device owner')
        self.acceptor.join(2)
        self.io.q.put(None,timeout=2);self.io.thread.join(5)
        if self.io.thread.is_alive():raise TimeoutError('File writer did not exit')
