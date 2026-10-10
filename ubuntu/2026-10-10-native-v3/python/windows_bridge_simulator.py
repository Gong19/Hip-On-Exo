"""Protocol reference / network test server. SIMULATED EMG ONLY; no Delsys SDK.

Runs on Windows/Linux Python 3.10+ using only the standard library.
Windows Codex should implement a separate Delsys-backed service against the
same protocol; this simulator is a conformance fixture, not a device adapter.
"""
import argparse
import hmac
import json
import math
import os
from pathlib import Path
import select
import socket
import threading
import time

PROTOCOL='hipexo-emg/1'
SIDS=[57614,57569,57566,57589,57586,57643,56683]


class BridgeSimulator:
    def __init__(self,host='127.0.0.1',port=0,token='',record_dir='emg_bridge_sim_records',
                 offset_ns=0,ready=True):
        self.token=token;self.record_dir=Path(record_dir);self.offset_ns=offset_ns;self.ready=ready
        self.exit=threading.Event();self.peer=None
        self.server=socket.socket();self.server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.server.bind((host,port));self.server.listen(1);self.server.settimeout(.2)
        self.port=self.server.getsockname()[1]
        self.thread=threading.Thread(target=self._serve,daemon=True)

    def start(self):
        self.thread.start();return self

    def close(self):
        self.exit.set()
        if self.peer:
            try:self.peer.shutdown(socket.SHUT_RDWR)
            except OSError:pass
        self.server.close();self.thread.join(3.)

    def _wall(self):
        return time.time_ns()+self.offset_ns

    def _serve(self):
        while not self.exit.is_set():
            try:peer,_=self.server.accept()
            except socket.timeout:continue
            except OSError:break
            self.peer=peer
            try:self._client(peer)
            except (OSError,ValueError,KeyError):pass
            finally:peer.close();self.peer=None

    def _client(self,peer):
        peer.settimeout(1.)
        buffer=bytearray();authorized=False;run=None;record=None;seq=0;sample=0
        next_data=0.;last_request=time.monotonic()
        def send(message):
            peer.sendall((json.dumps(message,allow_nan=False)+'\n').encode())
        def log(kind,**data):
            if record:
                record.write(json.dumps({'kind':kind,'windows_wall_ns':self._wall(),
                                         'windows_mono_ns':time.perf_counter_ns(),**data})+'\n')
        try:
            while not self.exit.is_set():
                if time.monotonic()-last_request>15:
                    log('watchdog_stop');break
                readable,_,_=select.select([peer],[],[],.01)
                if readable:
                    chunk=peer.recv(65536)
                    if not chunk:break
                    buffer.extend(chunk)
                    if len(buffer)>4*1024*1024:raise ValueError('oversized input')
                while b'\n' in buffer:
                    raw,_,remainder=buffer.partition(b'\n');buffer=bytearray(remainder)
                    request=json.loads(raw);t2=self._wall();last_request=time.monotonic()
                    kind=request['type'];rid=request['id']
                    def ack(**fields):send({'type':kind+'_ack','id':rid,**fields})
                    def error(message):send({'type':'error','id':rid,'message':message})
                    if kind=='hello':
                        if request.get('protocol')!=PROTOCOL or not hmac.compare_digest(str(request.get('token','')),self.token):
                            error('Protocol/token mismatch');return
                        authorized=True
                        ack(protocol=PROTOCOL,ready=self.ready,simulated=True,
                            channels=[{'sid':sid,'present':True,'is_rms':True,'mode':'SIMULATED RMS',
                                       'sample_rate':1000.,'battery':None} for sid in SIDS])
                    elif not authorized:
                        error('HELLO required');return
                    elif kind=='ping':
                        ack(t2_ns=t2,t3_ns=self._wall(),ready=self.ready)
                    elif kind=='start':
                        if not self.ready:error('Windows not armed');continue
                        if run:error('Acquisition already active');continue
                        run=request['run_id'];seq=sample=0
                        # Never use peer-supplied run ID as a filesystem path.
                        import uuid
                        self.record_dir.mkdir(parents=True,exist_ok=True)
                        path=self.record_dir/f'simulated_emg_{uuid.uuid4().hex}.jsonl'
                        record=path.open('x',encoding='utf-8')
                        anchor={'device_time_s':0.,'windows_wall_ns':self._wall(),
                                'windows_mono_ns':time.perf_counter_ns(),
                                'method':'simulator_clock','uncertainty_ns':0}
                        log('start',run_id=run,clock_anchor=anchor,sync_id=request['sync_id'],simulated=True)
                        record.flush()
                        ack(run_id=run,recording=True,record_path=str(path),clock_anchor=anchor)
                        next_data=time.monotonic()+.02
                    elif kind=='stop':
                        if not run or request.get('run_id')!=run:error('Unknown run');continue
                        log('stop',run_id=run,last_seq=seq-1);record.flush();record.close();record=None
                        ack(run_id=run,last_seq=seq-1);run=None
                    else:error('Unknown command')
                if run and time.monotonic()>=next_data:
                    message={'type':'data','run_id':run,'seq':seq,'windows_send_wall_ns':self._wall(),
                             'windows_send_mono_ns':time.perf_counter_ns(),
                             'channels':[{'slot':i,'device_time_s':[j/1000 for j in range(sample,sample+20)],
                                          'values_v':[.001+.0002*math.sin(j*.01+i) for j in range(sample,sample+20)]}
                                         for i in range(7)]}
                    log('data',message=message);send(message)
                    seq+=1;sample+=20;next_data+=.02
        finally:
            if record:
                log('connection_closed',run_id=run,last_seq=seq-1)
                record.flush();record.close()


def main():
    parser=argparse.ArgumentParser(description='SIMULATED EMG bridge — no Delsys acquisition')
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--record-dir',default='emg_bridge_sim_records')
    parser.add_argument('--clock-offset-ms',type=float,default=0)
    args=parser.parse_args()
    token=os.environ.get('HIPEXO_EMG_TOKEN','')
    if args.host not in ('127.0.0.1','localhost','::1') and not token:
        parser.error('Set HIPEXO_EMG_TOKEN before listening on the LAN')
    server=BridgeSimulator(args.host,args.port,token,args.record_dir,round(args.clock_offset_ms*1e6)).start()
    print(f'SIMULATED EMG ONLY — READY on {args.host}:{server.port}',flush=True)
    try:
        while server.thread.is_alive():time.sleep(.5)
    except KeyboardInterrupt:pass
    finally:server.close()

if __name__=='__main__':main()
