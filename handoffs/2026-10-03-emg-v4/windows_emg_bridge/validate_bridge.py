"""Explicit loopback verification. --real uses the physically attached Delsys."""
import argparse
import itertools
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import ubuntu_client_reference as remote
from bridge_service import Bridge
from bridge_sources import Clock, DelsysSource, SyntheticSource, SIDS


def messages(path):
    with Path(path).open(encoding='utf-8') as f:
        for line in f:
            e=json.loads(line)
            if e['kind']=='data':yield e['message']


def compare(windows, receiver):
    counts=[0]*7;first=[None]*7;last=[None]*7;dt_min=[None]*7;dt_max=[None]*7
    packets=0;run=None
    for a,b in itertools.zip_longest(messages(windows),messages(receiver)):
        if a!=b:raise AssertionError('Windows and receiver packet contents differ')
        if a['seq']!=packets:raise AssertionError('Sequence gap')
        if run is not None and a['run_id']!=run:raise AssertionError('Run changed')
        run=a['run_id'];packets+=1
        for ch in a['channels']:
            i=ch['slot']
            for t in ch['device_time_s']:
                if last[i] is not None:
                    d=t-last[i]
                    if d<=0:raise AssertionError('Source time not strictly increasing')
                    dt_min[i]=d if dt_min[i] is None else min(dt_min[i],d)
                    dt_max[i]=d if dt_max[i] is None else max(dt_max[i],d)
                else:first[i]=t
                last[i]=t;counts[i]+=1
    with Path(windows).open(encoding='utf-8') as f:
        metadata=json.loads(next(f));stop=None;raw_counts=[0]*7
        for line in f:
            event=json.loads(line)
            if event['kind']=='raw_poll':
                for ch in event['poll']['channels']:raw_counts[ch['slot']]+=len(ch['values_v'])
            if event['kind']=='stop':stop=event
    if not stop or not stop['recording_complete'] or stop['last_seq']!=packets-1:
        raise AssertionError('Stop record missing or incomplete')
    if raw_counts!=counts or stop['channel_sample_counts']!=counts:
        raise AssertionError('Raw poll / sent / stop counts differ')
    stats=[]
    for i,ch in enumerate(metadata['channels']):
        observed=(counts[i]-1)/(last[i]-first[i]) if counts[i]>1 else None
        stats.append({'slot':i,'sid':SIDS[i],'present':ch['present'],'samples':counts[i],
                      'sdk_rate_hz':ch['sample_rate'],'observed_time_axis_rate_hz':observed,
                      'first_device_s':first[i],'last_device_s':last[i],
                      'min_step_s':dt_min[i],'max_step_s':dt_max[i]})
    return {'packet_equality':'PASS','raw_poll_counts':'PASS','stop_tail':'PASS',
            'run_id':run,'packets':packets,'channel_stats':stats,'stop':stop}


def run(real=False,duration=10,output=None):
    root=Path(output or Path(__file__).parent/'validation'/(( 'real' if real else 'synthetic')+'-'+time.strftime('%Y%m%d-%H%M%S')))
    root.mkdir(parents=True,exist_ok=False)
    clock=Clock();source=DelsysSource(clock) if real else SyntheticSource(clock)
    service=Bridge(source,port=0,record_dir=root/'windows',clock=clock).start()
    client_clock=Clock()
    remote.time=SimpleNamespace(time_ns=client_clock.wall_ns,perf_counter_ns=time.perf_counter_ns,monotonic=time.monotonic)
    client=remote.RemoteWindowsSource({'remote_host':'127.0.0.1','remote_port':service.port,'sensor_ids':SIDS},threading.Event(),root/'receiver')
    result={'scope':'real Delsys -> Windows bridge -> Ubuntu client on Windows loopback' if real else 'synthetic source -> bridge -> Ubuntu client on Windows loopback',
            'remote_ubuntu_test':False,'hardware_sync_test':False,'requested_duration_s':duration,
            'client_wall_clock':'Windows precise UTC adapter used to emulate high-resolution Linux clock',
            'started_utc_ns':clock.wall_ns(),'status':'RUNNING'}
    try:
        channels=service.prepare().result(35)
        result['channels']=channels
        print(json.dumps({'stage':'configured','simulated':not real,'channels':channels},ensure_ascii=False),flush=True)
        service.arm().result(3)
        client.connect();client.synchronize()
        result['sync']=client.sync.copy()
        client.start()
        result['clock_anchor']=client.anchor.copy()
        start=time.monotonic();deadline=start+duration;last_report=start
        while time.monotonic()<deadline:
            client.poll()
            if time.monotonic()-last_report>=10:
                last_report=time.monotonic()
                print(json.dumps({'stage':'streaming','elapsed_s':round(last_report-start,1),
                                  'snapshot':service.snapshot()},ensure_ascii=False),flush=True)
        result['streaming_duration_s']=time.monotonic()-start
        client.stop();list(client.drain_pending())
        result['windows_file']=service.snapshot()['record_path']
        result['receiver_file']=client.status['journal']
        client.close()
        result['comparison']=compare(result['windows_file'],result['receiver_file'])
        result['status']='PASS'
        print(json.dumps({'stage':'complete','comparison':result['comparison']},ensure_ascii=False),flush=True)
    except Exception as exc:
        result['status']='FAIL';result['error']=type(exc).__name__+': '+str(exc)
        print(json.dumps({'stage':'failed','error':result['error']},ensure_ascii=False),flush=True)
    finally:
        try:client.close()
        except Exception as exc:result['client_close_error']=type(exc).__name__+': '+str(exc)
        try:service.shutdown()
        except Exception as exc:result['shutdown_error']=str(exc);result['status']='FAIL'
        result['ended_utc_ns']=clock.wall_ns()
        (root/'validation_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({'result_path':str(root/'validation_result.json'),'status':result['status']},ensure_ascii=False),flush=True)
    return result['status']=='PASS'


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--real',action='store_true')
    p.add_argument('--duration',type=float,default=10)
    p.add_argument('--output')
    args=p.parse_args()
    sys.exit(0 if run(args.real,args.duration,args.output) else 1)
