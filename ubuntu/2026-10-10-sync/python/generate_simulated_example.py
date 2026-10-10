"""Generate a 300 ms clearly synthetic example, never connect to hardware."""
import math,time,argparse
from pathlib import Path
from hipexo_pipeline import CycleRecorder,STREAMS,FIELDS,family
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
start=time.perf_counter_ns();wall=1791590400000000000
r=CycleRecorder(out/'SIMULATED_1000Hz_combined.csv',start_mono_ns=start,start_wall_ns=wall)
for stream in STREAMS:
    kind=family(stream);hz=200 if kind=='imu' else 50 if kind=='lidar' else 1000
    count=round(.3*hz)+1
    frames=[]
    for i in range(count):
        frame={k:math.sin(i/hz*2*math.pi) for k,_ in FIELDS[kind]}
        frame.update(simulated=True,valid=1,reference_id=0,reference_valid=0)
        if kind=='imu':frame.update(read_duration_ms=.8,repeated_register_block=0)
        if kind=='force':frame['adc_raw_count']=int(32768+1000*math.sin(i/hz*2*math.pi))
        if kind=='emg':frame.update(raw_v=.001*math.sin(i/hz*2*math.pi*50),envelope_v=.0004,mvc_ratio=.2)
        frames.append(frame)
    ns=[start+round(i/hz*1e9) for i in range(count)]
    r.enqueue_frames(stream,[(wall+n-start)/1e6 for n in ns],frames,ns,list(range(count)))
r.stop_ns=start+300_000_000
if not r.stop():raise RuntimeError(r.error)
(out/'README.txt').write_text('SIMULATED ONLY. No human or physical sensor recordings. 300 ms / 5 x 60 ms / 1000 Hz grid. cycles.jsonl indexes CSV bytes; no duplicate numeric payload.\n')
