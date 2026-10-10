"""Explicit maintenance command; interface must be closed. Never runs on import.
Only documented standard 10/50/100/200 Hz RRATE codes; no guessed 1 kHz command.
"""
import argparse,json,time,fcntl
from pathlib import Path
from datetime import datetime
from smbus2 import SMBus
DEVICES=[(7,0x50),(7,0x51),(1,0x52),(1,0x53)]
CODES={10:6,50:8,100:9,200:11}
def main():
    p=argparse.ArgumentParser();p.add_argument('--hz',type=int,choices=CODES,required=True);p.add_argument('--output',required=True);p.add_argument('--save',action='store_true');a=p.parse_args()
    result={'time':datetime.now().astimezone().isoformat(),'requested_hz':a.hz,'saved':a.save,'sensors':[]}
    destination=Path(a.output);destination.parent.mkdir(parents=True,exist_ok=True)
    for busid,addr in DEVICES:
        entry={'bus':busid,'address':addr};result['sensors'].append(entry)
        with open(f'/tmp/hipexo-i2c-{busid}.lock','w') as lease:
            fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
            with SMBus(busid) as b:
                def read(reg):
                    v=b.read_i2c_block_data(addr,reg,2);return v[0]|v[1]<<8
                def write(reg,value):b.write_i2c_block_data(addr,reg,[value&255,value>>8]);time.sleep(.05)
                try:
                    old=read(3);entry['before']=old
                    destination.write_text(json.dumps(result,indent=2))
                    write(0x69,0xb588);write(3,CODES[a.hz]);after=read(3);entry['after']=after
                    if after!=CODES[a.hz]:raise RuntimeError('RRATE readback mismatch')
                    if a.save:write(0,0)
                    entry['verified']=read(3)==CODES[a.hz]
                    if not entry['verified']:raise RuntimeError('Saved RRATE readback mismatch')
                except Exception as exc:
                    entry['error']=str(exc)
                    if 'before' in entry:
                        try:
                            write(0x69,0xb588);write(3,old)
                            if a.save:write(0,0)
                            entry['restored']=read(3)==old
                        except Exception as rollback:entry['restore_error']=str(rollback)
        destination.write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
