"""Read one 300 ms packet without loading the complete recording into RAM."""
import argparse,csv,json,io
from pathlib import Path

def read_cycle(directory,cycle_id):
    directory=Path(directory)
    packet=None
    with (directory/'cycles.jsonl').open(encoding='utf-8') as f:
        for line in f:
            if not line.strip():continue
            candidate=json.loads(line)
            if candidate['cycle_id']==cycle_id:packet=candidate;break
    if packet is None:raise KeyError(f'Cycle {cycle_id} not found')
    paths=list(directory.glob('*combined.csv'))
    if len(paths)!=1:raise ValueError('Expected exactly one combined CSV')
    length=packet['csv_byte_end']-packet['csv_byte_start']
    if packet['row_count']!=300 or not 0<length<=8*1024*1024:raise ValueError('Invalid bounded cycle index')
    with paths[0].open('rb') as f:
        header=next(csv.reader([f.readline().decode('utf-8')]))
        f.seek(packet['csv_byte_start']);data=f.read(packet['csv_byte_end']-packet['csv_byte_start'])
    rows=list(csv.reader(io.StringIO(data.decode('utf-8'))))
    if len(rows)!=packet['row_count'] or any(len(r)!=len(header) for r in rows):raise ValueError('Incomplete cycle payload')
    packet.update(columns=header,rows=rows)
    return packet

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('cycle',type=int);a=p.parse_args()
    packet=read_cycle(a.directory,a.cycle)
    print(json.dumps(packet,ensure_ascii=False))
