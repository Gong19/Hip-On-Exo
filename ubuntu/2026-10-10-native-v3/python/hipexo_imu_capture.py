"""One bounded raw-IMU capture process per I2C bus; references stay in parent."""
import fcntl,json,select,socket,sys,time
from smbus2 import SMBus
from hipexo_realtime import tune_current_process
from hipexo_capture_ipc import PacketSender


def child(fd,config):
    sock=socket.socket(fileno=fd);sender=PacketSender(sock)
    bus=None;lease=None
    try:
        lease=open(f"/tmp/hipexo-i2c-{config['bus']}.lock",'w')
        fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
        bus=SMBus(config['bus']);sender.send(dict(tuning=tune_current_process()))
        deadline=time.perf_counter();flush=deadline+.02;batch=[];retry={};last_temp={};temps={}
        while True:
            for idx,addr in config['devices']:
                now=time.perf_counter()
                if now<retry.get(idx,0):continue
                began=time.perf_counter_ns()
                try:
                    p1=bus.read_i2c_block_data(addr,0x34,12)
                    p2=bus.read_i2c_block_data(addr,0x3d,6)
                    completed=time.perf_counter_ns();wall=time.time_ns();block=p1+p2
                    if len(block)!=18 or not any(block) or all(x==255 for x in block):raise IOError('Invalid all-zero/all-FF IMU registers')
                    if now-last_temp.get(idx,-1e9)>=1:
                        raw=bus.read_i2c_block_data(addr,0x40,2)
                        temps[idx]=int.from_bytes(bytes(raw),'little',signed=True)/100;last_temp[idx]=now
                    batch.append(dict(idx=idx,raw=block,mono=completed,wall=wall,
                                      duration_ms=(completed-began)/1e6,temp=temps.get(idx)))
                except Exception as exc:
                    batch.append(dict(idx=idx,error=str(exc)));retry[idx]=now+config['retry_s']
            now=time.perf_counter()
            if now>=flush or len(batch)>=16:
                sender.send(dict(samples=batch,capture_ipc_peak_bytes=sender.peak));batch=[];flush=now+.02
                if select.select([sock],[],[],0)[0]:sock.recv(16);break
            sender.pump()
            deadline+=.005;delay=deadline-time.perf_counter()
            if delay>0:time.sleep(delay)
            elif delay<-.005:deadline=time.perf_counter()
        sender.drain()
    except Exception as exc:
        try:
            sender.drain();sender.send(dict(error=str(exc)));sender.drain()
        except Exception:pass
    finally:
        if bus:bus.close()
        if lease:lease.close()
        sock.close()

if __name__=='__main__':child(int(sys.argv[1]),json.loads(sys.argv[2]))
