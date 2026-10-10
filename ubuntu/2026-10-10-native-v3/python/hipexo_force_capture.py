"""ADS8688 capture in a dedicated process; bounded 20 ms IPC batches."""
import time,fcntl,json,socket,struct,sys,select
import spidev as _spidev
class ADS8688:
    """
    Minimal ADS8688 16-bit 8-ch SPI ADC driver.
    SPI Mode 1 (CPOL=0, CPHA=1).  No GPIO / RST needed — relies on
    power-on self-reset.  Data is in bytes [2:4] of each 32-bit frame.
    """
    REG_RANGE_BASE = 0x05   # CH0 range register; CH{n} = BASE + n

    def __init__(self, bus: int, device: int, speed: int):
        self._lease=open(f'/tmp/hipexo-spi-{bus}-{device}.lock','w')
        fcntl.flock(self._lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self._selected_channel = None
        self._spi = _spidev.SpiDev()
        self._spi.open(bus, device)
        self._spi.max_speed_hz = speed
        self._spi.mode         = 0b01
        self._spi.bits_per_word = 8
        # Software reset
        self._spi.xfer2([0x85, 0x00, 0x00, 0x00])
        time.sleep(0.05)

    def set_channel_range(self, ch: int, range_code: int):
        """Write documented low nibble and verify register readback."""
        self._selected_channel = None
        reg = (self.REG_RANGE_BASE + ch)
        cmd = ((reg << 1) | 0x01) & 0xFF
        self._spi.xfer2([cmd, range_code & 0x0F, 0x00, 0x00])
        readback=self._spi.xfer2([reg<<1,0,0,0])[2]
        if readback!=range_code:raise IOError(f"ADS8688 CH{ch} range readback {readback}, expected {range_code}")

    def read_raw(self, ch: int) -> int:
        """Manual channel select then NO_OP to clock out 16-bit result."""
        self._selected_channel = None
        sel = (0b11000000 | (ch << 2)) & 0xFF
        self._spi.xfer2([sel, 0x00, 0x00, 0x00])   # select
        r = self._spi.xfer2([0x00, 0x00, 0x00, 0x00])
        return (r[2] << 8) | r[3]                  # data in bytes 2-3

    def read_channels(self, channels):
        """One conversion per CS frame; command selects NEXT frame's channel.
        ADS8688 datasheet §8.4.2.6. Capture each read's host timestamp.
        """
        channels=list(channels)
        if not channels:return []
        if self._selected_channel!=channels[0]:
            self._spi.xfer2([0xC0 | (channels[0]<<2),0,0,0])
            self._selected_channel=channels[0]
        result=[]
        for i,ch in enumerate(channels):
            following=channels[(i+1)%len(channels)]
            r=self._spi.xfer2([0xC0 | (following<<2),0,0,0])
            mono=time.perf_counter_ns();wall=time.time_ns()
            self._selected_channel=following
            result.append(((r[2]<<8)|r[3],mono,wall))
        return result

    def close(self):
        try: self._spi.close()
        except Exception: pass
        self._lease.close()


def send_packet(sock,value):
    payload=json.dumps(value,separators=(',',':')).encode()
    sock.sendall(struct.pack('!I',len(payload))+payload)

def child(fd,config):
    from hipexo_capture_ipc import PacketSender
    sock=socket.socket(fileno=fd);sender=PacketSender(sock)
    adc=None
    try:
        adc=ADS8688(config['bus'],config['device'],config['speed'])
        for ch,code in config['ranges']:adc.set_channel_range(ch,code)
        from hipexo_realtime import tune_current_process
        sender.send(dict(tuning=tune_current_process()))
        period=1/max(1,config['hz']);deadline=time.perf_counter();batch=[]
        while True:
            batch.append(adc.read_channels(config['channels']))
            if len(batch)>=max(1,config['hz']//50):
                sender.send(dict(samples=batch,capture_ipc_peak_bytes=sender.peak));batch=[]
                if select.select([sock],[],[],0)[0]:
                    sock.recv(16);break
            sender.pump()
            deadline+=period;now=time.perf_counter()
            if deadline<now-5*period:deadline=now
            if deadline>now:time.sleep(deadline-now)
        sender.drain()
    except Exception as exc:
        try:
            sender.drain();sender.send(dict(error=str(exc)));sender.drain()
        except Exception:pass
    finally:
        if adc:adc.close()
        sock.close()

if __name__=='__main__':child(int(sys.argv[1]),json.loads(sys.argv[2]))
