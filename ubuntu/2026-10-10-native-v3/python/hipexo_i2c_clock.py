"""Scoped, board-checked Jetson I2C-1 source-clock tuning.

This is a runtime BPMP debug override, not a device-tree bus-frequency change.
Never infer actual SCL frequency from this report without measuring the wire.
"""
import fcntl,json,os,subprocess
from pathlib import Path

class I2cClockLease:
    def __init__(self,bus):
        self.bus=bus;self.lock=None;self.saved=None
        self.base=Path('/sys/kernel/debug/bpmp/debug/clk/i2c2')
        self.lock_path=Path('/tmp/hipexo-clock-i2c2.lock')
        self.report={'source_clock':'unchanged'}
    def _read(self,key):
        return int(subprocess.check_output(['sudo','-n','cat',str(self.base/key)],text=True,timeout=3).strip())
    def _write(self,key,value):
        subprocess.run(['sudo','-n','tee',str(self.base/key)],input=str(value)+'\n',text=True,capture_output=True,check=True,timeout=3)
    def _eligible(self):
        if self.bus!=1:return False
        node=Path('/sys/bus/i2c/devices/i2c-1/of_node')
        if node.resolve().name!='i2c@c240000':return False
        if (node/'clock-frequency').read_bytes()!=bytes.fromhex('000186a0'):return False
        if (node/'clocks').read_bytes()[4:8]!=bytes.fromhex('00000031'):return False
        return b'nvidia,tegra194-i2c' in (node/'compatible').read_bytes()
    def __enter__(self):
        if os.environ.get('HIPEXO_DISABLE_TUNING')=='1' or os.environ.get('HIPEXO_I2C_CLOCK_TUNING','1')=='0':return self
        try:
            if not self._eligible():return self
            self.lock=open(self.lock_path,'a+');fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            self.lock.seek(0);raw=self.lock.read().strip()
            saved=json.loads(raw) if raw else {'rate':self._read('rate'),'mrq_rate_locked':self._read('mrq_rate_locked')}
            if set(saved)!={'rate','mrq_rate_locked'} or saved['rate']!=136000000 or saved['mrq_rate_locked']!=0:
                raise ValueError('Unexpected prior I2C clock policy; refusing override')
            if self._read('max_rate')<204000000:raise ValueError('204 MHz exceeds reported maximum')
            self.saved=saved;self.lock.seek(0);self.lock.truncate();json.dump(saved,self.lock);self.lock.flush();os.fsync(self.lock.fileno())
            self._write('mrq_rate_locked',1);self._write('rate',204000000)
            actual=self._read('rate')
            if actual!=204000000:raise RuntimeError('Clock readback mismatch')
            self.report.update(source_clock_hz=actual,source_clock='scoped override',configured_scl_hz=100000)
        except Exception as exc:
            self.report['clock_error']=str(exc)
            # Restore partial writes immediately; a failed flock must never restore another owner.
            self.__exit__(None,None,None)
        return self
    def __exit__(self,*args):
        try:
            if self.saved is not None:
                self._write('rate',self.saved['rate']);self._write('mrq_rate_locked',self.saved['mrq_rate_locked'])
                if any(self._read(k)!=v for k,v in self.saved.items()):raise RuntimeError('Clock restore readback mismatch')
                self.report['restored']=True;self.saved=None
                self.lock.seek(0);self.lock.truncate();self.lock.flush();os.fsync(self.lock.fileno())
        except Exception as exc:self.report['restore_error']=str(exc)
        finally:
            if self.lock:self.lock.close();self.lock=None
