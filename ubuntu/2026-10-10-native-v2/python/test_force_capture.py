import unittest
from unittest.mock import patch
import hipexo_force_capture as f
class FakeSPI:
    def __init__(self):self.current=0;self.registers={};self.calls=[]
    def open(self,*a):pass
    def close(self):pass
    def xfer2(self,p):
        self.calls.append(p)
        if p[0]==0x85:return [0,0,0,0]
        if p[0]<0x80 and p[0]:
            reg=p[0]>>1
            if p[0]&1:self.registers[reg]=p[1]
            return [0,0,self.registers.get(reg,0),0]
        value=1000+self.current
        if p[0]&0xC0==0xC0:self.current=(p[0]>>2)&7
        return [0,0,value>>8,value&255]
class ForceTests(unittest.TestCase):
    def test_ranges_readback_and_pipeline_channel_identity(self):
        with patch.object(f._spidev,'SpiDev',FakeSPI):
            adc=f.ADS8688(98,98,1000000)
            try:
                adc.set_channel_range(5,0);adc.set_channel_range(4,6)
                self.assertEqual(adc._spi.registers,{10:0,9:6})
                for _ in range(3):
                    rows=adc.read_channels([5,4]);self.assertEqual([r[0] for r in rows],[1005,1004])
                    self.assertLessEqual(rows[0][1],rows[1][1])
                adc.read_raw(1)
                self.assertEqual([r[0] for r in adc.read_channels([5,4])],[1005,1004])
            finally:adc.close()
    def test_bad_range_readback_fails(self):
        with patch.object(f._spidev,'SpiDev',FakeSPI):
            adc=f.ADS8688(98,98,1000000)
            try:
                with self.assertRaises(OSError):adc.set_channel_range(0,32)
            finally:adc.close()
if __name__=='__main__':unittest.main()
