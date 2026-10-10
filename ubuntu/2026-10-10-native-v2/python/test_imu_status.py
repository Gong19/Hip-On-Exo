"""Four-slot IMU UI and dropout/reconnect tests using fake I2C reads only."""
import os,tempfile,time,unittest
from unittest.mock import patch
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
os.environ['HIPEXO_IMU_IN_PROCESS']='1'
from PyQt5 import QtWidgets
import hipexo_monitor as m
APP=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class ImuStatusTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.available=set(m.IMU_DEVICES)
        self.blocked_buses=set()
        owner=self
        class FakeBus:
            def __init__(self,bus_id):
                if bus_id in owner.blocked_buses:raise OSError('test bus missing')
                self.bus_id=bus_id
            def read_i2c_block_data(self,addr,reg,length):
                if (self.bus_id,addr) not in owner.available:raise OSError(121,'test disconnected')
                return ([0,8]*6)[:length]
            def close(self):pass
        self.patch=patch.multiple(m,SMBus=FakeBus,_SMBUS_OK=True,EXPORT_DIR=self.tmp.name,
                                  IMU_PERIOD_S=.01,IMU_OFFLINE_AFTER_S=.1,IMU_RESCAN_INTERVAL_S=.05)
        self.patch.start()
        self.dm=m.DataManager();self.worker=m.ImuWorker(self.dm);self.panel=m.ImuPanel(self.worker)

    def tearDown(self):
        self.worker.shutdown();self.panel._timer.stop();self.panel.close()
        self.dm._flush_timer.stop();self.dm._mem_timer.stop();APP.processEvents()
        self.patch.stop();self.tmp.cleanup()

    def pump(self,condition,timeout=2):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            APP.processEvents();self.panel._refresh();time.sleep(.01)
            if condition():return True
        return False

    def test_all_absent_still_shows_four_offline_cards(self):
        self.available.clear();self.panel._on_start()
        self.assertTrue(self.pump(lambda:all(self.worker.error_detail(i) for i in range(4))))
        self.assertEqual(len(self.panel._temp_labels),4)
        for i,label in enumerate(self.panel._temp_labels):
            self.assertIn('OFFLINE',label.text())
            self.assertIn(f'0x{m.IMU_DEVICES[i][1]:02X}',label.text())
            self.assertIn('test disconnected',label.toolTip())
        self.assertIn('ONLINE 0/4',self.panel.lbl_state.text())
        self.assertTrue(all(not times for times in self.panel._t))

    def test_dropout_recovery_and_stopped_are_distinct(self):
        self.panel._on_start()
        self.assertTrue(self.pump(lambda:all('ONLINE' in l.text() for l in self.panel._temp_labels)))
        self.available.remove(m.IMU_DEVICES[0]);self.available.remove(m.IMU_DEVICES[3])
        self.assertTrue(self.pump(lambda:'ONLINE 2/4' in self.panel.lbl_state.text()))
        self.panel._refresh()
        self.assertIn('ONLINE 2/4',self.panel.lbl_state.text())
        self.assertIn('OFFLINE',self.panel._temp_labels[0].text())
        self.assertIn('ONLINE',self.panel._temp_labels[1].text())
        old=list(self.panel._t[0]);other=len(self.dm.snapshot('imu_1_ax_g')[0])
        self.assertTrue(self.pump(lambda:len(self.dm.snapshot('imu_1_ax_g')[0])>other+2))
        self.assertEqual(list(self.panel._t[0]),old)
        self.available.update(m.IMU_DEVICES)
        self.assertTrue(self.pump(lambda:'ONLINE 4/4' in self.panel.lbl_state.text()))
        self.panel._refresh()
        self.assertIn('ONLINE 4/4',self.panel.lbl_state.text())
        self.assertEqual(self.dm.snapshot('imu_2_i2c_address')[1][-1],0x52)
        self.panel._on_stop();self.panel._refresh()
        self.assertTrue(all('STOPPED' in l.text() for l in self.panel._temp_labels))
        self.panel._on_start()
        self.assertTrue(self.pump(lambda:all(self.worker.is_online(i) for i in range(4))))

    def test_bus_open_failure_is_retried(self):
        self.blocked_buses.update((1,7));self.panel._on_start()
        self.assertTrue(self.pump(lambda:all('Cannot open' in self.worker.error_detail(i) for i in range(4))))
        self.assertTrue(self.worker._thread.is_alive())
        self.assertIn('ONLINE 0/4',self.panel.lbl_state.text())
        self.blocked_buses.clear()
        self.assertTrue(self.pump(lambda:all(self.worker.is_online(i) for i in range(4))))

    def test_other_sensors_do_not_shift_an_old_trace(self):
        base=time.perf_counter();self.panel._t0=base
        data={field:float(i) for i,(field,_) in enumerate(m._IMU_FIELDS)}
        for sensor,stamp in [(0,1.),(1,1.1),(0,1.2),(1,1.3)]:
            with patch.object(m.time,'perf_counter',return_value=base+stamp):
                self.panel._on_data(sensor,data,20.)
        with patch.object(m.time,'perf_counter',return_value=base+1.4):self.panel._refresh()
        first=self.panel._curves[0][0].xData.copy()
        with patch.object(m.time,'perf_counter',return_value=base+1.5):self.panel._on_data(1,data,20.)
        with patch.object(m.time,'perf_counter',return_value=base+1.6):self.panel._refresh()
        self.assertEqual(list(first),list(self.panel._curves[0][0].xData))
        self.assertAlmostEqual(first[0],1.)
        self.assertAlmostEqual(first[-1],1.2)


if __name__=='__main__':unittest.main()
