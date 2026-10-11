import unittest, tempfile, time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from hipexo_trend import trend_points
import hipexo_monitor as m
from PyQt5 import QtWidgets
APP=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
class TrendTests(unittest.TestCase):
    def test_bound_order_endpoints_and_unchanged_input(self):
        for n in [0,1,159,160,161,1000,10000]:
            for limit in [6,7,159,160]:
                x=np.arange(n);y=np.sin(x);before=y.copy();a,b=trend_points(x,y,limit)
                self.assertLessEqual(len(a),limit);self.assertTrue(np.all(np.diff(a)>0))
                np.testing.assert_equal(y,before)
                if n:self.assertEqual((a[0],a[-1]),(0,n-1))
    def test_peaks_and_gaps(self):
        y=np.zeros(1000);y[100]=100;y[120]=-100;y[500]=np.nan
        x,z=trend_points(range(1000),y)
        self.assertIn(100,z);self.assertIn(-100,z);self.assertTrue(np.isnan(z).any())
    def test_selector(self):
        with tempfile.TemporaryDirectory() as d,patch.object(m,'EXPORT_DIR',d):
            dm=m.DataManager();w=m.ImuWorker(dm);p=m.ImuPanel(w)
            try:
                for index,fields in enumerate([(6,7,8),(0,1,2),(3,4,5),tuple(range(9))]):
                    p._trend_group.setCurrentIndex(index)
                    self.assertEqual(p._trend_fields,fields)
                    self.assertEqual([i for i,x in enumerate(p._plots) if not x.isHidden()],list(fields))
            finally:
                w.shutdown();p._timer.stop();p.close();dm._flush_timer.stop();dm._mem_timer.stop()
    def test_record_growth_stall_error_stop(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'test.csv';path.write_text('data')
            dm=SimpleNamespace(_recording=True,_record_path=str(path),_recorder=SimpleNamespace(error=None))
            obj=SimpleNamespace(_dm=dm,_lbl_record_health=QtWidgets.QLabel())
            with patch.object(m.time,'monotonic',return_value=10):m.MainWindow._refresh_record_health(obj)
            self.assertIn('正在记录',obj._lbl_record_health.text())
            with patch.object(m.time,'monotonic',return_value=14):m.MainWindow._refresh_record_health(obj)
            self.assertIn('等待文件增长',obj._lbl_record_health.text())
            path.write_text('more data');m.MainWindow._refresh_record_health(obj)
            self.assertIn('正在记录',obj._lbl_record_health.text())
            dm._recorder.error='disk error';m.MainWindow._refresh_record_health(obj)
            self.assertIn('写入错误',obj._lbl_record_health.text())
            dm._recorder.error=None;dm._recording=False;m.MainWindow._refresh_record_health(obj)
            self.assertIn('未录制',obj._lbl_record_health.text())
