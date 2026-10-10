import csv,json,os,tempfile,time,unittest
from unittest.mock import patch
from pathlib import Path
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PyQt5 import QtWidgets
import hipexo_monitor as m
from test_imu_reference import frame
APP=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

class ReferenceUiTests(unittest.TestCase):
    def test_button_relative_plot_and_recorded_reference(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(m,'EXPORT_DIR',directory):
            dm=m.DataManager(); worker=m.ImuWorker(dm); panel=m.ImuPanel(worker)
            try:
                panel.btn_reference.click()
                self.assertIn('均在线',panel.lbl_reference.text())
                worker._running.set()
                worker._online={i:True for i in range(4)}
                worker._last_ok_time={i:time.perf_counter() for i in range(4)}
                panel.btn_reference.click()
                self.assertIsNotNone(worker.reference.pending)
                base=worker.reference.pending['start']
                for j in range(152):
                    for i in range(4): worker.reference.process(i,frame(),dm.session,m.IMU_DEVICES,now=base+j*.02)
                panel._refresh_status()
                self.assertTrue(all('已设置' in lbl.text() for lbl in panel._temp_labels))
                panel.chk_relative.setChecked(True)
                self.assertTrue(dm.start_recording())
                for i in range(4):
                    data=worker.reference.process(i,frame(),dm.session,m.IMU_DEVICES,now=base+3.2)
                    dm.append_dict(f'imu_{i}',1000,data)
                    panel._on_data(i,data,25)
                    self.assertAlmostEqual(panel._y[6][i][-1],0)
                self.assertTrue(dm.stop_recording())
                info=json.loads((Path(dm._record_path).parent/'recording_info.json').read_text())
                self.assertTrue(any(r['modality']=='imu_reference' for r in info['session_raw_references']))
                with open(dm._record_path) as f: rows=list(csv.DictReader(f))
                fields={r['field'] for r in rows}
                self.assertTrue({'roll_deg','rel_roll_deg','reference_valid','reference_id'} <= fields)
                panel.chk_relative.setChecked(False)
                panel._on_data(0,frame(),25)
                self.assertEqual(panel._y[6][0][-1],10)
                worker._set_online(0,False)
                panel._refresh_status()
                self.assertIn('需重新设置',panel._temp_labels[0].text())
            finally:
                worker.shutdown();panel._timer.stop();panel.close()
                dm.stop_recording();dm._flush_timer.stop();dm._mem_timer.stop()
                APP.processEvents()

if __name__=='__main__':unittest.main()
