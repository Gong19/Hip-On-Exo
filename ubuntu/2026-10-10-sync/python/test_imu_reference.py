import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from hipexo_imu_reference import ImuReference, rotation, euler
from hipexo_session import SessionManifest

DEVICES=[(7,0x50),(7,0x51),(1,0x52),(1,0x53)]
def frame(r=10,p=20,y=179):
    return dict(roll_deg=r,pitch_deg=p,yaw_deg=y,gx_dps=0,gy_dps=0,gz_dps=0,ax_g=0,ay_g=0,az_g=1)

class ReferenceTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.session=SessionManifest(self.tmp.name,subject_id='S_test',location='Lab')
        self.ref=ImuReference(4)
    def collect(self,data=None):
        self.ref.request([True]*4,self.session,now=0)
        for j in range(152):
            for i in range(4): self.ref.process(i,data or frame(),self.session,DEVICES,now=j*.02)
    def test_raw_preserved_and_matrix_relative_saved(self):
        self.collect()
        original=frame();out=self.ref.process(0,original,self.session,DEVICES,now=3.1)
        self.assertEqual(out['reference_valid'],1)
        for k,v in original.items(): self.assertEqual(out[k],v)
        np.testing.assert_allclose([out['rel_roll_deg'],out['rel_pitch_deg'],out['rel_yaw_deg']],0,atol=1e-10)
        target=rotation(10,20,179) @ rotation(15,-5,7)
        r,p,y=euler(target)
        out=self.ref.process(0,frame(r,p,y),self.session,DEVICES,now=3.2)
        np.testing.assert_allclose([out['rel_roll_deg'],out['rel_pitch_deg'],out['rel_yaw_deg']],[15,-5,7],atol=1e-10)
        refs=self.session.raw_references();self.assertEqual(refs[0]['modality'],'imu_reference')
        info=json.loads(Path(refs[0]['path']).read_text());self.assertEqual(len(info['sensors']),4)
        self.assertEqual(info['metadata']['subject_id'],'S_test')
    def test_wrap_at_180(self):
        self.collect(frame(0,0,179.5))
        out=self.ref.process(0,frame(0,0,-179.5),self.session,DEVICES,now=3.2)
        self.assertAlmostEqual(out['rel_yaw_deg'],1)
    def test_offline_and_motion_rejected(self):
        self.assertFalse(self.ref.request([True,True,False,True],self.session,now=0))
        self.ref.request([True]*4,self.session,now=0)
        data=frame();data['gx_dps']=10
        out=self.ref.process(0,data,self.session,DEVICES,now=.02)
        self.assertEqual(out['reference_valid'],0);self.assertIsNone(self.ref.pending)
        self.assertFalse(self.session.raw_references())
    def test_pose_change_and_gap_rejected(self):
        for second,now in [(frame(15),.02),(frame(),.3)]:
            self.ref.request([True]*4,self.session,now=0)
            self.ref.process(0,frame(),self.session,DEVICES,now=0)
            self.ref.process(0,second,self.session,DEVICES,now=now)
            self.assertIsNone(self.ref.pending)
    def test_drop_stop_and_session_reset(self):
        self.collect();self.ref.invalidate(2,'断线')
        out=self.ref.process(2,frame(),self.session,DEVICES,now=3.3)
        self.assertEqual(out['reference_valid'],0);self.assertIn(0,self.ref.refs)
        self.ref.invalidate(reason='停止');self.assertFalse(self.ref.refs)
        self.collect()
        new=SessionManifest(Path(self.tmp.name)/'new',subject_id='other')
        out=self.ref.process(0,frame(),new,DEVICES,now=3.4)
        self.assertEqual(out['reference_valid'],0);self.assertFalse(self.ref.refs)
    def test_disk_failure_does_not_apply(self):
        with patch.object(Path,'write_text',side_effect=OSError('disk full')): self.collect()
        self.assertFalse(self.ref.refs);self.assertIn('保存失败',self.ref.message)
    def test_missing_data_timeout(self):
        self.ref.request([True]*4,self.session,now=0)
        self.ref.status(self.session,now=4.1)
        self.assertIsNone(self.ref.pending);self.assertFalse(self.ref.refs)

if __name__=='__main__':unittest.main()
