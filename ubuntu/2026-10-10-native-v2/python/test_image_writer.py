import tempfile,unittest
from pathlib import Path
import cv2,numpy as np
from hipexo_image_writer import ImageWriter
class ImageWriterTests(unittest.TestCase):
 def test_png_is_lossless_uint16_and_input_is_bounded(self):
  with tempfile.TemporaryDirectory() as d:
   w=ImageWriter()
   try:
    data=np.random.default_rng(4).integers(0,65536,size=(32,48),dtype=np.uint16)
    path=str(Path(d)/'depth.png');w.write(path,data)
    np.testing.assert_array_equal(cv2.imread(path,cv2.IMREAD_UNCHANGED),data)
    with self.assertRaises(ValueError):w.write(path,np.zeros((1025,1025),dtype=np.uint16))
   finally:w.close()
   self.assertIsNotNone(w.process.poll())
 def test_encoder_exit_is_visible(self):
  w=ImageWriter();w.process.kill();w.process.wait()
  try:
   with self.assertRaises((EOFError,OSError)):w.write('/unused.png',np.zeros((2,2),dtype=np.uint16))
  finally:w.close(force=True)
if __name__=='__main__':unittest.main()
