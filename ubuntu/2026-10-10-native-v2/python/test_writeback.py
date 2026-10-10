import ctypes,errno,unittest
from unittest.mock import patch,Mock
import hipexo_writeback as w
class WritebackTests(unittest.TestCase):
 def test_requests_only_async_write_on_exact_owned_range(self):
  sync=Mock(return_value=0)
  with patch.object(w,'_sync',sync):self.assertTrue(w.submit(7,4096,3000))
  sync.assert_called_once_with(7,4096,3000,2)
 def test_unsupported_falls_back_but_io_failure_surfaces(self):
  with patch.object(w,'_sync',return_value=-1):
   ctypes.set_errno(errno.EOPNOTSUPP);self.assertFalse(w.submit(7))
   ctypes.set_errno(errno.EIO)
   with self.assertRaises(OSError):w.submit(7)
if __name__=='__main__':unittest.main()
