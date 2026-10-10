"""Linux data-writeback pacing for files owned by this recorder.

WRITE-only sync_file_range starts background IO; it is NOT fsync and does
not guarantee metadata or power-loss durability. No global VM settings.
"""
import ctypes,errno,os,sys
_sync=None
if sys.platform.startswith('linux'):
    try:
        _sync=ctypes.CDLL(None,use_errno=True).sync_file_range
        _sync.argtypes=[ctypes.c_int,ctypes.c_longlong,ctypes.c_longlong,ctypes.c_uint]
        _sync.restype=ctypes.c_int
    except AttributeError:pass

def submit(fd,offset=0,length=0):
    if _sync is None or os.environ.get('HIPEXO_ASYNC_WRITEBACK')=='0':return False
    if offset<0 or length<0:raise ValueError('Negative writeback range')
    if _sync(fd,offset,length,2)==0:return True  # SYNC_FILE_RANGE_WRITE only
    code=ctypes.get_errno()
    if code in (errno.ENOSYS,errno.EOPNOTSUPP,errno.EINVAL):return False
    raise OSError(code,os.strerror(code))
