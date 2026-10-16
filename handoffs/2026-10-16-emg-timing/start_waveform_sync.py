"""Run the original waveform GUI with local CSV saving and control-only TCP."""
import os
from pathlib import Path
import sys
import ctypes

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent / 'windows_emg_bridge'))
from start_emg_standalone import (SOURCE, DisabledClockBridge, is_source_discontinuity,
                                  controller_unavailable)
from bridge_settings import load_connection_token
from windows_control import install


def load_runtime(records_root):
    source = SOURCE.read_text(encoding='utf-8-sig')
    missing = '''from shared_status_ring import status_ring_layout, try_write_status
from realtime_timing import (
    RollingMinimumClockBridge,
    is_source_discontinuity,
    wait_until_perf_ns,
)'''
    replacements = {
        missing: '# Unavailable Final_System controller integration disabled.',
        'ENABLE_FILE_SAVING = False  # 是否允许子程序保存数据文件':
            'ENABLE_FILE_SAVING = True  # Windows local saving enabled by sync launcher',
        'PERIODIC_CSV_SAVE_ENABLED = False  # 仅在ENABLE_FILE_SAVING=True时启用周期CSV；正式默认不写EMG本地CSV':
            'PERIODIC_CSV_SAVE_ENABLED = True',
        'self.base_directory = os.getcwd()  # 使用当前工作目录':
            'self.base_directory = _emg_control_records_root',
        'timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")  # 格式化时间戳':
            'timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")',
    }
    for old, new in replacements.items():
        if source.count(old) != 1:
            raise RuntimeError('Original source changed; sync launcher needs review: '+old[:65])
        source = source.replace(old, new, 1)
    sys.path.insert(0, str(SOURCE.parent))
    os.chdir(SOURCE.parent)
    namespace = dict(__name__='emg_waveform_sync_runtime', __file__=str(SOURCE),
                     _emg_control_records_root=str(records_root),
                     RollingMinimumClockBridge=DisabledClockBridge,
                     is_source_discontinuity=is_source_discontinuity,
                     wait_until_perf_ns=controller_unavailable,
                     status_ring_layout=controller_unavailable, try_write_status=controller_unavailable)
    exec(compile(source, str(SOURCE), 'exec'), namespace)
    namespace['run_emg_controlled'] = controller_unavailable
    namespace['EMGApplication'].process_external_command = controller_unavailable
    return namespace


def main():
    guard = None
    if os.name == 'nt' and '--simulate' not in sys.argv and os.environ.get('EMG_SIMULATION_MODE') != '1':
        api = ctypes.WinDLL('kernel32', use_last_error=True)
        api.CreateMutexW.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_wchar_p]
        api.CreateMutexW.restype = ctypes.c_void_p
        guard = api.CreateMutexW(None, 0, 'Local\\HiPExoRealDelsysBridge')
        if not guard or ctypes.get_last_error() == 183:
            raise RuntimeError('请先关闭旧 Windows EMG 桥接服务或另一个联动波形窗口，再启动本程序。')
    records = HERE / 'records'
    records.mkdir(exist_ok=True)
    namespace = load_runtime(records)
    install(namespace, load_connection_token(), records)
    # The original main consumes --simulate itself; default is real hardware.
    try:
        namespace['main']()
    finally:
        if guard:
            api.CloseHandle.argtypes = [ctypes.c_void_p]
            api.CloseHandle(guard)


if __name__ == '__main__':
    main()
