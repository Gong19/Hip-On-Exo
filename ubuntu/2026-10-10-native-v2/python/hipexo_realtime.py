"""Scoped Linux acquisition tuning. No global clocks or scheduler changes."""
import ctypes
import fcntl
import os
from pathlib import Path
import subprocess
import threading


def tune_current_process():
    """Best effort; caller must expose returned status, never assume RT succeeded."""
    if os.environ.get('HIPEXO_DISABLE_TUNING') == '1':return {'disabled': True}
    report = {'timer_slack_ns': None, 'scheduler': 'normal'}
    try:
        if ctypes.CDLL(None, use_errno=True).prctl(29, 1, 0, 0, 0) != 0:
            raise OSError(ctypes.get_errno(), 'PR_SET_TIMERSLACK')
        report['timer_slack_ns'] = 1
    except Exception as exc:
        report['timer_error'] = str(exc)
    try:
        subprocess.run(['sudo', '-n', 'chrt', '-f', '-p', '10', str(threading.get_native_id())],
                       check=True, capture_output=True, timeout=3)
        report['scheduler'] = 'SCHED_FIFO:10'
    except Exception as exc:
        report['scheduler_error'] = str(exc)
    return report


class ControllerPowerLease:
    """Keep one controller awake while its child captures; restore on parent exit.

    The parent owns this lease, so forced child termination still restores power.
    A saved state permits recovery after an uncatchable parent SIGKILL on next use.
    """
    def __init__(self, device_path):
        self.control = None
        self.lock = None
        self.previous = None
        self.report = {'power_policy': 'unchanged'}
        path = Path(device_path).resolve()
        # Choose the controller, not the SPI peripheral's unsupported PM entry.
        for parent in path.parents:
            if parent.name.endswith(('.spi', '.i2c')):
                self.control = parent / 'power/control'
                break

    def _write(self, value):
        subprocess.run(['sudo', '-n', 'tee', str(self.control)], input=value+'\n',
                       text=True, capture_output=True, check=True, timeout=3)

    def __enter__(self):
        if os.environ.get('HIPEXO_DISABLE_TUNING') == '1':
            self.report = {'disabled': True}
            return self
        if self.control is None:
            self.report['power_error'] = 'Controller power control not found'
            return self
        try:
            key = self.control.parent.parent.name
            self.lock = open('/tmp/hipexo-power-'+key+'.lock', 'a+')
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.lock.seek(0)
            saved = self.lock.read().strip()
            self.previous = saved if saved in ('on', 'auto') else self.control.read_text().strip()
            if self.previous not in ('on', 'auto'):
                raise ValueError('Unexpected controller power policy')
            self.lock.seek(0); self.lock.truncate(); self.lock.write(self.previous); self.lock.flush()
            self._write('on')
            self.report.update(power_policy='on', previous=self.previous, controller=str(self.control))
        except Exception as exc:
            self.report['power_error'] = str(exc)
            # Do not restore another owner's policy when acquisition of flock failed.
            if self.previous is None and self.lock:
                self.lock.close(); self.lock = None
        return self

    def __exit__(self, *args):
        try:
            if self.previous is not None:
                self._write(self.previous)
                self.report['restored'] = self.previous
                self.lock.seek(0); self.lock.truncate(); self.lock.flush()
        except Exception as exc:
            self.report['restore_error'] = str(exc)
        finally:
            if self.lock:
                self.lock.close(); self.lock = None
