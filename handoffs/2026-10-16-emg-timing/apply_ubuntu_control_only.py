"""Back up first, then install the control-only panel into the reviewed Ubuntu GUI."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
import time

HERE = Path(__file__).resolve().parent
BASE_COMMIT = '6320378018f5b6d30d50eed722a78396e2ac5c60'
CHANGES = {
    'hipexo_monitor.py': [
        ('from hipexo_emg import EmgWorker, EmgPanel', 'from hipexo_emg_control import EmgWorker, EmgPanel'),
        ('            self._emg_worker.start()\n',
         '            if self._emg_worker.enabled:\n                self._emg_worker.connect_source()\n'),
        ('            self._recording = True\n            self._btn_rec.setText',
         '            self._recording = True\n            self._emg_worker.record_start(self._dm._record_path)\n            self._btn_rec.setText'),
        ('            recording_ok = self._dm.stop_recording()\n',
         '            if self._emg_worker.enabled:\n                self._emg_worker.stop()\n            recording_ok = self._dm.stop_recording()\n'),
        ('            self._emg_worker.stop()\n            self._imu_worker.stop()',
         '            if self._emg_worker.enabled:\n                self._emg_worker.stop()\n            self._imu_worker.stop()'),
        ('Remote EMG raw JSONL is saved automatically during Sync & Start.',
         'EMG is saved on Windows only; enable EMG linkage to send Record start/stop commands.'),
        ('Local host read completion; EMG mapped source time; not hardware synchronized',
         'Local host read completion; EMG stored on Windows only, separate control timestamps; not hardware synchronized'),
        ('OFFLINE PREVIEW — EMG simulated; physical devices disabled',
         'OFFLINE PREVIEW — EMG control only, no network; physical devices disabled'),
    ],
    'hipexo_pipeline.py': [
        ("+[f'emg_{i}' for i in range(7)]+['lidar_L','lidar_R']",
         "+['lidar_L','lidar_R']  # EMG control-only: no sample arrays or CSV columns"),
        ('EMG input is already processed onto 1kHz. Vendor RMS and raw EMG are distinct; see source metadata.',
         'EMG is acquired and saved only on Windows. This table has no EMG sample columns.'),
    ],
}


def patched(name, content):
    for old, new in CHANGES[name]:
        if content.count(old) != 1:
            raise RuntimeError(name+': source differs from reviewed Ubuntu version; merge required; no files changed')
        content = content.replace(old, new, 1)
    compile(content, name, 'exec')
    return content


def install(target, dry_run=False):
    target = Path(target).resolve()
    reviewed = json.loads((HERE/'upstream_sha256.json').read_text(encoding='utf-8-sig'))
    files = {}
    for name in CHANGES:
        path = target/name
        raw = path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != reviewed[name]:
            raise RuntimeError(name+': SHA256 differs from reviewed commit '+BASE_COMMIT+'; no files changed')
        files[name] = patched(name, raw.decode('utf-8')).encode('utf-8')
    for name in ('hipexo_emg_control.py', 'control_protocol.py'):
        files[name] = (HERE/name).read_bytes()
        compile(files[name], name, 'exec')
    if dry_run:
        return dict(dry_run=True, target=str(target), files=list(files))
    backup = Path(tempfile.mkdtemp(prefix='backup-before-emg-control-'+time.strftime('%Y%m%d-%H%M%S')+'-', dir=target.parent))
    manifest = {}
    for name in files:
        original = target/name
        manifest[name] = original.exists()
        if original.exists():
            shutil.copy2(original, backup/name)
    (backup/'manifest.json').write_text(json.dumps(manifest, indent=2))
    try:
        for name, data in files.items():
            temporary = target/(name+'.control-only.tmp')
            temporary.write_bytes(data)
            os.replace(temporary, target/name)
    except Exception:
        for name, existed in manifest.items():
            if existed:
                shutil.copy2(backup/name, target/name)
            elif (target/name).exists():
                (target/name).unlink()
        raise
    return dict(backup=str(backup), target=str(target), files=list(files), base_commit=BASE_COMMIT)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--target', required=True, help='Existing Ubuntu python directory containing hipexo_monitor.py')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    print(json.dumps(install(args.target, args.dry_run), ensure_ascii=False, indent=2))
