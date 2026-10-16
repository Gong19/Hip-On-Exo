"""Add click/receive timing to the reviewed, already-installed control-only Ubuntu GUI."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile

HERE = Path(__file__).resolve().parent
BASE_COMMIT = '8af0b24fb80f074f970619b9702d3156f7203659'
REPLACEMENTS = [
    ('from hipexo_emg_control import EmgWorker, EmgPanel',
     'from hipexo_emg_control import EmgWorker, EmgPanel\nfrom control_protocol import stamp as emg_control_stamp'),
    ('    def _toggle_collection(self):\n        if not self._collecting:\n            if self._emg_worker.enabled:\n                self._emg_worker.connect_source()',
     '    def _toggle_collection(self):\n        emg_click = emg_control_stamp()\n        if not self._collecting:\n            self._emg_worker.collect_all_requested(emg_click)'),
    ('    def _toggle_recording(self):\n        if not self._recording:',
     '    def _toggle_recording(self):\n        emg_click = emg_control_stamp()\n        if not self._recording:'),
    ('self._emg_worker.record_start(self._dm._record_path)',
     'self._emg_worker.record_start(self._dm._record_path, clicked=emg_click)'),
]


def patched_monitor(content):
    for old,new in REPLACEMENTS:
        if content.count(old) != 1:
            raise ValueError('Ubuntu monitor changed; merge required before installation')
        content = content.replace(old,new,1)
    if content.count('self._emg_worker.stop()') != 2:
        raise ValueError('Unexpected global stop call sites')
    content = content.replace('self._emg_worker.stop()', 'self._emg_worker.stop(clicked=emg_click)')
    compile(content, 'hipexo_monitor.py', 'exec')
    return content


def install(target, dry_run=False):
    target=Path(target).resolve()
    expected=json.loads((HERE/'timing_upstream_sha256.json').read_text(encoding='utf-8'))
    for name,digest in expected.items():
        if hashlib.sha256((target/name).read_bytes()).hexdigest() != digest:
            raise ValueError(name+': differs from reviewed Ubuntu '+BASE_COMMIT+'; no files changed')
    files={'hipexo_monitor.py':patched_monitor((target/'hipexo_monitor.py').read_text(encoding='utf-8')).encode('utf-8')}
    for name in ('hipexo_emg_control.py','control_protocol.py'):
        files[name]=(HERE/name).read_bytes()
        compile(files[name],name,'exec')
    if dry_run:return dict(dry_run=True,target=str(target),files=list(files))
    backup=Path(tempfile.mkdtemp(prefix='backup-before-emg-timing-',dir=target.parent))
    for name in files:shutil.copy2(target/name,backup/name)
    try:
        for name,data in files.items():
            tmp=target/(name+'.timing.tmp');tmp.write_bytes(data);os.replace(tmp,target/name)
    except Exception:
        for name in files:shutil.copy2(backup/name,target/name)
        raise
    return dict(backup=str(backup),target=str(target),files=list(files),base_commit=BASE_COMMIT)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--target',required=True);parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args();print(json.dumps(install(args.target,args.dry_run),ensure_ascii=False,indent=2))
