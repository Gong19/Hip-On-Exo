"""End-to-end test with the actual original waveform application, SIMULATED only."""
import os
os.environ['EMG_SIMULATION_MODE'] = '1'
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
import json
from pathlib import Path
import sys
import threading
import time
import traceback
import uuid
from start_waveform_sync import load_runtime
from windows_control import install
from control_protocol import ControlClient, stamp

ROOT = Path(__file__).resolve().parents[2]/'work'/('waveform-simulation-'+uuid.uuid4().hex[:8])
ROOT.mkdir(parents=True)
ns = load_runtime(ROOT)
assert ns['EMG_SIMULATION_MODE'] is True
assert ns['ENABLE_FILE_SAVING'] is True
os.chdir(ROOT)  # Any simulator auxiliary files stay in the test directory.
ns['_configure_qt_application_attributes']()
app = ns['QApplication']([])
install(ns, 'simulation-test-only', ROOT, port=0)
window = ns['EMGApplication']()
window.on_connect()
window.on_scan()
window._jetson_control.refresh()
outcome = {}


def run():
    client = ControlClient('127.0.0.1', window._jetson_control.server.port, 'simulation-test-only')
    try:
        status = client.connect()
        assert status['simulated'] and status['ready'], status
        clock = client.synchronize()
        run_id = str(uuid.uuid4())
        ack = client.request('start', run_id=run_id, clock_sync=clock, ubuntu_request=stamp())
        assert ack['status']['recording'], ack
        time.sleep(2.5)
        client.close()
        time.sleep(.5)
        status = client.connect()
        assert status['recording'] and status['run_id']==run_id, status
        stopped = client.request('stop', run_id=run_id)
        csv = Path(ack['status']['record_path'])
        text = csv.read_text(encoding='utf-8-sig')
        assert csv.stat().st_size > 100 and len(text.splitlines())>10, text[:500]
        sidecar = json.loads((ROOT/('sync_'+run_id+'.json')).read_text(encoding='utf-8'))
        assert sidecar['state']=='STOPPED' and sidecar['sample_timebase'], sidecar
        outcome.update(passed=True, simulated=True, original_GUI=True, csv_bytes=csv.stat().st_size,
                       csv_lines=len(text.splitlines()), start_ack=ack, stop_ack=stopped,
                       no_sample_transport=True, disconnect_continues=True, evidence_dir=str(ROOT))
    except Exception:
        outcome.update(passed=False, error=traceback.format_exc(), evidence_dir=str(ROOT))
    finally:
        client.close()


thread = threading.Thread(target=run)
thread.start()
deadline = time.monotonic()+100
while thread.is_alive() and time.monotonic()<deadline:
    app.processEvents()
    time.sleep(.005)
if thread.is_alive():
    outcome.update(passed=False, error='Simulation validation timeout')
if window.is_streaming:
    window.on_stop()
window._jetson_control.close()
(ROOT/'validation.json').write_text(json.dumps(outcome, ensure_ascii=False, indent=2), encoding='utf-8')
print('CONTROL_ONLY_VALIDATION '+json.dumps(outcome, ensure_ascii=False), flush=True)
sys.exit(0 if outcome.get('passed') else 1)
