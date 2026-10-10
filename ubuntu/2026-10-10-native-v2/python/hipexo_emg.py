"""HiPExo EMG worker and panel. Delsys SDK loads only on an explicit connection."""
import ast
from collections import deque
import copy
import json
import math
import os
from pathlib import Path
import queue
import sys
import threading
import time
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

MUSCLES = ('ECRL', 'ED', 'ECU', 'FCU', 'FCR', 'PT', 'SP')
SENSOR_IDS = (57614, 57569, 57566, 57589, 57586, 57643, 56683)
HERE = Path(__file__).resolve().parent
CONFIG_PATH = HERE / 'emg_config.json'
DEFAULT_CONFIG = {'source': 'windows', 'remote_host':'', 'remote_port':8765, 'remote_token':'',
                  'sdk_dll': str(HERE / 'resources' / 'DelsysAPI.dll'),
                  'credentials_file': str(HERE / 'emg_license.local.json'),
                  'mode': '', 'sensor_ids': list(SENSOR_IDS)}


def read_credentials(path):
    """Read literal values only; never execute the vendor example just for credentials."""
    path = Path(path)
    text = path.read_text(encoding='utf-8-sig')
    if path.suffix == '.json':
        record = json.loads(text)
    else:
        record = {}
        for node in ast.parse(text).body:
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if isinstance(target, ast.Name) and target.id in ('key', 'license'):
                        record[target.id] = ast.literal_eval(node.value)
    if not all(isinstance(record.get(k), str) and record[k].strip() for k in ('key', 'license')):
        raise ValueError('Credential file must contain non-empty key and license strings')
    return record['key'], record['license']


class DelsysSource:
    def __init__(self, config, cancel):
        self.config, self.cancel, self.api = config, cancel, None
        self.channels, self.guids = [], [None] * 7
        self.started = False

    def connect(self):
        dll = Path(self.config['sdk_dll']).expanduser().resolve()
        if not dll.is_file():
            raise FileNotFoundError('DelsysAPI.dll not found. Set SDK DLL and credentials in EMG Settings, or choose Simulation.')
        key, license_value = read_credentials(self.config['credentials_file'])
        try:
            if 'clr' not in sys.modules:
                from pythonnet import load
                load('coreclr')
            import clr
            if str(dll.parent) not in sys.path:
                sys.path.append(str(dll.parent))
            clr.AddReference(str(dll))
            clr.AddReference('System.Collections')
            from Aero import AeroPy
        except Exception as exc:
            raise RuntimeError(f'Delsys runtime unavailable on this machine: {type(exc).__name__}. Check SDK architecture, .NET and pythonnet.') from exc
        self.api = AeroPy()
        self.api.ValidateBase(key, license_value)
        task = self.api.ScanSensors()
        deadline = time.perf_counter() + 20
        while not bool(task.Wait(100)):
            if self.cancel.is_set():
                raise RuntimeError('EMG connection cancelled')
            if time.perf_counter() > deadline:
                raise TimeoutError('Delsys sensor scan exceeded 20 seconds; retry connection')
        if self.cancel.is_set():
            raise RuntimeError('EMG connection cancelled')
        sensors = list(self.api.GetScannedSensorsFound())
        sid_to_slot = {int(sid): i for i, sid in enumerate(self.config['sensor_ids'])}
        selected = []
        seen_sids = set()
        for index, sensor in enumerate(sensors):
            sid = int(sensor.Properties.Sid)
            if sid in sid_to_slot:
                if sid in seen_sids:
                    raise ValueError(f'Duplicate registered sensor ID: {sid}')
                seen_sids.add(sid)
                self.api.SelectSensor(index)
                if self.config.get('mode'):
                    modes = [str(m) for m in self.api.AvailableSensorModes(index)]
                    if self.config['mode'] not in modes:
                        raise ValueError(f'Sensor {sid} does not support requested mode; keep mode empty to use current mode')
                    self.api.SetSampleMode(index, self.config['mode'])
                selected.append((index, sid, sid_to_slot[sid]))
        if not selected:
            raise RuntimeError('No registered EMG sensors found. Check sensor IDs in EMG Settings.')
        self.api.Configure(False, False)
        if not self.api.IsPipelineConfigured():
            raise RuntimeError('Delsys pipeline could not be configured')
        self.channels = [dict(sid=int(sid), present=False, mode='missing', sample_rate=0.,
                              is_rms=False, battery=None, muscle=MUSCLES[i])
                         for i, sid in enumerate(self.config['sensor_ids'])]
        for index, sid, slot in selected:
            sensor = self.api.GetSensorObject(index)
            mode = str(sensor.Configuration.ModeString)
            emg = [c for c in sensor.TrignoChannels if bool(c.IsEnabled) and
                   (str(c.Type).upper() == 'EMG' or 'emg' in str(c.Name).lower())]
            if len(emg) > 1:
                raise ValueError(f'Sensor {sid} has multiple enabled EMG channels; choose an unambiguous mode')
            if not emg:
                continue
            ch = emg[0]
            self.guids[slot] = ch.Id
            self.channels[slot].update(present=True, mode=mode, sample_rate=float(ch.SampleRate),
                                       is_rms='RMS' in (mode + ' ' + str(ch.Name)).upper(),
                                       battery=float(sensor.Properties.BatteryPercent) * 100)
        if not any(c['present'] for c in self.channels):
            raise RuntimeError('Registered sensors have no enabled EMG channels')
        return self.channels

    def start(self):
        self.started = True  # cleanup also attempts Stop if Start partially fails
        self.api.Start(True)

    def poll(self):
        if not self.api.CheckYTDataQueue():
            return {}
        data = self.api.PollYTData()
        frames = {}
        for i, guid in enumerate(self.guids):
            if guid is None:
                continue
            contains = data.ContainsKey(guid) if hasattr(data, 'ContainsKey') else guid in data
            if not contains:
                continue
            times, values = [], []
            for sample in data[guid]:
                try:
                    times.append(float(sample.Item1))
                    values.append(float(sample.Item2))
                except (AttributeError, TypeError, ValueError):
                    if len(times) > len(values):
                        times.pop()
            frames[i] = (times, values)
        return frames

    def stop(self):
        if self.api and self.started:
            self.api.Stop()
            self.started = False

    def close(self):
        self.stop()
        if self.api and str(self.api.GetPipelineState()) == 'Armed':
            self.api.ResetPipeline()


class SimulatedSource:
    """Deterministic RMS-like test input, explicitly marked in UI and saved data."""
    def __init__(self, config, cancel):
        self.config, self.cancel = config, cancel
        self.count, self.start_perf = 0, None
        self.channels = [dict(sid=int(sid), muscle=MUSCLES[i], present=True,
                              mode='SIMULATED RMS 148.148 Hz', sample_rate=4000/27,
                              is_rms=True, battery=100.)
                         for i, sid in enumerate(config['sensor_ids'])]

    def connect(self):
        return self.channels

    def start(self):
        self.count, self.start_perf = 0, time.perf_counter()

    def poll(self):
        elapsed = time.perf_counter() - self.start_perf
        available = int(elapsed * (4000 / 27))
        end = min(available, self.count + 256)
        if end <= self.count:
            return {}
        times = [i / (4000 / 27) for i in range(self.count, end)]
        self.count = end
        return {ch: (times, [0.0001 + .001 * (0.5 + 0.5 * math.sin(2 * math.pi * .7 * t + ch * .3))
                            for t in times]) for ch in range(7)}

    def stop(self):
        pass

    def close(self):
        pass


class EmgWorker(QtCore.QObject):
    sig_status = QtCore.pyqtSignal(str)
    sig_state = QtCore.pyqtSignal(str)
    sig_configured = QtCore.pyqtSignal()
    sig_remote_status = QtCore.pyqtSignal(object)

    def __init__(self, data_manager, parent=None):
        super().__init__(parent)
        self._dm = data_manager
        self._lock = threading.RLock()
        self._commands = queue.Queue()
        self._cancel = threading.Event()
        self._exit = threading.Event()
        self._thread = None
        self.source = self.processor = None
        self.state = 'IDLE'
        self.config = copy.deepcopy(DEFAULT_CONFIG)
        try:
            if CONFIG_PATH.exists():
                self.config.update(json.loads(CONFIG_PATH.read_text()))
        except Exception as exc:
            self.config_error = f'Could not read EMG settings: {exc}'
        else:
            self.config_error = None
        try:
            self.config = self.validate_config(self.config)
        except (TypeError, ValueError) as exc:
            self.config_error = f'Invalid EMG settings; defaults restored: {exc}'
            self.config = copy.deepcopy(DEFAULT_CONFIG)
        self.history = [deque(maxlen=5000) for _ in range(7)]
        self.epoch = 0
        self._clock_offset_ns = None
        self._wall_minus_perf_ns = 0
        self._last_notice = ''
        self._last_source_perf = None
        self._stream_start_perf = None
        self._stale_notified = False
        self.session_id = 0
        self.remote_status = {'connected':False,'ready':False,'synced':False,'streaming':False}

    @staticmethod
    def validate_config(config):
        config = copy.deepcopy(config)
        if config.get('source') not in ('delsys', 'simulation', 'windows'):
            raise ValueError('EMG source must be windows, delsys or simulation')
        config['remote_port'] = int(config.get('remote_port',8765))
        if not 1 <= config['remote_port'] <= 65535:
            raise ValueError('Windows TCP port must be between 1 and 65535')
        ids = [int(s) for s in config.get('sensor_ids', [])]
        if len(ids) != 7 or len(set(ids)) != 7 or any(i <= 0 for i in ids):
            raise ValueError('Enter seven distinct positive sensor IDs in muscle order')
        config['sensor_ids'] = ids
        return config

    def _set_state(self, state):
        self.state = state
        self.sig_state.emit(state)

    def _submit(self, command):
        if self._exit.is_set():
            return
        if self._thread is None:
            self._thread = threading.Thread(target=self._loop, name='emg-worker', daemon=True)
            self._thread.start()
        self._commands.put(command)

    def connect_source(self):
        if self.state in ('IDLE', 'ERROR', 'READY'):
            self._cancel.clear()
            self._set_state('CONNECTING')
            self._submit('connect')

    def start(self):
        if self.state in ('RUNNING', 'CONNECTING', 'STARTING', 'STOPPING', 'SYNCING'):
            return
        self._cancel.clear()
        self._set_state('STARTING')
        self._submit('start')

    def sync_start(self):
        if self.config['source'] != 'windows' or self.state not in ('IDLE','READY','ERROR'):
            return
        self._cancel.clear()
        self._set_state('SYNCING')
        self._submit('sync_start')

    def _publish_remote(self):
        status = dict(getattr(self.source,'status',{}))
        status.setdefault('connected',False)
        if status != self.remote_status:
            self.remote_status = status
            self.sig_remote_status.emit(status)

    def stop(self):
        if self.state in ('RUNNING', 'CONNECTING', 'STARTING', 'SYNCING'):
            self._cancel.set()
            self._set_state('STOPPING')
            self._submit('stop')

    def shutdown(self):
        self._cancel.set()
        self._exit.set()
        if self._thread:
            self._thread.join(3.)
            return not self._thread.is_alive()
        return True

    def _connect(self):
        if getattr(self, "simulation_only", False):
            self.config["source"] = "simulation"
        if self.source:
            self.source.close()
        # Import numerical dependencies only when the user requests EMG.
        from hipexo_emg_core import EmgProcessor
        if self.config['source'] == 'windows':
            from hipexo_emg_remote import RemoteWindowsSource
            self.source = RemoteWindowsSource(self.config,self._cancel,self._dm.export_dir,
                                               session=getattr(self._dm,'session',None))
        else:
            factory = SimulatedSource if self.config['source'] == 'simulation' else DelsysSource
            self.source = factory(self.config, self._cancel)
        channels = self.source.connect()
        processor = EmgProcessor(channels, self.config['source'])
        with self._lock:
            previous = self.processor
            if previous and previous.fingerprint == processor.fingerprint:
                processor.load_calibration(previous.calibration_dict())
            self.processor = processor
            for buffer in self.history:
                buffer.clear()
            self.epoch += 1
        self.sig_configured.emit()
        count = sum(c['present'] for c in channels)
        self.sig_status.emit(f'{self.config["source"].upper()}: {count}/7 channels ready; missing slots remain invalid')
        self._set_state('READY')
        self._publish_remote()

    def _loop(self):
        try:
            while not self._exit.is_set():
                try:
                    cmd = self._commands.get(timeout=.004 if self.state == 'RUNNING' else .05)
                except queue.Empty:
                    cmd = None
                try:
                    if cmd == 'disconnect':
                        if self.source:
                            self.source.close()
                        self.source = None
                        self._publish_remote()
                    elif cmd == 'connect':
                        self._connect()
                    elif cmd in ('start','sync_start'):
                        if self.source is None or self.processor is None:
                            self._connect()
                        if self._cancel.is_set():
                            continue
                        if cmd == 'sync_start':
                            self._set_state('SYNCING')
                            self.source.synchronize()
                        with self._lock:
                            self.processor.reset_stream()
                            for buffer in self.history:
                                buffer.clear()
                            self.epoch += 1
                        self._clock_offset_ns = None
                        self._last_source_perf = None
                        self._stream_start_perf = time.perf_counter()
                        self._stale_notified = False
                        self.session_id += 1
                        self.source.start()
                        if hasattr(self.source,'mapped_time'):
                            self._clock_offset_ns = 0
                        self._set_state('RUNNING')
                        self._publish_remote()
                    elif cmd == 'stop':
                        if self.source:
                            self.source.stop()
                            for frames in getattr(self.source,'drain_pending',lambda:[])():
                                with self._lock:
                                    rows = self.processor.ingest(frames)
                                self._store(rows)
                        with self._lock:
                            if self.processor:
                                self.processor.job = None
                                self._store(self.processor.drain(force=True))
                        self._set_state('READY' if self.processor else 'IDLE')
                        self._cancel.clear()
                        self._publish_remote()
                        self.sig_status.emit('EMG stopped')
                    if self.state != 'RUNNING' or self._cancel.is_set():
                        if self.state == 'READY' and hasattr(self.source,'idle'):
                            self.source.idle()
                            self._publish_remote()
                        continue
                    frames = self.source.poll()
                    self._publish_remote()
                    now = time.perf_counter()
                    with self._lock:
                        if frames and any(len(pair[0]) for pair in frames.values()):
                            self._last_source_perf = now
                            if self._stale_notified:
                                self.sig_status.emit('EMG source resumed')
                                self._stale_notified = False
                            if hasattr(self.source,'mapped_time'):
                                self._clock_offset_ns = 0
                            elif self._clock_offset_ns is None:
                                latest = max(max(pair[0]) for pair in frames.values() if len(pair[0]))
                                perf = time.perf_counter_ns()
                                self._clock_offset_ns = perf - round(latest * 1e9)
                                self._wall_minus_perf_ns = time.time_ns() - perf
                        rows = self.processor.ingest(frames, now)
                        self.processor.finish_calibration(now)
                        notice = self.processor.last_calibration_message
                    self._store(rows)  # CSV/ring work must not hold the UI snapshot lock
                    if notice and notice != self._last_notice:
                        self._last_notice = notice
                        self.sig_status.emit(notice)
                    last_data = self._last_source_perf or self._stream_start_perf
                    if last_data and now - last_data > .5 and not self._stale_notified:
                        self._stale_notified = True
                        self.sig_status.emit('EMG data stale: no recent YT samples; recording contains a gap')
                except InterruptedError:
                    if self._cancel.is_set() or self._exit.is_set():
                        continue  # process the queued Stop; preserve buffered final packets
                    raise
                except Exception as exc:
                    # Buffered processed rows may precede the detected clock fault;
                    # retain raw evidence and do not publish them with an invalid map.
                    fault = dict(getattr(self.source,'status',{}))
                    if self.processor and self.source and not fault.get('clock_fault'):
                        try:
                            for frames in getattr(self.source,'drain_pending',lambda:[])():
                                self._store(self.processor.ingest(frames))
                            self._store(self.processor.drain(force=True))
                        except Exception as tail_error:
                            self.sig_status.emit(f'EMG tail drain failed: {tail_error}')
                    self._clock_offset_ns = None
                    try:
                        if self.source:
                            self.source.close()
                    except Exception:
                        pass
                    self.source = None
                    self._publish_remote()
                    if fault.get('clock_fault'):
                        self.remote_status = dict(fault,connected=False,ready=False,synced=False,streaming=False)
                        self.sig_remote_status.emit(self.remote_status)
                    self._set_state('ERROR')
                    self.sig_status.emit(f'EMG: {exc}')
        finally:
            try:
                if self.source:
                    self.source.stop()
                    for frames in getattr(self.source,'drain_pending',lambda:[])():
                        with self._lock:
                            rows = self.processor.ingest(frames)
                        self._store(rows)
                with self._lock:
                    if self.processor:
                        self.processor.job = None
                        self._store(self.processor.drain(force=True))
            except Exception as exc:
                self.sig_status.emit(f'EMG close: {exc}')
            finally:
                try:
                    if self.source:
                        self.source.close()
                except Exception as exc:
                    self.sig_status.emit(f'EMG disconnect: {exc}')
                self._publish_remote()

    def _store(self, rows):
        if not rows or self._clock_offset_ns is None:
            return
        origin = self.processor.origin
        times, monos, batches = [], [], {i: [] for i in range(7)}
        display_rows = [[] for _ in range(7)]
        for row in rows:
            source_s = origin + row['label_ms'] / 1000
            if hasattr(self.source,'mapped_time'):
                wall_ns, mono, _ = self.source.mapped_time(source_s)
                wall_ms = wall_ns / 1e6
            else:
                mono = self._clock_offset_ns + round(source_s * 1e9)
                wall_ms = (mono + self._wall_minus_perf_ns) / 1e6
            times.append(wall_ms)
            monos.append(mono)
            for i in range(7):
                values = {k: float(row[k][i]) for k in
                          ('raw_v', 'uncalibrated_v', 'envelope_v', 'mvc_ratio', 'baseline_v', 'mvc_v')}
                values.update(valid=int(row['valid'][i]), source_time_s=source_s,
                              simulated=int(self.config['source'] == 'simulation'),
                              sensor_id=self.config['sensor_ids'][i], session_id=self.session_id,
                              is_rms=int(self.processor.channels[i]['is_rms']),
                              source_sample_rate_hz=self.processor.channels[i].get('sample_rate',0),
                              filter_description="anti-alias-v1",
                              sdk_unit=self.processor.channels[i].get('sdk_unit','unverified'),
                              scale_to_v=self.processor.channels[i].get('scale_to_v'),
                              value_units=self.processor.channels[i].get('value_units','unverified'))
                if hasattr(self.source,'sample_metadata'):
                    values.update(self.source.sample_metadata(source_s))
                    values['simulated'] = values['remote_simulated']
                batches[i].append(values)
                display_rows[i].append((mono / 1e9, values['raw_v'], values['envelope_v'],
                                        values['mvc_ratio'], values['valid']))
        with self._lock:
            for i in range(7):
                self.history[i].extend(display_rows[i])
        for i in range(7):
            self._dm.append_batch(f'emg_{i}', times, batches[i], monos)

    def snapshot(self):
        with self._lock:
            now = time.perf_counter()
            channels = copy.deepcopy(self.processor.channels) if self.processor else []
            history = [list(b) for b in self.history]
            stale = ([t is None or now - t > .5 for t in self.processor.last_seen]
                     if self.processor else [True] * 7)
            return history, channels, stale, self.epoch

    def calibrate(self, kind, dof=0):
        with self._lock:
            if self.state != 'RUNNING' or not self.processor:
                raise ValueError('Start EMG acquisition before calibration')
            self.processor.begin_calibration(kind, dof)
            self.sig_status.emit(self.processor.last_calibration_message)

    def cancel_calibration(self):
        with self._lock:
            if self.processor:
                self.processor.job = None
                self.processor.last_calibration_message = 'Calibration cancelled; previous values retained'

    def save_calibration(self, path):
        with self._lock:
            if not self.processor:
                raise ValueError('Connect EMG first')
            record = self.processor.calibration_dict()
        Path(path).write_text(json.dumps(record, indent=2), encoding='utf-8')

    def load_calibration(self, path):
        record = json.loads(Path(path).read_text(encoding='utf-8'))
        with self._lock:
            if not self.processor:
                raise ValueError('Connect EMG first so sensor identity and mode can be checked')
            self.processor.load_calibration(record)
        self.sig_status.emit('Matching baseline and MVC loaded together')

    def configure(self, config):
        if self.state not in ('IDLE', 'READY', 'ERROR'):
            raise ValueError('Stop EMG before changing source or settings')
        config = self.validate_config(config)
        if getattr(self, "simulation_only", False) and config["source"] != "simulation":
            raise ValueError("Offline preview only permits simulated EMG")
        # Close via the worker's serialized command loop on the next connect.
        self.config = config
        # Dispose the previous idle connection in the worker thread.
        self._submit('disconnect')
        with self._lock:
            self.processor = None
            for buffer in self.history:
                buffer.clear()
        if not getattr(self, "simulation_only", False):
            CONFIG_PATH.write_text(json.dumps(config, indent=2), encoding='utf-8')
            CONFIG_PATH.chmod(0o600)  # remote connection token is local configuration
        self._set_state('IDLE')


class EmgSettingsDialog(QtWidgets.QDialog):
    def __init__(self, config, parent=None):
        super().__init__(parent)
        self.setWindowTitle('EMG Settings')
        self.resize(650, 350)
        layout = QtWidgets.QFormLayout(self)
        self.fields = {}
        for key, title in [('remote_host','Windows IP / hostname'), ('remote_port','Windows TCP port'),
                           ('remote_token','Windows connection token'),
                           ('sdk_dll', 'Local Delsys SDK DLL'), ('credentials_file', 'Local credentials JSON / Python'),
                           ('mode', 'Exact sensor mode (empty = current)'), ('sensor_ids', 'Sensor IDs: ECRL, ED, ECU, FCU, FCR, PT, SP')]:
            value = ', '.join(map(str, config[key])) if key == 'sensor_ids' else config.get(key, '')
            edit = QtWidgets.QLineEdit(str(value))
            if key == 'remote_token':
                edit.setEchoMode(QtWidgets.QLineEdit.Password)
            self.fields[key] = edit
            layout.addRow(title, edit)
            if key in ('sdk_dll', 'credentials_file'):
                button = QtWidgets.QPushButton('Browse…')
                button.clicked.connect(lambda checked=False, e=edit: self._browse(e))
                layout.addRow('', button)
        note = QtWidgets.QLabel('Windows mode: open and arm the bridge on Windows first; use the same token.\nDelsys SDK and license stay on Windows. Local DLL fields apply only to Local Delsys.\nChanging source, sensor IDs or mode clears calibration.')
        note.setWordWrap(True)
        layout.addRow(note)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

    def _browse(self, edit):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Select file', edit.text())
        if path:
            edit.setText(path)

    def values(self, config):
        result = dict(config)
        result.update({key: widget.text().strip() for key, widget in self.fields.items()})
        result['sensor_ids'] = [int(part.strip()) for part in result['sensor_ids'].split(',')]
        return result


class EmgPanel(QtWidgets.QWidget):
    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self.worker = worker
        self._theme = 'light'
        root = QtWidgets.QVBoxLayout(self)
        bar = QtWidgets.QHBoxLayout()
        self.source = QtWidgets.QComboBox()
        self.source_keys = ('windows','delsys','simulation')
        self.source.addItems(['Windows PC (remote EMG)', 'Local Delsys hardware', 'Simulation (no hardware)'])
        self.source.setCurrentIndex(self.source_keys.index(worker.config['source']))
        bar.addWidget(self.source)
        self.connect_btn = QtWidgets.QPushButton('Connect / Scan')
        self.start_btn = QtWidgets.QPushButton('Start EMG')
        self.stop_btn = QtWidgets.QPushButton('Stop EMG')
        self.settings_btn = QtWidgets.QPushButton('Settings')
        for button in (self.connect_btn, self.start_btn, self.stop_btn, self.settings_btn):
            bar.addWidget(button)
        root.addLayout(bar)
        sync_bar=QtWidgets.QHBoxLayout()
        self.sync_btn=QtWidgets.QPushButton('同步并开始 / Sync && Start')
        self.sync_btn.setToolTip('Connect to the armed Windows bridge, estimate clock offset, then start remote recording and transfer.')
        self.remote_label=QtWidgets.QLabel('Windows: 未连接 / DISCONNECTED')
        self.remote_label.setWordWrap(True)
        sync_bar.addWidget(self.sync_btn)
        sync_bar.addWidget(self.remote_label,1)
        root.addLayout(sync_bar)
        self.lbl_state = QtWidgets.QLabel('State: IDLE')
        self.message = QtWidgets.QLabel(worker.config_error or 'Connect a data source. Values are volts internally; plots show mV or %MVC.')
        self.message.setWordWrap(True)
        root.addWidget(self.lbl_state)
        root.addWidget(self.message)
        controls = QtWidgets.QHBoxLayout()
        self.baseline_btn = QtWidgets.QPushButton('Baseline · 5 s')
        self.dof = QtWidgets.QComboBox()
        self.dof.addItems(['Flexion / extension', 'Radial / ulnar deviation', 'Pronation / supination'])
        self.mvc_btn = QtWidgets.QPushButton('MVC · 5 s')
        self.cancel_btn = QtWidgets.QPushButton('Cancel calibration')
        for widget in (self.baseline_btn, self.dof, self.mvc_btn, self.cancel_btn):
            controls.addWidget(widget)
        root.addLayout(controls)
        tools = QtWidgets.QHBoxLayout()
        self.view = QtWidgets.QComboBox()
        self.view.addItems(['Envelope (mV)', 'Source on 1 kHz grid (mV)', 'Activation (%MVC)'])
        tools.addWidget(self.view)
        save = QtWidgets.QPushButton('Save calibration')
        load = QtWidgets.QPushButton('Load calibration')
        tools.addWidget(save)
        tools.addWidget(load)
        root.addLayout(tools)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        container = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(container)
        self.plots, self.curves, self.labels = [], [], []
        colors = ['#2962ff', '#00897b', '#8e24aa', '#ef6c00', '#d81b60', '#039be5', '#7cb342']
        for i, muscle in enumerate(MUSCLES):
            cell = QtWidgets.QWidget()
            layout = QtWidgets.QVBoxLayout(cell)
            label = QtWidgets.QLabel(f'{i+1} · {muscle}: not connected')
            plot = pg.PlotWidget()
            plot.setMinimumHeight(130)
            plot.setLabel('bottom', 'Time relative to now', units='s')
            plot.showGrid(x=True, y=True, alpha=.2)
            curve = plot.plot(pen=pg.mkPen(colors[i], width=1.5), connect='finite', antialias=False)
            layout.addWidget(label)
            layout.addWidget(plot)
            grid.addWidget(cell, i // 2, i % 2)
            self.plots.append(plot)
            self.curves.append(curve)
            self.labels.append(label)
        scroll.setWidget(container)
        root.addWidget(scroll, 1)
        worker.sig_status.connect(self.message.setText)
        worker.sig_state.connect(self._state)
        worker.sig_remote_status.connect(self._remote_state)
        self.sync_btn.clicked.connect(worker.sync_start)
        self.connect_btn.clicked.connect(worker.connect_source)
        self.start_btn.clicked.connect(worker.start)
        self.stop_btn.clicked.connect(worker.stop)
        self.settings_btn.clicked.connect(self._settings)
        self.source.currentIndexChanged.connect(self._source_changed)
        self.baseline_btn.clicked.connect(lambda: self._calibrate('baseline'))
        self.mvc_btn.clicked.connect(lambda: self._calibrate('mvc'))
        self.cancel_btn.clicked.connect(worker.cancel_calibration)
        save.clicked.connect(lambda: self._file(True))
        load.clicked.connect(lambda: self._file(False))
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(100)  # display at 10 Hz; acquisition/recording remain full-rate
        self._view_mode = None
        self._state(worker.state)
        self.set_theme('light')

    def _state(self, state):
        self.lbl_state.setText(f'State: {state} | {self.worker.config["source"].upper()}')
        busy = state in ('CONNECTING', 'STARTING', 'STOPPING', 'RUNNING', 'SYNCING')
        for widget in (self.source, self.settings_btn, self.connect_btn):
            widget.setEnabled(not busy)
        self.start_btn.setEnabled(not busy)
        self.stop_btn.setEnabled(busy and state != 'STOPPING')
        self.baseline_btn.setEnabled(state == 'RUNNING')
        self.mvc_btn.setEnabled(state == 'RUNNING')
        self.sync_btn.setEnabled(not busy and self.worker.config['source']=='windows')

    def _remote_state(self,status):
        if not status.get('connected'):
            text='Windows: 未连接 / DISCONNECTED'; color='#888'
        else:
            text='Windows: 已连接 | '+('待命 READY' if status.get('ready') else '未待命 NOT ARMED')
            color='#2E7D32' if status.get('ready') else '#EF6C00'
            if status.get('synced'):
                text+=f" | offset {status['offset_ms']:+.3f} ms | RTT {status['rtt_ms']:.3f} ms"
                text+=f" | 单向估计 {status.get('one_way_est_ms',0):.3f} ms (RTT/2)"
            if 'initial_delivery_ms' in status:
                text+=f" | 首包发送→接收估计 {status['initial_delivery_ms']:.3f} ms"
            if status.get('streaming'):
                text+=' | 正在传输 STREAMING'
            if 'drift_ms' in status:
                text+=f" | drift {status['drift_ms']:+.3f} ms"
            if status.get('simulated'):
                text+=' | SIMULATED DATA'
            if status.get('clock_quality') in ('suspect','network_uncertain'):
                text+=' | 对时待确认 '+status['clock_quality']; color='#EF6C00'
        if status.get('clock_fault'):
            text+=' | 同步失效，请重新 Sync & Start'; color='#C62828'
        self.remote_label.setText(text)
        self.remote_label.setToolTip('RTT/2 is an estimate, not device acquisition latency. Run timing file: '+status.get('timing_report','created on Sync & Start'))
        self.remote_label.setStyleSheet(f'color:{color}; font-weight:600;')

    def _source_changed(self, index):
        try:
            config = dict(self.worker.config, source=self.source_keys[index])
            self.worker.configure(config)
            self.message.setText('Source changed; calibration cleared. Connect or start EMG.')
        except Exception as exc:
            self.message.setText(str(exc))
            self.source.blockSignals(True)
            self.source.setCurrentIndex(self.source_keys.index(self.worker.config['source']))
            self.source.blockSignals(False)

    def _settings(self):
        dialog = EmgSettingsDialog(self.worker.config, self)
        if dialog.exec_() == QtWidgets.QDialog.Accepted:
            try:
                self.worker.configure(dialog.values(self.worker.config))
                self.message.setText('Settings saved; reconnect to validate hardware and modes')
            except Exception as exc:
                self.message.setText(str(exc))

    def _calibrate(self, kind):
        try:
            self.worker.calibrate(kind, self.dof.currentIndex())
            self.message.setText('Keep still for 5 seconds' if kind == 'baseline' else
                                 f'Perform {self.dof.currentText()} for 5 seconds')
        except Exception as exc:
            self.message.setText(str(exc))

    def _file(self, save):
        fn = QtWidgets.QFileDialog.getSaveFileName if save else QtWidgets.QFileDialog.getOpenFileName
        path, _ = fn(self, 'EMG calibration', '', 'Calibration JSON (*.json)')
        if not path:
            return
        try:
            if save:
                self.worker.save_calibration(path)
            else:
                self.worker.load_calibration(path)
            self.message.setText('Calibration saved' if save else 'Matching baseline and MVC loaded')
        except Exception as exc:
            self.message.setText(str(exc))

    def _refresh(self):
        if not self.isVisible():
            return
        history, channels, stale, _ = self.worker.snapshot()
        now = time.perf_counter()
        mode = self.view.currentIndex()
        value_index = (2, 1, 3)[mode]
        factor = 100 if mode == 2 else 1000
        units = '%MVC' if mode == 2 else 'mV'
        for i, (plot, curve, label) in enumerate(zip(self.plots, self.curves, self.labels)):
            samples = history[i]
            if samples:
                # Downsample only the visual path; the full-rate rows go to recording.
                samples = samples[::max(1, len(samples) // 500)]
                curve.setData([s[0] - now for s in samples],
                              [s[value_index] * factor if s[4] else float('nan') for s in samples])
            else:
                curve.clear()
            plot.setXRange(-5, 0, padding=0)
            if self._view_mode != mode:
                plot.setLabel('left', units)
                if mode == 2:
                    plot.setYRange(0, 100, padding=.05)
                else:
                    plot.enableAutoRange(axis='y', enable=True)
            if not channels:
                label.setText(f'{i+1} · {MUSCLES[i]}: not connected')
                continue
            info = channels[i]
            state = 'MISSING' if not info['present'] else ('STALE' if stale[i] else 'LIVE')
            if mode == 2 and samples and not math.isfinite(samples[-1][3]) and state == 'LIVE':
                state = 'MVC NOT CALIBRATED'
            battery = f'{info["battery"]:.0f}%' if info.get('battery') is not None else '?'
            label.setText(f'{i+1} · {MUSCLES[i]} | {state} | {info["sample_rate"]:.1f} Hz | battery {battery}')
            label.setToolTip(f'SID {info["sid"]} | {info["mode"]}')
        self._view_mode = mode

    def set_theme(self, theme):
        self._theme = theme
        bg, fg = ('#0F1115', '#C3CEE3') if theme == 'dark' else ('#FFFFFF', '#263238')
        for plot in self.plots:
            plot.setBackground(bg)
            for name in ('left', 'bottom'):
                plot.getAxis(name).setPen(fg)
                plot.getAxis(name).setTextPen(fg)
