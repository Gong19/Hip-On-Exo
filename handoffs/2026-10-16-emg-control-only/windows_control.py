"""Attach control-only commands to the existing waveform GUI, on its Qt thread."""
from concurrent.futures import Future
import copy
import json
import os
from pathlib import Path
import queue
import threading
import uuid

from control_protocol import ControlServer, stamp


class GuiControl:
    def __init__(self, window, namespace, token, records_root, port=8766):
        self.window, self.ns = window, namespace
        self.root = Path(records_root)
        self.commands = queue.Queue(maxsize=8)
        self.lock = threading.Lock()
        self.cached = {}
        self.run = None
        self.completed = {}
        self.last_pulse = None
        self.closed = False
        self.sync_error = ''
        self.server = None
        # Qt classes are obtained from the original GUI's binding.
        if namespace['QT_BINDING'] == 'PyQt5':
            from PyQt5.QtWidgets import QToolBar, QCheckBox, QLabel
        else:
            from PySide6.QtWidgets import QToolBar, QCheckBox, QLabel
        toolbar = QToolBar('Jetson 同步', window)
        window.addToolBar(toolbar)
        self.allow = QCheckBox('允许 Jetson 联动开始/停止（数据仅保存在 Windows）')
        self.allow.setChecked(True)
        self.label = QLabel('同步服务初始化')
        toolbar.addWidget(self.allow)
        toolbar.addWidget(self.label)
        window.sync_pulse_signal.connect(self.pulse)
        self.refresh()
        self.server = ControlServer(token, self.dispatch, self.snapshot, port=port)
        self.timer = namespace['QTimer'](window)
        self.timer.timeout.connect(self.tick)
        self.timer.start(200)
        namespace['QApplication'].instance().aboutToQuit.connect(self.close)

    def pulse(self):
        self.last_pulse = stamp()

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.cached)

    def refresh(self):
        w = self.window
        manager = getattr(w, 'plotting_manager', None)
        saving = bool(w.is_streaming and manager and manager.streaming_mode == 'saving')
        saver = getattr(w, 'file_save_manager', None)
        status = dict(connected=False, ready=bool(self.allow.isChecked() and w.start_btn.isEnabled() and not saving),
                      recording=saving, allow_remote=self.allow.isChecked(),
                      run_id=self.run['run_id'] if self.run else None,
                      state='RECORDING' if saving else 'READY' if w.start_btn.isEnabled() else 'NOT_READY',
                      simulated=bool(self.ns.get('EMG_SIMULATION_MODE', False)),
                      data_transport=False, local_saving_enabled=bool(self.ns['ENABLE_FILE_SAVING']),
                      record_path=str(getattr(saver, 'envelope_file_path', '')),
                      saved_rows=int(getattr(saver, 'envelope_row_counter', 0)),
                      save_ok=bool(getattr(saver, 'last_final_save_success', True)), sync_error=self.sync_error)
        if self.server:
            status['connected'] = self.server.connected
        with self.lock:
            self.cached = status
        self.label.setText('  TCP %s | Jetson %s | %s' %
                           (self.server.port if self.server else 8766,
                            '已连接' if status['connected'] else '未连接', status['state']))

    def dispatch(self, message):
        future = Future()
        self.commands.put_nowait((message, future))
        try:
            return future.result(80)
        except TimeoutError:
            # A timed-out, not-yet-executed request must never start later.
            future.cancel()
            raise TimeoutError('GUI command outcome unknown; reconnect and query status before retrying')

    def tick(self):
        try:
            message, future = self.commands.get_nowait()
        except queue.Empty:
            pass
        else:
            if future.set_running_or_notify_cancel():
                try:
                    future.set_result(self.execute(message))
                except Exception as exc:
                    future.set_exception(exc)
        # A tiny time-anchor sidecar only; no waveform access or transmission.
        if self.run and not self.run.get('sample_timebase'):
            processor = getattr(self.window.plotting_manager, 'preprocessor', None)
            if processor and processor.data_lock.acquire(blocking=False):
                try:
                    base = getattr(processor, 'save_time_base', None)
                    if base is not None:
                        self.run['sample_timebase'] = dict(device_seconds=float(base),
                            label_zero_perf_ns=getattr(processor, 'label_zero_perf_ns', None),
                            actual_sync_start_perf_ns=getattr(processor, 'actual_sync_start_perf_ns', None),
                            observed_at=stamp(), method='original_GUI_save_time_base',
                            physical_sample_delay_ns=None)
                finally:
                    processor.data_lock.release()
                if self.run.get('sample_timebase'):
                    try:
                        self.persist()
                    except OSError as exc:
                        self.sync_error = '同步元数据保存失败：'+str(exc)
                        self.run.pop('sample_timebase', None)
        self.refresh()

    def persist(self):
        path = Path(self.run['sidecar'])
        data = json.dumps(self.run, ensure_ascii=False, indent=2, allow_nan=False)
        temp = path.with_suffix('.tmp')
        with temp.open('w', encoding='utf-8') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)

    def execute(self, message):
        kind = message['type']
        run_id = str(uuid.UUID(message.get('run_id', '')))
        self.refresh()
        if kind == 'start':
            if self.run and self.run['run_id'] == run_id:
                return dict(status=self.snapshot(), run_id=run_id, already_started=True)
            if run_id in self.completed:
                raise ValueError('This run has already ended; use a new run ID')
            if self.run or not self.snapshot()['ready']:
                raise ValueError('Windows not ready: scan sensors and enable remote control first')
            self.root.mkdir(parents=True, exist_ok=True)
            sidecar = self.root / ('sync_'+run_id+'.json')
            if sidecar.exists():
                raise ValueError('Run ID already exists on disk; inspect it before retrying')
            self.run = dict(protocol='hipexo-emg-control/1', run_id=run_id,
                sidecar=str(sidecar), command_received=message['received'],
                ubuntu_request=message.get('ubuntu_request'), clock_sync=message.get('clock_sync'),
                ubuntu_record_path=message.get('ubuntu_record_path'),
                state='START_REQUESTED', data_transport=False,
                timestamp_note='Software command/pulse timestamps; not hardware sample synchronization',
                windows_data_format='Original GUI envelope_data.csv (processed data, not raw SDK samples)')
            try:
                self.persist()  # Verify the metadata destination before starting hardware.
                self.last_pulse = None
                self.run['gui_start_begin'] = stamp()
                self.window.on_start()
                self.run['gui_start_returned'] = stamp()
                self.run['gui_sync_pulse'] = self.last_pulse
                self.refresh()
                if not self.snapshot()['recording']:
                    self.run['state'] = 'START_FAILED'
                    self.persist()
                    self.completed[run_id] = self.run
                    self.run = None
                    raise RuntimeError('Original GUI did not enter saving mode; check its status bar')
                self.run['state'] = 'RECORDING'
                self.run['windows_record_path'] = self.snapshot()['record_path']
                self.persist()
                return dict(run_id=run_id, status=self.snapshot(), gui_sync_pulse=self.last_pulse,
                            gui_start_returned=self.run['gui_start_returned'], sidecar=str(sidecar))
            except Exception:
                # Keep any running acquisition and its evidence visible after an ACK failure.
                self.refresh()
                raise
        if kind == 'stop':
            if run_id in self.completed:
                result = self.completed[run_id]
                if result.get('state') != 'STOPPED':
                    raise RuntimeError('Previous stop/save failed; inspect Windows')
                return dict(run_id=run_id, stopped=True, already_stopped=True, status=self.snapshot())
            if not self.run or self.run['run_id'] != run_id:
                raise ValueError('Unknown run ID; query Windows status')
            if not self.allow.isChecked():
                raise ValueError('Remote control disabled; stop on Windows')
            self.run['stop_received'] = message['received']
            success = self.window.on_stop()  # wrapped local/remote stop writes the same receipt
            if not success:
                raise RuntimeError('Windows stop/final CSV save failed; inspect Windows')
            return dict(run_id=run_id, stopped=True, status=self.snapshot())
        if kind == 'mark':
            if not self.run or self.run['run_id'] != run_id:
                raise ValueError('Unknown run ID')
            # Bounded summary, never append per-sample or unlimited history.
            self.run['last_clock_check'] = dict(received=message['received'], clock_sync=message.get('clock_sync'))
            self.persist()
            return dict(run_id=run_id)
        raise ValueError('Unknown command')

    def after_local_stop(self, success):
        if self.run:
            self.run['stop_returned'] = stamp()
            self.run['state'] = 'STOPPED' if success else 'STOP_OR_SAVE_FAILED'
            self.run['save_ok'] = bool(success)
            self.persist()
            if success:
                self.completed[self.run['run_id']] = self.run
                while len(self.completed) > 32:
                    del self.completed[next(iter(self.completed))]
                self.run = None
        self.refresh()

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.timer.stop()
        self.server.close()


def install(namespace, token, records_root, port=8766):
    cls = namespace['EMGApplication']
    original_init, original_stop, original_close = cls.__init__, cls.on_stop, cls.closeEvent

    def init(window, *args, **kwargs):
        original_init(window, *args, **kwargs)
        window.setWindowTitle(window.windowTitle() + ' [Windows本地保存 / Jetson仅同步]')
        window._jetson_control = GuiControl(window, namespace, token, records_root, port)

    def stop(window):
        result = original_stop(window)
        adapter = getattr(window, '_jetson_control', None)
        if adapter:
            try:
                adapter.after_local_stop(result)
            except OSError as exc:
                adapter.sync_error = '同步收尾元数据保存失败：'+str(exc)
                adapter.refresh()
                print(adapter.sync_error, flush=True)
                return False
        return result

    def close_event(window, event):
        original_close(window, event)
        adapter = getattr(window, '_jetson_control', None)
        if event.isAccepted() and adapter:
            try:
                adapter.after_local_stop(window.file_save_manager.last_final_save_success)
            except OSError as exc:
                print('同步收尾元数据保存失败：'+str(exc), flush=True)
            adapter.close()

    cls.__init__, cls.on_stop = init, stop
    cls.closeEvent = close_event
