"""Jetson EMG status/commands only: no SDK, NumPy, samples, plots or EMG CSV."""
import json
import os
from pathlib import Path
import queue
import threading
import time
import uuid
from PyQt5 import QtCore, QtWidgets
from control_protocol import ControlClient, stamp

HERE = Path(__file__).resolve().parent
CONFIG = HERE / 'emg_control_private.json'


class EmgWorker(QtCore.QObject):
    sig_status = QtCore.pyqtSignal(str)
    sig_state = QtCore.pyqtSignal(str)
    sig_configured = QtCore.pyqtSignal()
    sig_remote_status = QtCore.pyqtSignal(object)

    def __init__(self, data_manager, parent=None):
        super().__init__(parent)
        self._dm = data_manager
        self.config = dict(source='windows', remote_host='', remote_port=8766, remote_token='')
        # Preserve the existing token/host locally, but never migrate its streaming port.
        old = HERE / 'emg_config.json'
        self.config_error = ''
        try:
            path = CONFIG if CONFIG.exists() else old
            if path.exists():
                settings = json.loads(path.read_text(encoding='utf-8'))
                for key in ('remote_host', 'remote_token'):
                    self.config[key] = settings.get(key, '')
                if path == CONFIG:
                    self.config['remote_port'] = int(settings.get('remote_port', 8766))
        except Exception as exc:
            self.config_error = str(exc)
        self.enabled = False
        self.simulation_only = False
        self.processor = None  # compatibility only; never instantiated
        self.source = None
        self.state = 'IDLE'
        self.remote_status = dict(connected=False, recording=False, ready=False)
        self._commands = queue.Queue(maxsize=16)
        self._exit = threading.Event()
        self._thread = None
        self.run_id = None
        self.journal = None
        self._last_probe = 0
        self._active_config = None

    def _set_state(self, state):
        self.state = state
        self.sig_state.emit(state)

    def _submit(self, command, **fields):
        if self._exit.is_set():
            return
        if self._thread is None:
            self._thread = threading.Thread(target=self._loop, name='EMG-status-control', daemon=True)
            self._thread.start()
        try:
            self._commands.put_nowait((command, fields))
        except queue.Full:
            self.sig_status.emit('控制队列已满；等待当前指令完成')

    def connect_source(self):
        self._submit('connect', config=dict(self.config))

    def start(self):
        self.sync_start()

    def sync_start(self):
        # Explicit panel action is independent of the optional global linkage.
        self._submit('start', config=dict(self.config), export_dir=str(self._dm.export_dir),
                     ubuntu_request=stamp(), ubuntu_record_path=None)

    def record_start(self, record_path):
        if self.enabled:
            self._submit('start', config=dict(self.config), export_dir=str(self._dm.export_dir),
                         ubuntu_request=stamp(), ubuntu_record_path=str(record_path))

    def stop(self):
        self._submit('stop')

    def _log(self, kind, **fields):
        if self.journal:
            # Control events only, including sparse clock probes; no periodic status dump.
            with self.journal.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(dict(kind=kind, at=stamp(), **fields), ensure_ascii=False)+'\n')

    def _status(self, status):
        self.remote_status = dict(status, connected=True)
        self.run_id = str(uuid.UUID(status['run_id'])) if status.get('run_id') else None
        self._set_state('RUNNING' if status.get('recording') else 'READY')
        self.sig_remote_status.emit(dict(self.remote_status))

    def _connect(self, config):
        if self.simulation_only:
            raise RuntimeError('预览模式禁止连接真实 Windows；退出预览后操作')
        if self.source:
            self.source.close()
        self._active_config = dict(config)
        self.source = ControlClient(config['remote_host'], config['remote_port'], config['remote_token'])
        self._set_state('CONNECTING')
        self._status(self.source.connect())

    def _loop(self):
        next_status = 0
        while not self._exit.is_set():
            try:
                command, fields = self._commands.get(timeout=.2)
            except queue.Empty:
                command, fields = None, {}
            try:
                if command == 'disconnect':
                    if self.source:
                        self.source.close()
                    self.source = None
                    self.remote_status = dict(connected=False, ready=False, recording=None)
                    self.sig_remote_status.emit(dict(self.remote_status))
                    self._set_state('IDLE')
                elif command == 'connect':
                    self._connect(fields['config'])
                elif command == 'start':
                    if not self.source or not self.source.sock:
                        self._connect(fields['config'])
                    status = self.source.request('status')['status']
                    if status.get('recording'):
                        self._status(status)
                        if not self.run_id:
                            raise RuntimeError('Windows 正在手动采集；请先在 Windows 停止，再创建新的联动实验')
                        directory = Path(fields['export_dir'])
                        directory.mkdir(parents=True, exist_ok=True)
                        self.journal = directory / ('emg_control_'+self.run_id+'.jsonl')
                        self._log('already_recording', run_id=self.run_id,
                                  ubuntu_record_path=fields['ubuntu_record_path'], ubuntu_request=fields['ubuntu_request'])
                        self.sig_status.emit('Windows 已在采集；沿用当前运行，已记录本次 Record 关联')
                        continue
                    self._set_state('SYNCING')
                    clock = self.source.synchronize()
                    self.run_id = str(uuid.uuid4())
                    directory = Path(fields['export_dir'])
                    directory.mkdir(parents=True, exist_ok=True)
                    self.journal = directory / ('emg_control_'+self.run_id+'.jsonl')
                    self._log('start_request', run_id=self.run_id, clock_sync=clock,
                              ubuntu_request=fields['ubuntu_request'], ubuntu_record_path=fields['ubuntu_record_path'])
                    reply = self.source.request('start', run_id=self.run_id, clock_sync=clock,
                        ubuntu_request=fields['ubuntu_request'], ubuntu_record_path=fields['ubuntu_record_path'])
                    self._log('start_ack', reply=reply)
                    if reply.get('run_id') != self.run_id or reply.get('status', {}).get('recording') is not True:
                        raise RuntimeError('Windows 未确认本次运行进入保存状态；请核对 Windows 界面')
                    self._last_probe = time.monotonic()
                    self._status(reply['status'])
                    self.sig_status.emit('Windows 开始本地保存；Jetson 不接收 EMG 样本')
                elif command == 'stop':
                    if self.source and self.source.sock and self.run_id:
                        self._set_state('STOPPING')
                        self._log('stop_request', run_id=self.run_id)
                        reply = self.source.request('stop', run_id=self.run_id)
                        self._log('stop_ack', reply=reply)
                        if reply.get('stopped') is not True or reply.get('status', {}).get('recording') is not False or reply.get('status', {}).get('save_ok') is not True:
                            raise RuntimeError('Windows 未确认停止及保存成功，请在 Windows 核对')
                        self._status(reply['status'])
                        self.sig_status.emit('Windows 已停止，已返回最终保存结果')
                    elif self.run_id:
                        raise RuntimeError('连接中断；Windows 可能继续采集，请重连或在 Windows 点击停止')
                if self.source and self.source.sock and time.monotonic() >= next_status:
                    self._status(self.source.request('status')['status'])
                    next_status = time.monotonic()+2
                    if self.run_id and time.monotonic()-self._last_probe >= 60:
                        clock = self.source.synchronize(probes=3)
                        self._log('clock_check', clock_sync=clock)
                        self.source.request('mark', run_id=self.run_id, clock_sync=clock)
                        self._last_probe = time.monotonic()
            except Exception as exc:
                self._set_state('ERROR')
                self.sig_status.emit(str(exc))
                try:
                    self._log('error', message=str(exc), run_id=self.run_id)
                except OSError:
                    pass
                if self.source:
                    self.source.close()
                self.source = None
                self.remote_status = dict(connected=False, ready=False, recording=None)
                self.sig_remote_status.emit(dict(self.remote_status))
        if self.source:
            self.source.close()

    def shutdown(self):
        # Closing the Jetson UI disconnects only. The Windows UI owns acquisition.
        self._exit.set()
        if self.source:
            self.source.close()
        if self._thread:
            self._thread.join(2)
            return not self._thread.is_alive()
        return True


class EmgPanel(QtWidgets.QWidget):
    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self.worker = worker
        layout = QtWidgets.QVBoxLayout(self)
        intro = QtWidgets.QLabel('EMG 在 Windows 采集、显示与保存\n本页仅连接状态和同步指令；不接收波形、不重采样、不写 EMG 数据表。')
        intro.setWordWrap(True)
        layout.addWidget(intro)
        self.link = QtWidgets.QCheckBox('联动 Windows EMG：全局 Record 开始/停止时同步发送指令')
        self.link.setChecked(False)
        self.link.toggled.connect(lambda value:setattr(worker, 'enabled', value))
        layout.addWidget(self.link)
        form = QtWidgets.QFormLayout()
        self.host = QtWidgets.QLineEdit(worker.config['remote_host'])
        self.port = QtWidgets.QSpinBox(); self.port.setRange(1,65535); self.port.setValue(worker.config['remote_port'])
        self.token = QtWidgets.QLineEdit(worker.config['remote_token']); self.token.setEchoMode(QtWidgets.QLineEdit.Password)
        form.addRow('Windows IP', self.host); form.addRow('控制端口', self.port); form.addRow('固定令牌', self.token)
        layout.addLayout(form)
        row = QtWidgets.QHBoxLayout()
        for title, callback in (('连接 / 重连', self._connect), ('同步并开始 EMG', worker.sync_start),
                                ('停止 Windows EMG', worker.stop), ('仅断开连接', lambda:worker._submit('disconnect'))):
            button = QtWidgets.QPushButton(title); button.clicked.connect(callback); row.addWidget(button)
        layout.addLayout(row)
        self.lbl_state = QtWidgets.QLabel('未连接')
        self.detail = QtWidgets.QLabel(worker.config_error or '联动只控制全局 Record；单独开始/停止按钮始终可用。')
        self.detail.setWordWrap(True)
        layout.addWidget(self.lbl_state); layout.addWidget(self.detail)
        self.file_status = QtWidgets.QLabel('Windows 保存信息尚未获取')
        self.file_status.setWordWrap(True)
        layout.addWidget(self.file_status)
        warning = QtWidgets.QLabel('断开网络或关闭 Jetson 界面后，Windows 会继续本地采集。需要停止时请点击停止并核对回执，或在 Windows 停止。')
        warning.setWordWrap(True); layout.addWidget(warning)
        layout.addStretch()
        worker.sig_state.connect(lambda state:self.lbl_state.setText(state))
        worker.sig_status.connect(self.detail.setText)
        worker.sig_remote_status.connect(self._remote)

    def _connect(self):
        self.worker.config.update(remote_host=self.host.text().strip(), remote_port=self.port.value(), remote_token=self.token.text().strip())
        try:
            if CONFIG.exists():
                backup = CONFIG.with_name(CONFIG.name+'.before-'+time.strftime('%Y%m%d-%H%M%S'))
                if not backup.exists():
                    backup.write_bytes(CONFIG.read_bytes())
            temporary = CONFIG.with_suffix('.tmp')
            temporary.write_text(json.dumps(self.worker.config, indent=2), encoding='utf-8')
            os.replace(temporary, CONFIG)
            if os.name != 'nt':
                CONFIG.chmod(0o600)
        except OSError as exc:
            self.detail.setText('配置保存失败：'+str(exc)); return
        self.worker.connect_source()

    def _remote(self, status):
        if not status.get('connected'):
            self.lbl_state.setText('连接已断开；Windows 采集状态未知')
            return
        self.lbl_state.setText(('模拟源 | ' if status.get('simulated') else '')+
            ('Windows 正在采集' if status.get('recording') else 'Windows 就绪' if status.get('ready') else '已连接，Windows 尚未就绪'))
        self.lbl_state.setToolTip('Windows 文件：'+status.get('record_path',''))
        self.file_status.setText(f"Windows 报告保存行数：{status.get('saved_rows', 0)} · 最后保存：{'成功' if status.get('save_ok') else '未确认'}\n{status.get('record_path', '')}")
        if status.get('sync_error'):
            self.detail.setText(status['sync_error'])

    def set_theme(self, theme):
        pass
