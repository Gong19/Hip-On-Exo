"""Windows operator panel for the real Delsys bridge (simulation is explicit)."""
import argparse
import json
from pathlib import Path
import socket
import sys
import threading
import time

from bridge_sources import Clock, DelsysSource, SyntheticSource, PROJECT
from bridge_service import Bridge, BUILD, PROTOCOL
from bridge_settings import load_connection_token

HERE=Path(__file__).resolve().parent


def addresses():
    try:
        import psutil
        found=[(name,a.address) for name,items in psutil.net_if_addrs().items() for a in items
               if a.family==socket.AF_INET and not a.address.startswith(('127.','169.254.'))]
        found.sort(key=lambda x:(0 if ('wifi' in x[0].lower() or 'ethernet' in x[0].lower()) else 1,x[0]))
        return found+[('本机测试','127.0.0.1')]
    except Exception:
        return [('本机测试','127.0.0.1')]


def gui(simulated=False, smoke_output=None):
    from PyQt5 import QtCore, QtGui, QtWidgets
    app=QtWidgets.QApplication(sys.argv[:1])
    try:
        connection_token=load_connection_token()
    except (OSError, ValueError) as exc:
        QtWidgets.QMessageBox.critical(None,'固定令牌配置错误',str(exc))
        return 1
    # Explicit font loading also makes offscreen Windows verification reliable.
    font_path=Path('C:/Windows/Fonts/msyh.ttc')
    if font_path.is_file():
        font_id=QtGui.QFontDatabase.addApplicationFont(str(font_path))
        families=QtGui.QFontDatabase.applicationFontFamilies(font_id)
        if families:app.setFont(QtGui.QFont(families[0],10))

    class Window(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.service=None;self.future=None;self.exit_ok=False;self.exit_result=None
            self.setWindowTitle('HiPExo · Windows EMG 桥接 '+BUILD+(' [SIMULATED]' if simulated else ' [真实 Delsys]'))
            self.resize(940,700)
            layout=QtWidgets.QVBoxLayout(self)
            title=QtWidgets.QLabel('模拟数据 / SIMULATED DATA ONLY' if simulated else 'Windows Delsys → Ubuntu Hip Exo')
            title.setStyleSheet('font-size:22px;font-weight:bold;color:'+('#b45309' if simulated else '#126172'))
            layout.addWidget(title)
            instructions=QtWidgets.QLabel('1. 关闭旧 EMG 程序　2. 连接并扫描　3. 检查通道后手动待命　4. Ubuntu 同步并开始')
            instructions.setWordWrap(True);layout.addWidget(instructions)
            form=QtWidgets.QFormLayout();layout.addLayout(form)
            self.host=QtWidgets.QComboBox();self.host.setEditable(True)
            for name,address in addresses():self.host.addItem(address+'  ·  '+name,address)
            self.port=QtWidgets.QSpinBox();self.port.setRange(1024,65535);self.port.setValue(8765)
            self.token=QtWidgets.QLineEdit(connection_token);self.token.setEchoMode(QtWidgets.QLineEdit.Password)
            self.token.setReadOnly(True)
            tokenrow=QtWidgets.QHBoxLayout();tokenrow.addWidget(self.token)
            copy=QtWidgets.QPushButton('复制令牌');copy.clicked.connect(lambda:app.clipboard().setText(self.token.text()))
            show=QtWidgets.QCheckBox('显示');show.toggled.connect(lambda yes:self.token.setEchoMode(QtWidgets.QLineEdit.Normal if yes else QtWidgets.QLineEdit.Password))
            tokenrow.addWidget(copy);tokenrow.addWidget(show)
            self.directory=QtWidgets.QLineEdit(str(HERE/'records'))
            saverow=QtWidgets.QHBoxLayout();saverow.addWidget(self.directory)
            browse=QtWidgets.QPushButton('选择目录');browse.clicked.connect(self.choose_directory);saverow.addWidget(browse)
            form.addRow('Windows 实验网卡 IP',self.host);form.addRow('TCP 端口',self.port)
            form.addRow('两端相同连接令牌',tokenrow);form.addRow('Windows 原始记录目录',saverow)
            note=QtWidgets.QLabel('使用本机保存的固定令牌，重启后保持不变；Ubuntu 保存同一个值即可。')
            note.setWordWrap(True);layout.addWidget(note)
            buttons=QtWidgets.QHBoxLayout();layout.addLayout(buttons)
            self.connect=QtWidgets.QPushButton('① 连接基站 / 扫描')
            self.connect.clicked.connect(self.prepare);buttons.addWidget(self.connect)
            self.arm=QtWidgets.QPushButton('② 进入待命 ARMED');self.arm.clicked.connect(self.arm_device);self.arm.setEnabled(False);buttons.addWidget(self.arm)
            self.stop=QtWidgets.QPushButton('解除待命 / 本机停止');self.stop.clicked.connect(self.disarm);self.stop.setEnabled(False);buttons.addWidget(self.stop)
            self.status=QtWidgets.QLabel('DISARMED · 尚未连接基站');self.status.setWordWrap(True);self.status.setStyleSheet('font-size:17px;padding:10px;background:#edf3f5');layout.addWidget(self.status)
            self.table=QtWidgets.QTableWidget(7,8);self.table.setHorizontalHeaderLabels(['槽位 / 肌肉','SID','状态','采样率 Hz','RMS','电量 %','SDK 单位 → V','SDK 模式'])
            self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
            self.table.horizontalHeader().setStretchLastSection(True);layout.addWidget(self.table)
            self.stats=QtWidgets.QLabel('尚未开始记录');self.stats.setWordWrap(True);layout.addWidget(self.stats)
            self.handshake=QtWidgets.QLabel('版本 '+BUILD+' · '+PROTOCOL+' · 尚未收到 HELLO')
            self.handshake.setWordWrap(True);layout.addWidget(self.handshake)
            self.errors=QtWidgets.QLabel('');self.errors.setWordWrap(True);self.errors.setStyleSheet('color:#a02020');layout.addWidget(self.errors)
            self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.refresh);self.timer.start(200)

        def choose_directory(self):
            selected=QtWidgets.QFileDialog.getExistingDirectory(self,'选择原始记录目录',self.directory.text())
            if selected:self.directory.setText(selected)

        def prepare(self):
            try:
                if self.service is None:
                    clock=Clock();source=SyntheticSource(clock) if simulated else DelsysSource(clock)
                    host=self.host.currentText().split()[0]
                    self.service=Bridge(source,host=host,port=self.port.value(),token=self.token.text(),
                                        record_dir=self.directory.text(),clock=clock).start()
                    for field in (self.host,self.port,self.token,self.directory):field.setEnabled(False)
                self.future=self.service.prepare();self.connect.setEnabled(False)
            except Exception as exc:
                self.errors.setText(str(exc));self.service=None

        def arm_device(self):
            if self.service:self.future=self.service.arm()

        def disarm(self):
            if self.service:self.future=self.service.disarm()

        def refresh(self):
            if self.exit_result is not None:
                result=self.exit_result;self.exit_result=None
                if result is True:self.exit_ok=True;self.close();return
                self.errors.setText('停止未确认：'+str(result)+'；请保留窗口，稍后重试退出。')
            if self.future and self.future.done():
                try:self.future.result();self.errors.setText('')
                except Exception as exc:self.errors.setText(str(exc))
                self.future=None
            if not self.service:return
            s=self.service.snapshot()
            busy=s['state'] in ('SCANNING','STARTING','STREAMING','STOPPING') or self.future is not None
            self.connect.setEnabled(not busy)
            self.arm.setEnabled(not busy and self.service.prepared and not s['ready'])
            self.stop.setEnabled(s['ready'] or s['state']=='STREAMING')
            self.status.setText(f"{s['state']}  |  {s['host']}:{s['port']}  |  Ubuntu: "+(s['peer'] if s['connected'] else '未连接'))
            hello=s['last_hello']
            labels={'OK':'握手成功','PROTOCOL_MISMATCH':'协议不匹配','TOKEN_MISMATCH':'令牌不匹配，请重新复制当前令牌',
                    'TOKEN_FORMAT_INVALID':'令牌格式不正确','ALREADY_AUTHENTICATED':'重复握手'}
            detail=(labels.get(hello['code'],hello['code'])+' · '+hello['peer']) if hello else '尚未收到 HELLO'
            self.handshake.setText(f"版本 {s['build']} · 实例 {s['instance_id']} · {PROTOCOL} · {detail}")
            for i,ch in enumerate(s['channels']):
                values=[f"{i} / {ch['muscle']}",str(ch['sid']),'在线' if ch['present'] else 'MISSING',
                        f"{ch['sample_rate']:.6g}" if ch['present'] else '配置值（非实测）',
                        str(ch['is_rms']),'-' if ch['battery'] is None else f"{ch['battery']:.0f}",
                        (str(ch.get('sdk_unit','模拟 V' if simulated else '未配置'))+
                         (' × '+str(ch['scale_to_v']) if 'scale_to_v' in ch else '')),ch['mode']]
                for j,value in enumerate(values):self.table.setItem(i,j,QtWidgets.QTableWidgetItem(value))
            self.stats.setText(f"run_id: {s['run_id'] or '—'}\n数据包: {s['packets']}　各通道点数: {s['samples']}　队列: {s['io_queue']} / 峰值 {s['io_peak']}\n本机原始文件: {s['record_path'] or '等待 Ubuntu 开始'}")
            if s['error']:self.errors.setText(s['error'])

        def closeEvent(self,event):
            if self.exit_ok or self.service is None:event.accept();return
            event.ignore()
            if getattr(self,'closing',False):return
            self.closing=True;self.status.setText('正在停止设备并排空原始文件，请稍候…')
            def finish():
                try:self.service.shutdown();self.exit_result=True
                except Exception as exc:self.exit_result=str(exc)
                finally:self.closing=False
            threading.Thread(target=finish,name='EMG-shutdown',daemon=True).start()

    win=Window();win.show()
    if smoke_output:
        if not simulated:raise ValueError('GUI smoke test must be simulated')
        win.host.setCurrentText('127.0.0.1');win.port.setMinimum(0);win.port.setValue(0)
        QtCore.QTimer.singleShot(100,win.prepare)
        QtCore.QTimer.singleShot(500,win.arm_device)
        def capture():
            win.refresh()
            if not win.service.snapshot()['ready']:raise RuntimeError('GUI smoke did not reach ARMED')
            win.grab().save(str(smoke_output))
            win.close()
        QtCore.QTimer.singleShot(1000,capture)
    return app.exec_()


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--simulate',action='store_true',help='Explicit simulated source; never real EMG')
    p.add_argument('--smoke-output',help=argparse.SUPPRESS)
    args=p.parse_args()
    return gui(args.simulate,args.smoke_output)


if __name__=='__main__':sys.exit(main())
