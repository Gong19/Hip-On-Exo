"""Shared D6 acquisition and UI, independent of motor/camera modules."""

import argparse
import json
import math
import os
import sys
import time
import uuid
from datetime import datetime
from pathlib import Path
from threading import Event, Lock, Thread

from PyQt5 import QtCore, QtGui, QtWidgets

import lidar_d6

try:
    import serial
except ImportError:
    serial = None


def load_config(path=None):
    path = path or os.environ.get("HIPEXO_LIDAR_CONFIG")
    path = Path(path) if path else Path(__file__).with_name("lidar_config.json")
    with path.open(encoding="utf-8") as stream:
        config = json.load(stream)
    for side in ("L", "R"):
        unit = config.setdefault(side, {})
        if not isinstance(unit.get("port", ""), str):
            raise ValueError(f"{side}.port must be a string")
        if not isinstance(unit.get("send_commands", False), bool):
            raise ValueError(f"{side}.send_commands must be boolean")
    ports = [os.path.realpath(config[side]["port"]) for side in ("L", "R")
             if config[side].get("port")]
    if len(ports) != len(set(ports)):
        raise ValueError("L/R cannot share a serial port")
    return config


class LidarWorker(QtCore.QObject):
    sig_update = QtCore.pyqtSignal(dict)
    sig_status = QtCore.pyqtSignal(str)
    sig_conn_status = QtCore.pyqtSignal(bool)
    _claims = set()
    _claims_lock = Lock()

    def __init__(self, side, port, data_manager=None, parent=None,
                 config=None, reserved_ports=()):
        super().__init__(parent)
        self.side = side
        self.config = load_config() if config is None else config
        self.port = port or self.config.get(side, {}).get("port", "")
        self.send_commands = self.config.get(side, {}).get("send_commands", False)
        self._reserved_ports = tuple(reserved_ports)
        self._dm = data_manager
        self._online = False
        self._thread = None
        self._stop = Event()
        self._record = Event()
        self._last_frame = 0.0
        self._latest = None
        self._latest_lock = Lock()
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._publish)
        self._timer.start()

    def is_online(self):
        return self._online

    def set_recording(self, enabled):
        if enabled:
            self._record.set()
        else:
            self._record.clear()

    def _set_online(self, online):
        if self._online != online:
            self._online = online
            self.sig_conn_status.emit(online)

    def _publish(self):
        with self._latest_lock:
            result, self._latest = self._latest, None
        if result is not None and not self._stop.is_set():
            self.sig_update.emit(result)

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        if not self.port:
            self.sig_status.emit(f"{self.side}: configure a LiDAR port first")
            return
        if serial is None:
            self.sig_status.emit("Missing pyserial")
            return
        self._stop.clear()
        self._thread = Thread(target=self._loop, daemon=True, name=f"lidar-{self.side}")
        self._thread.start()

    def stop(self):
        self._stop.set()
        with self._latest_lock:
            self._latest = None
        self._set_online(False)

    def shutdown(self):
        self.stop()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._timer.stop()

    def _open_serial(self):
        selected = os.path.realpath(self.port)
        reserved = {os.path.realpath(port) for port in self._reserved_ports}
        if selected in reserved:
            raise ValueError("LiDAR port conflicts with a configured motor port")
        with self._claims_lock:
            if selected in self._claims:
                raise ValueError("Serial port already claimed by another LiDAR")
            self._claims.add(selected)
        try:
            device = serial.Serial(self.port, lidar_d6.BAUD_RATE, timeout=0.1,
                                   write_timeout=0.2, exclusive=True)
            try:
                if self.send_commands:
                    device.write(bytes.fromhex("aa55f00f"))
            except Exception:
                device.close()
                raise
            return device, selected
        except Exception:
            with self._claims_lock:
                self._claims.discard(selected)
            raise

    def _loop(self):
        record = None
        record_session = None
        session = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        stream_id = uuid.uuid4().hex
        scan_index = 0
        total_bytes = 0

        def close_record(error=None):
            nonlocal record, record_session
            if record is not None:
                filename = record.name
                record.close()
                record = None
                if record_session:
                    record_session.event('lidar_raw_closed',path=filename,error=error)
                record_session = None

        try:
            while not self._stop.is_set():
                device = None
                claimed = None
                parser = lidar_d6.D6StreamParser()
                previous_frame = None
                try:
                    device, claimed = self._open_serial()
                    connection_id = uuid.uuid4().hex
                    self.sig_status.emit(f"{self.side}: {self.port}, receive-only={not self.send_commands}")
                    self._last_frame = time.monotonic()
                    while not self._stop.is_set():
                        if record is not None and (not self._record.is_set() or
                                record_session is not getattr(self._dm,'session',None)):
                            close_record()
                        read_started_wall_ns, read_started_mono_ns = time.time_ns(), time.perf_counter_ns()
                        data = device.read(4096)
                        read_finished_wall_ns, read_finished_mono_ns = time.time_ns(), time.perf_counter_ns()
                        now = time.monotonic()
                        total_bytes += len(data)
                        revolutions = parser.feed(data)
                        if now - self._last_frame > 1.0:
                            self._set_online(False)
                        if not self._record.is_set() and record is not None:
                            close_record()
                        for points in revolutions:
                            if self._stop.is_set():
                                break
                            measured_hz = 0.0 if previous_frame is None or now<=previous_frame else 1.0 / (now-previous_frame)
                            previous_frame = now
                            self._last_frame = now
                            self._set_online(True)
                            scan_index += 1
                            result = {
                                "valid": False, "calibrated": False,
                                "side": self.side, "port": self.port,
                                "wall_time": read_finished_wall_ns/1e9, "monotonic_time": now,
                                "stream_id": stream_id, "connection_id": connection_id,
                                "scan_index": scan_index,
                                "timestamp_semantics": "host_read_completion_not_point_sampling_time",
                                "host_read_started_wall_ns": read_started_wall_ns,
                                "host_read_started_mono_ns": read_started_mono_ns,
                                "host_read_finished_wall_ns": read_finished_wall_ns,
                                "host_read_finished_mono_ns": read_finished_mono_ns,
                                "scan_assembled_mono_ns": time.perf_counter_ns(),
                                "device_scan_start_ns": None, "device_scan_end_ns": None,
                                "device_timestamp_available": False,
                                "n_scan_points": len(points),
                                "nonzero_points": sum(point.distance_mm > 0 for point in points),
                                "valid_packets": parser.valid_packets,
                                "bad_packets": parser.bad_packets,
                                "bytes": total_bytes,
                                "reported_hz": parser.reported_hz,
                                "arrival_hz": measured_hz,
                                "points": [[point.angle_deg, point.distance_mm / 1000.0,
                                            point.intensity] for point in points],
                            }
                            if self._dm is not None:
                                self._dm.append_frame(f"lidar_{self.side}", result["wall_time"] * 1000,
                                                     {key: result[key] for key in
                                                      ("n_scan_points", "nonzero_points", "bad_packets", "reported_hz",
                                                       "scan_index", "stream_id", "connection_id", "timestamp_semantics",
                                                       "host_read_started_wall_ns", "host_read_started_mono_ns",
                                                       "host_read_finished_wall_ns", "host_read_finished_mono_ns",
                                                       "scan_assembled_mono_ns", "device_timestamp_available")},
                                                     t_mono_ns=read_finished_mono_ns)
                            if self._record.is_set():
                                try:
                                    if record is None:
                                        record_session = getattr(self._dm,'session',None)
                                        directory = (record_session.directory / 'lidar_raw' if record_session else
                                                     Path(self.config.get("record_directory", "~/HiPExo_Lidar_Records")).expanduser())
                                        directory.mkdir(parents=True, exist_ok=True)
                                        context = record_session.file_stem(f'Lidar-{self.side}') if record_session else session
                                        filename = directory / f"D6_{self.side}_{context}_{time.time_ns()}.jsonl"
                                        record = filename.open("x", encoding="utf-8", buffering=1)
                                        if record_session:
                                            record_session.artifact('lidar_raw',filename,side=self.side,stream_id=stream_id)
                                        record.write(json.dumps({"type": "metadata", "config": self.config,
                                                                 "port": self.port, "units": "degrees,metres,intensity",
                                                                 "stream_id": stream_id,
                                                                 "session_id": record_session.session_id if record_session else None,
                                                                 "timing_note": "Read intervals refer to the final serial chunk. A scan can span reads; multiple scans can share one read. Device/point sampling times are unavailable."}) + "\n")
                                        self.sig_status.emit(f"Recording: {filename}")
                                    record.write(json.dumps(result) + "\n")
                                except OSError as error:
                                    self._record.clear()
                                    close_record(str(error))
                                    self.sig_status.emit(f"Recording failed: {error}")
                            with self._latest_lock:
                                self._latest = result
                        if now - self._last_frame > 3.0:
                            raise TimeoutError("No valid complete scan for 3 seconds; reconnecting")
                        if not data:
                            self._stop.wait(0.01)
                except Exception as error:
                    self.sig_status.emit(f"{self.side}: {error}")
                finally:
                    self._set_online(False)
                    if device is not None:
                        try:
                            if self.send_commands:
                                device.write(bytes.fromhex("aa55f50a"))
                        except Exception:
                            pass
                        finally:
                            try:
                                device.close()
                            except Exception as error:
                                self.sig_status.emit(f"Close failed: {error}")
                    with self._claims_lock:
                        self._claims.discard(claimed)
                self._stop.wait(1.0)
        finally:
            close_record()


class ScanPlot(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.points = []
        self.range_m = 3.0
        self.setMinimumSize(220, 220)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#101a29"))
        center = QtCore.QPointF(self.width() / 2, self.height() / 2)
        radius = max(1, min(self.width(), self.height()) / 2 - 25)
        painter.setPen(QtGui.QColor("#52647b"))
        for fraction in (0.25, 0.5, 0.75, 1.0):
            painter.drawEllipse(center, radius * fraction, radius * fraction)
        painter.drawLine(QtCore.QPointF(center.x() - radius, center.y()),
                         QtCore.QPointF(center.x() + radius, center.y()))
        painter.drawLine(QtCore.QPointF(center.x(), center.y() - radius),
                         QtCore.QPointF(center.x(), center.y() + radius))
        painter.setPen(QtGui.QColor("#dbeaff"))
        painter.drawText(8, 18, f"RAW sensor XY | range {self.range_m:g} m | 0 deg right")
        painter.setPen(QtGui.QPen(QtGui.QColor("#48dfdf"), 3))
        for angle, distance, intensity in self.points:
            if 0 < distance <= self.range_m:
                angle = math.radians(angle)
                scale = distance * radius / self.range_m
                painter.drawPoint(QtCore.QPointF(center.x() + math.cos(angle) * scale,
                                                 center.y() - math.sin(angle) * scale))
        painter.end()


class LidarPanel(QtWidgets.QWidget):
    def __init__(self, workers, parent=None):
        super().__init__(parent)
        self._workers = dict(zip(("L", "R"), workers))
        self._side_widgets = {}
        self._last_updates = {}
        layout = QtWidgets.QVBoxLayout(self)
        toolbar = QtWidgets.QHBoxLayout()
        self.btn_start = QtWidgets.QPushButton("Start selected")
        self.btn_stop = QtWidgets.QPushButton("Stop")
        self.record = QtWidgets.QCheckBox("Record raw scans (JSONL)")
        self.range_box = QtWidgets.QDoubleSpinBox()
        self.range_box.setRange(0.2, 12.0)
        self.range_box.setValue(3.0)
        self.range_box.setSuffix(" m")
        for widget in (self.btn_start, self.btn_stop, self.record, self.range_box):
            toolbar.addWidget(widget)
        layout.addLayout(toolbar)
        self.lbl_state = QtWidgets.QLabel("IDLE — no motor hardware is started by this panel")
        self.lbl_state.setWordWrap(True)
        layout.addWidget(self.lbl_state)
        cards = QtWidgets.QHBoxLayout()
        for side, worker in self._workers.items():
            card = QtWidgets.QVBoxLayout()
            enabled = QtWidgets.QCheckBox(f"LiDAR {side}: {worker.port or 'not configured'}")
            enabled.setChecked(bool(worker.port))
            conn = QtWidgets.QLabel("OFFLINE")
            values = QtWidgets.QLabel("No scan yet")
            values.setWordWrap(True)
            plot = ScanPlot()
            card.addWidget(enabled)
            card.addWidget(conn)
            card.addWidget(plot, 1)
            card.addWidget(values)
            cards.addLayout(card)
            self._side_widgets[side] = {"enabled": enabled, "conn": conn, "values": values, "plot": plot}
            worker.sig_update.connect(lambda result, side=side: self._on_update(side, result))
            worker.sig_conn_status.connect(lambda online, side=side: self._on_conn_status(side, online))
            worker.sig_status.connect(self.lbl_state.setText)
        layout.addLayout(cards, 1)
        layout.addWidget(QtWidgets.QLabel("Uncalibrated: raw sensor-plane coordinates only; terrain/control output disabled."))
        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop.clicked.connect(self._on_stop)
        self.record.toggled.connect(self._record_changed)
        self.range_box.valueChanged.connect(self._range_changed)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh_age)
        self._timer.start(200)

    def _on_start(self):
        for side, worker in self._workers.items():
            if self._side_widgets[side]["enabled"].isChecked():
                worker.start()

    def _on_stop(self):
        for worker in self._workers.values():
            worker.stop()
        self.lbl_state.setText("STOPPED")

    def _record_changed(self, enabled):
        for worker in self._workers.values():
            worker.set_recording(enabled)

    def _range_changed(self, value):
        for widgets in self._side_widgets.values():
            widgets["plot"].range_m = value
            widgets["plot"].update()

    def _on_conn_status(self, side, online):
        self._side_widgets[side]["conn"].setText("ONLINE" if online else "OFFLINE")
        if not online:
            self._side_widgets[side]["plot"].points = []
            self._side_widgets[side]["plot"].update()

    def _on_update(self, side, result):
        self._last_updates[side] = result
        plot = self._side_widgets[side]["plot"]
        plot.points = result["points"]
        plot.update()
        self._refresh_age()

    def _refresh_age(self):
        for side, result in self._last_updates.items():
            age = max(0.0, time.monotonic() - result["monotonic_time"])
            self._side_widgets[side]["values"].setText(
                f"{result['n_scan_points']} points ({result['nonzero_points']} nonzero) | "
                f"sensor {result['reported_hz']:.1f} Hz | age {age:.1f} s\n"
                f"packets OK {result['valid_packets']} / bad {result['bad_packets']} | uncalibrated")

    def set_theme(self, theme):
        self.update()

    def closeEvent(self, event):
        for worker in self._workers.values():
            worker.shutdown()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description="D6-only GUI: never imports motor, IMU or camera modules")
    parser.add_argument("--config")
    parser.add_argument("--port", help="Override L port for bench test")
    parser.add_argument("--right-port")
    arguments = parser.parse_args()
    config = load_config(arguments.config)
    if arguments.port:
        config["L"]["port"] = arguments.port
    if arguments.right_port:
        config["R"]["port"] = arguments.right_port
    app = QtWidgets.QApplication(sys.argv[:1])
    workers = tuple(LidarWorker(side, "", config=config) for side in ("L", "R"))
    panel = LidarPanel(workers)
    app.aboutToQuit.connect(lambda: [worker.shutdown() for worker in workers])
    panel.setWindowTitle("HiPExo — LiDAR ONLY / no motor control")
    panel.resize(1000, 640)
    panel.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
