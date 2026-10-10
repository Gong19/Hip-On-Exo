#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HiPExo Monitor: motor, IMU, ADS8688 force, EMG, camera and LiDAR.

Native-v2 architecture (2026-10-10): isolated ADC and per-I2C-bus IMU
capture; independent motor-port threads; bounded 20ms transfers; isolated
300ms cycle writer on a 1000Hz processing grid. Grid frequency does not
imply hardware sample rate. See README_NATIVE_V2_20261010.md for evidence.

Display rings have maxlen=8000 and evict incrementally. Legacy-only 30s
housekeeping must not stop cycle-mode acquisition. Preview never starts
physical hardware. Launch: python3 hipexo_monitor19.py
"""

# ── stdlib ──────────────────────────────────────────────────────────────────
import os, sys, csv, json, math, re, statistics, time, traceback
from datetime import datetime
from threading import Thread, Event, Lock
from collections import deque, defaultdict

# ── Qt / pyqtgraph ───────────────────────────────────────────────────────────
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtGui import QKeySequence, QIcon, QFont
import pyqtgraph as pg
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hipexo_emg import EmgWorker, EmgPanel
from hipexo_recording import FrameRecorder
from hipexo_session import SessionManifest, camera_frame_timing
from hipexo_recording_layout import RecordingBundle, batch_summary, sensors_in
from hipexo_imu_reference import ImuReference
from hipexo_recording_process import ProcessCycleRecorder as CycleRecorder
PIPELINE_ENABLED = os.environ.get("HIPEXO_RECORDING_MODE", "cycles") != "legacy"
VISION_IMAGE_ONLY = os.environ.get("HIPEXO_VISION_MODE", "images") == "images"
import uuid

# Antialiased line drawing on every plot repaint is expensive on the Jetson
# Orin Nano's software/CPU-bound Qt rendering path. With many curves
# refreshing at UI_REFRESH_HZ this was a measurable contributor to lag —
# turned off globally instead of per-panel. See WORKING_LOG.md 2026-08-17.
pg.setConfigOptions(antialias=False, useOpenGL=False)

# ── Unitree SDK (optional – GUI stays alive without it) ──────────────────────
import ctypes
import numpy as np   # already a transitive dependency of pyqtgraph; used directly
                      # by VisionPanel for depth/projection image display

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_LIB_DIR    = os.environ.get('HIPEXO_SDK_LIB_DIR',os.path.normpath(os.path.join(_SCRIPT_DIR, '..', 'lib')))

# Own directory first — `import vision_terrain` (below) lives next to this
# file. Plain `python3 hipexo_monitor.py` already gets this for free (the
# interpreter puts the script's directory at sys.path[0]), but this file is
# also loaded via importlib in headless test harnesses, which does NOT add
# it automatically — so make it explicit rather than relying on how we
# happen to be invoked.
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

# Add lib dir to sys.path so the .cpython .so wrapper is importable
sys.path.insert(0, _LIB_DIR)

# Pre-load the native Arm64 shared library via ctypes.
# This is the reliable fix on Jetson: without it, the Python wrapper
# fails with "cannot open shared object file" even when sys.path is correct,
# because dlopen() needs LD_LIBRARY_PATH or a ctypes pre-load.
_NATIVE_SO = os.path.join(_LIB_DIR, 'libUnitreeMotorSDK_Arm64.so')
try:
    ctypes.CDLL(_NATIVE_SO)
except OSError as _ctypes_err:
    print(f"[WARN] Could not preload {_NATIVE_SO}: {_ctypes_err}")

try:
    try:
        from hipexo_unitree_sdk import MotorCmd, MotorData, MotorType, MotorMode, SerialPort, queryMotorMode, queryGearRatio
        _MOTOR_GIL_RELEASED = True
    except ImportError:
        from unitree_actuator_sdk import MotorCmd, MotorData, MotorType, MotorMode, SerialPort, queryMotorMode, queryGearRatio
        _MOTOR_GIL_RELEASED = False
    _SDK_OK = True
    print(f"[INFO] unitree_actuator_sdk loaded OK from {_LIB_DIR}")
except Exception as _e:
    _SDK_OK = False
    print(f"[WARN] unitree_actuator_sdk not found: {_e}  (motor panel disabled)")

# ── smbus2 (optional – IMU panel disabled gracefully) ────────────────────────
try:
    from smbus2 import SMBus
    _SMBUS_OK = True
except ImportError:
    _SMBUS_OK = False
    print("[WARN] smbus2 not found: pip3 install smbus2  (IMU panel disabled)")

# ── spidev for ADS8688 force sensor ADC (optional) ───────────────────────────
try:
    import spidev as _spidev
    _SPIDEV_OK = True
except ImportError:
    _SPIDEV_OK = False
    print("[WARN] spidev not found: sudo apt install python3-spidev  (Force panel disabled)")

# ── D435i depth camera + terrain CNN for the Vision panel (optional) ─────────
try:
    import vision_terrain as _vt
    _VISION_MODULE_OK = _vt._REALSENSE_OK and _vt._TORCH_OK and _vt._CV2_OK
    if not _VISION_MODULE_OK:
        missing = [n for n, ok in (("pyrealsense2", _vt._REALSENSE_OK),
                                    ("torch", _vt._TORCH_OK),
                                    ("opencv-python", _vt._CV2_OK)) if not ok]
        print(f"[WARN] vision deps missing: {missing}  (Vision panel disabled)")
except Exception as _e:
    _vt = None
    _VISION_MODULE_OK = False
    print(f"[WARN] vision_terrain import failed: {_e}  (Vision panel disabled)")

# Terrain -> control-mode switching framework (pure Python, no hardware
# deps — always available once vision_terrain itself imports).
from terrain_mode_switch import TerrainModeSwitcher, LABELS as _TERRAIN_LABELS

# ── COIN-D6 LiDAR support (optional) ──────────────────────────────────────────
try:
    import lidar_d6 as _ld6
    _LIDAR_MODULE_OK = True
except Exception as _e:
    _ld6 = None
    _LIDAR_MODULE_OK = False
    print(f"[WARN] lidar_d6 import failed: {_e}  (LiDAR panel disabled)")

try:
    import serial as _pyserial
    _PYSERIAL_OK = True
except ImportError:
    _PYSERIAL_OK = False
    print("[WARN] pyserial not found: pip3 install pyserial  (LiDAR panel disabled)")

# ── psutil for memory guard (optional) ───────────────────────────────────────
try:
    import psutil as _psutil
    _PSUTIL_OK = True
except ImportError:
    _PSUTIL_OK = False


_PREVIEW = "--preview" in sys.argv
if _PREVIEW:
    _SDK_OK = _SMBUS_OK = _SPIDEV_OK = _VISION_MODULE_OK = False

# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                       RESPONSIVE SCALE FACTOR                           ║
# ║  _S is computed once at import time from the primary screen geometry.   ║
# ║  Reference resolution: 1280×720.  All fixed pixel sizes are × _S.      ║
# ║  1024×600 → _S≈0.80   1920×1080 → _S=1.0   2560×1440 → _S=1.33        ║
# ╚══════════════════════════════════════════════════════════════════════════╝
def _compute_scale() -> float:
    try:
        _tmp_app = QtWidgets.QApplication.instance()
        if _tmp_app is None:
            return 1.0
        screen = _tmp_app.primaryScreen()
        if screen is None:
            return 1.0
        g = screen.availableGeometry()
        s = min(g.width() / 1280.0, g.height() / 720.0)
        return max(0.65, min(s, 2.0))   # clamp [0.65, 2.0]
    except Exception:
        return 1.0

_S = 1.0   # will be re-computed in main() after QApplication is created

def _px(base: int) -> int:
    """Scale a base pixel value by _S, return int."""
    return max(1, int(round(base * _S)))


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                          CONFIGURATION BLOCK                            ║
# ║  Edit everything here.  Nothing hardware-specific lives elsewhere.      ║
# ╚══════════════════════════════════════════════════════════════════════════╝

# ── Motor hardware ───────────────────────────────────────────────────────────
MOTOR_DEVICES = [("/dev/ttyUSB0", 0), ("/dev/ttyUSB1", 1)]
# FT4232H: each motor on its own port, motor_id=0 per port.
# ttyUSB0 → Channel A → Motor 0
# ttyUSB1 → Channel B → Motor 1

SWMOTOR_PATH = os.path.join(
    _SCRIPT_DIR, '..', 'motor_tools',
    'Unitree_MotorTools_v1.2.4_arm64_Linux', 'swmotor'
)

MOTOR_PARAMS = {
    0: {"MODE": "DQ", "KP": 0.0, "KD": 0.00, "TAU": 0.0,
        "Q_SET": 0.0, "DQ_SET": None, "DQ_SCALE": "GEAR",
        "K": 0.0, "D": 0.0, "M": 0.0,
        "TRAJ": {"type": "CONST", "q0": 0.0, "amp": 0.0, "freq": 0.5, "phase": 0.0}},
    1: {"MODE": "DQ", "KP": 0.0, "KD": 0.00, "TAU": 0.0,
        "Q_SET": 0.0, "DQ_SET": None, "DQ_SCALE": "GEAR",
        "K": 0.0, "D": 0.0, "M": 0.0,
        "TRAJ": {"type": "CONST", "q0": 0.0, "amp": 0.0, "freq": 0.5, "phase": 0.0}},
}

TAU_LIMIT  = 6.0
DQ_LIMIT   = 50.0
# GO-M8010-6: gear ratio 6.33, 15-bit absolute encoder on rotor side.
# MotorData.q is the RAW ROTOR angle in rad (cumulative, unbounded).
# Output shaft angle = q / MOTOR_GEAR_RATIO
# Relative position   = (q / MOTOR_GEAR_RATIO) mod 2π  → always in [0, 2π)
MOTOR_GEAR_RATIO = 6.33
DEFAULT_TRAJ = {"type": "CONST", "q0": 0.0, "amp": 0.0, "freq": 0.5, "phase": 0.0}

# ── IMU hardware ─────────────────────────────────────────────────────────────
IMU_BUS_ID = 7                        # legacy/default bus; use IMU_DEVICES below
IMU_DEVICES = [
    (7, 0x50),
    (7, 0x51),
    (1, 0x52),
    (1, 0x53),
]                                      # (i2c bus, WT9011G4K address)
# Restore the four fixed slots from exo-9.21-version, including absent sensors.
# On 2026-10-02, live reads responded at 7/0x51 and 1/0x52; retain the other slots.
IMU_ADDRS  = [addr for _, addr in IMU_DEVICES] # kept for profile/UI compatibility
                                       # inactive devices are auto-skipped
IMU_PERIOD_S    = 0.02                 # Preserve this PC's existing 50 Hz poll loop.
IMU_PLOT_WINDOW = 10.0                 # seconds of history shown in plot
IMU_OFFLINE_AFTER_S   = 0.5            # no good sample for this long → mark sensor OFFLINE
IMU_RESCAN_INTERVAL_S = 1.0            # how often to retry sensors that are offline/never seen

# WT9011G4K REG.h word-addressed registers (each = 2 bytes on wire)
IMU_AX_REG    = 0x34
IMU_BLOCK_LEN = 26     # 13 regs × 2 bytes: AX..TEMP


# ── Force Sensor (ADS8688 + DY510) ───────────────────────────────────────────
FORCE_SPI_BUS    = 0          # /dev/spidev{BUS}.{DEVICE}
FORCE_SPI_DEVICE = 0
FORCE_SPI_SPEED  = 4_000_000  # verified SPI register readback; original was 1 MHz

# Two independent load-cell channels sharing one ADS8688 board.
# AIN channel: 0-7 (physical pin on ADS8688 header)
# output_mode: "0_10V" | "0_5V" | "pm10V" | "pm5V" | "4_20mA"
# sensor_max_kg: load-cell rated capacity
# label: displayed in the UI
FORCE_CHANNELS = [
    {"ain": 5, "output_mode": "pm10V", "sensor_max_kg": 50.0,  "label": "Load Cell L"},
    {"ain": 4, "output_mode": "pm10V", "sensor_max_kg": 50.0,  "label": "Load Cell R"},
]

FORCE_SAMPLE_HZ  = 1000         # target native reads per channel, independently measured
FORCE_PLOT_WIN_S = 10.0       # seconds of history shown in plots
FORCE_OFFLINE_AFTER_S     = 1.0   # no good sample for this long → mark channel OFFLINE
FORCE_RECONNECT_INTERVAL_S = 2.0  # retry ADC open this often while disconnected

# ── Vision (Intel RealSense D435i + terrain CNN) ─────────────────────────────
# The depth stream itself can run at full camera FPS cheaply, but CNN
# inference (+ the Qt image conversion for display) is the expensive part —
# same "decouple hardware I/O from GUI/logging rate" lesson as the motor/IMU
# perf pass. See WORKING_LOG.md 2026-08-18.
VISION_MODEL_NAME       = "projection_cnn_clean.pt"
VISION_CAPTURE_FPS      = 15       # D435i depth stream FPS — the algorithm only needs
                                    # VISION_INFER_HZ worth of frames; requesting the
                                    # sensor's full 30 FPS just burns USB/CPU decoding
                                    # frames that get thrown away. 15 still comfortably
                                    # covers VISION_INFER_HZ below with margin.
VISION_INFER_HZ         = 8        # CNN inference rate
VISION_DISPLAY_HZ       = 5        # depth/projection preview image regeneration rate —
                                    # deliberately <= VISION_INFER_HZ; the cv2 colormap
                                    # call is the single most expensive step per frame
VISION_HISTORY_SIZE     = 10       # majority-vote window for the "Stable" label (frames @ VISION_INFER_HZ)
VISION_OFFLINE_AFTER_S      = 1.0  # no good frame for this long → mark camera OFFLINE
VISION_RECONNECT_INTERVAL_S = 3.0  # retry opening the RealSense pipeline this often while disconnected
VISION_LOG_HZ           = 4        # DataManager logging rate for terrain predictions
LIDAR_PORT_L = ""
LIDAR_PORT_R = ""

# ADS8688 datasheet table 15, low nibble. Internal reference nominal 4.096 V.
_ADS_RANGE = {"pm2p5V":2,"pm5V":1,"pm10V":0,"0_5V":6,"0_10V":5}
_ADS_VRANGE = {"pm2p5V":(-2.56,2.56),"pm5V":(-5.12,5.12),
               "pm10V":(-10.24,10.24),"0_5V":(0.,5.12),
               "0_10V":(0.,10.24),"4_20mA":(0.,5.12)}
_ADS_MODE_TO_RANGE = {
    "0_5V":   "0_5V",   "0_10V":  "0_10V",
    "pm5V":   "pm5V",   "pm10V":  "pm10V",
    "4_20mA": "0_5V",
}



# ── Data / logging ───────────────────────────────────────────────────────────
EXPORT_DIR = os.path.expanduser(os.environ.get("HIPEXO_EXPORT_DIR", "~/hipexo_preview_logs" if _PREVIEW else "~/hipexo_logs"))
FLUSH_INTERVAL_S    = 30     # legacy housekeeping only; cycle-mode rings need no bulk trim
DISPLAY_KEEP_PTS    = 500    # points kept in RAM after each flush
BUFFER_HARD_MAX     = 8000   # absolute deque cap per channel (was 20000 — see WORKING_LOG.md;
                              # 20000 was also (mis)used as the live-plot buffer size, which was
                              # the single biggest cause of UI freezes, see MOTOR_DISPLAY_PTS below)
MEMORY_WARN_MB      = 200    # RSS warning threshold
MEMORY_CRIT_MB      = 350    # RSS critical → force flush

# ── UI timing ────────────────────────────────────────────────────────────────
UI_REFRESH_HZ   = 20        # plot/label redraw rate (was 50 Hz — halves redraw cost, still smooth
                              # for a monitoring display; see WORKING_LOG.md 2026-08-17)
CTRL_PERIOD_S   = 0.0002   # legacy lower bound; V2 caps each port at MOTOR_LOG_HZ, actual rate link-limited

# V2 requests up to 1000 validated feedback samples/s independently per port.
# Synchronous USB round trips set the achieved rate. GUI signals run at 50Hz;
# data enters the recorder in bounded 20ms batches with original timestamps.
MOTOR_LOG_HZ        = 1000
MOTOR_LOG_DECIMATE  = max(1, int(round((1.0 / CTRL_PERIOD_S) / MOTOR_LOG_HZ)))
MOTOR_OFFLINE_AFTER_S = 0.5   # no successful sendRecv for this long → mark motor OFFLINE

# Live-plot buffer sizes (decoupled from BUFFER_HARD_MAX / DataManager).
# These only need enough points to fill the visible time window — NOT the
# full logging buffer. Previously the Motor plot buffers reused
# BUFFER_HARD_MAX (20000 pts) and were copied to a fresh Python list on
# every 50 Hz refresh tick: ~4 curves x 20000 pts x 50 Hz ≈ 4,000,000
# element copies/second, which alone was enough to freeze a Jetson Orin
# Nano. See WORKING_LOG.md 2026-08-17.
MOTOR_DISPLAY_PTS   = 600

# ── Subject / session tracking ─────────────────────────────────────────────────
# Touch-screen operation (no keyboard) needs the "who/where" prompt reduced to
# the bare minimum — a subject ID and a location — with the save destination
# handled automatically. See SessionManager / SessionDialog and WORKING_LOG.md
# 2026-08-17 "触摸屏保存位置与受试者流程" entry for the full design rationale.
SESSION_STATE_PATH = os.path.expanduser(os.environ.get("HIPEXO_SESSION_STATE_PATH", "~/hipexo_preview_session_state.json" if _PREVIEW else "~/hipexo_session_state.json"))

# Fixed set of experiment locations. Kept as a short, closed list (rather
# than free text) so the touch-screen dialog can show them as one-tap
# buttons — add/remove entries here to change what's offered.
EXPERIMENT_LOCATIONS = ["Vicon Lab", "Campus Outdoor"]

# ── Misc ──────────────────────────────────────────────────────────────────────
TOAST_BEEP = True


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                            THEME STYLES                                 ║
# ╚══════════════════════════════════════════════════════════════════════════╝

LIGHT_QSS = """
* { font-family: "Inter","Roboto","Segoe UI","PingFang SC","Microsoft YaHei",sans-serif;
    font-size: 12pt; color: #222; }
QWidget                 { background: #F4F6FB; }
QFrame[card="true"]     { background: #FFFFFF; border: 1px solid #E0E5EF;
                          border-radius: 12px; }
QFrame[sidebar="true"]  { background: #1A2B5F; border: none; border-radius: 0; }
QPushButton             { background: #F7F9FC; border: 1px solid #DDE3EA;
                          border-radius: 8px; padding: 6px 14px; }
QPushButton:hover       { background: #EEF5FF; border-color: #B5CEFF; }
QPushButton:pressed     { background: #DCEBFF; border-color: #8EB6FF; }
QPushButton[accent="true"]         { background: #2962FF; color:#FFF; border:1px solid #2962FF; }
QPushButton[accent="true"]:hover   { background: #3D6EFF; }
QPushButton[accent="true"]:pressed { background: #2453CC; }
QPushButton[danger="true"]         { background: #D32F2F; color:#FFF; border:1px solid #B71C1C; }
QPushButton[danger="true"]:hover   { background: #E53935; }
QTabWidget::pane        { border: 1px solid #E0E5EF; border-radius: 8px; background: #FAFBFF; }
QTabBar::tab            { background: #F0F3FA; border: 1px solid #E0E5EF;
                          padding: 6px 14px; border-top-left-radius: 8px;
                          border-top-right-radius: 8px; margin-right: 3px; }
QTabBar::tab:selected   { background: #EEF5FF; border-color: #B5CEFF; color: #1A2B5F; }
QScrollBar:vertical     { width: 6px; background: transparent; }
QScrollBar::handle:vertical { background: #CBD3E0; border-radius: 3px; min-height: 24px; }
"""

DARK_QSS = """
* { font-family: "Inter","Roboto","Segoe UI","PingFang SC","Microsoft YaHei",sans-serif;
    font-size: 12pt; color: #E6EAF2; }
QWidget                 { background: #0F1115; }
QFrame[card="true"]     { background: #151922; border: 1px solid #2A3140;
                          border-radius: 12px; }
QFrame[sidebar="true"]  { background: #0A0D12; border: none; border-radius: 0; }
QPushButton             { background: #1B2230; border: 1px solid #2A3140;
                          border-radius: 8px; padding: 6px 14px; color: #E6EAF2; }
QPushButton:hover       { background: #233048; border-color: #37507A; }
QPushButton:pressed     { background: #1E2A41; border-color: #4D6BA1; }
QPushButton[accent="true"]         { background: #2F6BFF; color:#FFF; border:1px solid #2F6BFF; }
QPushButton[accent="true"]:hover   { background: #4077FF; }
QPushButton[accent="true"]:pressed { background: #2A5FDA; }
QPushButton[danger="true"]         { background: #B71C1C; color:#FFF; border:1px solid #9A1515; }
QPushButton[danger="true"]:hover   { background: #D32F2F; }
QTabWidget::pane        { border: 1px solid #2A3140; border-radius: 8px; background: #151922; }
QTabBar::tab            { background: #1B2230; border: 1px solid #2A3140;
                          padding: 6px 14px; border-top-left-radius: 8px;
                          border-top-right-radius: 8px; margin-right: 3px; color: #E6EAF2; }
QTabBar::tab:selected   { background: #233048; border-color: #37507A; color: #FFFFFF; }
QScrollBar:vertical     { width: 6px; background: transparent; }
QScrollBar::handle:vertical { background: #2A3140; border-radius: 3px; min-height: 24px; }
"""

def apply_theme(app, theme: str):
    base = DARK_QSS if theme == 'dark' else LIGHT_QSS
    fs = max(8, int(10 * _S))
    scaled = base.replace('font-size: 12pt', f'font-size: {fs}pt')
    app.setStyleSheet(scaled)

def _plot_theme_params(theme: str) -> dict:
    if theme == 'dark':
        return dict(bg='#0F1115', axis='#6C7A92', text='#C3CEE3', grid=0.18,
                    cross=(180,180,180))
    return dict(bg='w', axis='#B8C1CC', text='#445566', grid=0.22,
                cross=(120,120,120))


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                          DATA MANAGER                                   ║
# ║  Central ring-buffer for all sensor streams.                            ║
# ║  Periodic flush to CSV + trim; memory guard via psutil.                 ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def _percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    vals = sorted(float(v) for v in values)
    if len(vals) == 1:
        return vals[0]
    pos = (len(vals) - 1) * (pct / 100.0)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return vals[lo]
    return vals[lo] + (vals[hi] - vals[lo]) * (pos - lo)

def _nearest_offsets_ms(a_times: list[float], b_times: list[float]) -> list[float]:
    """Absolute nearest-neighbour offsets from series A to series B."""
    if not a_times or not b_times:
        return []
    a = sorted(float(t) for t in a_times)
    b = sorted(float(t) for t in b_times)
    offsets = []
    j = 0
    for t in a:
        while j + 1 < len(b) and abs(b[j + 1] - t) <= abs(b[j] - t):
            j += 1
        offsets.append(abs(b[j] - t))
    return offsets

class DataManager(QtCore.QObject):
    """
    Owns all data buffers.  Thread-safe append via Lock.
    Qt timer drives periodic flush + memory check.

    Buffer key convention:
      "motor_{id}_{field}"   e.g. "motor_0_q", "motor_1_temp"
      "imu_{idx}_{field}"    e.g. "imu_0_roll_deg", "imu_2_ax_g"
      "force_{ch}_{field}"   (future)
      "emg_{ch}_{field}"     (seven fixed logical slots)

    Display buffers are bounded and periodically trimmed. Recording is fed
    directly at acquisition time into a bounded background CSV queue, so
    display eviction cannot silently erase unflushed recording samples.
    """
    sig_record_error = QtCore.pyqtSignal(str)
    sig_memory_warn = QtCore.pyqtSignal(float)   # current MB
    sig_cycle_ready = QtCore.pyqtSignal(object)  # 300 ms packet; optional local subscribers
    sig_flushed     = QtCore.pyqtSignal(str)      # path flushed to

    def __init__(self, parent=None):
        super().__init__(parent)
        self._lock      = Lock()
        self._buffers   : dict[str, deque] = {}
        self._t_buffers : dict[str, deque] = {}   # timestamp per stream key
        self._seq_buffers : dict[str, deque] = {}  # sample id per stream key
        self._stream_seq : dict[str, int] = defaultdict(int)
        self._session_start = datetime.now()
        self._flush_file    = None     # current open CSV file path
        self._flush_writer  = None
        self._flush_fh      = None
        self._flush_headers_written = set()
        self._last_recorded_t_ms = None

        self._export_dir = EXPORT_DIR
        os.makedirs(self._export_dir, exist_ok=True)
        self.session = SessionManifest(self._export_dir)
        self._record_session = None
        self._record_bundle = None

        # Periodic flush timer
        self._flush_timer = QtCore.QTimer(self)
        self._flush_timer.timeout.connect(self._on_flush_tick)
        self._flush_timer.start(int(FLUSH_INTERVAL_S * 1000))

        # Memory guard timer (every 5 s)
        self._mem_timer = QtCore.QTimer(self)
        self._mem_timer.timeout.connect(self._on_memory_check)
        self._mem_timer.start(5000)

        self._recording = False
        self._record_path = None
        self._recorder = None

    # ── Public API ────────────────────────────────────────────────────────
    def append(self, key: str, t_ms: float, value: float):
        prefix, field = self._split_stream_field(key)
        self.append_frame(prefix, t_ms, {field: value})

    def append_frame(self, prefix: str, t_ms: float, values: dict,
                     t_mono_ns: int | None = None, sample_id: int | None = None) -> int:
        """
        Append one acquisition frame.

        Every field in the frame shares the same wall-clock timestamp,
        monotonic timestamp and sample id.  This makes later analysis able to
        prove which fields came from the same hardware poll/read cycle.
        """
        if t_mono_ns is None:
            t_mono_ns = time.perf_counter_ns()
        with self._lock:
            if sample_id is None:
                sample_id = self._stream_seq[prefix]
                self._stream_seq[prefix] += 1
            frame = dict(values)
            frame["sample_id"] = sample_id
            frame["t_mono_ns"] = int(t_mono_ns)
            for field, value in frame.items():
                self._append_locked(f"{prefix}_{field}", t_ms, value, sample_id)
            if self._recording and self._recorder:
                if hasattr(self._recorder, 'enqueue_frames'):
                    self._recorder.enqueue_frames(prefix,[t_ms],[values],[int(t_mono_ns)],[sample_id])
                else:
                    self._recorder.enqueue([[t_ms, prefix, field, sample_id, int(t_mono_ns), value]
                                            for field, value in values.items()],
                                           batch_summary(prefix,[t_ms],[values]))
        return sample_id

    def append_batch(self, prefix, times_ms, frames, mono_ns):
        if not (len(times_ms) == len(frames) == len(mono_ns)):
            raise ValueError("Batch timestamps and frames must have equal lengths")
        if not frames:
            return
        fields = tuple(frames[0])
        if any(tuple(frame) != fields for frame in frames):
            raise ValueError("All frames in a batch must share a schema")
        with self._lock:
            first = self._stream_seq[prefix]
            sample_ids = list(range(first, first + len(frames)))
            self._stream_seq[prefix] += len(frames)
            columns = {field: [frame[field] for frame in frames] for field in fields}
            columns.update(sample_id=sample_ids, t_mono_ns=mono_ns)
            for field, values in columns.items():
                if PIPELINE_ENABLED and prefix.startswith('emg_') and field not in ('raw_v','envelope_v','mvc_ratio','valid','sample_id','t_mono_ns'):
                    continue
                if PIPELINE_ENABLED and prefix.startswith('force_') and field in ('adc_reference_v','adc_range_code','conversion_version'):
                    continue
                key = f"{prefix}_{field}"
                if key not in self._buffers:
                    self._buffers[key] = deque(maxlen=BUFFER_HARD_MAX)
                    self._t_buffers[key] = deque(maxlen=BUFFER_HARD_MAX)
                    self._seq_buffers[key] = deque(maxlen=BUFFER_HARD_MAX)
                self._buffers[key].extend(values)
                self._t_buffers[key].extend(times_ms)
                self._seq_buffers[key].extend(sample_ids)
            if self._recording and self._recorder:
                if hasattr(self._recorder, 'enqueue_frames'):
                    self._recorder.enqueue_frames(prefix,times_ms,frames,mono_ns,sample_ids)
                else:
                    self._recorder.enqueue([
                        [stamp, prefix, field, sid, int(mono), value]
                        for stamp, frame, sid, mono in zip(times_ms, frames, sample_ids, mono_ns)
                        for field, value in frame.items()],batch_summary(prefix,times_ms,frames))

    def record_camera_image(self, depth, timing):
        with self._lock:
            if self._recording and hasattr(self._recorder, 'enqueue_image'):
                return self._recorder.enqueue_image(depth, timing)
        return False

    def append_dict(self, prefix: str, t_ms: float, d: dict):
        """Convenience: append multiple fields from a dict."""
        self.append_frame(prefix, t_ms, d)

    def snapshot(self, key: str) -> tuple[list, list]:
        """Return (timestamps, values) copies — safe to read from UI thread."""
        with self._lock:
            return (list(self._t_buffers.get(key, [])),
                    list(self._buffers.get(key, [])))

    def quality_report(self) -> dict:
        """Return per-stream timing and completeness metrics for diagnostics."""
        with self._lock:
            keys = sorted(self._buffers.keys())
            snapshots = {
                k: (list(self._t_buffers.get(k, [])),
                    list(self._buffers.get(k, [])),
                    list(self._seq_buffers.get(k, [])))
                for k in keys
            }

        streams = sorted({
            k[:-10] for k in keys if k.endswith("_sample_id")
        })
        report = {"streams": {}, "sync": {}}
        for stream in streams:
            t_key = f"{stream}_sample_id"
            times, sample_ids, _seqs = snapshots.get(t_key, ([], [], []))
            times = [float(t) for t in times]
            sample_ids = [int(v) for v in sample_ids]
            dts = [b - a for a, b in zip(times, times[1:]) if b >= a]
            missing = 0
            if sample_ids:
                missing = max(0, (max(sample_ids) - min(sample_ids) + 1) - len(set(sample_ids)))
            stream_keys = [k for k in keys if k.startswith(f"{stream}_")]
            counts = [len(snapshots[k][0]) for k in stream_keys]
            report["streams"][stream] = {
                "samples": len(times),
                "fields": len(stream_keys),
                "field_count_min": min(counts) if counts else 0,
                "field_count_max": max(counts) if counts else 0,
                "duration_s": ((times[-1] - times[0]) / 1000.0) if len(times) > 1 else 0.0,
                "observed_hz": (1000.0*(len(times)-1)/(times[-1]-times[0])) if len(times)>1 and times[-1]>times[0] else 0.0,
                "inverse_median_interval_hz": (1000.0/statistics.median(dts)) if dts and statistics.median(dts)>0 else 0.0,
                "median_dt_ms": statistics.median(dts) if dts else 0.0,
                "max_gap_ms": max(dts) if dts else 0.0,
                "jitter_p95_ms": _percentile([abs(dt - statistics.median(dts)) for dt in dts], 95) if dts else 0.0,
                "duplicate_timestamps": len(times) - len(set(times)),
                "missing_sample_ids": missing,
                "field_counts_match": len(set(counts)) <= 1 if counts else True,
            }

        stream_times = {
            s: snapshots.get(f"{s}_sample_id", ([], [], []))[0]
            for s in streams
        }
        for i, a in enumerate(streams):
            for b in streams[i + 1:]:
                offsets = _nearest_offsets_ms(stream_times.get(a, []), stream_times.get(b, []))
                if offsets:
                    report["sync"][f"{a}<->{b}"] = {
                        "pairs": len(offsets),
                        "median_abs_offset_ms": statistics.median(offsets),
                        "p95_abs_offset_ms": _percentile(offsets, 95),
                        "max_abs_offset_ms": max(offsets),
                    }
        return report

    def keys(self) -> list[str]:
        with self._lock:
            return list(self._buffers.keys())

    def start_recording(self, path: str | None = None):
        if self._recorder and not self.stop_recording():
            return False
        try:
            bundle = None
            if path is None:
                bundle = RecordingBundle(self._export_dir,self.session.metadata,self.session.session_id)
                path = str(bundle.csv_path)
            if PIPELINE_ENABLED and bundle:
                acquisition = dict(schema='hipexo-acquisition/2',software_release='native-v3.1-995hz-20261010',
                    capture_architecture='isolated ADC, per-bus IMU and native zero-output motor processes; active motor control uses SDK port threads; 20ms bounded batches',
                    tuning_requested=os.environ.get('HIPEXO_DISABLE_TUNING')!='1',tuning_note='Scoped controller awake + FIFO10; board-checked I2C-1 source clock 136 to 204 MHz while capturing, then restored; no CPU/GPU overclock',i2c_clock_tuning_requested=os.environ.get('HIPEXO_I2C_CLOCK_TUNING','1')!='0',imu_devices=IMU_DEVICES,
                    imu_poll_targets_hz={'i2c_7':200,'i2c_1':200},imu_config_note='RRATE 200Hz verified on 2026-10-10; hardware register output rate is not measured host/native update rate',
                    force_channels=FORCE_CHANNELS,force_target_hz=FORCE_SAMPLE_HZ,force_spi_hz=FORCE_SPI_SPEED,force_adc='ADS8688',force_adc_reference_v=4.096,force_conversion_version='ADS8688-datasheet-v1',
                    motor_native_target_hz=__import__('hipexo_motor_process').configured_target_hz(),motor_log_target_hz=MOTOR_LOG_HZ,motor_gear_ratio=MOTOR_GEAR_RATIO,motor_native_monitor_requested=os.environ.get('HIPEXO_MOTOR_NATIVE','1')!='0',motor_timestamp_note='Native monitor uses host validated-frame time; asynchronous replies are not paired to requests; round-trip field is NaN; inspect motor_*_transport_quality.json',
                    camera_mode='images' if VISION_IMAGE_ONLY else 'inference',camera_target_fps=VISION_CAPTURE_FPS,
                    timestamp_semantics='Local host read completion; EMG mapped source time; not hardware synchronized')
                (bundle.directory/'acquisition_config.json').write_text(json.dumps(acquisition,ensure_ascii=False,indent=2),encoding='utf-8')
            recorder = (CycleRecorder(path, self.sig_record_error.emit, self.sig_cycle_ready.emit if self.receivers(self.sig_cycle_ready) else None)
                        if PIPELINE_ENABLED and bundle else FrameRecorder(path, self.sig_record_error.emit))
            try:
                self.session.artifact('processed_csv', path,
                                      scope='processed/summary frames; camera samples and LiDAR raw scans are separate')
            except Exception:
                recorder.stop()
                raise
            with self._lock:
                self._recorder = recorder
                self._record_session = self.session
                self._record_bundle = bundle
                self._record_path = path
                self._recording = True
            return True
        except Exception as exc:
            self.sig_record_error.emit(f"Cannot start recording: {exc}")
            return False

    def stop_recording(self):
        with self._lock:
            self._recording = False
            if self._record_bundle:
                self._record_bundle.mark_stopping()
            recorder = self._recorder
        if recorder is None:
            return True
        ok = recorder.stop()
        # Retain a draining recorder so shutdown / a later stop can check it again.
        if not recorder._thread.is_alive():
            self._recorder = None
            if self._record_bundle:
                original = str(recorder.path)
                try:
                    self._record_bundle.finalize(recorder.stats,ok,recorder.error,
                                                 self._record_session.raw_references())
                except Exception as exc:
                    ok = False
                    self.sig_record_error.emit(f"CSV retained, but naming/summary failed: {exc}")
                recorder.path = str(self._record_bundle.csv_path)
                self._record_path = recorder.path
                try:
                    self._record_session.event('recording_finalized',original_path=original,
                                               path=recorder.path,complete=ok,
                                               sensors=sensors_in(recorder.stats))
                except OSError as exc:
                    ok = False
                    self.sig_record_error.emit(f"Session manifest write failed: {exc}")
                self._record_bundle = None
            if self._record_session:
                try:
                    self._record_session.event('csv_closed', path=str(recorder.path),
                                               complete=ok, error=recorder.error)
                except OSError as exc:
                    self.sig_record_error.emit(f"Session manifest write failed: {exc}")
                    ok = False
                self._record_session = None
        if ok:
            self.sig_flushed.emit(recorder.path)
        return ok

    def suggested_export_path(self):
        with self._lock:
            stats = {}
            for key, values in self._buffers.items():
                stream,field = self._split_stream_field(key)
                if field=='sample_id':
                    valid = self._buffers.get(stream+'_valid',())
                    count = sum(v==1 for v in valid) if stream.startswith('emg_') else len(values)
                    stats[stream]={'valid_samples':count}
        sensors = '-'.join(sensors_in(stats)) or 'NoValidData'
        return os.path.join(self._export_dir,self.session.file_stem(sensors)+'__snapshot.csv')

    def _append_locked(self, key: str, t_ms: float, value: float, sample_id: int | None):
        if key not in self._buffers:
            self._buffers[key]   = deque(maxlen=BUFFER_HARD_MAX)
            self._t_buffers[key] = deque(maxlen=BUFFER_HARD_MAX)
            self._seq_buffers[key] = deque(maxlen=BUFFER_HARD_MAX)
        self._buffers[key].append(value)
        self._t_buffers[key].append(float(t_ms))
        self._seq_buffers[key].append(sample_id)

    def export_snapshot_csv(self, path: str):
        """
        One-shot export.  All channels are aligned to a shared time axis by
        nearest-neighbour lookup keyed on t_ms.  This correctly handles
        channels with different sample rates (e.g. motor @ 5 kHz vs IMU @ 50 Hz).

        Output columns:
          t_ms  |  motor_0_q  motor_0_dq  motor_0_temp  motor_1_*  |
                   imu_0_ax_g  imu_0_ay_g  ...  imu_0_temp_c  imu_1_*  ...
        Rows are indexed by the union of all timestamps, sorted ascending.
        Each cell contains the most-recent known value for that channel at
        that timestamp (forward-fill / step interpolation).
        """
        with self._lock:
            keys = sorted(self._buffers.keys())
            if not keys:
                return 0
            rows = self._build_aligned_rows(keys)
        if not rows:
            return 0
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_ms'] + keys)
            w.writerows(rows)
        return len(rows)

    # ── Internal ──────────────────────────────────────────────────────────
    def _build_aligned_rows(self, keys: list) -> list:
        """
        Build time-aligned rows from all buffers (must be called under lock).

        Strategy:
          1. Collect all unique timestamps across every channel.
          2. Sort them ascending.
          3. For each timestamp, each channel value = the most recent sample
             at or before that timestamp (nearest-neighbour, forward-fill).
             Channels that have no data yet for that time get "".

        Motor runs at ~5 kHz; IMU at 50 Hz.  The union timestamp grid will
        have ~5000 pts/s, with motor columns fully populated and IMU columns
        holding their last value for 100 consecutive rows then updating.
        This is the standard approach used in ROS bag CSV exports and
        commercial motion-capture loggers.
        """
        # Gather all (t_ms, key, value) events
        all_t = set()
        events: dict[str, list] = {}   # key -> sorted list of (t_ms, value)
        for k in keys:
            tb = list(self._t_buffers[k])
            vb = list(self._buffers[k])
            pairs = sorted(zip(tb, vb), key=lambda pair: pair[0])
            events[k] = pairs
            all_t.update(tb)

        if not all_t:
            return []

        sorted_t = sorted(all_t)

        # For each channel maintain a cursor and last known value
        cursors   = {k: 0   for k in keys}
        last_vals = {k: ""  for k in keys}

        rows = []
        for t in sorted_t:
            for k in keys:
                pairs  = events[k]
                cur    = cursors[k]
                # Advance cursor while next sample's timestamp <= t
                while cur < len(pairs) and pairs[cur][0] <= t:
                    last_vals[k] = pairs[cur][1]
                    cur += 1
                cursors[k] = cur
            row = [t] + [last_vals[k] for k in keys]
            rows.append(row)
        return rows

    @staticmethod
    def _split_stream_field(key: str) -> tuple[str, str]:
        parts = key.split("_")
        if len(parts) >= 3 and (parts[1].isdigit() or parts[1] in ('L', 'R')):
            return "_".join(parts[:2]), "_".join(parts[2:])
        if len(parts) >= 2:
            return parts[0], "_".join(parts[1:])
        return key, "value"

    def _build_long_rows_since(self, keys: list, last_t_ms: float | None) -> list:
        """Build fixed-schema recording rows from buffers (must hold lock)."""
        t_mono_by_sample = {}
        for key in keys:
            stream, field = self._split_stream_field(key)
            if field != "t_mono_ns":
                continue
            for value, sample_id in zip(self._buffers[key], self._seq_buffers[key]):
                if sample_id is not None:
                    t_mono_by_sample[(stream, int(sample_id))] = int(value)

        rows = []
        for key in keys:
            stream, field = self._split_stream_field(key)
            if field in ("sample_id", "t_mono_ns"):
                continue
            for t_ms, value, sample_id in zip(self._t_buffers[key],
                                              self._buffers[key],
                                              self._seq_buffers[key]):
                if last_t_ms is not None and t_ms <= last_t_ms:
                    continue
                sid = "" if sample_id is None else int(sample_id)
                t_mono_ns = t_mono_by_sample.get((stream, sid), "") if sid != "" else ""
                rows.append([t_ms, stream, field, sid, t_mono_ns, value])
        rows.sort(key=lambda r: (r[0], r[1], r[2]))
        return rows

    def _on_flush_tick(self):
        # Recording is streamed independently; these buffers serve snapshots only.
        self._trim_buffers()

    def _flush_to_open_csv(self):
        # Compatibility for the memory guard. The recorder flushes every second.
        return

    def _trim_buffers(self):
        """Legacy trimming only; bounded cycle-mode rings evict incrementally.

        Rebuilding all rings every 30s held the GIL/data lock for ~200ms and
        stalled motor reads. Their maxlen already provides a hard memory bound.
        """
        if PIPELINE_ENABLED:return
        with self._lock:
            for k in list(self._buffers.keys()):
                buf = self._buffers[k]
                tbuf = self._t_buffers[k]
                if len(buf) > DISPLAY_KEEP_PTS:
                    trimmed   = deque(list(buf)[-DISPLAY_KEEP_PTS:],
                                      maxlen=BUFFER_HARD_MAX)
                    trimmed_t = deque(list(tbuf)[-DISPLAY_KEEP_PTS:],
                                      maxlen=BUFFER_HARD_MAX)
                    trimmed_s = deque(list(self._seq_buffers[k])[-DISPLAY_KEEP_PTS:],
                                      maxlen=BUFFER_HARD_MAX)
                    self._buffers[k]   = trimmed
                    self._t_buffers[k] = trimmed_t
                    self._seq_buffers[k] = trimmed_s

    def _on_memory_check(self):
        if not _PSUTIL_OK:
            return
        try:
            rss_mb = _psutil.Process(os.getpid()).memory_info().rss / 1e6
        except Exception:
            return
        if rss_mb >= MEMORY_CRIT_MB:
            self._on_flush_tick()   # emergency flush + trim
            self.sig_memory_warn.emit(rss_mb)
        elif rss_mb >= MEMORY_WARN_MB:
            self.sig_memory_warn.emit(rss_mb)

    @property
    def export_dir(self) -> str:
        return self._export_dir

    def set_export_dir(self, path: str, **metadata):
        """Point future recordings/exports at a new directory (e.g. when the
        operator switches subject/session). Does not affect a recording
        that's already in progress — stop it first if you need to redirect
        mid-session."""
        session = SessionManifest(path, **metadata)
        self._export_dir = path
        self.session = session


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                    SESSION / SAVE-LOCATION MANAGEMENT                   ║
# ║  Touch-screen friendly subject tracking + collision-free auto folders.  ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def _safe_path_component(s: str) -> str:
    """Sanitize a string for safe use as a single folder/file name component."""
    s = (s or "").strip()
    if not s:
        return "unknown"
    out = []
    for ch in s:
        if ch.isalnum() or ch in ("-", "_"):
            out.append(ch)
        elif ch.isspace():
            out.append("_")
        # anything else (/, \, :, etc.) is dropped
    result = "".join(out).strip("_")
    return result or "unknown"


def make_unique_session_dir(base_dir: str, subject_id: str, location: str,
                             when: datetime = None) -> str:
    """
    Build and CREATE a fresh, collision-free session directory:
        {base_dir}/{subject}_{location}_{YYYYMMDD_HHMMSS}[_{n}]/

    Guarantees no two calls ever collide/overwrite each other — including
    across app restarts, which was the specific failure mode this needed to
    solve (operator restarts the app for the same subject/location within
    the same run, or the wall-clock second happens to repeat). If the
    timestamped name is already taken, a numeric suffix is appended and
    incremented until an unused name is found.
    """
    when = when or datetime.now()
    ts   = when.strftime("%Y%m%d_%H%M%S")
    stem = f"{_safe_path_component(subject_id)}_{_safe_path_component(location)}_{ts}"
    os.makedirs(base_dir, exist_ok=True)
    candidate = os.path.join(base_dir, stem)
    n = 1
    while True:
        try:
            os.makedirs(candidate, exist_ok=False)
            return candidate
        except FileExistsError:
            n += 1
            candidate = os.path.join(base_dir, f"{stem}_{n}")


class SessionManager:
    """
    Replaces the old free-form multi-field ProfileDialog with the minimum
    an operator needs on a touch screen: a subject ID and a location.

    Save-location policy:
      • Auto mode (default) — every confirmed (subject, location) gets its
        own freshly created, collision-free folder under EXPORT_DIR, named
        "{subject}_{location}_{timestamp}" (see make_unique_session_dir()).
        A restart of the app (same subject/location) simply gets another
        timestamped folder — never overwrites a previous one.
      • Manual mode — operator picks an exact folder once via the Session
        dialog; every session then writes directly into that folder until
        they explicitly switch back to Auto.

    Subject IDs and each subject's location history are persisted to
    SESSION_STATE_PATH (~/hipexo_session_state.json) so the dialog can
    default to "continue where we left off" without any typing.
    """

    _SUBJECT_RE = re.compile(r"^S(\d+)$")

    def __init__(self, state_path: str = SESSION_STATE_PATH, base_dir: str = EXPORT_DIR):
        self._path     = state_path
        self._base_dir = base_dir
        self._state    = self._load()

    # ── persistence ──────────────────────────────────────────────────────
    def _load(self) -> dict:
        state = {
            "subjects": {},          # subject_id -> {"locations": [...]}
            "last_subject": "",
            "last_location": "",
            "manual_save_path": None,
        }
        if os.path.exists(self._path):
            try:
                with open(self._path) as f:
                    loaded = json.load(f)
                for k in state:
                    if k in loaded:
                        state[k] = loaded[k]
            except Exception as e:
                print(f"[SessionManager] Failed to load {self._path}: {e}")
        return state

    def save(self):
        try:
            with open(self._path, 'w') as f:
                json.dump(self._state, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"[SessionManager] Save failed: {e}")

    # ── subject / location bookkeeping ────────────────────────────────────
    def known_subjects(self) -> list:
        return sorted(self._state["subjects"].keys())

    def locations_for(self, subject_id: str) -> list:
        return list(self._state["subjects"].get(subject_id, {}).get("locations", []))

    def all_locations(self) -> list:
        seen = []
        for info in self._state["subjects"].values():
            for loc in info.get("locations", []):
                if loc not in seen:
                    seen.append(loc)
        return seen

    def next_subject_id(self) -> str:
        nums = [int(m.group(1)) for sid in self._state["subjects"]
                if (m := self._SUBJECT_RE.match(sid))]
        return f"S{(max(nums) + 1) if nums else 1:03d}"

    def last_subject(self) -> str:
        return self._state.get("last_subject") or self.next_subject_id()

    def last_location(self) -> str:
        return self._state.get("last_location") or ""

    def record_use(self, subject_id: str, location: str):
        """Remember that a session was just started for (subject, location)."""
        info = self._state["subjects"].setdefault(subject_id, {"locations": []})
        if location and location not in info["locations"]:
            info["locations"].append(location)
        self._state["last_subject"]  = subject_id
        self._state["last_location"] = location
        self.save()

    # ── manual save-path override ─────────────────────────────────────────
    @property
    def manual_path(self):
        return self._state.get("manual_save_path")

    def set_manual_path(self, path):
        self._state["manual_save_path"] = path
        self.save()

    # ── session directory resolution ────────────────────────────────────
    def resolve_session_dir(self, subject_id: str, location: str) -> str:
        """Create (if needed) and return the directory this session's data
        should be written into."""
        if self.manual_path:
            os.makedirs(self.manual_path, exist_ok=True)
            return self.manual_path
        return make_unique_session_dir(self._base_dir, subject_id, location)


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                            IMU WORKER                                   ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def _le_i16(lo: int, hi: int) -> int:
    v = lo | (hi << 8)
    return v - 0x10000 if (v & 0x8000) else v

def _read_imu_block(bus, addr: int) -> bytes:
    """Three tegra-i2c-safe transactions (≤12 bytes each)."""
    p1 = bus.read_i2c_block_data(addr, 0x34, 12)   # AX..GZ
    p2 = bus.read_i2c_block_data(addr, 0x3A, 12)   # HX..Yaw
    p3 = bus.read_i2c_block_data(addr, 0x40,  2)   # TEMP
    return bytes(p1) + bytes(p2) + bytes(p3)


def _parse_imu_block(buf: bytes):
    """Return (data_dict, temp_c) or (None, None) on bad data."""
    if len(buf) != IMU_BLOCK_LEN:
        return None, None
    raw = [_le_i16(buf[2*i], buf[2*i+1]) for i in range(13)]
    if all(v == 0 for v in raw[:12]):
        return None, None
    d = {
        "ax_g":      raw[0]  / 32768.0 * 16.0,
        "ay_g":      raw[1]  / 32768.0 * 16.0,
        "az_g":      raw[2]  / 32768.0 * 16.0,
        "gx_dps":    raw[3]  / 32768.0 * 2000.0,
        "gy_dps":    raw[4]  / 32768.0 * 2000.0,
        "gz_dps":    raw[5]  / 32768.0 * 2000.0,
        "roll_deg":  raw[9]  / 32768.0 * 180.0,
        "pitch_deg": raw[10] / 32768.0 * 180.0,
        "yaw_deg":   raw[11] / 32768.0 * 180.0,
    }
    return d, raw[12] / 100.0


class ImuWorker(QtCore.QObject):
    """
    Runs a background thread that polls all configured IMU devices.
    Emits sig_update(sensor_idx, data_dict, temp_c) at IMU_PERIOD_S rate.
    Emits sig_conn_status(sensor_idx, online) whenever a sensor's connection
    state changes (present ↔ absent, or drops out mid-run and reconnects).
    Supports up to 4 sensors (IMU_DEVICES).
    """
    sig_update      = QtCore.pyqtSignal(int, dict, float)  # idx, data, temp_c
    sig_status      = QtCore.pyqtSignal(str)               # info / error text
    sig_conn_status = QtCore.pyqtSignal(int, bool)         # idx, online

    def __init__(self, data_manager: DataManager, parent=None):
        super().__init__(parent)
        self._dm      = data_manager
        self._running = Event()
        self._alive   = True
        self._thread  = None
        self._buses   = {}
        # Per-sensor connection bookkeeping (idx -> ...)
        self._online       = {}   # idx -> bool
        self._last_ok_time = {}   # idx -> perf_counter() of last good sample
        self._last_probe   = {}   # idx -> perf_counter() of last presence retry
        self._errors = {}
        self.reference = ImuReference(len(IMU_DEVICES))
        self._rate_times = {i:deque(maxlen=200) for i in range(len(IMU_DEVICES))}

    def is_online(self, idx: int) -> bool:
        return self._online.get(idx, False)

    def error_detail(self, idx: int) -> str:
        return self._errors.get(idx, '')

    def actual_rate(self, idx):
        stamps=list(self._rate_times[idx])
        if len(stamps)<2 or time.perf_counter()-stamps[-1]>.5:return 0.
        return (len(stamps)-1)/(stamps[-1]-stamps[0]) if stamps[-1]>stamps[0] else 0.

    def request_reference(self):
        now = time.perf_counter()
        online = [self._running.is_set() and self.is_online(i) and
                  now-self._last_ok_time.get(i, -1e9) <= IMU_OFFLINE_AFTER_S
                  for i in range(len(IMU_DEVICES))]
        return self.reference.request(online, self._dm.session)

    def start(self):
        if not _SMBUS_OK:
            self.sig_status.emit("[IMU] smbus2 not installed")
            return False
        self._last_probe.clear()
        self._running.set()
        if self._thread and self._thread.is_alive():
            return True
        self._thread = Thread(target=self._loop, daemon=True, name="imu-worker")
        self._thread.start()
        return True

    def stop(self):
        self._running.clear()
        self.reference.invalidate(reason="已停止，需重新设置")
        for idx in range(len(IMU_DEVICES)):
            self._set_online(idx, False)

    def shutdown(self):
        self._alive = False
        self.stop()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _set_online(self, idx: int, online: bool):
        """Emit sig_conn_status only on an actual state transition."""
        prev = self._online.get(idx)
        self._online[idx] = online
        if prev != online:
            if not online:
                self.reference.invalidate(idx, "断线后需重新设置")
            self.sig_conn_status.emit(idx, online)

    def _loop(self):
        workers = [Thread(target=self._bus_loop, args=(bus_id,), daemon=True,
                          name=f"imu-i2c-{bus_id}") for bus_id in sorted({b for b,_ in IMU_DEVICES})]
        for worker in workers: worker.start()
        for worker in workers: worker.join()

    def _bus_loop(self,bus_id):
        if os.environ.get('HIPEXO_IMU_IN_PROCESS')=='1':return self._bus_loop_legacy(bus_id)
        import socket,struct,subprocess,json
        from hipexo_realtime import ControllerPowerLease
        from hipexo_i2c_clock import I2cClockLease
        indices=[i for i,(b,_) in enumerate(IMU_DEVICES) if b==bus_id]
        last_bytes={};last_gui={}
        while self._alive:
            if not self._running.is_set():time.sleep(.02);continue
            sock,other=socket.socketpair();process=None
            power=ControllerPowerLease(f'/sys/class/i2c-dev/i2c-{bus_id}/device');power.__enter__()
            clock=I2cClockLease(bus_id);clock.__enter__()
            self.sig_status.emit(f'[IMU i2c-{bus_id}] controller tuning: {power.report}; clock: {clock.report}')
            try:
                config=dict(bus=bus_id,devices=[(i,IMU_DEVICES[i][1]) for i in indices],retry_s=IMU_RESCAN_INTERVAL_S)
                process=subprocess.Popen([sys.executable,os.path.join(os.path.dirname(__file__),'hipexo_imu_capture.py'),
                    str(other.fileno()),json.dumps(config)],pass_fds=(other.fileno(),),stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                other.close();sock.settimeout(.2);pending=bytearray();stopping=False;last_packet=time.perf_counter()
                while True:
                    if (not self._alive or not self._running.is_set()) and not stopping:
                        sock.sendall(b'stop');stopping=True;stop_deadline=time.perf_counter()+1
                    if stopping and time.perf_counter()>stop_deadline:raise TimeoutError('IMU stop drain timed out')
                    try:
                        data=sock.recv(65536)
                        if not data:
                            if stopping:break
                            raise EOFError('IMU process disconnected')
                        pending.extend(data)
                    except socket.timeout:
                        if process.poll() is not None:raise RuntimeError('IMU process exited')
                        if time.perf_counter()-last_packet>IMU_OFFLINE_AFTER_S:
                            for idx in indices:self._errors[idx]='IMU IPC timeout';self._set_online(idx,False)
                        continue
                    while len(pending)>=4:
                        n=struct.unpack('!I',pending[:4])[0]
                        if n>1024*1024:raise ValueError('IMU packet exceeds limit')
                        if len(pending)<4+n:break
                        packet=json.loads(pending[4:4+n]);del pending[:4+n]
                        if 'error' in packet:raise RuntimeError(packet['error'])
                        if 'tuning' in packet:
                            self.sig_status.emit(f'[IMU i2c-{bus_id}] process tuning: {packet["tuning"]}');continue
                        last_packet=time.perf_counter()
                        groups={i:[] for i in indices}
                        for row in packet['samples']:
                            idx=row['idx'];addr=IMU_DEVICES[idx][1]
                            if 'error' in row:
                                self._errors[idx]=row['error']
                                # A failed transaction is explicit evidence, not a slow display update.
                                self._set_online(idx,False);continue
                            block=bytes(row['raw']);parsed,_=_parse_imu_block(block[:12]+bytes(6)+block[12:]+bytes(2))
                            if parsed is None:raise ValueError('Invalid IMU packet')
                            now=row['mono']/1e9;self._last_ok_time[idx]=now;self._rate_times[idx].append(now)
                            self._errors.pop(idx,None);self._set_online(idx,True)
                            parsed.update(read_duration_ms=row['duration_ms'],repeated_register_block=int(last_bytes.get(idx)==block))
                            last_bytes[idx]=block
                            with self.reference.lock:
                                frame=self.reference.process(idx,parsed,self._dm.session,IMU_DEVICES)
                                frame=dict(frame,i2c_bus=bus_id,i2c_address=addr,temp_c=row['temp'])
                                groups[idx].append((row['wall']/1e6,frame,row['mono']))
                            if now-last_gui.get(idx,0)>=.02:
                                last_gui[idx]=now;self.sig_update.emit(idx,frame,row['temp'])
                        for idx,rows in groups.items():
                            if rows:self._dm.append_batch(f'imu_{idx}',[r[0] for r in rows],[r[1] for r in rows],[r[2] for r in rows])
            except Exception as exc:
                self.sig_status.emit(f'[IMU i2c-{bus_id}] {exc}')
                for idx in indices:self._errors[idx]=str(exc);self._set_online(idx,False)
            finally:
                other.close();sock.close()
                if process:
                    process.terminate()
                    try:process.wait(timeout=1)
                    except subprocess.TimeoutExpired:process.kill();process.wait()
                clock.__exit__(None,None,None)
                if 'restore_error' in clock.report:self.sig_status.emit(f'[IMU] {clock.report["restore_error"]}')
                power.__exit__(None,None,None)
                if 'restore_error' in power.report:self.sig_status.emit(f'[IMU] {power.report["restore_error"]}')
                if not self._running.is_set():
                    for idx in indices:self._set_online(idx,False)
            if self._alive and self._running.is_set():time.sleep(.5)

    def _bus_loop_legacy(self, bus_id):
        import fcntl
        indices=[i for i,(b,_) in enumerate(IMU_DEVICES) if b==bus_id]
        # Target 200 Hz on each bus; native timestamps reveal actual achieved rate.
        period = min(IMU_PERIOD_S, 1.0/200)
        bus=None; lease=None; next_probe=0.; deadline=time.perf_counter()
        temperatures={};last_temp={};last_gui={};last_bytes={}
        try:
            lease=open(f'/tmp/hipexo-i2c-{bus_id}.lock','w')
            fcntl.flock(lease, fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError as exc:
            for i in indices:
                self._errors[i]=f'I2C bus already owned: {exc}';self._set_online(i,False)
            if lease:lease.close()
            return
        from hipexo_realtime import ControllerPowerLease,tune_current_process
        power=ControllerPowerLease(f'/sys/class/i2c-dev/i2c-{bus_id}/device')
        power.__enter__()
        self.sig_status.emit(f'[IMU i2c-{bus_id}] tuning: {tune_current_process()}, {power.report}')
        try:
            while self._alive:
                if not self._running.is_set():
                    time.sleep(.02);deadline=time.perf_counter();continue
                now=time.perf_counter()
                if bus is None:
                    if now<next_probe:time.sleep(.02);continue
                    next_probe=now+IMU_RESCAN_INTERVAL_S
                    try:bus=SMBus(bus_id);self._buses[bus_id]=bus
                    except Exception as exc:
                        for i in indices:
                            self._errors[i]=f'Cannot open i2c-{bus_id}: {exc}';self._set_online(i,False)
                        continue
                for idx in indices:
                    addr=IMU_DEVICES[idx][1];now=time.perf_counter()
                    if not self._online.get(idx,False) and now-self._last_probe.get(idx,-1e9)<IMU_RESCAN_INTERVAL_S:continue
                    self._last_probe[idx]=now
                    try:
                        began=time.perf_counter_ns()
                        p1=bus.read_i2c_block_data(addr,0x34,12)
                        p2=bus.read_i2c_block_data(addr,0x3d,6)
                        completed=time.perf_counter_ns();wall=time.time_ns()
                        block=bytes(p1+p2)
                        if len(block)!=18 or all(v==0 for v in block) or all(v==255 for v in block):raise IOError('all-zero/all-FF block (sensor not responding)')
                        # Existing parser expects unused magnetic registers in the middle.
                        parsed,temp=_parse_imu_block(bytes(p1)+bytes(6)+bytes(p2)+bytes(2))
                        if parsed is None:raise IOError('bad IMU block')
                        if now-last_temp.get(idx,-1e9)>=1:
                            raw=bus.read_i2c_block_data(addr,0x40,2)
                            temperatures[idx]=_le_i16(*raw)/100;last_temp[idx]=now
                        self._last_ok_time[idx]=now;self._rate_times[idx].append(now);self._errors.pop(idx,None);self._set_online(idx,True)
                        parsed.update(read_duration_ms=(completed-began)/1e6,
                                      repeated_register_block=int(last_bytes.get(idx)==block))
                        last_bytes[idx]=block
                        with self.reference.lock:
                            data=self.reference.process(idx,parsed,self._dm.session,IMU_DEVICES)
                            frame=dict(data,i2c_bus=bus_id,i2c_address=addr,temp_c=temperatures.get(idx,float('nan')))
                            self._dm.append_frame(f'imu_{idx}',wall/1e6,frame,t_mono_ns=completed)
                        if now-last_gui.get(idx,-1e9)>=.02:
                            self.sig_update.emit(idx,data,temperatures.get(idx,float('nan')));last_gui[idx]=now
                    except Exception as exc:
                        self._errors[idx]=str(exc)
                        if now-self._last_ok_time.get(idx,-1e9)>=IMU_OFFLINE_AFTER_S:self._set_online(idx,False)
                deadline+=period
                delay=deadline-time.perf_counter()
                if delay>0:time.sleep(delay)
                elif delay < -period:deadline=time.perf_counter()  # bound catch-up to one period
        finally:
            if bus:bus.close()
            self._buses.pop(bus_id,None)
            power.__exit__(None,None,None)
            lease.close()


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                          MOTOR WORKER                                   ║
# ║  Logic identical to interface13.py DualMotorWorker.                     ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def _sat(x: float, limit: float):
    if limit is None or limit <= 0:
        return x
    return max(-limit, min(limit, x))

def _traj_eval(traj_cfg: dict, t: float):
    cfg = dict(DEFAULT_TRAJ)
    if traj_cfg:
        cfg.update(traj_cfg)
    if str(cfg.get("type", "CONST")).upper() == "SINE":
        q0  = float(cfg.get("q0", 0.0))
        A   = float(cfg.get("amp", 0.0))
        f   = float(cfg.get("freq", 0.5))
        phi = float(cfg.get("phase", 0.0))
        w   = 2 * math.pi * f
        return (q0 + A*math.sin(w*t + phi),
                A*w*math.cos(w*t + phi),
                -A*w*w*math.sin(w*t + phi))
    return float(cfg.get("q0", 0.0)), 0.0, 0.0


def _ensure_motor_mode_for_ports(ports, swmotor_path):
    if not os.path.exists(swmotor_path):
        raise FileNotFoundError(f"swmotor not found: {swmotor_path}")
    for p in sorted(set(ports)):
        os.system(f"sudo chmod 777 {swmotor_path} {p}")
        ret = os.system(f"sudo {swmotor_path} {p}")
        if ret != 0:
            raise RuntimeError(f"swmotor failed for {p}, ret={ret}")
    time.sleep(1.0)


class MotorWorker(QtCore.QObject):
    sig_update      = QtCore.pyqtSignal(int, float, float, float, int)
    sig_error       = QtCore.pyqtSignal(str)
    sig_conn_status = QtCore.pyqtSignal(int, bool)   # idx, online

    def __init__(self, devices, data_manager: DataManager, parent=None):
        super().__init__(parent)
        self._dm      = data_manager
        self.devices  = devices
        self.port_to_ids = defaultdict(list)
        for p, mid in devices:
            self.port_to_ids[p].append(mid)
        self.serials  = {}
        self._port_leases = {}
        self._fast_handles = {}
        self._fast_disabled_ports = set()
        self.transport_stats = {}
        from threading import RLock
        self._io_lock = RLock()
        self._port_locks = {p:RLock() for p in self.port_to_ids}
        self._motor_state = {}
        self.cmd      = MotorCmd()  if _SDK_OK else None
        self.data     = MotorData() if _SDK_OK else None
        self.running  = Event()
        self.collecting = Event()
        self._thread  = None
        self._alive   = True
        self.t0       = None
        self._current_t  = 0.0
        self._loop_count = 0
        # Per-motor connection bookkeeping (idx -> ...)
        self._online        = {}
        self._last_ok_time  = {}
        self._last_err_emit = {}

    def is_online(self, idx: int) -> bool:
        return self._online.get(idx, False)

    def _set_online(self, idx: int, online: bool):
        prev = self._online.get(idx)
        self._online[idx] = online
        if prev != online:
            self.sig_conn_status.emit(idx, online)

    def init_links(self):
        if not _SDK_OK:
            self.sig_error.emit("[Motor] SDK not available")
            return False
        try:
            for port in self.port_to_ids:
                if port not in self.serials:
                    import fcntl
                    lease=open('/tmp/hipexo-motor-'+os.path.basename(port)+'.lock','w')
                    try:
                        fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
                        self.serials[port] = SerialPort(port)
                        self._port_leases[port]=lease
                    except Exception:
                        lease.close();raise
            return True
        except Exception as e:
            self.sig_error.emit(f"[Motor] Open serial: {e}")
            return False

    def _compose_and_send(self, serial, motor_id: int, active: bool):
        port=next(p for p,link in self.serials.items() if link is serial)
        with self._port_locks[port]:
            key=(port,motor_id)
            if key not in self._motor_state:self._motor_state[key]=(MotorCmd(),MotorData())
            return self._compose_locked(serial,motor_id,active and self.running.is_set(),*self._motor_state[key])

    def _compose_locked(self, serial, motor_id: int, active: bool, cmd=None, data=None):
        cmd = self.cmd if cmd is None else cmd
        data = self.data if data is None else data
        cfg = MOTOR_PARAMS.get(motor_id, {
            "MODE": "ZERO", "KP": 0.0, "KD": 0.0, "TAU": 0.0,
            "Q_SET": 0.0, "DQ_SET": None, "DQ_SCALE": "GEAR",
            "K": 0.0, "D": 0.0, "M": 0.0, "TRAJ": DEFAULT_TRAJ
        })
        if not _SDK_OK or cmd is None or data is None:
            return 0.0, 0.0, 0.0, 0

        mode = str(cfg.get("MODE", "ZERO")).upper()
        kp   = float(cfg.get("KP",  0.0))
        kd   = float(cfg.get("KD",  0.0))
        tau  = float(cfg.get("TAU", 0.0))
        qset = float(cfg.get("Q_SET", 0.0))
        dqset   = cfg.get("DQ_SET", None)
        dqscale = cfg.get("DQ_SCALE", "GEAR")

        data.motorType = MotorType.GO_M8010_6
        cmd.motorType  = MotorType.GO_M8010_6
        cmd.mode       = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
        cmd.id         = motor_id

        if not active or mode == "ZERO":
            cmd.q=0.0; cmd.dq=0.0; cmd.kp=0.0; cmd.kd=0.0; cmd.tau=0.0
        elif mode == "DQ":
            dq_cmd = float(dqset) if dqset is not None else (
                float(dqscale) if isinstance(dqscale,(int,float))
                else 6.28*queryGearRatio(MotorType.GO_M8010_6))
            cmd.q=0.0; cmd.dq=_sat(dq_cmd,DQ_LIMIT); cmd.kp=kp; cmd.kd=kd; cmd.tau=tau
        elif mode == "Q":
            cmd.q=qset; cmd.dq=0.0; cmd.kp=kp; cmd.kd=kd; cmd.tau=tau
        elif mode == "TAU":
            cmd.q=0.0; cmd.dq=0.0; cmd.kp=kp; cmd.kd=kd; cmd.tau=_sat(tau,TAU_LIMIT)
        elif mode in ("IMP","IMPEDANCE"):
            K=float(cfg.get("K",0.0)); D=float(cfg.get("D",0.0)); M=float(cfg.get("M",0.0))
            qd,dqd,ddqd = _traj_eval(cfg.get("TRAJ",None), self._current_t)
            tc = K*(qd-float(data.q)) + D*(dqd-float(data.dq)) + M*ddqd
            cmd.q=0.0; cmd.dq=0.0; cmd.kp=0.0; cmd.kd=0.0; cmd.tau=_sat(tc,TAU_LIMIT)
        else:
            cmd.q=0.0; cmd.dq=0.0; cmd.kp=0.0; cmd.kd=0.0; cmd.tau=0.0

        received=serial.sendRecv(cmd, data)
        if received is False or not getattr(data,'correct',True):
            raise IOError('Motor feedback failed validation; stale values not recorded')
        return data.q, data.dq, data.temp, data.merror

    def _loop(self):
        # Independent ports never wait on each other's USB round trip.
        workers=[Thread(target=self._port_loop,args=(port,),daemon=True,
                        name='motor-'+os.path.basename(port)) for port in self.port_to_ids]
        for worker in workers:worker.start()
        for worker in workers:worker.join()

    def _port_loop(self, port):
        from hipexo_realtime import tune_current_process
        tuning=tune_current_process()
        if 'scheduler_error' in tuning or 'timer_error' in tuning:
            self.sig_error.emit(f'[Motor {port}] timing optimization unavailable: {tuning}')
        indices=[(idx,mid) for idx,(p,mid) in enumerate(self.devices) if p==port]
        deadline=time.perf_counter();last_gui={};next_log={};batches={mid:[] for _,mid in indices};last_flush=deadline
        def flush():
            for mid,rows in batches.items():
                if rows:
                    self._dm.append_batch(f'motor_{mid}',[r[0] for r in rows],[r[1] for r in rows],[r[2] for r in rows])
                    rows.clear()
        for idx,mid in indices:
            self._last_ok_time[idx]=time.monotonic();self._last_err_emit[idx]=0.
        while self._alive:
            now=time.perf_counter()
            if now-last_flush>=.02 or any(len(rows)>=32 for rows in batches.values()):
                flush();last_flush=now
            if not self.running.is_set() and not self.collecting.is_set():
                time.sleep(.01);deadline=time.perf_counter();continue
            if (not self.running.is_set() and len(indices)==1 and port.startswith('/dev/')
                    and port not in self._fast_disabled_ports
                    and os.environ.get('HIPEXO_MOTOR_NATIVE','1')!='0'
                    and os.path.isfile(os.path.join(os.path.dirname(__file__),'hipexo_motor_capture'))):
                flush()
                self._fast_monitor(port,*indices[0])
                deadline=time.perf_counter();last_flush=deadline
                continue
            self._current_t=time.monotonic()-(self.t0 or time.monotonic())
            for idx,mid in indices:
                try:
                    serial=self.serials.get(port)
                    if serial is None:raise IOError('Serial port unavailable')
                    began=time.perf_counter_ns()
                    q,dq,temp,merror=self._compose_and_send(serial,mid,self.running.is_set())
                    completed=time.perf_counter_ns();wall=time.time_ns();now=completed/1e9
                    self._last_ok_time[idx]=time.monotonic();self._set_online(idx,True)
                    if now>=next_log.get(idx,0):
                        next_log[idx]=max(next_log.get(idx,now)+1/MOTOR_LOG_HZ,now)
                        q_output=q/MOTOR_GEAR_RATIO;q_rel=q_output%(2*math.pi)
                        batches[mid].append((wall/1e6,dict(
                            q_rotor=q,q_output=q_output,q_rel_rad=q_rel,q_deg=math.degrees(q_rel),
                            dq=dq,temp=temp,merror=merror,read_duration_ms=(completed-began)/1e6,timestamp_basis='host_request_reply_completion',motor_transport='sdk_synchronous'),
                            completed))
                    if now-last_gui.get(idx,0)>=.02:
                        last_gui[idx]=now;self.sig_update.emit(idx,q,dq,temp,merror)
                except Exception as exc:
                    now=time.monotonic()
                    if now-self._last_ok_time.get(idx,now)>=MOTOR_OFFLINE_AFTER_S:self._set_online(idx,False)
                    if now-self._last_err_emit.get(idx,0)>=1:
                        self._last_err_emit[idx]=now;self.sig_error.emit(f'[{port} ID{mid}] {exc}')
            deadline+=max(CTRL_PERIOD_S,1/MOTOR_LOG_HZ)
            delay=deadline-time.perf_counter()
            if delay>0:time.sleep(delay)
            elif delay < -.005:deadline=time.perf_counter()
        flush()

    def _fast_monitor(self,port,idx,mid):
        from hipexo_motor_process import MotorCapture
        capture=None;last_gui=0.;last_report=0.;last_received=time.monotonic()
        def report(final=False):
            nonlocal last_report
            stats=dict(capture.stats,mode='native_zero_output',target_hz=capture.target_hz,timestamp_basis='host_validated_frame',
                       request_reply_matching=False,final=final,tuning_error=capture.tuning_error,cpu_affinity=capture.cpu_affinity,
                       counter_scope='capture process lifetime; may include time outside recording')
            self.transport_stats[port]=stats
            path=getattr(self._dm,'_record_path',None)
            if path and (final or getattr(self._dm,'_recording',False)):
                from pathlib import Path
                import json
                target=Path(path).parent/f'motor_{mid}_transport_quality.json'
                if target.parent.exists():
                    tmp=target.with_suffix('.json.tmp');tmp.write_text(json.dumps(stats,indent=2));tmp.replace(target)
            last_report=time.monotonic()
        try:
            with self._port_locks[port]:
                if not self._alive or not self.collecting.is_set() or self.running.is_set():return
                capture=MotorCapture(port,mid);self._fast_handles[port]=capture
            if capture.tuning_error:self.sig_error.emit(f'[Motor {port}] native scheduling: {capture.tuning_error}')
            while True:
                if not self._alive or not self.collecting.is_set() or self.running.is_set():capture.request_stop()
                rows=capture.receive()
                if rows is None:break
                if rows:
                    frames=[];monos=[];walls=[]
                    for mono,wall,q,dq,temp,error in rows:
                        q_output=q/MOTOR_GEAR_RATIO;q_rel=q_output%(2*math.pi)
                        frames.append(dict(q_rotor=q,q_output=q_output,q_rel_rad=q_rel,q_deg=math.degrees(q_rel),
                                           dq=dq,temp=temp,merror=error,read_duration_ms=float('nan'),timestamp_basis='host_validated_frame',motor_transport='native_zero_output'))
                        monos.append(mono);walls.append(wall/1e6)
                    self._dm.append_batch(f'motor_{mid}',walls,frames,monos)
                    last_received=time.monotonic();self._last_ok_time[idx]=last_received;self._set_online(idx,True)
                    if last_received-last_gui>=.02:
                        last_gui=last_received;self.sig_update.emit(idx,q,dq,temp,error)
                elif time.monotonic()-last_received>=MOTOR_OFFLINE_AFTER_S:self._set_online(idx,False)
                if time.monotonic()-last_report>=1:report()
            capture.close();report(final=True)
        except Exception as exc:
            self._fast_disabled_ports.add(port)
            self.sig_error.emit(f'[Motor {port}] native monitor stopped; SDK fallback: {exc}')
        finally:
            if capture:
                capture.stop()
                if not capture.sock._closed:
                    try:capture.close()
                    except Exception:pass
            with self._port_locks[port]:self._fast_handles.pop(port,None)

    def start(self):
        if not self.serials and not self.init_links():
            return
        if self._thread is None:
            self._thread = Thread(target=self._loop, daemon=True, name="motor-worker")
            self._thread.start()
        if self.t0 is None:
            self.t0 = time.monotonic()
        self.collecting.set()
        self.running.set()

    def stop(self):
        self.running.clear()
        self.collecting.clear()
        self._zero_all()

    def shutdown(self):
        self._alive = False
        self.running.clear()
        self.collecting.clear()
        self._zero_all()
        time.sleep(0.05)
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._thread is None or not self._thread.is_alive():
            for lease in self._port_leases.values():lease.close()
            self._port_leases.clear()
        else:self.sig_error.emit('[Motor] Shutdown incomplete; port ownership retained')

    def _zero_all(self):
        # Same port lock as acquisition: stop can never interleave serial frames.
        if not _SDK_OK:return
        for port,mid in self.devices:
            serial=self.serials.get(port)
            if serial is None:continue
            with self._port_locks[port]:
                try:
                    capture=self._fast_handles.get(port)
                    if capture:capture.stop()
                    key=(port,mid)
                    if key not in self._motor_state:self._motor_state[key]=(MotorCmd(),MotorData())
                    self._compose_locked(serial,mid,False,*self._motor_state[key])
                except Exception:pass

    def start_monitoring(self):
        """Start feedback collection only. Commands remain zero output."""
        if not self.serials and not self.init_links():
            return
        if self._thread is None:
            self._thread = Thread(target=self._loop, daemon=True, name="motor-worker")
            self._thread.start()
        if self.t0 is None:
            self.t0 = time.monotonic()
        self.running.clear()
        self.collecting.set()

    def stop_monitoring(self):
        self.running.clear()
        self.collecting.clear()
        self._zero_all()


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                         MOTOR PANEL                                     ║
# ╚══════════════════════════════════════════════════════════════════════════╝

class MotorSettingsDialog(QtWidgets.QDialog):
    """Per-motor parameter tuning (from interface13 AdvancedSettingsDialog)."""
    def __init__(self, parent, devices):
        super().__init__(parent)
        self.setWindowTitle("Motor Settings")
        self.setMinimumWidth(540)
        self.devices = devices
        v = QtWidgets.QVBoxLayout(self)
        self.tabs  = QtWidgets.QTabWidget()
        self.pages = {}
        for _,mid in devices:
            self.tabs.addTab(self._make_page(mid), f"Motor {mid}")
        v.addWidget(self.tabs)
        btns = QtWidgets.QHBoxLayout()
        btn_ok  = QtWidgets.QPushButton("Apply & Close")
        btn_ok.setProperty("accent", True)
        btns.addStretch(1); btns.addWidget(btn_ok)
        v.addLayout(btns)
        btn_ok.clicked.connect(self._on_apply)

    def _make_page(self, mid):
        cfg  = MOTOR_PARAMS.get(mid, {})
        page = QtWidgets.QWidget()
        g    = QtWidgets.QGridLayout(page)
        row  = 0

        def lbl(text): return QtWidgets.QLabel(text)
        def dspin(lo, hi, dec=3, step=0.01, val=0.0):
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(lo, hi); s.setDecimals(dec); s.setSingleStep(step)
            s.setValue(float(val)); return s

        g.addWidget(lbl("Mode"), row, 0)
        cb_mode = QtWidgets.QComboBox()
        cb_mode.addItems(["ZERO","DQ","Q","TAU","IMP"])
        cb_mode.setCurrentText(str(cfg.get("MODE","ZERO")).upper())
        g.addWidget(cb_mode, row, 1, 1, 3); row+=1

        g.addWidget(lbl("KP"), row, 0); sp_kp = dspin(0,10000,4,0.01,cfg.get("KP",0)); g.addWidget(sp_kp,row,1)
        g.addWidget(lbl("KD"), row, 2); sp_kd = dspin(0,1000,4,0.01,cfg.get("KD",0));  g.addWidget(sp_kd,row,3); row+=1
        g.addWidget(lbl("TAU (Nm)"), row, 0); sp_tau  = dspin(-1000,1000,3,0.05,cfg.get("TAU",0));  g.addWidget(sp_tau,row,1)
        g.addWidget(lbl("Q_SET (rad)"), row, 2); sp_qset = dspin(-1000,1000,4,0.01,cfg.get("Q_SET",0)); g.addWidget(sp_qset,row,3); row+=1

        chk_gear = QtWidgets.QCheckBox("DQ = gear × 6.28")
        use_gear = (str(cfg.get("DQ_SCALE","GEAR")).upper()=="GEAR" and cfg.get("DQ_SET") is None)
        chk_gear.setChecked(use_gear)
        g.addWidget(chk_gear,row,0,1,2)
        g.addWidget(lbl("DQ_SET (rad/s)"),row,2)
        sp_dq = dspin(-10000,10000,3,0.1, cfg.get("DQ_SET",0) or 0); sp_dq.setEnabled(not use_gear)
        g.addWidget(sp_dq,row,3); row+=1
        chk_gear.toggled.connect(lambda c: sp_dq.setEnabled(not c))

        sep = QtWidgets.QFrame(); sep.setFrameShape(QtWidgets.QFrame.HLine)
        g.addWidget(sep,row,0,1,4); row+=1
        g.addWidget(lbl("Impedance"), row,0,1,2)
        g.addWidget(lbl("K (Nm/rad)"),row,2); sp_K=dspin(0,5000,3,0.5,cfg.get("K",0)); g.addWidget(sp_K,row,3); row+=1
        g.addWidget(lbl("D (Nm·s/rad)"),row,0); sp_D=dspin(0,500,3,0.1,cfg.get("D",0)); g.addWidget(sp_D,row,1)
        g.addWidget(lbl("M (Nm·s²/rad)"),row,2); sp_M=dspin(0,500,3,0.1,cfg.get("M",0)); g.addWidget(sp_M,row,3); row+=1

        traj = dict(cfg.get("TRAJ", DEFAULT_TRAJ))
        g.addWidget(lbl("Trajectory"),row,0)
        cb_traj = QtWidgets.QComboBox(); cb_traj.addItems(["CONST","SINE"])
        cb_traj.setCurrentText(str(traj.get("type","CONST")).upper()); g.addWidget(cb_traj,row,1)
        g.addWidget(lbl("q0 (rad)"),row,2); sp_q0=dspin(-1000,1000,4,0.01,traj.get("q0",0)); g.addWidget(sp_q0,row,3); row+=1
        g.addWidget(lbl("amp (rad)"),row,0); sp_amp=dspin(0,1000,4,0.01,traj.get("amp",0)); g.addWidget(sp_amp,row,1)
        g.addWidget(lbl("freq (Hz)"),row,2); sp_freq=dspin(0,50,3,0.1,traj.get("freq",0.5)); g.addWidget(sp_freq,row,3); row+=1
        g.addWidget(lbl("phase (rad)"),row,0); sp_phase=dspin(-6.3,6.3,3,0.1,traj.get("phase",0)); g.addWidget(sp_phase,row,1)

        self.pages[mid] = dict(cb_mode=cb_mode, sp_kp=sp_kp, sp_kd=sp_kd,
                               sp_tau=sp_tau, sp_qset=sp_qset,
                               chk_gear=chk_gear, sp_dq=sp_dq,
                               sp_K=sp_K, sp_D=sp_D, sp_M=sp_M,
                               cb_traj=cb_traj, sp_q0=sp_q0, sp_amp=sp_amp,
                               sp_freq=sp_freq, sp_phase=sp_phase)
        return page

    def _on_apply(self):
        for _,mid in self.devices:
            w = self.pages[mid]
            out = MOTOR_PARAMS.setdefault(mid, {})
            out["MODE"]  = w["cb_mode"].currentText()
            out["KP"]    = w["sp_kp"].value()
            out["KD"]    = w["sp_kd"].value()
            out["TAU"]   = w["sp_tau"].value()
            out["Q_SET"] = w["sp_qset"].value()
            if w["chk_gear"].isChecked():
                out["DQ_SET"]=None; out["DQ_SCALE"]="GEAR"
            else:
                out["DQ_SET"]=w["sp_dq"].value(); out["DQ_SCALE"]=1.0
            out["K"]=w["sp_K"].value(); out["D"]=w["sp_D"].value(); out["M"]=w["sp_M"].value()
            out["TRAJ"]=dict(type=w["cb_traj"].currentText(),
                             q0=w["sp_q0"].value(), amp=w["sp_amp"].value(),
                             freq=w["sp_freq"].value(), phase=w["sp_phase"].value())
        self.accept()


class MotorPanel(QtWidgets.QWidget):
    """
    Layout (left→right per motor column):
      • dq plot    — live speed, time-axis, full history
      • q_rel plot — output-shaft relative position (mod 2π), time-axis
      • Polar dial — current output-shaft angle as a point on a unit circle
                     (most intuitive for a periodically rotating joint)

    Conversion  (GO-M8010-6, gear ratio 6.33, rotor-side 15-bit abs encoder):
      output_angle_rad = MotorData.q / MOTOR_GEAR_RATIO
      q_rel            = output_angle_rad mod 2π   → [0, 2π)
      q_deg            = q_rel × 180/π             → [0°, 360°)

    The raw MotorData.q (unbounded rotor radians) is stored in DataManager
    alongside q_rel and q_deg for complete logging.
    """

    # Polar dial is drawn as a fixed-size SVG-like pyqtgraph polar plot.
    # We use a ViewBox with a unit circle overlay drawn once, and a single
    # ScatterPlotItem for the current position dot.

    def __init__(self, worker: MotorWorker, parent=None):
        super().__init__(parent)
        self._worker = worker
        self._theme  = 'light'

        layout = QtWidgets.QVBoxLayout(self)

        # ── control bar ───────────────────────────────────────────────
        ctrl = QtWidgets.QFrame(); ctrl.setProperty("card","true")
        hb   = QtWidgets.QHBoxLayout(ctrl); hb.setContentsMargins(10,8,10,8)

        self.lbl_state = QtWidgets.QLabel("State: IDLE")
        self.lbl_state.setStyleSheet("font-weight:bold;")
        self.btn_start    = QtWidgets.QPushButton("▶  Start"); self.btn_start.setProperty("accent","true")
        self.btn_stop     = QtWidgets.QPushButton("■  Stop");  self.btn_stop.setProperty("danger","true")
        self.btn_cross    = QtWidgets.QPushButton("Crosshair")
        self.btn_settings = QtWidgets.QPushButton("Settings")
        self.btn_reset    = QtWidgets.QPushButton("Reset View")

        for b in (self.btn_start, self.btn_stop, self.btn_cross, self.btn_settings, self.btn_reset):
            b.setFixedHeight(_px(30)); hb.addWidget(b)
        hb.addStretch(1); hb.addWidget(self.lbl_state)
        layout.addWidget(ctrl)

        # ── plot area: 2 rows × 3 cols ────────────────────────────────
        # Col 0: M0 time-series (dq top, q_rel bottom)
        # Col 1: M1 time-series (dq top, q_rel bottom)
        # Col 2: Polar dials    (M0 top, M1 bottom)
        plot_frame = QtWidgets.QFrame(); plot_frame.setProperty("card","true")
        grid = QtWidgets.QGridLayout(plot_frame); grid.setSpacing(6)
        grid.setColumnStretch(0, 3); grid.setColumnStretch(1, 3); grid.setColumnStretch(2, 2)

        self._color_m0 = "#1E88E5"
        self._color_m1 = "#E53935"
        self._color_dq = {"m0": "#00ACC1", "m1": "#FB8C00"}

        # NOTE: this used to be BUFFER_HARD_MAX (20000 pts). Every refresh
        # tick did `list(buf)` on all 4 of these deques and handed the
        # result to setData() — at 50 Hz that was up to ~4,000,000 element
        # copies/second and was the single biggest cause of UI freezes on
        # Jetson Orin Nano. These buffers only need to hold what's visible
        # on screen, not the whole logging history (DataManager/CSV already
        # keeps the full-resolution data independently).
        # See WORKING_LOG.md 2026-08-17.
        MAX_PTS = int(MOTOR_DISPLAY_PTS)
        # buf layout: 0=M0_dq  1=M0_qrel  2=M1_dq  3=M1_qrel
        self._bufs   = [deque(maxlen=MAX_PTS) for _ in range(4)]
        self._plots  = []
        self._curves = []
        self._vlines = []

        ts_specs = [
            ("M0  dq  (rad/s)",        self._color_dq["m0"], 0, 0),
            ("M0  position  (rad)",    self._color_m0,        1, 0),
            ("M1  dq  (rad/s)",        self._color_dq["m1"], 0, 1),
            ("M1  position  (rad)",    self._color_m1,        1, 1),
        ]
        for title, color, row, col in ts_specs:
            pw = pg.PlotWidget()
            pw.setTitle(title, color=color, size="10pt", bold=True)
            pw.setLabel("bottom", "samples", size="8pt")
            curve = pw.plot(pen=pg.mkPen(color=color, width=2))
            vl = pg.InfiniteLine(angle=90, movable=False,
                                  pen=pg.mkPen((120,120,120), width=1))
            pw.addItem(vl)
            self._plots.append(pw); self._curves.append(curve); self._vlines.append(vl)
            grid.addWidget(pw, row, col)

        # Link X axes of same-column plots
        self._plots[0].setXLink(self._plots[1])   # M0 dq ↔ M0 pos
        self._plots[2].setXLink(self._plots[3])   # M1 dq ↔ M1 pos

        # ── Polar dials ───────────────────────────────────────────────
        self._dial_m0 = self._make_dial(self._color_m0, "M0  position")
        self._dial_m1 = self._make_dial(self._color_m1, "M1  position")
        grid.addWidget(self._dial_m0['widget'], 0, 2)
        grid.addWidget(self._dial_m1['widget'], 1, 2)

        self._crosshair_on = True
        self._cur_idx      = None
        for pw in self._plots:
            pw.scene().sigMouseMoved.connect(self._on_mouse_moved)
            pw.scene().sigMouseClicked.connect(self._on_dbl_click)

        layout.addWidget(plot_frame)

        # ── temperature + live readout labels ─────────────────────────
        temp_frame = QtWidgets.QFrame(); temp_frame.setProperty("card","true")
        tb = QtWidgets.QHBoxLayout(temp_frame); tb.setContentsMargins(10,6,10,6)
        self._lbl_temp_m0 = QtWidgets.QLabel("M0   T = --.- °C")
        self._lbl_temp_m1 = QtWidgets.QLabel("M1   T = --.- °C")
        self._lbl_temp_m0.setStyleSheet("color:#1565C0; font-weight:600;")
        self._lbl_temp_m1.setStyleSheet("color:#B71C1C; font-weight:600;")
        sep = QtWidgets.QFrame(); sep.setFrameShape(QtWidgets.QFrame.VLine)
        sep.setFrameShadow(QtWidgets.QFrame.Sunken)
        tb.addWidget(self._lbl_temp_m0); tb.addWidget(sep)
        tb.addWidget(self._lbl_temp_m1); tb.addStretch(1)
        layout.addWidget(temp_frame)

        status_frame = QtWidgets.QFrame(); status_frame.setProperty("card","true")
        sb = QtWidgets.QHBoxLayout(status_frame); sb.setContentsMargins(8,4,8,4)
        self.lbl_m0 = QtWidgets.QLabel("M0: —")
        self.lbl_m1 = QtWidgets.QLabel("M1: —")
        self.lbl_m0.setStyleSheet("color:#1565C0; font-weight:bold;")
        self.lbl_m1.setStyleSheet("color:#B71C1C; font-weight:bold;")
        self.lbl_conn_m0 = QtWidgets.QLabel("● OFFLINE")
        self.lbl_conn_m1 = QtWidgets.QLabel("● OFFLINE")
        for l in (self.lbl_conn_m0, self.lbl_conn_m1):
            l.setStyleSheet("color:#9E9E9E; font-weight:700;")
        sb.addWidget(self.lbl_m0); sb.addWidget(self.lbl_conn_m0)
        sb.addStretch(1)
        sb.addWidget(self.lbl_conn_m1); sb.addWidget(self.lbl_m1)
        layout.addWidget(status_frame)

        self._last = [(0., 0., 0., '−'), (0., 0., 0., '−')]
        self._apply_theme(self._theme)

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(int(1000 / UI_REFRESH_HZ))

        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_cross.clicked.connect(self._toggle_cross)
        self.btn_reset.clicked.connect(self._reset_view)
        self.btn_settings.clicked.connect(self._open_settings)
        worker.sig_update.connect(self._on_data)
        worker.sig_error.connect(lambda m: self.lbl_state.setText(f"ERR: {m[:60]}"))
        worker.sig_conn_status.connect(self._on_conn_status)

    @QtCore.pyqtSlot(int, bool)
    def _on_conn_status(self, idx: int, online: bool):
        lbl = self.lbl_conn_m0 if idx == 0 else (self.lbl_conn_m1 if idx == 1 else None)
        if lbl is None:
            return
        if online:
            lbl.setText("● ONLINE")
            lbl.setStyleSheet("color:#2E7D32; font-weight:700;")
        else:
            lbl.setText("● OFFLINE")
            lbl.setStyleSheet("color:#D32F2F; font-weight:700;")

    # ── Polar dial factory ────────────────────────────────────────────
    def _make_dial(self, color: str, title: str) -> dict:
        """
        Build a pyqtgraph ViewBox-based polar dial.
        Returns dict with keys: widget, dot, needle, angle_label.

        Design:
          - Grey unit circle + cardinal tick marks drawn once as static items
          - A filled dot (ScatterPlotItem) at (cos θ, sin θ) for current angle
          - A thin line from origin to dot (needle)
          - A text label showing the current angle in degrees
        """
        pw = pg.PlotWidget()
        pw.setTitle(title, color=color, size="10pt", bold=True)
        pw.setAspectLocked(True)
        pw.setXRange(-1.35, 1.35, padding=0)
        pw.setYRange(-1.35, 1.35, padding=0)
        pw.hideAxis('bottom'); pw.hideAxis('left')
        pw.setBackground('w')
        pw.setMouseEnabled(x=False, y=False)

        # Static circle
        theta = [i * 2 * math.pi / 360 for i in range(361)]
        cx = [math.cos(t) for t in theta]
        cy = [math.sin(t) for t in theta]
        pw.plot(cx, cy, pen=pg.mkPen('#CCCCCC', width=1.5))

        # Cardinal ticks + labels  (0°=right, 90°=up, consistent with math convention)
        cardinals = [(0, '0°'), (math.pi/2, '90°'), (math.pi, '180°'), (3*math.pi/2, '270°')]
        for ang, lbl in cardinals:
            x0, y0 = 0.88*math.cos(ang), 0.88*math.sin(ang)
            x1, y1 = 1.02*math.cos(ang), 1.02*math.sin(ang)
            pw.plot([x0, x1], [y0, y1], pen=pg.mkPen('#AAAAAA', width=1))
            ti = pg.TextItem(lbl, anchor=(0.5, 0.5), color='#888888')
            ti.setPos(1.20*math.cos(ang), 1.20*math.sin(ang))
            pw.addItem(ti)

        # Minor ticks every 30°
        for deg in range(0, 360, 30):
            ang = math.radians(deg)
            pw.plot([0.93*math.cos(ang), 1.0*math.cos(ang)],
                    [0.93*math.sin(ang), 1.0*math.sin(ang)],
                    pen=pg.mkPen('#DDDDDD', width=1))

        # Needle (line from origin to dot)
        needle = pw.plot([0, 1], [0, 0],
                          pen=pg.mkPen(color, width=2.5))

        # Current-position dot
        dot = pg.ScatterPlotItem(
            [1.0], [0.0],
            symbol='o', size=14,
            pen=pg.mkPen(color, width=2),
            brush=pg.mkBrush(color)
        )
        pw.addItem(dot)

        # Angle text
        angle_lbl = pg.TextItem("0.00°", anchor=(0.5, 0.5), color=color)
        angle_lbl.setPos(0, -1.20)
        f = angle_lbl.textItem.font(); f.setPointSize(11); f.setBold(True)
        angle_lbl.textItem.setFont(f)
        pw.addItem(angle_lbl)

        return {'widget': pw, 'dot': dot, 'needle': needle,
                'angle_label': angle_lbl, 'pw': pw}

    def _update_dial(self, dial: dict, q_rel_rad: float):
        """Move the needle and dot to the new output-shaft angle."""
        # q_rel_rad is in [0, 2π); map to standard math angle
        # 0° = 3 o'clock, 90° = 12 o'clock (match motor convention: CCW positive)
        x = math.cos(q_rel_rad)
        y = math.sin(q_rel_rad)
        dial['dot'].setData([x], [y])
        dial['needle'].setData([0, x], [0, y])
        deg = math.degrees(q_rel_rad) % 360.0
        dial['angle_label'].setText(f"{deg:.2f}°")

    # ── Slots & helpers ───────────────────────────────────────────────
    def _on_start(self):
        if not _SDK_OK:
            self.lbl_state.setText("State: SDK missing")
            self.lbl_state.setStyleSheet("color:#D32F2F; font-weight:bold;")
            return
        self._worker.start()
        self.lbl_state.setText("State: RUNNING")

    def _on_stop(self):
        if not _SDK_OK:
            return
        self._worker.stop()
        self.lbl_state.setText("State: IDLE")

    def _open_settings(self):
        MotorSettingsDialog(self, MOTOR_DEVICES).exec_()

    def _toggle_cross(self):
        self._crosshair_on = not self._crosshair_on
        for v in self._vlines: v.setVisible(self._crosshair_on)
        self.btn_cross.setText("Crosshair OFF" if self._crosshair_on else "Crosshair ON")

    def _reset_view(self):
        for pw in self._plots:
            pw.enableAutoRange('xy', True); pw.autoRange()

    def set_theme(self, theme: str):
        self._theme = theme
        self._apply_theme(theme)

    def _apply_theme(self, theme: str):
        p = _plot_theme_params(theme)
        bg_dial = '#0F1115' if theme == 'dark' else 'w'
        for pw in self._plots:
            pw.setBackground(p['bg'])
            pw.showGrid(x=True, y=True, alpha=p['grid'])
            pw.getAxis('left').setPen(pg.mkPen(p['axis']))
            pw.getAxis('bottom').setPen(pg.mkPen(p['axis']))
            pw.getAxis('left').setTextPen(p['text'])
            pw.getAxis('bottom').setTextPen(p['text'])
        for v in self._vlines:
            v.setPen(pg.mkPen(p['cross'], width=1))
        for dial in (self._dial_m0, self._dial_m1):
            dial['pw'].setBackground(bg_dial)

    @QtCore.pyqtSlot(int, float, float, float, int)
    def _on_data(self, idx, q_raw, dq, temp, merror):
        # Convert rotor angle → output shaft relative position
        q_output = q_raw / MOTOR_GEAR_RATIO          # output shaft, unbounded rad
        q_rel    = q_output % (2 * math.pi)           # [0, 2π)
        if idx == 0:
            self._bufs[0].append(dq)
            self._bufs[1].append(q_rel)
            self._last[0] = (q_rel, dq, temp, merror)
        elif idx == 1:
            self._bufs[2].append(dq)
            self._bufs[3].append(q_rel)
            self._last[1] = (q_rel, dq, temp, merror)

    def _refresh(self):
        # Hidden pages retain their bounded data buffers but do not redraw.
        # Standalone unshown panels remain usable by offline inspection/tests.
        if self.window().isVisible() and not self.isVisible():return
        for i, (curve, buf) in enumerate(zip(self._curves, self._bufs)):
            if buf:
                y = list(buf); curve.setData(range(len(y)), y)

        q0, dq0, t0, e0 = self._last[0]
        q1, dq1, t1, e1 = self._last[1]

        self._update_dial(self._dial_m0, q0)
        self._update_dial(self._dial_m1, q1)

        self._lbl_temp_m0.setText(f"M0   T = {t0:+.1f} °C")
        self._lbl_temp_m1.setText(f"M1   T = {t1:+.1f} °C")
        self.lbl_m0.setText(
            f"M0  pos={math.degrees(q0):.1f}°  dq={dq0:.3f} rad/s  err={e0}")
        self.lbl_m1.setText(
            f"M1  pos={math.degrees(q1):.1f}°  dq={dq1:.3f} rad/s  err={e1}")

    def _on_mouse_moved(self, pos):
        if not self._crosshair_on: return
        non_empty = [len(b) for b in self._bufs if b]
        if not non_empty: return
        mx = max(non_empty) - 1
        if mx < 0: return
        # Find which plot generated the event and use its viewbox
        for pw in self._plots:
            if pw.sceneBoundingRect().contains(pos):
                x = pw.plotItem.vb.mapSceneToView(pos).x()
                self._cur_idx = max(0, min(int(round(x)), mx))
                for v in self._vlines: v.setPos(self._cur_idx)
                return

    def _on_dbl_click(self, ev):
        try:
            if not (getattr(ev,'double',False) or
                    (hasattr(ev,'double') and ev.double())):
                return
            self._reset_view(); ev.accept()
        except Exception:
            pass

    def snapshot_csv(self) -> dict:
        return {f: list(self._bufs[i]) for i, f in enumerate(
            ['m0_dq','m0_q_rel_rad','m1_dq','m1_q_rel_rad'])}


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                            IMU PANEL                                    ║
# ╚══════════════════════════════════════════════════════════════════════════╝

_IMU_FIELDS = [
    ("ax_g",      "Accel X (g)"),
    ("ay_g",      "Accel Y (g)"),
    ("az_g",      "Accel Z (g)"),
    ("gx_dps",    "Gyro X (°/s)"),
    ("gy_dps",    "Gyro Y (°/s)"),
    ("gz_dps",    "Gyro Z (°/s)"),
    ("roll_deg",  "Roll (°)"),
    ("pitch_deg", "Pitch (°)"),
    ("yaw_deg",   "Yaw (°)"),
]
_IMU_COLS  = ["Accelerometer", "Gyroscope", "Euler Angles"]
_IMU_COLORS = ["#1E88E5","#43A047","#F4511E","#8E24AA"]   # one per sensor

class ImuPanel(QtWidgets.QWidget):
    """
    3×3 live plot grid (Accel / Gyro / Euler) for up to 4 IMU sensors.
    Temperature shown as text labels top-right of each column.
    Uses pyqtgraph — no matplotlib, no secondary event loop.
    """
    def __init__(self, worker: ImuWorker, parent=None):
        super().__init__(parent)
        self._worker    = worker
        self._theme     = 'light'
        self._relative = False
        self._last_reference_ids = [None]*len(IMU_DEVICES)
        self._n_sensors = len(IMU_DEVICES)
        # At 500 Hz, 10 s window = 5000 pts per sensor per channel.
        # The UI refresh (UI_REFRESH_HZ) decimates for drawing;
        # the full 5000-pt buffer is kept for data fidelity.
        maxlen = max(500, int(IMU_PLOT_WINDOW / IMU_PERIOD_S))

        # y_data[field_idx][sensor_idx] = deque
        self._y  = [[deque(maxlen=maxlen) for _ in range(self._n_sensors)]
                    for _ in range(len(_IMU_FIELDS))]
        # Each sensor owns its timestamps: a disconnected sensor's old trace
        # must not move forward when another sensor continues sending data.
        self._t = [deque(maxlen=maxlen) for _ in range(self._n_sensors)]
        self._t0 = time.perf_counter()
        self._last_temp = [float('nan')] * self._n_sensors
        self._sensor_online = [False] * self._n_sensors
        self._last_data_at = [None] * self._n_sensors
        self._ever_started = False

        layout = QtWidgets.QVBoxLayout(self)

        # ── control bar ────────────────────────────────────────────────
        ctrl = QtWidgets.QFrame(); ctrl.setProperty("card","true")
        hb   = QtWidgets.QHBoxLayout(ctrl); hb.setContentsMargins(10,8,10,8)
        self.lbl_state = QtWidgets.QLabel("State: IDLE"); self.lbl_state.setStyleSheet("font-weight:bold;")
        self.btn_start  = QtWidgets.QPushButton("▶  Start"); self.btn_start.setProperty("accent","true")
        self.btn_stop   = QtWidgets.QPushButton("■  Stop");  self.btn_stop.setProperty("danger","true")
        self.btn_reset  = QtWidgets.QPushButton("Reset View")
        for b in (self.btn_start, self.btn_stop, self.btn_reset):
            b.setFixedHeight(_px(30)); hb.addWidget(b)
        hb.addStretch(1); hb.addWidget(self.lbl_state)
        layout.addWidget(ctrl)

        reference_row = QtWidgets.QHBoxLayout()
        self.btn_reference = QtWidgets.QPushButton("设置参考姿态（静止 3 秒）")
        self.btn_reference.clicked.connect(self._set_reference)
        self.chk_relative = QtWidgets.QCheckBox("显示相对姿态")
        self.chk_relative.toggled.connect(self._toggle_relative)
        reference_row.addWidget(self.btn_reference)
        reference_row.addWidget(self.chk_relative)
        reference_row.addStretch()
        layout.addLayout(reference_row)
        self.lbl_reference = QtWidgets.QLabel()
        self.lbl_reference.setWordWrap(True)
        layout.addWidget(self.lbl_reference)

        # ── 3×3 plot grid ──────────────────────────────────────────────
        plot_frame = QtWidgets.QFrame(); plot_frame.setProperty("card","true")
        grid = QtWidgets.QGridLayout(plot_frame); grid.setSpacing(4)
        # Equal stretch for all 3 columns so Euler (col 2) is never squeezed out
        grid.setColumnStretch(0, 1); grid.setColumnStretch(1, 1); grid.setColumnStretch(2, 1)

        self._plots  = []    # [field_idx] → PlotWidget
        self._curves = []    # [field_idx][sensor_idx] → PlotDataItem

        for fi, (field, ylabel) in enumerate((_IMU_FIELDS)):
            col = fi // 3
            row = fi % 3
            pw  = pg.PlotWidget()
            pw.setLabel("left", ylabel, size="9pt")
            if row == 2:
                pw.setLabel("bottom", "t (s)", size="9pt")
            if row == 0:
                pw.setTitle(_IMU_COLS[col], size="10pt", bold=True)
            pw.showGrid(x=True, y=True, alpha=0.2)
            sensor_curves = []
            for si in range(self._n_sensors):
                c = pw.plot(pen=pg.mkPen(_IMU_COLORS[si % 4], width=1.2),
                            name=f"IMU {si}" if self._n_sensors > 1 else None,
                            connect='finite')
                sensor_curves.append(c)
            self._plots.append(pw)
            self._curves.append(sensor_curves)
            grid.addWidget(pw, row, col)

        layout.addWidget(plot_frame)

        # ── Always-visible status cards, including sensors with no data ─
        temp_frame = QtWidgets.QFrame(); temp_frame.setProperty("card","true")
        tb = QtWidgets.QGridLayout(temp_frame); tb.setContentsMargins(10,6,10,6)
        tb.setColumnStretch(0, 1); tb.setColumnStretch(1, 1)
        self._temp_labels = []
        for si in range(self._n_sensors):
            bus_id, addr = IMU_DEVICES[si]
            lbl  = QtWidgets.QLabel()
            lbl.setToolTip(f"i2c-{bus_id} / 0x{addr:02X}")
            lbl.setStyleSheet(f"color:{_IMU_COLORS[si % 4]}; font-weight:600;")
            self._temp_labels.append(lbl)
            tb.addWidget(lbl, si // 2, si % 2)
        layout.insertWidget(1, temp_frame)

        self._apply_theme(self._theme)

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(int(1000 / UI_REFRESH_HZ))

        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_reset.clicked.connect(self._reset_view)
        worker.sig_update.connect(self._on_data)
        worker.sig_status.connect(lambda m: self.lbl_state.setText(m[:80]))
        worker.sig_conn_status.connect(self._on_conn_status)
        self._refresh_status()

    @QtCore.pyqtSlot(int, bool)
    def _on_conn_status(self, idx: int, online: bool):
        """
        Sensor went offline/online — keep plotting/collecting with whatever
        data keeps arriving, just make the state visible so a dropped IMU
        during a session isn't mistaken for a flat-lined reading.
        """
        if idx >= self._n_sensors:
            return
        self._sensor_online[idx] = online
        self._refresh_status()

    def _set_reference(self):
        self._worker.request_reference()
        self._refresh_status()

    def _toggle_relative(self, checked):
        self._relative = checked
        self._clear_angle_history()
        for fi, axis in enumerate(('Roll','Pitch','Yaw'), 6):
            self._plots[fi].setLabel('left', ('Relative ' if checked else '')+axis+' (°)')

    def _clear_angle_history(self, idx=None):
        for si in (range(self._n_sensors) if idx is None else [idx]):
            for fi in range(6,9):
                self._y[fi][si].clear()
                self._y[fi][si].extend([float('nan')]*len(self._t[si]))

    def _toast_status(self, msg: str):
        self.lbl_state.setText(msg)
        self.lbl_state.setStyleSheet(
            "color:#D32F2F; font-weight:bold;" if "OFFLINE" in msg
            else "color:#2E7D32; font-weight:bold;")

    def _on_start(self):
        if self._worker.start():
            self._ever_started = True
            self._refresh_status()
    def _on_stop(self):
        self._worker.stop()
        self._refresh_status()
    def _reset_view(self):
        for pw in self._plots:
            try:
                pw.enableAutoRange("xy", True)
                pw.autoRange()
            except Exception:
                pass

    def set_theme(self, theme: str):
        self._theme = theme
        self._apply_theme(theme)

    def _apply_theme(self, theme: str):
        p = _plot_theme_params(theme)
        for pw in self._plots:
            pw.setBackground(p['bg'])
            pw.showGrid(x=True, y=True, alpha=p['grid'])
            pw.getAxis('left').setPen(pg.mkPen(p['axis']))
            pw.getAxis('bottom').setPen(pg.mkPen(p['axis']))
            pw.getAxis('left').setTextPen(p['text'])
            pw.getAxis('bottom').setTextPen(p['text'])

    @QtCore.pyqtSlot(int, dict, float)
    def _on_data(self, idx: int, d: dict, temp_c: float):
        if idx >= self._n_sensors:
            return
        identity = (d.get('reference_valid',0), d.get('reference_id',0))
        if self._relative and identity != self._last_reference_ids[idx]:
            self._clear_angle_history(idx)
        self._last_reference_ids[idx] = identity
        now = time.perf_counter()
        tt = now - self._t0
        times = self._t[idx]
        if times and tt-times[-1] > IMU_OFFLINE_AFTER_S:
            times.append((times[-1]+tt)/2)
            for channels in self._y:
                channels[idx].append(float('nan'))
        times.append(tt)
        self._last_data_at[idx] = now
        for fi, (field, _) in enumerate(_IMU_FIELDS):
            key = 'rel_'+field if self._relative and fi >= 6 else field
            v = d.get(key, float('nan'))
            self._y[fi][idx].append(float('nan') if v is None else float(v))
        if not math.isnan(temp_c):
            self._last_temp[idx] = temp_c

    # Display decimation: draw at most this many points per curve.
    # 500 Hz x 10 s = 5000 raw pts; draw a decimated view for UI speed.
    _DISPLAY_MAX_PTS = 500

    def _refresh_status(self):
        running = self._worker._running.is_set()
        self._ever_started = self._ever_started or running
        now = time.perf_counter()
        reference_states, message, pending = self._worker.reference.status(self._worker._dm.session)
        self.lbl_reference.setText(message)
        self.btn_reference.setEnabled(not pending)
        online_count = 0
        for si, lbl in enumerate(self._temp_labels):
            bus_id, addr = IMU_DEVICES[si]
            fresh = self._last_data_at[si] is not None and now-self._last_data_at[si] <= IMU_OFFLINE_AFTER_S
            online = running and self._sensor_online[si] and fresh
            if not running:
                status = 'STOPPED 已停止' if self._ever_started else 'IDLE 未采集'
                color = '#888888'
            elif online:
                status, color = 'ONLINE 在线', '#2E7D32'
                online_count += 1
            else:
                status, color = 'OFFLINE 无响应', '#D32F2F'
            temp = self._last_temp[si]
            val = f'{temp:+.1f} °C' if online and math.isfinite(temp) else '--.- °C'
            lbl.setText(
                f'<b style="color:{_IMU_COLORS[si % 4]}">IMU {si}</b> · I²C-{bus_id} / 0x{addr:02X}'
                f'<br><b style="color:{color}">● {status}</b> &nbsp; T = {val}'
                f'<br>参考姿态：{reference_states[si]} · 实读 {self._worker.actual_rate(si):.1f} Hz / 目标 200 Hz')
            detail = self._worker.error_detail(si)
            lbl.setToolTip(f'i2c-{bus_id} / 0x{addr:02X}' + (f'\nLast read error: {detail}' if detail else ''))
        if running:
            self.lbl_state.setText(f'采集中 | ONLINE {online_count}/{self._n_sensors} | OFFLINE {self._n_sensors-online_count}')
            self.lbl_state.setStyleSheet('font-weight:bold; color:'+('#2E7D32;' if online_count==self._n_sensors else '#D32F2F;'))
        elif self._ever_started:
            self.lbl_state.setText('State: STOPPED · 已停止采集')
            self.lbl_state.setStyleSheet('font-weight:bold; color:#888;')

    def _refresh(self):
        # Hidden pages retain their bounded data buffers but do not redraw.
        # Standalone unshown panels remain usable by offline inspection/tests.
        if self.window().isVisible() and not self.isVisible():return
        self._refresh_status()
        xmax = max(1.0, time.perf_counter()-self._t0)
        xmin = max(0.0, xmax - IMU_PLOT_WINDOW)
        # Time slicing is shared by all nine measurements for each IMU.
        slices={}
        for si in range(self._n_sensors):
            x=list(self._t[si])
            first=next((i for i,stamp in enumerate(x) if stamp>=xmin),len(x))
            stride=max(1,(len(x)-first)//self._DISPLAY_MAX_PTS)
            slices[si]=(first,stride,x[first::stride])
        for fi in range(len(_IMU_FIELDS)):
            self._plots[fi].setXRange(xmin,xmax,padding=0)
            for si in range(self._n_sensors):
                first,stride,x=slices[si];y=list(self._y[fi][si])
                self._curves[fi][si].setData(x,y[first::stride])


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                     STUB PANELS  (plug-in ports)                        ║
# ║  Each stub follows the same interface: set_theme(str).                  ║
# ║  Replace the QLabel body with a real panel when hardware is ready.      ║
# ╚══════════════════════════════════════════════════════════════════════════╝

class _StubPanel(QtWidgets.QWidget):
    def __init__(self, title: str, description: str, parent=None):
        super().__init__(parent)
        v = QtWidgets.QVBoxLayout(self)
        v.setAlignment(QtCore.Qt.AlignCenter)
        icon = QtWidgets.QLabel(title)
        icon.setStyleSheet("font-size:28pt; font-weight:600; color:#888;")
        icon.setAlignment(QtCore.Qt.AlignCenter)
        desc = QtWidgets.QLabel(description)
        desc.setStyleSheet("font-size:11pt; color:#999;")
        desc.setAlignment(QtCore.Qt.AlignCenter)
        desc.setWordWrap(True)
        v.addWidget(icon); v.addWidget(desc)

    def set_theme(self, _theme): pass





# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                      ADS8688 DRIVER (no GPIO)                           ║
# ╚══════════════════════════════════════════════════════════════════════════╝

try:
    from hipexo_force_capture import ADS8688 as _ADS8688
except ImportError:
    _ADS8688 = None


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                       FORCE SENSOR WORKER                               ║
# ╚══════════════════════════════════════════════════════════════════════════╝

class ForceSensorWorker(QtCore.QObject):
    """
    Background thread: reads two ADS8688 channels at FORCE_SAMPLE_HZ.
    Emits sig_update(ch_idx, voltage_V, weight_kg) for each sample.
    Reads channel config from the module-level FORCE_CHANNELS list
    (hot-reloaded on restart so settings dialog changes take effect).
    """
    sig_update      = QtCore.pyqtSignal(int, float, float)   # ch, V, kg
    sig_status      = QtCore.pyqtSignal(str)
    sig_conn_status = QtCore.pyqtSignal(int, bool)            # ch, online
    sig_adc_status  = QtCore.pyqtSignal(bool)                 # ADC link itself online?

    def __init__(self, data_manager: DataManager, parent=None):
        super().__init__(parent)
        self._dm      = data_manager
        self._running = Event()
        self._alive   = True
        self._thread  = None
        self._adc     = None
        self._online        = {}   # ch idx -> bool
        self._last_ok_time  = {}   # ch idx -> time.time()
        self._adc_online    = False
        self._rate_times = {i:deque(maxlen=2000) for i in range(len(FORCE_CHANNELS))}

    def actual_rate(self,idx):
        ts=list(self._rate_times.get(idx,()))
        if len(ts)<2 or time.perf_counter_ns()-ts[-1]>1_000_000_000:return 0.
        return (len(ts)-1)*1e9/(ts[-1]-ts[0]) if ts[-1]>ts[0] else 0.

    def is_online(self, idx: int) -> bool:
        return self._online.get(idx, False)

    def _set_online(self, idx: int, online: bool):
        prev = self._online.get(idx)
        self._online[idx] = online
        if prev != online:
            self.sig_conn_status.emit(idx, online)

    def _set_adc_online(self, online: bool):
        if self._adc_online != online:
            self._adc_online = online
            self.sig_adc_status.emit(online)

    # ── public API ────────────────────────────────────────────────────
    def start(self):
        if not _SPIDEV_OK:
            self.sig_status.emit("[Force] spidev not installed")
            return
        if self._thread and self._thread.is_alive():
            self._running.set()
            return
        self._thread = Thread(target=self._loop, daemon=True, name="force-worker")
        self._thread.start()
        self._running.set()

    def stop(self):
        self._running.clear()

    def shutdown(self):
        self._alive = False
        self._running.clear()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    # ── internal ──────────────────────────────────────────────────────
    def _open_adc(self):
        try:
            adc = _ADS8688(FORCE_SPI_BUS, FORCE_SPI_DEVICE, FORCE_SPI_SPEED)
            for ch_cfg in FORCE_CHANNELS:
                ain  = int(ch_cfg["ain"])
                mode = ch_cfg["output_mode"]
                rkey = _ADS_MODE_TO_RANGE.get(mode, "pm10V")
                adc.set_channel_range(ain, _ADS_RANGE[rkey])
            return adc
        except Exception as e:
            self.sig_status.emit(f"[Force] SPI open failed: {e}")
            return None

    @staticmethod
    def _raw_to_weight(raw: int, cfg: dict) -> tuple[float, float]:
        mode  = cfg["output_mode"]
        max_kg = float(cfg["sensor_max_kg"])
        vmin, vmax = _ADS_VRANGE.get(mode, (-10.0, 10.0))
        v = vmin + (raw / 65536.0) * (vmax - vmin)
        if mode == "0_10V":
            w = (v / 10.0) * max_kg
        elif mode == "0_5V":
            w = (v / 5.0)  * max_kg
        elif mode == "pm10V":
            w = (v / 10.0) * max_kg
        elif mode == "pm5V":
            w = (v / 5.0)  * max_kg
        elif mode == "4_20mA":
            w = max(0.0, (v - 1.0) / 4.0 * max_kg)
        else:
            w = 0.0
        return v, w

    def _loop(self):
        import socket,struct,subprocess,json
        last_gui=0.
        while self._alive:
            if not self._running.is_set():
                time.sleep(.02);continue
            sock,other=socket.socketpair();process=None
            from hipexo_realtime import ControllerPowerLease
            power=ControllerPowerLease(f"/sys/class/spidev/spidev{FORCE_SPI_BUS}.{FORCE_SPI_DEVICE}/device")
            power.__enter__()
            self.sig_status.emit("[Force] controller tuning: "+json.dumps(power.report))
            try:
                config=dict(bus=FORCE_SPI_BUS,device=FORCE_SPI_DEVICE,speed=FORCE_SPI_SPEED,hz=FORCE_SAMPLE_HZ,
                            channels=[int(c['ain']) for c in FORCE_CHANNELS],
                            ranges=[(int(c['ain']),_ADS_RANGE[_ADS_MODE_TO_RANGE[c['output_mode']]]) for c in FORCE_CHANNELS])
                process=subprocess.Popen([sys.executable,os.path.join(os.path.dirname(__file__),'hipexo_force_capture.py'),
                                          str(other.fileno()),json.dumps(config)],pass_fds=(other.fileno(),),
                                         stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                other.close();sock.settimeout(.2)
                pending=bytearray();stopping=False;last_packet=time.perf_counter()
                while True:
                    if (not self._alive or not self._running.is_set()) and not stopping:
                        sock.sendall(b'stop');stopping=True;stop_deadline=time.perf_counter()+1
                    if stopping and time.perf_counter()>stop_deadline:raise TimeoutError('ADC stop drain timed out')
                    try:
                        data=sock.recv(65536)
                        if not data:
                            if stopping:break
                            raise EOFError('ADC process disconnected')
                        pending.extend(data)
                    except socket.timeout:
                        if process.poll() is not None:raise RuntimeError('ADC process exited')
                        if time.perf_counter()-last_packet>FORCE_OFFLINE_AFTER_S:
                            self._set_adc_online(False)
                            for idx in range(len(FORCE_CHANNELS)):self._set_online(idx,False)
                        continue
                    while len(pending)>=4:
                        n=struct.unpack('!I',pending[:4])[0]
                        if n>1024*1024:raise ValueError('ADC packet exceeds limit')
                        if len(pending)<4+n:break
                        packet=json.loads(pending[4:4+n]);del pending[:4+n]
                        if 'error' in packet:raise RuntimeError(packet['error'])
                        if 'tuning' in packet:
                            self.sig_status.emit('[Force] process tuning: '+json.dumps(packet['tuning']))
                            continue
                        last_packet=time.perf_counter()
                        self._set_adc_online(True)
                        now=time.perf_counter();display=now-last_gui>=.02
                        for idx,cfg in enumerate(FORCE_CHANNELS):
                            readings=[s[idx] for s in packet['samples']]
                            self._rate_times[idx].extend(r[1] for r in readings)
                            frames=[dict(V=self._raw_to_weight(r[0],cfg)[0],kg=self._raw_to_weight(r[0],cfg)[1],adc_raw_count=r[0],adc_reference_v=4.096,adc_range_code=_ADS_RANGE[_ADS_MODE_TO_RANGE[cfg['output_mode']]],conversion_version='ADS8688-datasheet-v1') for r in readings]
                            self._dm.append_batch(f'force_{idx}',[r[2]/1e6 for r in readings],frames,[r[1] for r in readings])
                            self._last_ok_time[idx]=time.time();self._set_online(idx,True)
                            if display:self.sig_update.emit(idx,frames[-1]['V'],frames[-1]['kg'])
                        if display:last_gui=now
            except Exception as exc:
                self.sig_status.emit('[Force] '+str(exc))
                self._set_adc_online(False)
                for idx in range(len(FORCE_CHANNELS)):self._set_online(idx,False)
            finally:
                other.close();sock.close()
                if process:
                    process.terminate()
                    try:process.wait(timeout=1)
                    except subprocess.TimeoutExpired:process.kill();process.wait()
                power.__exit__(None,None,None)
                if "restore_error" in power.report:self.sig_status.emit("[Force] "+power.report["restore_error"])
            if self._alive and self._running.is_set():time.sleep(.5)



# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                    FORCE SENSOR SETTINGS DIALOG                         ║
# ╚══════════════════════════════════════════════════════════════════════════╝

class ForceSensorSettingsDialog(QtWidgets.QDialog):
    """Live-editable per-channel config: AIN, output mode, max kg, label."""

    _MODES = ["pm10V", "pm5V", "0_10V", "0_5V", "4_20mA"]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Force Sensor Settings")
        self.setMinimumWidth(480)
        v = QtWidgets.QVBoxLayout(self)

        # SPI hardware row
        spi_frame = QtWidgets.QGroupBox("SPI Hardware")
        sg = QtWidgets.QFormLayout(spi_frame)
        self._sp_bus    = QtWidgets.QSpinBox(); self._sp_bus.setRange(0,3);   self._sp_bus.setValue(FORCE_SPI_BUS)
        self._sp_dev    = QtWidgets.QSpinBox(); self._sp_dev.setRange(0,3);   self._sp_dev.setValue(FORCE_SPI_DEVICE)
        self._sp_speed  = QtWidgets.QSpinBox(); self._sp_speed.setRange(100_000, 17_000_000)
        self._sp_speed.setSingleStep(500_000);  self._sp_speed.setValue(FORCE_SPI_SPEED)
        self._sp_hz     = QtWidgets.QSpinBox(); self._sp_hz.setRange(1, 1000); self._sp_hz.setValue(FORCE_SAMPLE_HZ)
        sg.addRow("SPI Bus",      self._sp_bus)
        sg.addRow("SPI Device",   self._sp_dev)
        sg.addRow("SPI Speed Hz", self._sp_speed)
        sg.addRow("Sample Hz",    self._sp_hz)
        v.addWidget(spi_frame)

        # Per-channel rows
        self._ch_widgets = []
        for idx, cfg in enumerate(FORCE_CHANNELS):
            gb = QtWidgets.QGroupBox(f"Channel {idx}")
            fl = QtWidgets.QFormLayout(gb)

            le_label = QtWidgets.QLineEdit(cfg.get("label", f"CH{idx}"))
            sp_ain   = QtWidgets.QSpinBox(); sp_ain.setRange(0, 7); sp_ain.setValue(int(cfg["ain"]))
            cb_mode  = QtWidgets.QComboBox(); cb_mode.addItems(self._MODES)
            cb_mode.setCurrentText(cfg.get("output_mode", "pm10V"))
            sp_max   = QtWidgets.QDoubleSpinBox()
            sp_max.setRange(0.1, 10000.0); sp_max.setDecimals(1); sp_max.setSingleStep(10)
            sp_max.setValue(float(cfg.get("sensor_max_kg", 50.0)))

            fl.addRow("Label",           le_label)
            fl.addRow("AIN channel",     sp_ain)
            fl.addRow("Output mode",     cb_mode)
            fl.addRow("Max load (kg)",   sp_max)
            v.addWidget(gb)
            self._ch_widgets.append(dict(label=le_label, ain=sp_ain,
                                         mode=cb_mode, max_kg=sp_max))

        # Buttons
        hb = QtWidgets.QHBoxLayout()
        btn_ok  = QtWidgets.QPushButton("Apply & Restart"); btn_ok.setProperty("accent","true")
        btn_can = QtWidgets.QPushButton("Cancel")
        hb.addStretch(1); hb.addWidget(btn_can); hb.addWidget(btn_ok)
        v.addLayout(hb)
        btn_ok.clicked.connect(self._apply)
        btn_can.clicked.connect(self.reject)

    def _apply(self):
        global FORCE_SPI_BUS, FORCE_SPI_DEVICE, FORCE_SPI_SPEED, FORCE_SAMPLE_HZ
        FORCE_SPI_BUS    = self._sp_bus.value()
        FORCE_SPI_DEVICE = self._sp_dev.value()
        FORCE_SPI_SPEED  = self._sp_speed.value()
        FORCE_SAMPLE_HZ  = self._sp_hz.value()
        for idx, w in enumerate(self._ch_widgets):
            FORCE_CHANNELS[idx]["label"]          = w["label"].text()
            FORCE_CHANNELS[idx]["ain"]            = w["ain"].value()
            FORCE_CHANNELS[idx]["output_mode"]    = w["mode"].currentText()
            FORCE_CHANNELS[idx]["sensor_max_kg"]  = w["max_kg"].value()
        self.accept()


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                       FORCE SENSOR PANEL                                ║
# ╚══════════════════════════════════════════════════════════════════════════╝

class ForceSensorPanel(QtWidgets.QWidget):
    """
    Live dual-channel load-cell readout via ADS8688 + DY510 transmitter.

    Layout
    ──────
    • Control bar  – Start / Stop / Tare / Settings
    • Big live readout cards  – one per channel (kg + V)
    • Dual time-series plots  – scrolling window
    • Status bar
    """

    _CH_COLORS = ["#1E88E5", "#E53935"]   # blue, red

    def __init__(self, worker: "ForceSensorWorker", parent=None):
        super().__init__(parent)
        self._worker  = worker
        self._theme   = "light"
        self._n       = len(FORCE_CHANNELS)
        maxpts        = max(50, int(FORCE_PLOT_WIN_S * FORCE_SAMPLE_HZ))

        # Ring buffers: [ch_idx] → deque of (t_s, kg)
        self._t_bufs  = [deque(maxlen=maxpts) for _ in range(self._n)]
        self._kg_bufs = [deque(maxlen=maxpts) for _ in range(self._n)]
        self._t0      = time.time()
        self._tare    = [0.0] * self._n          # tare offsets (kg)
        self._last    = [(0.0, 0.0)] * self._n   # (V, kg) for readout cards

        layout = QtWidgets.QVBoxLayout(self)
        layout.setSpacing(6)

        # ── control bar ────────────────────────────────────────────────
        ctrl = QtWidgets.QFrame(); ctrl.setProperty("card","true")
        hb   = QtWidgets.QHBoxLayout(ctrl); hb.setContentsMargins(10,8,10,8)
        self.btn_start    = QtWidgets.QPushButton("▶  Start");    self.btn_start.setProperty("accent","true")
        self.btn_stop     = QtWidgets.QPushButton("■  Stop");     self.btn_stop.setProperty("danger","true")
        self.btn_tare     = QtWidgets.QPushButton("⊘  Tare All")
        self.btn_settings = QtWidgets.QPushButton("⚙  Settings")
        self.lbl_state    = QtWidgets.QLabel("State: IDLE");      self.lbl_state.setStyleSheet("font-weight:bold;")
        for b in (self.btn_start, self.btn_stop, self.btn_tare, self.btn_settings):
            b.setFixedHeight(36); hb.addWidget(b)
        hb.addStretch(1); hb.addWidget(self.lbl_state)
        layout.addWidget(ctrl)

        # ── live readout cards ─────────────────────────────────────────
        cards_frame = QtWidgets.QFrame(); cards_frame.setProperty("card","true")
        cards_hb    = QtWidgets.QHBoxLayout(cards_frame)
        cards_hb.setContentsMargins(12, 10, 12, 10); cards_hb.setSpacing(20)
        self._card_kg_lbls = []
        self._card_v_lbls  = []
        self._card_name_lbls = []
        self._cards        = []
        self._card_colors  = []
        self._channel_online = [False] * self._n
        for idx in range(self._n):
            cfg   = FORCE_CHANNELS[idx]
            color = self._CH_COLORS[idx % len(self._CH_COLORS)]
            card  = QtWidgets.QFrame()
            card.setStyleSheet(
                f"QFrame {{ border: 2px solid {color}; border-radius: 10px;"
                f" background: transparent; padding: 6px; }}"
            )
            cv = QtWidgets.QVBoxLayout(card); cv.setSpacing(2)
            lbl_name = QtWidgets.QLabel(cfg.get("label", f"CH{idx}"))
            lbl_name.setStyleSheet(f"color:{color}; font-weight:700; font-size:11pt; border:none;")
            lbl_kg   = QtWidgets.QLabel("0.00 kg")
            lbl_kg.setStyleSheet(f"color:{color}; font-size:22pt; font-weight:700; border:none;")
            lbl_v    = QtWidgets.QLabel("0.0000 V")
            lbl_v.setStyleSheet("color:#888; font-size:9pt; border:none;")
            cv.addWidget(lbl_name); cv.addWidget(lbl_kg); cv.addWidget(lbl_v)
            cards_hb.addWidget(card)
            self._card_kg_lbls.append(lbl_kg)
            self._card_v_lbls.append(lbl_v)
            self._card_name_lbls.append(lbl_name)
            self._cards.append(card)
            self._card_colors.append(color)
        layout.addWidget(cards_frame)

        # ── time-series plots ──────────────────────────────────────────
        plot_frame = QtWidgets.QFrame(); plot_frame.setProperty("card","true")
        pg_layout  = QtWidgets.QVBoxLayout(plot_frame); pg_layout.setContentsMargins(4,4,4,4)
        self._plots  = []
        self._curves = []
        for idx in range(self._n):
            cfg   = FORCE_CHANNELS[idx]
            color = self._CH_COLORS[idx % len(self._CH_COLORS)]
            pw    = pg.PlotWidget()
            pw.setTitle(cfg.get("label", f"CH{idx}"), color=color, size="10pt", bold=True)
            pw.setLabel("left",   "Weight (kg)", size="9pt")
            pw.setLabel("bottom", "t (s)",        size="9pt")
            pw.showGrid(x=True, y=True, alpha=0.2)
            curve = pw.plot(pen=pg.mkPen(color=color, width=2))
            self._plots.append(pw); self._curves.append(curve)
            pg_layout.addWidget(pw)
        # Link X axes
        if len(self._plots) > 1:
            self._plots[1].setXLink(self._plots[0])
        layout.addWidget(plot_frame, 1)

        # ── AIN assignment display ─────────────────────────────────────
        info_frame = QtWidgets.QFrame(); info_frame.setProperty("card","true")
        ih = QtWidgets.QHBoxLayout(info_frame); ih.setContentsMargins(10,4,10,4)
        self._lbl_info = QtWidgets.QLabel(self._build_info_str())
        self._lbl_info.setStyleSheet("color:#888; font-size:9pt;")
        ih.addWidget(self._lbl_info); ih.addStretch(1)
        layout.addWidget(info_frame)

        self._apply_theme(self._theme)

        # ── timer ──────────────────────────────────────────────────────
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(int(1000 / UI_REFRESH_HZ))

        # ── connections ────────────────────────────────────────────────
        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_tare.clicked.connect(self._on_tare)
        self.btn_settings.clicked.connect(self._on_settings)
        worker.sig_update.connect(self._on_data)
        worker.sig_status.connect(lambda m: self.lbl_state.setText(m[:80]))
        worker.sig_conn_status.connect(self._on_conn_status)
        worker.sig_adc_status.connect(self._on_adc_status)

    @QtCore.pyqtSlot(int, bool)
    def _on_conn_status(self, idx: int, online: bool):
        if idx >= self._n:
            return
        self._channel_online[idx] = online
        color = self._card_colors[idx]
        name  = FORCE_CHANNELS[idx].get("label", f"CH{idx}")
        if online:
            self._cards[idx].setStyleSheet(
                f"QFrame {{ border: 2px solid {color}; border-radius: 10px;"
                f" background: transparent; padding: 6px; }}")
            self._card_name_lbls[idx].setText(name)
            self._card_name_lbls[idx].setStyleSheet(
                f"color:{color}; font-weight:700; font-size:11pt; border:none;")
        else:
            self._cards[idx].setStyleSheet(
                "QFrame { border: 2px solid #D32F2F; border-radius: 10px;"
                " background: transparent; padding: 6px; }")
            self._card_name_lbls[idx].setText(f"{name}  ● OFFLINE 掉线")
            self._card_name_lbls[idx].setStyleSheet(
                "color:#D32F2F; font-weight:700; font-size:11pt; border:none;")

    @QtCore.pyqtSlot(bool)
    def _on_adc_status(self, online: bool):
        if not online:
            self.lbl_state.setText("State: ADC 掉线，正在重连…")
            self.lbl_state.setStyleSheet("color:#D32F2F; font-weight:bold;")

    # ── helpers ───────────────────────────────────────────────────────
    def _build_info_str(self) -> str:
        parts = []
        for idx, cfg in enumerate(FORCE_CHANNELS):
            parts.append(
                f"CH{idx}: AIN{cfg['ain']}  {cfg['output_mode']}  "
                f"max {cfg['sensor_max_kg']} kg"
            )
        return "   |   ".join(parts)

    # ── slots ─────────────────────────────────────────────────────────
    def _on_start(self):
        if not _SPIDEV_OK:
            self.lbl_state.setText("State: spidev missing")
            self.lbl_state.setStyleSheet("color:#D32F2F; font-weight:bold;")
            return
        self._worker.start()
        self.lbl_state.setText("State: RUNNING")
        self.lbl_state.setStyleSheet("color:#2E7D32; font-weight:bold;")

    def _on_stop(self):
        self._worker.stop()
        self.lbl_state.setText("State: IDLE")
        self.lbl_state.setStyleSheet("font-weight:bold;")

    def _on_tare(self):
        """Capture current readings as tare offset."""
        for idx in range(self._n):
            self._tare[idx] = self._last[idx][1]

    def _on_settings(self):
        dlg = ForceSensorSettingsDialog(self)
        if dlg.exec_() == QtWidgets.QDialog.Accepted:
            # Restart worker to pick up new config
            self._worker.stop()
            time.sleep(0.15)
            # Rebuild ADC channel ranges then restart
            self._worker.start()
            # Update plot titles and info bar
            for idx in range(self._n):
                cfg   = FORCE_CHANNELS[idx]
                color = self._CH_COLORS[idx % len(self._CH_COLORS)]
                self._plots[idx].setTitle(cfg.get("label", f"CH{idx}"),
                                          color=color, size="10pt", bold=True)
            self._lbl_info.setText(self._build_info_str())

    @QtCore.pyqtSlot(int, float, float)
    def _on_data(self, ch_idx: int, voltage: float, weight_kg: float):
        if ch_idx >= self._n:
            return
        tt = time.time() - self._t0
        self._t_bufs[ch_idx].append(tt)
        self._kg_bufs[ch_idx].append(weight_kg - self._tare[ch_idx])
        self._last[ch_idx] = (voltage, weight_kg)

    def _refresh(self):
        # Hidden pages retain their bounded data buffers but do not redraw.
        # Standalone unshown panels remain usable by offline inspection/tests.
        if self.window().isVisible() and not self.isVisible():return
        for idx in range(self._n):
            v, w_raw = self._last[idx]
            w = w_raw - self._tare[idx]
            self._card_kg_lbls[idx].setText(f"{w:+.2f} kg")
            self._card_v_lbls[idx].setText(f"{v:+.4f} V · 实读 {self._worker.actual_rate(idx):.1f} Hz / 目标 {FORCE_SAMPLE_HZ} Hz")

            t = list(self._t_bufs[idx])
            y = list(self._kg_bufs[idx])
            if len(t) > 1:
                xmax = t[-1]; xmin = max(0.0, xmax - FORCE_PLOT_WIN_S)
                self._plots[idx].setXRange(xmin, xmax, padding=0)
                self._curves[idx].setData(t, y)

    # ── theme ─────────────────────────────────────────────────────────
    def set_theme(self, theme: str):
        self._theme = theme
        self._apply_theme(theme)

    def _apply_theme(self, theme: str):
        p = _plot_theme_params(theme)
        for pw in self._plots:
            pw.setBackground(p['bg'])
            pw.showGrid(x=True, y=True, alpha=p['grid'])
            pw.getAxis('left').setPen(pg.mkPen(p['axis']))
            pw.getAxis('bottom').setPen(pg.mkPen(p['axis']))
            pw.getAxis('left').setTextPen(p['text'])
            pw.getAxis('bottom').setTextPen(p['text'])






# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                          VISION WORKER                                  ║
# ║  D435i depth → x-z projection → CNN terrain classifier (vision_terrain) ║
# ╚══════════════════════════════════════════════════════════════════════════╝

# Fixed label <-> numeric id mapping for DataManager/CSV storage (buffers are
# float-only). Kept in sync with terrain_mode_switch.LABELS (the full,
# forward-looking taxonomy — see hip_on_vision/TERRAIN_EXPANSION_RESEARCH_CN.md
# 2026-08-18) so DataManager already has a stable numeric id for every
# terrain class the moment a retrained model starts emitting it, even
# though only flat/stairs_up/stairs_down have a trained model today.
# Labels the currently-loaded model doesn't know about simply never appear
# in the data — VISION_LABEL_IDS.get(label, -1) below handles that safely.
VISION_LABEL_IDS = {label: i for i, label in enumerate(_TERRAIN_LABELS)}


class VisionWorker(QtCore.QObject):
    """
    Background thread: D435i depth stream → x-z projection → CNN → majority
    vote. Runs TWO independently-trained classifiers on the same projection
    image — the stairs model (flat/stairs_up/stairs_down) and the slope
    model (flat/slope_up/slope_down, added 2026-08-18) — and combines them
    via vision_terrain.fuse_stairs_and_slope(). The slope model is optional:
    if its weights fail to load, the worker just runs stairs-only, same as
    before. Mirrors the other workers' online/offline + rate-decoupling
    design:

      • Camera frames are grabbed at native FPS (cheap), but CNN inference
        (+ the image processing needed for the on-screen preview) is
        decimated to VISION_INFER_HZ / VISION_DISPLAY_HZ — running a CNN at
        30 Hz for a rehab monitor UI is not necessary and was exactly the
        kind of "hardware I/O rate == GUI rate" mistake that caused the
        original freezes (see WORKING_LOG.md 2026-08-17). DataManager
        logging is decimated further still, to VISION_LOG_HZ.
      • If the camera is unplugged/pipeline.start() fails, the worker keeps
        retrying every VISION_RECONNECT_INTERVAL_S instead of dying, and
        emits sig_conn_status(False)/(True) only on actual state transitions
        — same contract as Imu/Motor/ForceSensorWorker.

    sig_update(raw_label, stable_label, confidence, probs_dict) — cheap,
        emitted every inference tick.
    sig_frame(depth_vis_bgr_or_None, projection_gray) — numpy arrays by
        reference; the panel redraws from the latest one on its own QTimer,
        decoupling paint cost from arrival rate.
    """
    sig_update      = QtCore.pyqtSignal(str, str, float, dict)
    sig_frame       = QtCore.pyqtSignal(object, object)
    sig_status      = QtCore.pyqtSignal(str)
    sig_conn_status = QtCore.pyqtSignal(bool)

    def __init__(self, data_manager: DataManager, parent=None):
        super().__init__(parent)
        self._dm      = data_manager
        self._running = Event()
        self._alive   = True
        self._thread  = None
        self._pipeline = None
        self._online       = False
        self._last_ok_time = 0.0
        # Latest frame, kept for on-demand dataset capture (see
        # capture_dataset_sample). Guarded by its own lock since it's
        # written from the worker thread and read from the GUI thread when
        # the operator presses "Capture Sample" — separate from self._dm's
        # lock because this is GUI-triggered, not a continuous data path.
        self._capture_lock = Lock()
        self._latest_depth_raw  = None
        self._latest_projection = None
        self._latest_depth_scale = None
        self._latest_intrinsics  = None
        self._latest_timing = None

    def is_online(self) -> bool:
        return self._online

    def capture_dataset_sample(self, label: str, output_root: str) -> str | None:
        """
        Save the most recent depth frame as a labeled training sample under
        output_root/label/ (see vision_terrain.save_dataset_sample — same
        4-file layout hip_on_vision's own recorders use, so it drops
        straight into train_projection_cnn.py / train_slope_cnn.py later).
        Returns the saved .npy path, or None if no frame has arrived yet
        (e.g. camera not started/still connecting).
        """
        with self._capture_lock:
            if self._latest_depth_raw is None:
                return None
            depth_raw  = self._latest_depth_raw.copy()
            projection = self._latest_projection.copy() if self._latest_projection is not None else None
            depth_scale = self._latest_depth_scale
            intrinsics  = dict(self._latest_intrinsics)
            timing = dict(self._latest_timing or {})
        session = self._dm.session
        if timing.get('session_id',session.session_id) != session.session_id:
            return None  # wait for a frame from the newly selected session
        if projection is None:
            projection = _vt.depth_to_projection(depth_raw, intrinsics, depth_scale=depth_scale)
        path = _vt.save_dataset_sample(output_root, label, depth_raw, projection,
                                       depth_scale, intrinsics, extra_meta={'frame_timing': timing,
                                       'session_id': session.session_id,
                                       'subject_id': session.metadata.get('subject_id','unknown'),
                                       'location': session.metadata.get('location','unknown')})
        stem = os.path.basename(path).removeprefix('depth_raw_').removesuffix('.npy')
        directory = os.path.dirname(path)
        session.artifact('camera_sample', path, frame_timing=timing, label=label,
                         metadata_path=os.path.join(directory,f'meta_{stem}.json'),
                         projection_path=os.path.join(directory,f'projection_{stem}.png'),
                         depth_vis_path=os.path.join(directory,f'depth_vis_{stem}.png'))
        return path

    def start(self):
        if not _VISION_MODULE_OK:
            message = ("[Vision] Offline preview: physical camera disabled" if _PREVIEW else
                       "[Vision] dependencies missing (torch/pyrealsense2/opencv)")
            self.sig_status.emit(message)
            return False
        if self._thread and self._thread.is_alive():
            self._running.set()
            return True
        self._thread = Thread(target=self._loop, daemon=True, name="vision-worker")
        self._thread.start()
        self._running.set()
        return True

    def stop(self):
        self._running.clear()

    def shutdown(self):
        # Joining here matters more than for the other workers: this thread
        # can be blocked inside a native pyrealsense2 call
        # (wait_for_frames). Letting the interpreter tear down while that's
        # still in flight (daemon threads don't get waited for otherwise)
        # was observed to segfault the whole process on shutdown — see
        # WORKING_LOG.md 2026-08-18.
        self._alive = False
        self._running.clear()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _set_online(self, online: bool):
        if self._online != online:
            self._online = online
            self.sig_conn_status.emit(online)

    def _open_pipeline(self):
        """Try depth+accel first (richer, matches training/benchmark setup),
        fall back to depth-only if the accel stream can't be enabled — same
        graceful-degradation the original hip_on_vision script used."""
        import pyrealsense2 as rs
        pipeline = rs.pipeline()
        cfg_full = rs.config()
        cfg_full.enable_stream(rs.stream.depth, _vt.WIDTH, _vt.HEIGHT, rs.format.z16, VISION_CAPTURE_FPS)
        if not VISION_IMAGE_ONLY:
            cfg_full.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 100)
        try:
            profile = pipeline.start(cfg_full)
        except RuntimeError:
            pipeline = rs.pipeline()
            cfg_depth = rs.config()
            cfg_depth.enable_stream(rs.stream.depth, _vt.WIDTH, _vt.HEIGHT, rs.format.z16, VISION_CAPTURE_FPS)
            profile = pipeline.start(cfg_depth)
        depth_scale = float(profile.get_device().first_depth_sensor().get_depth_scale())
        return pipeline, depth_scale

    def _image_loop(self):
        import socket,struct,subprocess
        child=None;peer=None;buffer=bytearray();last_display=0.;last_status=0.
        def close_child():
            nonlocal child,peer
            if peer:peer.close();peer=None
            if child:
                child.terminate()
                try:child.wait(timeout=.5)
                except subprocess.TimeoutExpired:child.kill();child.wait(timeout=.5)
                child=None
        try:
            while self._alive:
                if not self._running.is_set():
                    close_child();buffer.clear();time.sleep(.05);continue
                if child is None:
                    peer,other=socket.socketpair();peer.settimeout(.1)
                    child=subprocess.Popen([sys.executable,os.path.join(os.path.dirname(__file__),'hipexo_camera_capture.py'),
                        '--fd',str(other.fileno()),'--fps',str(VISION_CAPTURE_FPS)],pass_fds=(other.fileno(),),
                        stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                    other.close();buffer.clear()
                try:
                    try:chunk=peer.recv(1024*1024)
                    except socket.timeout:continue
                    if not chunk:raise ConnectionError('Camera process exited')
                    buffer.extend(chunk)
                    while len(buffer)>=8:
                        nh,nd=struct.unpack('!II',buffer[:8])
                        if nh>65536 or nd>2*1024*1024:raise ValueError('Invalid camera frame size')
                        if len(buffer)<8+nh+nd:break
                        timing=json.loads(bytes(buffer[8:8+nh]));raw=bytes(buffer[8+nh:8+nh+nd]);del buffer[:8+nh+nd]
                        if 'error' in timing:
                            self._set_online(False)
                            if time.monotonic()-last_status>1:
                                self.sig_status.emit('Camera: '+timing['error']);last_status=time.monotonic()
                            continue
                        depth=np.frombuffer(raw,dtype=np.uint16).reshape(timing['shape']).copy()
                        timing['session_id']=self._dm.session.session_id
                        self._dm.record_camera_image(depth,timing)
                        self._set_online(True);self._last_ok_time=time.time()
                        with self._capture_lock:
                            self._latest_depth_raw=depth;self._latest_projection=None
                            self._latest_depth_scale=timing['depth_scale_m'];self._latest_intrinsics=timing['intrinsics']
                            self._latest_timing=timing
                        if time.perf_counter()-last_display>=1./VISION_DISPLAY_HZ:
                            self.sig_frame.emit(_vt.make_depth_vis(depth),None);last_display=time.perf_counter()
                except Exception as exc:
                    self._set_online(False);self.sig_status.emit('Camera: '+str(exc));close_child();time.sleep(.2)
        finally:close_child()

    def _loop(self):
        import pyrealsense2 as rs

        if VISION_IMAGE_ONLY:
            return self._image_loop()

        try:
            device = _vt.torch.device("cuda" if _vt.torch.cuda.is_available() else "cpu")
            stairs_model, stairs_labels, _meta = _vt.load_model(VISION_MODEL_NAME, device)
        except Exception as e:
            self.sig_status.emit(f"[Vision] Model load failed: {e}")
            return
        # Slope model is optional — if its weights are missing, VisionWorker
        # still runs on stairs-only (fuse_stairs_and_slope degrades to just
        # trusting the stairs model whenever slope_model is None).
        slope_model = slope_labels = None
        try:
            slope_model, slope_labels, _slope_meta = _vt.load_model(_vt.SLOPE_MODEL_NAME, device)
            self.sig_status.emit(
                f"[Vision] Models loaded ({device}): stairs={stairs_labels}, slope={slope_labels}")
        except Exception as e:
            self.sig_status.emit(
                f"[Vision] Stairs model loaded ({device}); slope model unavailable: {e}")

        history = deque(maxlen=VISION_HISTORY_SIZE)
        infer_period    = 1.0 / max(1, VISION_INFER_HZ)
        infer_decimate  = max(1, int(round(VISION_CAPTURE_FPS / max(1, VISION_INFER_HZ))))
        log_decimate    = max(1, int(round(VISION_INFER_HZ / max(1, VISION_LOG_HZ))))
        # depth_vis (cv2 colormap on a full 640x480 frame) is the single most
        # expensive per-frame step after the CNN itself — only regenerate it
        # at VISION_DISPLAY_HZ, not on every inference tick.
        vis_decimate    = max(1, int(round(VISION_INFER_HZ / max(1, VISION_DISPLAY_HZ))))
        frame_count = 0
        infer_count = 0
        last_infer_t = 0.0
        last_open_attempt = 0.0

        while self._alive:
            if not self._running.is_set():
                if self._pipeline is not None:
                    try: self._pipeline.stop()
                    except Exception: pass
                    self._pipeline = None
                time.sleep(0.1)
                continue

            if self._pipeline is None:
                now = time.time()
                if (now - last_open_attempt) < VISION_RECONNECT_INTERVAL_S:
                    time.sleep(0.1)
                    continue
                last_open_attempt = now
                try:
                    self._pipeline, depth_scale = self._open_pipeline()
                    camera_stream_id = uuid.uuid4().hex
                    self.sig_status.emit("[Vision] Camera connected")
                except Exception as e:
                    self._set_online(False)
                    self.sig_status.emit(f"[Vision] Camera open failed: {e}")
                    continue

            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
                received_wall_ns, received_mono_ns = time.time_ns(), time.perf_counter_ns()
                frame_session = self._dm.session
                depth_frame = frames.get_depth_frame()
                if not depth_frame:
                    raise RuntimeError("no depth frame")
            except Exception as e:
                if (time.time() - self._last_ok_time) >= VISION_OFFLINE_AFTER_S:
                    self._set_online(False)
                self.sig_status.emit(f"[Vision] Frame read failed: {e}")
                try: self._pipeline.stop()
                except Exception: pass
                self._pipeline = None
                continue

            frame_count += 1
            self._last_ok_time = time.time()
            self._set_online(True)

            # Throttle the expensive part (projection + CNN + image prep) to
            # VISION_INFER_HZ regardless of the camera's native frame rate.
            if (frame_count % infer_decimate) != 0:
                continue
            now = time.time()
            if (now - last_infer_t) < infer_period * 0.5:
                continue
            last_infer_t = now

            try:
                frame_timing = camera_frame_timing(depth_frame, received_wall_ns,
                                                  received_mono_ns, camera_stream_id)
                frame_timing['session_id'] = frame_session.session_id
                frame_timing['inference_started_mono_ns'] = time.perf_counter_ns()
                depth_raw = np.asanyarray(depth_frame.get_data())
                intr = _vt.intrinsics_from_realsense(depth_frame)
                projection = _vt.depth_to_projection(depth_raw, intr, depth_scale=depth_scale)
                # Both models are tiny (~5ms each on CPU, see WORKING_LOG.md
                # 2026-08-18) and run on the SAME projection image (verified
                # they use identical depth_to_projection parameters) — well
                # within the VISION_INFER_HZ budget even running both.
                stairs_label, stairs_conf, stairs_probs = _vt.classify_projection(
                    stairs_model, stairs_labels, projection, device)
                if slope_model is not None:
                    slope_label, slope_conf, slope_probs = _vt.classify_projection(
                        slope_model, slope_labels, projection, device)
                    raw_label, confidence, probs = _vt.fuse_stairs_and_slope(
                        stairs_label, stairs_conf, stairs_probs,
                        slope_label, slope_conf, slope_probs)
                else:
                    raw_label, confidence, probs = stairs_label, stairs_conf, stairs_probs
                frame_timing.update(inference_done_wall_ns=time.time_ns(),
                                    inference_done_mono_ns=time.perf_counter_ns())
                with self._capture_lock:
                    self._latest_depth_raw = depth_raw.copy()
                    self._latest_projection = projection
                    self._latest_depth_scale = depth_scale
                    self._latest_intrinsics = intr
                    self._latest_timing = frame_timing
            except Exception as e:
                self.sig_status.emit(f"[Vision] Inference error: {e}")
                continue

            history.append(raw_label)
            stable_label = max(set(history), key=history.count) if history else raw_label
            infer_count += 1

            t_ms = received_wall_ns / 1e6
            if (infer_count % log_decimate) == 0 and frame_session is self._dm.session:
                self._dm.append_frame("vision", t_ms, {
                    **frame_timing,
                    "terrain_id":   float(VISION_LABEL_IDS.get(raw_label, -1)),
                    "confidence":   confidence,
                    "prob_flat":        probs.get("flat", 0.0),
                    "prob_stairs_up":   probs.get("stairs_up", 0.0),
                    "prob_stairs_down": probs.get("stairs_down", 0.0),
                    "prob_slope_up":    probs.get("slope_up", 0.0),
                    "prob_slope_down":  probs.get("slope_down", 0.0),
                }, t_mono_ns=received_mono_ns)

            self.sig_update.emit(raw_label, stable_label, confidence, probs)

            if (infer_count % vis_decimate) == 0:
                depth_vis = _vt.make_depth_vis(depth_raw)
                self.sig_frame.emit(depth_vis, projection)

        if self._pipeline is not None:
            try: self._pipeline.stop()
            except Exception: pass
            self._pipeline = None


class VisionPanel(QtWidgets.QWidget):
    """
    Live D435i terrain-recognition panel.

    Layout: control bar / big depth+projection preview / Stable-label
    readout with per-class probabilities / connection status — same visual
    language (● ONLINE/OFFLINE badge, card frames) as the other panels.
    """

    _LABEL_COLORS = {"flat": "#43A047", "stairs_up": "#FB8C00", "stairs_down": "#E53935",
                      "slope_up": "#8E24AA", "slope_down": "#3949AB"}

    def __init__(self, worker: VisionWorker, parent=None):
        super().__init__(parent)
        self._worker = worker
        self._theme  = 'light'
        self._last_frame = (None, None)   # (depth_vis_bgr, projection_gray)
        self._last_probs = {}
        self._online = False

        layout = QtWidgets.QVBoxLayout(self)

        # ── control bar ───────────────────────────────────────────────
        ctrl = QtWidgets.QFrame(); ctrl.setProperty("card", "true")
        hb = QtWidgets.QHBoxLayout(ctrl); hb.setContentsMargins(10, 8, 10, 8)
        self.btn_start = QtWidgets.QPushButton("▶  Start"); self.btn_start.setProperty("accent", "true")
        self.btn_stop  = QtWidgets.QPushButton("■  Stop");  self.btn_stop.setProperty("danger", "true")
        for b in (self.btn_start, self.btn_stop):
            b.setFixedHeight(_px(30)); hb.addWidget(b)
        hb.addStretch(1)
        self.lbl_conn = QtWidgets.QLabel("● OFFLINE")
        self.lbl_conn.setStyleSheet("color:#9E9E9E; font-weight:700;")
        hb.addWidget(self.lbl_conn)
        self.lbl_state = QtWidgets.QLabel("State: IDLE")
        self.lbl_state.setStyleSheet("font-weight:bold;")
        hb.addWidget(self.lbl_state)
        layout.addWidget(ctrl)

        # ── image preview ────────────────────────────────────────────
        img_frame = QtWidgets.QFrame(); img_frame.setProperty("card", "true")
        img_hb = QtWidgets.QHBoxLayout(img_frame); img_hb.setContentsMargins(8, 8, 8, 8)
        self.lbl_depth_img = QtWidgets.QLabel("(no camera)")
        self.lbl_depth_img.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_depth_img.setMinimumSize(_px(320), _px(240))
        self.lbl_depth_img.setStyleSheet("background:#111; color:#888; border-radius:8px;")
        self.lbl_proj_img = QtWidgets.QLabel("(no camera)")
        self.lbl_proj_img.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_proj_img.setMinimumSize(_px(160), _px(160))
        self.lbl_proj_img.setStyleSheet("background:#111; color:#888; border-radius:8px;")
        img_hb.addWidget(self.lbl_depth_img, 2)
        img_hb.addWidget(self.lbl_proj_img, 1)
        layout.addWidget(img_frame, 1)

        # ── readout ───────────────────────────────────────────────────
        readout = QtWidgets.QFrame(); readout.setProperty("card", "true")
        rv = QtWidgets.QVBoxLayout(readout); rv.setContentsMargins(14, 10, 14, 10)
        self.lbl_stable = QtWidgets.QLabel("Stable: —")
        self.lbl_stable.setStyleSheet("font-size:20pt; font-weight:800;")
        self.lbl_raw = QtWidgets.QLabel("Raw: —    Confidence: —")
        self.lbl_raw.setStyleSheet("color:#888;")
        self.lbl_probs = QtWidgets.QLabel("")
        self.lbl_probs.setStyleSheet("font-family:monospace;")
        rv.addWidget(self.lbl_stable)
        rv.addWidget(self.lbl_raw)
        rv.addWidget(self.lbl_probs)
        layout.addWidget(readout)

        # ── field data collection (2026-08-18 — see WORKING_LOG.md
        #    "数据采集工具细化" and hip_on_vision/TERRAIN_EXPANSION_
        #    RESEARCH_CN.md §5) ─────────────────────────────────────────
        # Saves samples in the exact layout hip_on_vision's own
        # train_projection_cnn.py / train_slope_cnn.py already expect
        # (vision_terrain.save_dataset_sample), under the CURRENT
        # session's folder — so a field data-collection trip is
        # automatically organized by subject+location+time the same way
        # a normal recording session already is (see SessionManager).
        capture_frame = QtWidgets.QFrame(); capture_frame.setProperty("card", "true")
        cv_ = QtWidgets.QVBoxLayout(capture_frame); cv_.setContentsMargins(14, 8, 14, 8)
        lbl_capture_title = QtWidgets.QLabel("数据采集  Field Data Collection")
        lbl_capture_title.setStyleSheet("font-weight:700;")
        cv_.addWidget(lbl_capture_title)

        cap_row = QtWidgets.QHBoxLayout()
        self.cb_capture_label = QtWidgets.QComboBox()
        self.cb_capture_label.setMinimumHeight(_px(40))
        self.cb_capture_label.addItems(list(_TERRAIN_LABELS))
        cap_row.addWidget(self.cb_capture_label, 1)
        self.btn_capture = QtWidgets.QPushButton("📸  采集样本 Capture Sample")
        self.btn_capture.setProperty("accent", "true")
        self.btn_capture.setMinimumHeight(_px(40))
        cap_row.addWidget(self.btn_capture)
        cv_.addLayout(cap_row)

        self.lbl_capture_status = QtWidgets.QLabel("尚未采集任何样本 / no samples captured yet")
        self.lbl_capture_status.setStyleSheet("color:#888;")
        self.lbl_capture_status.setWordWrap(True)
        cv_.addWidget(self.lbl_capture_status)
        layout.addWidget(capture_frame)

        # ── auto control-mode switching (off by default; see
        #    terrain_mode_switch.py — MainWindow owns the actual
        #    TerrainModeSwitcher and wires this checkbox to it) ──────────
        auto_frame = QtWidgets.QFrame(); auto_frame.setProperty("card", "true")
        av = QtWidgets.QVBoxLayout(auto_frame); av.setContentsMargins(14, 8, 14, 8)
        self.chk_auto_mode = QtWidgets.QCheckBox(
            "⚠ 自动地形切换控制策略 (Auto terrain → control-mode switching)")
        self.chk_auto_mode.setStyleSheet("color:#B71C1C; font-weight:700;")
        self.lbl_auto_status = QtWidgets.QLabel(
            "关闭 — 手动控制不受影响 (disabled — manual control unaffected)")
        self.lbl_auto_status.setStyleSheet("color:#888;")
        av.addWidget(self.chk_auto_mode)
        av.addWidget(self.lbl_auto_status)
        layout.addWidget(auto_frame)
        if VISION_IMAGE_ONLY:
            self.lbl_proj_img.hide()
            self.lbl_stable.setText("图像采集模式 / Images only")
            self.lbl_raw.setText("Record 保存深度 PNG 和帧时间戳；地形识别已停用")
            self.lbl_probs.hide()
            self.chk_auto_mode.setEnabled(False)
            self.lbl_auto_status.setText("图像模式下自动地形控制不可用")

        self._apply_theme(self._theme)

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(int(1000 / VISION_DISPLAY_HZ))

        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop.clicked.connect(self._on_stop)
        worker.sig_update.connect(self._on_update)
        worker.sig_frame.connect(self._on_frame)
        worker.sig_status.connect(lambda m: self.lbl_state.setText(m[:90]))
        worker.sig_conn_status.connect(self._on_conn_status)

        if not _VISION_MODULE_OK:
            self.lbl_state.setText("OFFLINE PREVIEW — camera disabled" if _PREVIEW else
                                  "State: dependencies missing (torch / pyrealsense2 / opencv)")
            self.lbl_state.setStyleSheet("color:#D32F2F; font-weight:bold;")

    # ── slots ─────────────────────────────────────────────────────────
    def _on_start(self):
        if self._worker.start():
            self.lbl_state.setText("State: STARTING — waiting for camera/model")

    def _on_stop(self):
        self._worker.stop()
        self.lbl_state.setText("State: IDLE")
        self._set_conn(False)

    @QtCore.pyqtSlot(bool)
    def _on_conn_status(self, online: bool):
        self._set_conn(online)

    def _set_conn(self, online: bool):
        self._online = online
        if online:
            self.lbl_conn.setText("● ONLINE")
            self.lbl_conn.setStyleSheet("color:#2E7D32; font-weight:700;")
        else:
            self.lbl_conn.setText("● OFFLINE")
            self.lbl_conn.setStyleSheet("color:#D32F2F; font-weight:700;")

    @QtCore.pyqtSlot(str, str, float, dict)
    def _on_update(self, raw_label, stable_label, confidence, probs):
        self._last_probs = probs
        color = self._LABEL_COLORS.get(stable_label, "#444")
        self.lbl_stable.setText(f"Stable: {stable_label}")
        self.lbl_stable.setStyleSheet(f"font-size:20pt; font-weight:800; color:{color};")
        self.lbl_raw.setText(f"Raw: {raw_label}    Confidence: {confidence:.2f}")
        self.lbl_probs.setText("   ".join(f"{k}: {v:.2f}" for k, v in probs.items()))

    @QtCore.pyqtSlot(object, object)
    def _on_frame(self, depth_vis, projection):
        self._last_frame = (depth_vis, projection)

    def _refresh(self):
        # Hidden pages retain their bounded data buffers but do not redraw.
        # Standalone unshown panels remain usable by offline inspection/tests.
        if self.window().isVisible() and not self.isVisible():return
        depth_vis, projection = self._last_frame
        if depth_vis is not None:
            self.lbl_depth_img.setPixmap(self._ndarray_bgr_to_pixmap(depth_vis, self.lbl_depth_img.size()))
        if projection is not None:
            self.lbl_proj_img.setPixmap(self._ndarray_gray_to_pixmap(projection, self.lbl_proj_img.size()))

    @staticmethod
    def _ndarray_bgr_to_pixmap(arr, target_size):
        h, w = arr.shape[:2]
        arr = np.ascontiguousarray(arr[:, :, ::-1])  # BGR -> RGB, contiguous for QImage
        qimg = QtGui.QImage(arr.data, w, h, arr.strides[0], QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(qimg)
        return pix.scaled(target_size, QtCore.Qt.KeepAspectRatio, QtCore.Qt.FastTransformation)

    @staticmethod
    def _ndarray_gray_to_pixmap(arr, target_size):
        arr = np.ascontiguousarray(arr)
        h, w = arr.shape[:2]
        qimg = QtGui.QImage(arr.data, w, h, arr.strides[0], QtGui.QImage.Format_Grayscale8)
        pix = QtGui.QPixmap.fromImage(qimg)
        return pix.scaled(target_size, QtCore.Qt.KeepAspectRatio, QtCore.Qt.FastTransformation)

    # ── theme ─────────────────────────────────────────────────────────
    def set_theme(self, theme: str):
        self._theme = theme
        self._apply_theme(theme)

    def _apply_theme(self, theme: str):
        fg = "#E6EAF2" if theme == "dark" else "#222"
        self.lbl_probs.setStyleSheet(f"font-family:monospace; color:{fg};")


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                          LIDAR WORKER                                   ║
# ║  COIN-D6 (国科光芯) 360° 2D scanning dToF — see lidar_d6.py for the      ║
# ║  protocol/geometry and hip_on_vision/LIDAR_FUSION_DESIGN_CN.md for the  ║
# ║  necessity analysis this was built from. See WORKING_LOG.md 2026-08-20. ║
# ╚══════════════════════════════════════════════════════════════════════════╝

from lidar_interface import LidarWorker as _SharedLidarWorker, LidarPanel


class LidarWorker(_SharedLidarWorker):
    def __init__(self, side, port, data_manager, parent=None):
        super().__init__(side, port, data_manager, parent,
                         reserved_ports=[device[0] for device in MOTOR_DEVICES])

    def start(self):
        if _PREVIEW:
            self.sig_status.emit("Preview: physical LiDAR disabled")
            return
        super().start()


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                          SESSION DIALOG                                 ║
# ║  Touch-screen friendly: subject ID + location, nothing else required.   ║
# ╚══════════════════════════════════════════════════════════════════════════╝

class SessionDialog(QtWidgets.QDialog):
    """
    Replaces the old free-form multi-field ProfileDialog. An operator on a
    touch screen (no physical keyboard) only ever needs to set two things
    per subject: their ID and where the experiment is happening. Everything
    else (save folder, timestamps) is automatic — see SessionManager.

    On accept(), the dialog exposes `.subject_id`, `.location` and
    `.manual_path` (None ⇒ auto folder mode) for the caller to act on.
    """

    def __init__(self, parent, session_mgr: SessionManager):
        super().__init__(parent)
        self.setWindowTitle("受试者 / Subject")
        self.setMinimumWidth(_px(560))
        self._mgr = session_mgr
        self._manual_path = session_mgr.manual_path

        big = f"font-size:{max(12, int(13 * _S))}pt;"
        btn_h = _px(46)

        v = QtWidgets.QVBoxLayout(self)
        v.setSpacing(_px(12))

        lbl_subj = QtWidgets.QLabel("受试者编号 / 姓名  Subject ID / name")
        lbl_subj.setStyleSheet(big + "font-weight:700;")
        v.addWidget(lbl_subj)

        row1 = QtWidgets.QHBoxLayout()
        self._cb_subject = QtWidgets.QComboBox()
        self._cb_subject.setEditable(True)
        self._cb_subject.setToolTip("此处的信息会写入每次记录的文件夹、文件名和记录说明。")
        self._cb_subject.setStyleSheet(big)
        self._cb_subject.setMinimumHeight(btn_h)
        known = session_mgr.known_subjects()
        self._cb_subject.addItems(known)
        default_subject = session_mgr.last_subject()
        if default_subject not in known:
            self._cb_subject.addItem(default_subject)
        self._cb_subject.setCurrentText(default_subject)
        row1.addWidget(self._cb_subject, 1)

        btn_new_subject = QtWidgets.QPushButton("+ 新受试者")
        btn_new_subject.setMinimumHeight(btn_h)
        btn_new_subject.clicked.connect(self._assign_new_subject)
        row1.addWidget(btn_new_subject)
        v.addLayout(row1)

        lbl_loc = QtWidgets.QLabel("实验地点  Location")
        lbl_loc.setStyleSheet(big + "font-weight:700;")
        v.addWidget(lbl_loc)

        # Fixed, closed set of locations (EXPERIMENT_LOCATIONS) — shown as
        # big one-tap buttons rather than a dropdown, since there are only
        # ever two of them and a touch screen shouldn't need to open a menu
        # to pick between two choices.
        row_loc = QtWidgets.QHBoxLayout()
        self._loc_group   = QtWidgets.QButtonGroup(self)
        self._loc_group.setExclusive(True)
        self._loc_buttons = {}
        default_loc = session_mgr.last_location()
        if default_loc not in EXPERIMENT_LOCATIONS:
            default_loc = EXPERIMENT_LOCATIONS[0]
        for loc in EXPERIMENT_LOCATIONS:
            b = QtWidgets.QPushButton(loc)
            b.setCheckable(True)
            b.setMinimumHeight(_px(56))
            b.setStyleSheet(f"""
                QPushButton {{ {big} border:2px solid #DDE3EA; border-radius:10px;
                              background:#F7F9FC; }}
                QPushButton:checked {{ border-color:#2962FF; background:#EEF5FF;
                              color:#1A2B5F; font-weight:700; }}
            """)
            b.setChecked(loc == default_loc)
            b.clicked.connect(self._update_preview)
            self._loc_group.addButton(b)
            row_loc.addWidget(b)
            self._loc_buttons[loc] = b
        v.addLayout(row_loc)

        sep = QtWidgets.QFrame(); sep.setFrameShape(QtWidgets.QFrame.HLine)
        v.addWidget(sep)

        lbl_save = QtWidgets.QLabel("数据保存位置  Save Location")
        lbl_save.setStyleSheet(big + "font-weight:700;")
        v.addWidget(lbl_save)

        self._lbl_preview = QtWidgets.QLabel()
        self._lbl_preview.setWordWrap(True)
        self._lbl_preview.setStyleSheet("color:#888;")
        v.addWidget(self._lbl_preview)

        row2 = QtWidgets.QHBoxLayout()
        btn_manual = QtWidgets.QPushButton("手动选择文件夹…")
        btn_manual.setMinimumHeight(btn_h)
        btn_manual.clicked.connect(self._pick_manual_path)
        row2.addWidget(btn_manual)
        self._btn_auto = QtWidgets.QPushButton("恢复自动模式")
        self._btn_auto.setMinimumHeight(btn_h)
        self._btn_auto.clicked.connect(self._clear_manual_path)
        row2.addWidget(self._btn_auto)
        v.addLayout(row2)

        self._cb_subject.editTextChanged.connect(self._update_preview)
        self._update_preview()

        v.addStretch(1)
        btns = QtWidgets.QHBoxLayout()
        btn_cancel = QtWidgets.QPushButton("取消")
        btn_cancel.setMinimumHeight(btn_h)
        btn_ok = QtWidgets.QPushButton("确定，开始新会话")
        btn_ok.setProperty("accent", True)
        btn_ok.setMinimumHeight(btn_h)
        btns.addWidget(btn_cancel); btns.addStretch(1); btns.addWidget(btn_ok)
        v.addLayout(btns)
        btn_cancel.clicked.connect(self.reject)
        btn_ok.clicked.connect(self._on_confirm)

    def _assign_new_subject(self):
        nid = self._mgr.next_subject_id()
        existing = {self._cb_subject.itemText(i) for i in range(self._cb_subject.count())}
        while nid in existing:
            number = int(re.match(r"S(\d+)", nid).group(1))
            nid = f"S{number + 1:03d}"
        self._cb_subject.insertItem(0, nid)
        self._cb_subject.setCurrentText(nid)

    def _selected_location(self) -> str:
        for loc, btn in self._loc_buttons.items():
            if btn.isChecked():
                return loc
        return EXPERIMENT_LOCATIONS[0]

    def _pick_manual_path(self):
        start_dir = self._manual_path or EXPORT_DIR
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "选择保存文件夹", start_dir)
        if d:
            self._manual_path = d
            self._update_preview()

    def _clear_manual_path(self):
        self._manual_path = None
        self._update_preview()

    def _update_preview(self, *_args):
        if not hasattr(self, "_lbl_preview"):
            return   # widgets not built yet — button signals can fire during __init__
        subject  = self._cb_subject.currentText().strip() or self._mgr.next_subject_id()
        location = self._selected_location()
        if self._manual_path:
            self._lbl_preview.setText(f"手动模式：所有数据将保存到\n{self._manual_path}")
            self._btn_auto.setEnabled(True)
        else:
            preview_name = f"{_safe_path_component(subject)}_{_safe_path_component(location)}_YYYYMMDD_HHMMSS"
            self._lbl_preview.setText(
                "自动模式：本次会话将新建文件夹\n"
                f"{os.path.join(EXPORT_DIR, preview_name)}\n"
                "（实际文件夹名以确认时刻的时间为准；重名会自动加编号，绝不覆盖旧数据）")
            self._btn_auto.setEnabled(False)

    def _on_confirm(self):
        self.subject_id  = self._cb_subject.currentText().strip() or self._mgr.next_subject_id()
        self.location    = self._selected_location()
        self.manual_path = self._manual_path
        self.accept()


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                           MAIN WINDOW                                   ║
# ╚══════════════════════════════════════════════════════════════════════════╝

class MainWindow(QtWidgets.QWidget):
    """
    Sidebar navigation (left) + QStackedWidget (centre) architecture.

    Panel registry — to add a new panel:
      1. Create a QWidget subclass with a set_theme(str) method.
      2. Add an entry to _PANEL_REGISTRY below (name, icon_char, class, args).
      3. Done — it appears automatically in the sidebar.
    """

    # ── Panel registry ────────────────────────────────────────────────────
    # Each entry: (display_name, unicode_icon, widget_class, constructor_kwargs)
    # constructor_kwargs is passed as **kwargs to the class.
    # Workers are injected after creation via set_worker() if present.
    _PANEL_REGISTRY = [
        ("Motor",   "⚙",  MotorPanel,       {}),
        ("IMU",     "📡", ImuPanel,          {}),
        ("Force",   "⚖",  ForceSensorPanel,  {}),
        ("EMG",     "📈", EmgPanel,          {}),
        ("Vision",  "👁",  VisionPanel,       {}),
        ("Lidar",   "🛰",  LidarPanel,        {}),
    ]

    def __init__(self):
        super().__init__()
        self.setWindowTitle("HiPExo Monitor")
        # ── Borderless fullscreen ──────────────────────────────────────
        self.setWindowFlags(
            QtCore.Qt.FramelessWindowHint |
            QtCore.Qt.WindowStaysOnTopHint
        )
        screen_geo = QtWidgets.QApplication.primaryScreen().availableGeometry()
        self.setGeometry(screen_geo)
        self.showFullScreen()
        self._theme   = 'light'
        self._session_mgr = SessionManager(state_path=SESSION_STATE_PATH, base_dir=EXPORT_DIR)
        self._current_subject     = self._session_mgr.last_subject()
        self._current_location    = self._session_mgr.last_location()
        self._current_session_dir = None
        self._recording = False
        self._collecting = False

        # Fix XDG warning on headless setups
        if "XDG_RUNTIME_DIR" not in os.environ:
            p = f"/tmp/runtime-{os.getenv('USER','user')}"
            os.environ["XDG_RUNTIME_DIR"] = p
            try: os.makedirs(p, exist_ok=True); os.chmod(p, 0o700)
            except Exception: pass

        # ── Data manager ───────────────────────────────────────────────
        self._dm = DataManager(self)
        self._dm.sig_record_error.connect(self._on_record_error)
        self._dm.sig_memory_warn.connect(self._on_memory_warn)
        self._dm.sig_flushed.connect(lambda p: self._toast(f"Flushed → {os.path.basename(p)}", True))

        # Resolve where THIS run's data goes right away — no dialog needed.
        # Auto mode: a brand-new, collision-free folder every launch (see
        # SessionManager/make_unique_session_dir — restarting the app never
        # overwrites a previous run's data). Manual mode: whatever folder
        # the operator last picked. Operator can change subject/location at
        # any time via the sidebar "受试者" button.
        self._start_new_session(self._current_subject, self._current_location,
                                 toast=False)

        # ── Workers ────────────────────────────────────────────────────
        self._imu_worker    = ImuWorker(self._dm, self)
        self._motor_worker  = MotorWorker(MOTOR_DEVICES, self._dm, self)
        self._force_worker  = ForceSensorWorker(self._dm, self)
        self._vision_worker = VisionWorker(self._dm, self)
        self._emg_worker = EmgWorker(self._dm, self)
        self._emg_worker.simulation_only = _PREVIEW
        if _PREVIEW:
            self._emg_worker.config["source"] = "simulation"
        self._lidar_worker_l = LidarWorker("L", LIDAR_PORT_L, self._dm, self)
        self._lidar_worker_r = LidarWorker("R", LIDAR_PORT_R, self._dm, self)

        # Terrain -> control-mode switcher. Disabled by default — see
        # terrain_mode_switch.py for the full safety rationale (dwell +
        # confidence gating, fail-static on camera loss, presets are
        # currently safe/passive placeholders). Wired up below, after the
        # Vision panel (which owns the arming checkbox) is built.
        self._terrain_switcher = TerrainModeSwitcher()

        # ── Panels ─────────────────────────────────────────────────────
        self._panels: dict[str, QtWidgets.QWidget] = {}
        self._stack  = QtWidgets.QStackedWidget()

        worker_map = {"EMG": self._emg_worker, "Motor": self._motor_worker, "IMU": self._imu_worker,
                      "Force": self._force_worker, "Vision": self._vision_worker,
                      "Lidar": (self._lidar_worker_l, self._lidar_worker_r)}
        for name, _icon, cls, kwargs in self._PANEL_REGISTRY:
            if name in worker_map:
                panel = cls(worker_map[name], parent=self, **kwargs)
            else:
                panel = cls(parent=self, **kwargs)
            self._panels[name] = panel
            self._stack.addWidget(panel)

        # Wire the Vision panel's arming checkbox + the vision worker's
        # predictions into the terrain switcher.
        vision_panel = self._panels["Vision"]
        vision_panel.chk_auto_mode.toggled.connect(self._on_auto_terrain_mode_toggled)
        self._vision_worker.sig_update.connect(self._on_vision_terrain_update)
        vision_panel.btn_capture.clicked.connect(self._on_capture_dataset_sample)

        # ── Layout ─────────────────────────────────────────────────────
        root = QtWidgets.QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # Sidebar
        self._sidebar = self._build_sidebar()
        root.addWidget(self._sidebar)

        # Centre (toolbar + stack)
        centre = QtWidgets.QVBoxLayout()
        centre.setContentsMargins(0, 0, 0, 0)
        centre.setSpacing(0)
        centre.addWidget(self._build_toolbar())
        centre.addWidget(self._stack, 1)
        centre.addWidget(self._build_statusbar())
        centre_w = QtWidgets.QWidget()
        centre_w.setLayout(centre)
        root.addWidget(centre_w, 1)

        # ── Initial theme ──────────────────────────────────────────────
        apply_theme(QtWidgets.QApplication.instance(), self._theme)
        self._propagate_theme()

        # ── Motor mode pre-run ─────────────────────────────────────────
        if _SDK_OK:
            try:
                _ensure_motor_mode_for_ports([p for p,_ in MOTOR_DEVICES], SWMOTOR_PATH)
            except Exception as e:
                self._toast(f"swmotor: {e}", False, 3000)

    # ── Sidebar ───────────────────────────────────────────────────────────
    def _build_sidebar(self) -> QtWidgets.QFrame:
        sb = QtWidgets.QFrame()
        sb.setProperty("sidebar","true")
        sb.setFixedWidth(_px(90))
        v = QtWidgets.QVBoxLayout(sb)
        v.setContentsMargins(0, 12, 0, 12)
        v.setSpacing(2)

        # App logo / title
        logo = QtWidgets.QLabel("HiPExo")
        logo.setAlignment(QtCore.Qt.AlignCenter)
        logo.setStyleSheet(f"color:#FFFFFF; font-size:{max(9,int(12*_S))}pt; font-weight:700; padding:{_px(6)}px 0;")
        v.addWidget(logo)
        v.addSpacing(8)

        self._nav_btns: dict[str, QtWidgets.QPushButton] = {}
        for name, icon, _cls, _kw in self._PANEL_REGISTRY:
            btn = QtWidgets.QPushButton(f"{icon}\n{name}")
            btn.setFixedHeight(_px(58))
            btn.setStyleSheet(f"""
                QPushButton {{ background:transparent; border:none; color:#A0B0CC;
                              font-size:{max(7,int(8*_S))}pt; border-radius:8px; padding:{_px(3)}px; }}
                QPushButton:hover   {{ background:rgba(255,255,255,0.10); color:#FFFFFF; }}
                QPushButton:checked {{ background:rgba(255,255,255,0.18); color:#FFFFFF; font-weight:700; }}
            """)
            btn.setCheckable(True)
            btn.clicked.connect(lambda _, n=name: self._switch_panel(n))
            v.addWidget(btn)
            self._nav_btns[name] = btn

        v.addStretch(1)

        # Subject / session button
        self._btn_profile = QtWidgets.QPushButton("👤\n受试者")
        self._btn_profile.setFixedHeight(_px(48))
        self._btn_profile.setStyleSheet("""
            QPushButton { background:transparent; border:none; color:#A0B0CC;
                          font-size:10pt; border-radius:8px; }
            QPushButton:hover { background:rgba(255,255,255,0.10); color:#FFF; }
        """)
        self._btn_profile.clicked.connect(self._open_session_dialog)
        v.addWidget(self._btn_profile)

        # Activate first panel
        first_name = self._PANEL_REGISTRY[0][0]
        self._nav_btns[first_name].setChecked(True)
        return sb

    def _switch_panel(self, name: str):
        for n, btn in self._nav_btns.items():
            btn.setChecked(n == name)
        self._stack.setCurrentWidget(self._panels[name])

    # ── Toolbar ───────────────────────────────────────────────────────────
    def _build_toolbar(self) -> QtWidgets.QFrame:
        bar = QtWidgets.QFrame(); bar.setProperty("card","true")
        bar.setFixedHeight(_px(44))
        hb = QtWidgets.QHBoxLayout(bar); hb.setContentsMargins(_px(10),_px(4),_px(10),_px(4))

        self._lbl_profile = QtWidgets.QLabel()
        self._lbl_profile.setStyleSheet("font-weight:600;")
        self._update_session_label()

        self._btn_collect = QtWidgets.QPushButton("▶  Collect All")
        self._btn_collect.setProperty("accent", "true")
        self._btn_collect.clicked.connect(self._toggle_collection)
        self._btn_rec   = QtWidgets.QPushButton("⏺  Record")
        self._btn_rec.setToolTip("Records processed/summary CSV. For raw LiDAR use Raw scans; for camera frames use Capture Sample. Remote EMG raw JSONL is saved automatically during Sync & Start.")
        self._btn_rec.setProperty("accent","true")
        self._btn_export = QtWidgets.QPushButton("💾  Export CSV")
        self._btn_theme  = QtWidgets.QPushButton("🌙  Dark")
        self._lbl_mem    = QtWidgets.QLabel("")

        for b in (self._btn_collect, self._btn_rec, self._btn_export, self._btn_theme):
            b.setFixedHeight(_px(30)); hb.addWidget(b)
        hb.addStretch(1)
        hb.addWidget(self._lbl_mem)
        hb.addWidget(self._lbl_profile)

        self._btn_rec.clicked.connect(self._toggle_recording)
        self._btn_export.clicked.connect(self._on_export)
        self._btn_theme.clicked.connect(self._toggle_theme)

        # Shortcuts
        QtWidgets.QShortcut(QKeySequence("D"), self).activated.connect(self._toggle_theme)
        QtWidgets.QShortcut(QKeySequence("S"), self).activated.connect(self._on_export)
        QtWidgets.QShortcut(QKeySequence("Escape"), self).activated.connect(self.close)
        QtWidgets.QShortcut(QKeySequence("Alt+F4"), self).activated.connect(self.close)

        return bar

    def _build_statusbar(self) -> QtWidgets.QFrame:
        bar = QtWidgets.QFrame(); bar.setProperty("card","true")
        bar.setFixedHeight(_px(28))
        hb  = QtWidgets.QHBoxLayout(bar); hb.setContentsMargins(_px(10),0,_px(10),0)
        self._lbl_status = QtWidgets.QLabel("Ready")
        self._lbl_status.setStyleSheet("color:#888; font-size:10pt;")
        self._toast_lbl  = QtWidgets.QLabel("")
        self._toast_lbl.setAlignment(QtCore.Qt.AlignCenter)
        self._toast_timer = QtCore.QTimer(self); self._toast_timer.setSingleShot(True)
        self._toast_timer.timeout.connect(lambda: self._toast_lbl.setVisible(False))
        hb.addWidget(self._lbl_status)
        hb.addStretch(1)
        hb.addWidget(self._toast_lbl)
        hb.addStretch(1)
        return bar

    # ── Theme ─────────────────────────────────────────────────────────────
    def _toggle_theme(self):
        self._theme = 'dark' if self._theme == 'light' else 'light'
        apply_theme(QtWidgets.QApplication.instance(), self._theme)
        self._propagate_theme()
        self._btn_theme.setText("☀️  Light" if self._theme=='dark' else "🌙  Dark")

    def _propagate_theme(self):
        for panel in self._panels.values():
            if hasattr(panel, 'set_theme'):
                panel.set_theme(self._theme)

    # ── Recording / Export ────────────────────────────────────────────────
    def _toggle_collection(self):
        if not self._collecting:
            self._emg_worker.start()
            self._imu_worker.start()
            self._force_worker.start()
            self._motor_worker.start_monitoring()
            self._panels["Vision"]._on_start()
            self._panels["Lidar"]._on_start()
            self._collecting = True
            self._btn_collect.setText("■  Stop All")
            self._btn_collect.setStyleSheet("background:#D32F2F; color:white; border-radius:8px;")
            self._lbl_status.setText("Acquisition requested — check each page for device status")
            self._set_panel_state("IMU", "State: RUNNING")
            self._set_panel_state("Force", "State: RUNNING")
            self._set_panel_state("Motor", "State: MONITORING")
            self._toast("Acquisition requested; Motor uses zero-output feedback", True)
        else:
            self._emg_worker.stop()
            self._imu_worker.stop()
            self._force_worker.stop()
            self._motor_worker.stop_monitoring()
            self._panels["Vision"]._on_stop()
            self._panels["Lidar"]._on_stop()
            self._collecting = False
            self._btn_collect.setText("▶  Collect All")
            self._btn_collect.setProperty("accent","true")
            self._btn_collect.setStyleSheet("")
            self._lbl_status.setText("Collection stopped")
            self._set_panel_state("IMU", "State: IDLE")
            self._set_panel_state("Force", "State: IDLE")
            self._set_panel_state("Motor", "State: IDLE")
            self._toast("All data collection stopped", True)

    def _set_panel_state(self, panel_name: str, text: str):
        panel = self._panels.get(panel_name)
        lbl = getattr(panel, "lbl_state", None)
        if lbl is not None:
            lbl.setText(text)

    def _toggle_recording(self):
        if not self._recording:
            if not self._dm.start_recording():
                return
            self._recording = True
            self._btn_rec.setText("⏹  Stop Rec")
            self._btn_rec.setStyleSheet("background:#D32F2F; color:white; border-radius:8px;")
            self._btn_rec.setToolTip(f"本次记录：{self._dm._record_path}\n停止后按实际有效数据命名。")
            self._toast(f"Recording 1000 Hz grid · 5×60 ms: {self._current_subject} @ {self._current_location}" if PIPELINE_ENABLED else "Recording legacy CSV", True)
        else:
            recording_ok = self._dm.stop_recording()
            self._recording = False
            self._btn_rec.setText("⏺  Record")
            self._btn_rec.setProperty("accent","true")
            self._btn_rec.setStyleSheet("")
            self._toast("Recording saved" if recording_ok else "Recording stopped with an error; check status", recording_ok)
            self._btn_rec.setToolTip(f"上次记录：{self._dm._record_path}")
            if recording_ok:
                self._lbl_status.setText(f"Saved: {self._dm._record_path}")

    @QtCore.pyqtSlot(str)
    def _on_record_error(self, message):
        self._recording = False
        with self._dm._lock:
            self._dm._recording = False
        self._btn_rec.setText("⚠ Record error")
        self._lbl_status.setText(message)
        self._toast(message, False, 10000)

    def _on_export(self):
        suggested = self._dm.suggested_export_path()
        fname, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export CSV", suggested, "CSV (*.csv);;All (*)")
        if not fname:
            return
        try:
            n = self._dm.export_snapshot_csv(fname)
            self._toast(f"Saved {n} rows → {os.path.basename(fname)}", True)
        except Exception as e:
            self._toast(f"Export failed: {e}", False, 3000)

    # ── Subject / session ────────────────────────────────────────────────────
    def _open_session_dialog(self):
        if self._emg_worker.config['source']=='windows' and self._emg_worker.state not in ('IDLE','READY','ERROR'):
            self._toast("Stop remote EMG before changing subject/session", False, 5000)
            return
        dlg = SessionDialog(self, self._session_mgr)
        if dlg.exec_() == QtWidgets.QDialog.Accepted:
            self._session_mgr.set_manual_path(dlg.manual_path)
            self._start_new_session(dlg.subject_id, dlg.location)

    def _start_new_session(self, subject_id: str, location: str, toast: bool = True):
        """
        Point all future recording/export at a fresh session directory for
        (subject_id, location). Auto mode creates a brand-new, collision-free
        timestamped folder every time this is called (including at app
        startup) — see SessionManager.resolve_session_dir /
        make_unique_session_dir. Manual mode reuses whatever folder the
        operator last picked.
        """
        emg = getattr(self,'_emg_worker',None)
        for name in ('_lidar_worker_l','_lidar_worker_r'):
            lidar = getattr(self,name,None)
            if lidar and lidar._record.is_set():
                self._toast("Turn off LiDAR Raw scans before changing subject/session", False, 5000)
                return
        if emg and emg.config['source']=='windows':
            if emg.state not in ('IDLE','READY','ERROR'):
                self._toast("Stop remote EMG before changing subject/session", False, 5000)
                return
            emg._submit('disconnect')
            emg.processor = None
            emg._set_state('IDLE')
        if self._recording or self._dm._recorder:
            if not self._dm.stop_recording():
                self._toast("Previous recording has not drained; session unchanged", False, 5000)
                return
            self._recording = False
            self._btn_rec.setText("⏺  Record")
            self._btn_rec.setStyleSheet("")
        session_dir = self._session_mgr.resolve_session_dir(subject_id, location)
        imu = getattr(self, '_imu_worker', None)
        if imu is None:  # Initial session is created before the workers.
            self._dm.set_export_dir(session_dir, subject_id=subject_id, location=location)
        else:
            with imu.reference.lock:
                self._dm.set_export_dir(session_dir, subject_id=subject_id, location=location)
                imu.reference.status(self._dm.session)
        self._session_mgr.record_use(subject_id, location)
        self._current_subject     = subject_id
        self._current_location    = location
        self._current_session_dir = session_dir
        self._update_session_label()
        if toast:
            self._toast(f"新会话：{subject_id} @ {location}", True)

    def _update_session_label(self):
        if not hasattr(self, "_lbl_profile"):
            return
        self._lbl_profile.setText(f"受试者 {self._current_subject}  @ {self._current_location}")
        self._lbl_profile.setToolTip(f"保存位置：{self._current_session_dir}")

    # ── Terrain -> control-mode auto-switching ──────────────────────────────
    @QtCore.pyqtSlot(bool)
    def _on_auto_terrain_mode_toggled(self, checked: bool):
        vision_panel = self._panels["Vision"]
        if checked and VISION_IMAGE_ONLY:
            self._terrain_switcher.disable()
            vision_panel.lbl_auto_status.setText("图像模式未运行识别，自动地形控制不可用")
            return
        if not checked:
            self._terrain_switcher.disable()
            vision_panel.lbl_auto_status.setText(
                "关闭 — 手动控制不受影响 (disabled — manual control unaffected)")
            vision_panel.lbl_auto_status.setStyleSheet("color:#888;")
            return

        reply = QtWidgets.QMessageBox.warning(
            self, "启用自动地形切换 / Enable auto terrain switching",
            "启用后，Motor 面板的控制参数 (MOTOR_PARAMS) 会根据摄像头识别的"
            "地形（flat / stairs_up / stairs_down）自动切换，切换前会要求"
            "连续、高置信度地观察到新地形一段时间（防抖动）。\n\n"
            "当前三种地形的预设参数都是占位的安全值（不产生助力），"
            "在真正标定好每种地形的控制增益之前启用不会有实际助力效果，"
            "但会开始覆盖你手动设置的 MOTOR_PARAMS。\n\n"
            "确定要启用吗？",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No)
        if reply != QtWidgets.QMessageBox.Yes:
            vision_panel.chk_auto_mode.blockSignals(True)
            vision_panel.chk_auto_mode.setChecked(False)
            vision_panel.chk_auto_mode.blockSignals(False)
            return

        self._terrain_switcher.enable()
        vision_panel.lbl_auto_status.setText(
            f"已启用 — 当前控制模式: {self._terrain_switcher.committed_mode or '(等待识别)'}")
        vision_panel.lbl_auto_status.setStyleSheet("color:#B71C1C; font-weight:700;")
        self._toast("自动地形切换已启用", True)

    @QtCore.pyqtSlot(str, str, float, dict)
    def _on_vision_terrain_update(self, raw_label, stable_label, confidence, probs):
        if not self._terrain_switcher.enabled:
            return
        ev = self._terrain_switcher.update(
            stable_label, confidence, camera_online=self._vision_worker.is_online())
        if ev is None:
            return
        motor_ids = [mid for _port, mid in MOTOR_DEVICES]
        self._terrain_switcher.apply_to_motor_params(MOTOR_PARAMS, motor_ids)
        vision_panel = self._panels.get("Vision")
        if vision_panel is not None:
            vision_panel.lbl_auto_status.setText(
                f"已启用 — 当前控制模式: {ev.to_mode}  (由 {ev.from_mode or '—'} 切换, "
                f"conf={ev.confidence:.2f})")
        self._toast(f"控制模式切换：{ev.from_mode or '—'} → {ev.to_mode}", True, 3000)

    # ── Field data collection ────────────────────────────────────────────
    def _on_capture_dataset_sample(self):
        vision_panel = self._panels["Vision"]
        label = vision_panel.cb_capture_label.currentText()
        # Collected under the CURRENT session's folder — ties each sample
        # to whichever subject/location was active when it was taken, same
        # as everything else DataManager/recording already does.
        output_root = os.path.join(self._current_session_dir, "vision_dataset")
        try:
            path = self._vision_worker.capture_dataset_sample(label, output_root)
        except Exception as e:
            vision_panel.lbl_capture_status.setText(f"采集失败 / capture failed: {e}")
            vision_panel.lbl_capture_status.setStyleSheet("color:#D32F2F;")
            self._toast(f"采集失败: {e}", False, 3000)
            return
        if path is None:
            vision_panel.lbl_capture_status.setText(
                "还没有相机画面，请先按 Start 启动摄像头 / no camera frame yet — press Start first")
            vision_panel.lbl_capture_status.setStyleSheet("color:#D32F2F;")
            return
        label_dir = os.path.join(output_root, label)
        try:
            n_total = len([f for f in os.listdir(label_dir) if f.startswith("depth_raw_")])
        except OSError:
            n_total = "?"
        vision_panel.lbl_capture_status.setText(
            f"已保存 {label}（{output_root} 下累计 {n_total} 个样本）\nsaved → {path}")
        vision_panel.lbl_capture_status.setStyleSheet("color:#2E7D32;")
        self._toast(f"已采集样本：{label} (第 {n_total} 个)", True, 1500)

    # ── Memory / status ───────────────────────────────────────────────────
    @QtCore.pyqtSlot(float)
    def _on_memory_warn(self, mb: float):
        color = "color:#D32F2F; font-weight:700;" if mb >= MEMORY_CRIT_MB else "color:#F57C00; font-weight:700;"
        self._lbl_mem.setStyleSheet(color)
        self._lbl_mem.setText(f"RAM {mb:.0f} MB")
        if mb >= MEMORY_CRIT_MB:
            self._toast(f"Memory critical ({mb:.0f} MB) — buffer trimmed", False, 4000)

    # ── Toast ─────────────────────────────────────────────────────────────
    def _toast(self, msg: str, ok: bool = True, msec: int = 2000):
        fg = "#2E7D32" if ok else "#B71C1C"
        self._toast_lbl.setStyleSheet(f"color:{fg}; font-weight:600;")
        self._toast_lbl.setText(msg)
        self._toast_lbl.setVisible(True)
        self._toast_timer.start(msec)
        if TOAST_BEEP:
            QtWidgets.QApplication.beep()

    # ── Cleanup ───────────────────────────────────────────────────────────
    def closeEvent(self, ev):
        # Stop producers first; then drain all already accepted recording rows.
        if not self._emg_worker.shutdown():
            self._toast("EMG is still stopping; close again after the SDK returns", False, 5000)
            ev.ignore()
            return
        try: self._motor_worker.shutdown()
        except Exception: pass
        try: self._imu_worker.shutdown()
        except Exception: pass
        try: self._force_worker.shutdown()
        except Exception: pass
        for worker in (self._vision_worker, self._lidar_worker_l, self._lidar_worker_r):
            worker.shutdown()
            if worker._thread and worker._thread.is_alive():
                self._toast("Sensor still stopping; close again after it returns", False, 5000)
                ev.ignore()
                return
        self._dm.stop_recording()
        if self._dm._recorder and self._dm._recorder._thread.is_alive():
            self._toast("CSV is still draining; close again when complete", False, 5000)
            ev.ignore()
            return
        self._session_mgr.save()
        super().closeEvent(ev)


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                              ENTRY POINT                                ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def main():
    # High-DPI attributes MUST be set before QApplication is created
    try:
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps)
    except Exception:
        pass

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("HiPExo Monitor")
    app.setOrganizationName("HiPExo Lab")

    # Compute responsive scale factor from primary screen geometry.
    # Reference: 1280×720. Clamped to [0.65, 2.0].
    # 1024×600 → 0.80   1280×720 → 1.00   1920×1080 → 1.33 (clamped 2.0 max)
    global _S
    _S = _compute_scale()

    apply_theme(app, 'light')
    win = MainWindow()
    if _PREVIEW:
        win.setWindowTitle("HiPExo Monitor — offline preview")
        win._lbl_status.setText("OFFLINE PREVIEW — EMG simulated; physical devices disabled")
        win._switch_panel("EMG")
        win._emg_worker.start()
    # showFullScreen() already called inside __init__; ensure it takes effect
    win.showFullScreen()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
