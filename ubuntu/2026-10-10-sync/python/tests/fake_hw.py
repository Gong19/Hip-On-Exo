"""
Fake hardware layer for hipexo_monitor.py testing.

Injects fake `smbus2`, `unitree_actuator_sdk`, `spidev` modules into
sys.modules *before* hipexo_monitor.py is imported, so its normal
`from smbus2 import SMBus` etc. import machinery picks up the fakes and
_SMBUS_OK / _SDK_OK / _SPIDEV_OK all come out True — exercising the exact
same code paths that would run on real Jetson hardware, without needing
real I2C/SPI/serial devices.

Also exposes a shared `State` object the test/bench script can mutate at
runtime to simulate a sensor unplug / replug event while the worker
threads are running.
"""
import sys
import time
import math
import types


class State:
    def __init__(self):
        self.imu_disconnected = set()      # {(bus, addr), ...}
        self.motor_disconnected = set()    # {port_str, ...}
        self.force_open_fail = False       # ADS8688 open() raises
        self.force_disconnected = False    # ADS8688 xfer2() raises


def install() -> State:
    state = State()

    # ── fake smbus2 ──────────────────────────────────────────────────
    smbus2_mod = types.ModuleType("smbus2")

    class FakeSMBus:
        def __init__(self, bus, force=False):
            self.bus = bus

        def read_byte(self, addr):
            if (self.bus, addr) in state.imu_disconnected:
                raise OSError(121, "Remote I/O error")
            return 0

        def read_i2c_block_data(self, addr, reg, length):
            if (self.bus, addr) in state.imu_disconnected:
                raise OSError(121, "Remote I/O error")
            t = time.time()
            base = int(1000 * math.sin(t * 3 + addr)) & 0xFFFF
            out = []
            for i in range(length // 2):
                v = (base + i * 37 + int(t * 100)) & 0xFFFF
                out.append(v & 0xFF)
                out.append((v >> 8) & 0xFF)
            while len(out) < length:
                out.append(0)
            return out[:length]

        def close(self):
            pass

    smbus2_mod.SMBus = FakeSMBus
    sys.modules["smbus2"] = smbus2_mod

    # ── fake unitree_actuator_sdk ────────────────────────────────────
    ua_mod = types.ModuleType("unitree_actuator_sdk")

    class MotorType:
        GO_M8010_6 = 0

    class MotorMode:
        FOC = 0

    def queryMotorMode(t, m):
        return 1

    def queryGearRatio(t):
        return 6.33

    class MotorCmd:
        def __init__(self):
            self.motorType = 0
            self.mode = 0
            self.id = 0
            self.q = 0.0
            self.dq = 0.0
            self.kp = 0.0
            self.kd = 0.0
            self.tau = 0.0

    class MotorData:
        def __init__(self):
            self.motorType = 0
            self.q = 0.0
            self.dq = 0.0
            self.temp = 25.0
            self.merror = 0

    class SerialPort:
        def __init__(self, port):
            self.port = port
            self._t0 = time.time()

        def sendRecv(self, cmd, data):
            if self.port in state.motor_disconnected:
                raise IOError(f"simulated link down on {self.port}")
            t = time.time() - self._t0
            data.q = 6.33 * (2 * math.pi * 0.5 * t)   # spinning rotor
            data.dq = 6.28
            data.temp = 30.0 + 2 * math.sin(t)
            data.merror = 0

    ua_mod.MotorType = MotorType
    ua_mod.MotorMode = MotorMode
    ua_mod.MotorCmd = MotorCmd
    ua_mod.MotorData = MotorData
    ua_mod.SerialPort = SerialPort
    ua_mod.queryMotorMode = queryMotorMode
    ua_mod.queryGearRatio = queryGearRatio
    sys.modules["unitree_actuator_sdk"] = ua_mod

    # ── fake spidev ──────────────────────────────────────────────────
    spidev_mod = types.ModuleType("spidev")

    class SpiDev:
        def __init__(self):
            self.max_speed_hz = 0
            self.mode = 0
            self.bits_per_word = 8

        def open(self, bus, device):
            if state.force_open_fail:
                raise IOError("simulated SPI open failure")
            self.bus = bus
            self.device = device

        def xfer2(self, data):
            if state.force_disconnected:
                raise IOError("simulated SPI xfer failure")
            if data[0] == 0x85 or (data[0] & 0b11000000) == 0b11000000:
                return [0, 0, 0, 0]
            val = int(32768 + 5000 * math.sin(time.time())) & 0xFFFF
            return [0, 0, (val >> 8) & 0xFF, val & 0xFF]

        def close(self):
            pass

    spidev_mod.SpiDev = SpiDev
    sys.modules["spidev"] = spidev_mod

    return state
