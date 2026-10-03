"""Device adapters. The bridge actor is the only caller of these objects."""
import ast
import ctypes
import math
import os
from pathlib import Path
import sys
import threading
import time
import warnings

SIDS = [57614, 57569, 57566, 57589, 57586, 57643, 56683]
MUSCLES = ['ECRL', 'ED', 'ECU', 'FCU', 'FCR', 'PT', 'SP']
PROJECT = Path(r'C:\Users\YOUR_WINDOWS_USER\OneDrive\Desktop\trigno_code\Example-Applications-main\Example-Applications-main\Python')


class Clock:
    def __init__(self):
        self.offset_ns = 0  # simulation/fault injection only
        self.api = None
        if sys.platform == 'win32':
            self.api = ctypes.WinDLL('kernel32', use_last_error=True).GetSystemTimePreciseAsFileTime
            self.api.argtypes = [ctypes.POINTER(ctypes.c_ulonglong)]
            self.api.restype = None
        self.method = 'GetSystemTimePreciseAsFileTime' if self.api else 'time.time_ns'

    def wall_ns(self):
        if self.api:
            value = ctypes.c_ulonglong()
            self.api(ctypes.byref(value))
            return (value.value - 116444736000000000) * 100 + self.offset_ns
        return time.time_ns() + self.offset_ns

    def stamps(self):
        before = time.perf_counter_ns()
        wall = self.wall_ns()
        after = time.perf_counter_ns()
        return {'wall_ns': wall, 'mono_ns': (before + after)//2,
                'read_bracket_ns': after-before}


def default_channels():
    # Non-streaming / absent slot metadata; never a claimed measured rate.
    return [dict(sid=s, present=False, is_rms=False, mode='not_configured',
                 sample_rate=1000., sample_rate_source='nominal_missing_slot_only',
                 battery=None, muscle=MUSCLES[i], guid=None) for i, s in enumerate(SIDS)]


def read_credentials(path):
    path = Path(path)
    if path.suffix.lower() == '.json':
        import json
        values = json.loads(path.read_text(encoding='utf-8-sig'))
    else:
        values = {}
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', SyntaxWarning)
            tree = ast.parse(path.read_text(encoding='utf-8-sig'))
        for node in tree.body:
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if isinstance(target, ast.Name) and target.id in ('key', 'license'):
                        values[target.id] = ast.literal_eval(node.value)
    if not all(isinstance(values.get(k), str) and values[k].strip() for k in ('key', 'license')):
        raise ValueError('Existing credential file has no nonempty literal key/license')
    return values['key'], values['license']


def voltage_scale_to_v(unit):
    """Use the SDK channel Unit enum, never infer scaling from signal amplitude."""
    normalized=str(unit).strip().lower()
    scales={'volts':1.,'volt':1.,'v':1.,'millivolts':1e-3,'millivolt':1e-3,
            'mv':1e-3,'microvolts':1e-6,'microvolt':1e-6,'uv':1e-6,'µv':1e-6}
    if normalized not in scales:
        raise ValueError('EMG channel Unit is unknown; voltage conversion must be verified before acquisition')
    return scales[normalized]


class DelsysSource:
    simulated = False

    def __init__(self, clock, project=PROJECT):
        self.clock, self.project = clock, Path(project)
        self.api = None
        self.owner = None
        self.started = False
        self.guids = [None]*7
        self.channels = default_channels()
        self.dll_directory = None
        self.scanned_ids = []
        self.device_mutex = None
        self.mutex_api = None

    def _owned(self):
        ident = threading.get_ident()
        if self.owner is None:
            self.owner = ident
        if ident != self.owner:
            raise RuntimeError('Delsys SDK accessed outside its single acquisition thread')

    def prepare(self):
        self._owned()
        self.close()
        if sys.platform == 'win32':
            self.mutex_api=ctypes.WinDLL('kernel32',use_last_error=True)
            self.mutex_api.CreateMutexW.argtypes=[ctypes.c_void_p,ctypes.c_int,ctypes.c_wchar_p]
            self.mutex_api.CreateMutexW.restype=ctypes.c_void_p
            self.mutex_api.CloseHandle.argtypes=[ctypes.c_void_p]
            handle=self.mutex_api.CreateMutexW(None,0,'Local\\HiPExoRealDelsysBridge')
            if not handle:raise RuntimeError('Cannot acquire Delsys bridge instance guard')
            if ctypes.get_last_error()==183:
                self.mutex_api.CloseHandle(handle)
                raise RuntimeError('Another real EMG bridge is open; close it before continuing')
            self.device_mutex=handle
        resources = self.project/'resources'
        key, license_value = read_credentials(self.project/'AeroPy'/'TrignoBase.py')
        if not (resources/'DelsysAPI.dll').is_file():
            raise FileNotFoundError('DelsysAPI.dll missing in existing project/resources')
        try:
            if 'clr' not in sys.modules:
                from pythonnet import load
                load('coreclr')
            import clr
            if str(resources) not in sys.path:
                sys.path.insert(0, str(resources))
            if hasattr(os, 'add_dll_directory') and self.dll_directory is None:
                self.dll_directory = os.add_dll_directory(str(resources))
            clr.AddReference(str(resources/'DelsysAPI.dll'))
            clr.AddReference('System.Collections')
            from Aero import AeroPy
            self.api = AeroPy()
            self.api.ValidateBase(key, license_value)
        except Exception as exc:
            # SDK errors can include license material: don't propagate their text.
            raise RuntimeError('Delsys runtime/base validation failed: '+type(exc).__name__) from None
        finally:
            key = license_value = None
        try:
            task = self.api.ScanSensors()
            deadline = time.monotonic()+25
            while not task.Wait(100):
                if time.monotonic() > deadline:
                    raise TimeoutError('Sensor scan timed out')
            sensors = list(self.api.GetScannedSensorsFound())
            self.scanned_ids = [int(s.Properties.Sid) for s in sensors]
            selected = []
            for index, sensor in enumerate(sensors):
                sid = int(sensor.Properties.Sid)
                if sid in SIDS:
                    if sid in [s for _, s in selected]:
                        raise ValueError('Duplicate registered sensor ID')
                    self.api.SelectSensor(index)
                    selected.append((index, sid))
            if not selected:
                raise ValueError('No registered sensor found. Wake sensors and scan again.')
            # Keep the actual modes established by the user's working program.
            self.api.Configure(False, False)
            if not self.api.IsPipelineConfigured():
                raise RuntimeError('Pipeline configuration failed')
            previous = self.channels
            self.channels = default_channels()
            self.guids = [None]*7
            for i, old in enumerate(previous):
                if old.get('sample_rate_source') == 'SDK':
                    self.channels[i].update(mode=old['mode'], sample_rate=old['sample_rate'],
                                            is_rms=old['is_rms'], sample_rate_source='last_observed_configuration')
            for index, sid in selected:
                sensor = self.api.GetSensorObject(index)
                emgs = [c for c in sensor.TrignoChannels if c.IsEnabled and
                        (str(c.Type).upper() == 'EMG' or 'emg' in str(c.Name).lower())]
                if len(emgs) != 1:
                    raise ValueError('Registered sensor must have exactly one enabled EMG channel')
                ch = emgs[0]
                sdk_unit = str(ch.Unit)
                scale_to_v = voltage_scale_to_v(sdk_unit)
                mode = str(sensor.Configuration.ModeString)
                rate = float(ch.SampleRate)
                if not math.isfinite(rate) or rate <= 0:
                    raise ValueError('SDK returned invalid sample rate')
                slot = SIDS.index(sid)
                self.guids[slot] = ch.Id
                try:
                    battery = float(sensor.Properties.BatteryPercent)*100
                    if not math.isfinite(battery): battery = None
                except Exception:
                    battery = None
                self.channels[slot].update(present=True, mode=mode, sample_rate=rate,
                    sample_rate_source='SDK', is_rms='RMS' in (mode+' '+str(ch.Name)).upper(),
                    battery=battery, guid=str(ch.Id),sdk_unit=sdk_unit,
                    scale_to_v=scale_to_v,value_units='V',
                    unit_evidence='SDK ChannelTrigno.Unit; physical calibration not performed')
            return self.channels
        except (ValueError, TimeoutError):
            raise
        except Exception as exc:
            raise RuntimeError('Delsys scan/configuration failed: '+type(exc).__name__) from None

    def start(self):
        self._owned()
        self.started = True  # stop is required even for a partial Start failure
        try:
            self.api.Start(True)
        except Exception as exc:
            raise RuntimeError('Delsys Start failed: '+type(exc).__name__) from None

    def poll(self):
        self._owned()
        try:
            if not self.api.CheckYTDataQueue(): return None
            before = self.clock.stamps()
            raw = self.api.PollYTData()
            received = self.clock.stamps()
            channels = []
            for slot, guid in enumerate(self.guids):
                if guid is None or not raw.ContainsKey(guid): continue
                times, values = [], []
                for sample in raw[guid]:
                    times.append(float(sample.Item1))
                    values.append(float(sample.Item2))
                if times:
                    scale = self.channels[slot]['scale_to_v']
                    channels.append({'slot':slot, 'device_time_s':times, 'values_v':values,
                                     'sid':SIDS[slot], 'guid':str(guid),
                                     'sdk_values':values[:], 'sdk_unit':self.channels[slot]['sdk_unit'],
                                     'scale_to_v':scale})
                    channels[-1]['values_v']=[v*scale for v in values]
            return {'poll_start':before, 'poll_received':received, 'channels':channels}
        except Exception as exc:
            raise RuntimeError('Delsys Poll failed; run incomplete: '+type(exc).__name__) from None

    def stop(self):
        self._owned()
        if self.api is not None and self.started:
            try:
                self.api.Stop()
                self.started = False
            except Exception as exc:
                raise RuntimeError('Delsys Stop failed: '+type(exc).__name__) from None

    def close(self):
        self._owned()
        self.stop()
        if self.api is not None:
            try:
                if str(self.api.GetPipelineState()) == 'Armed': self.api.ResetPipeline()
            except Exception as exc:
                raise RuntimeError('Delsys reset failed: '+type(exc).__name__) from None
            self.api = None
        if self.device_mutex is not None:
            self.mutex_api.CloseHandle(self.device_mutex);self.device_mutex=None


class SyntheticSource:
    """Explicitly simulated source with a final tail to exercise STOP drainage."""
    simulated = True

    def __init__(self, clock, missing=(), rate=1000.):
        self.clock, self.rate, self.missing = clock, rate, set(missing)
        self.started = False
        self.tail = 0
        self.count = 0
        self.channels = default_channels()
        self.scanned_ids = [s for i,s in enumerate(SIDS) if i not in self.missing]
        self.owner = None

    def _owned(self):
        if self.owner is None: self.owner = threading.get_ident()
        assert threading.get_ident() == self.owner, 'Synthetic SDK ownership violated'

    def prepare(self):
        self._owned()
        for i,ch in enumerate(self.channels):
            ch.update(present=i not in self.missing, is_rms=True, mode='SIMULATED RMS',
                      sample_rate=self.rate, sample_rate_source='simulation', guid='sim-'+str(i))
        return self.channels

    def start(self):
        self._owned(); self.started = True; self.count = self.tail = 0
        self.begin = time.perf_counter()

    def poll(self):
        self._owned()
        if not self.started and not self.tail: return None
        n = min(20, max(0,int((time.perf_counter()-self.begin)*self.rate)-self.count)) if self.started else self.tail
        if n <= 0: return None
        before=self.clock.stamps()
        times=[j/self.rate for j in range(self.count,self.count+n)]
        frames=[{'slot':i,'sid':SIDS[i],'guid':'sim-'+str(i), 'device_time_s':times[:],
                 'values_v':[.001+.0002*math.sin(t*6+i) for t in times]} for i in range(7) if i not in self.missing]
        self.count += n
        if not self.started: self.tail=0
        return {'poll_start':before,'poll_received':self.clock.stamps(),'channels':frames}

    def stop(self):
        self._owned()
        if self.started: self.started=False; self.tail=3

    def close(self):
        self._owned(); self.started=False; self.tail=0
