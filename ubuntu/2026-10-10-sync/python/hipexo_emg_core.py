"""Seven-slot EMG envelope processing adapted from the supplied EMG program.

YT times are device seconds; values are volts. This module does not control motors.
Output uses a 1000 Hz grid and the original order-2 Bessel 5 Hz low-pass.
Calibration always observes the envelope BEFORE calibration/normalization.
"""
from collections import deque
import json
import math
import time
import numpy as np
from scipy import signal

MUSCLES = ('ECRL', 'ED', 'ECU', 'FCU', 'FCR', 'PT', 'SP')
SENSOR_IDS = (57614, 57569, 57566, 57589, 57586, 57643, 56683)
RATE = 1000


class EmgProcessor:
    def __init__(self, channels, source_kind='delsys'):
        if len(channels) != 7:
            raise ValueError('Exactly seven logical channel slots are required')
        self.channels = channels
        self.fingerprint = {'source': source_kind, 'channels': [
            {k: ch.get(k) for k in ('sid', 'mode', 'sample_rate', 'present', 'sdk_unit', 'scale_to_v', 'value_units')}
            for ch in channels]}
        self.b, self.a = signal.bessel(2, 5, fs=RATE, btype='low')
        self.baseline = np.zeros(7)
        self.mvc = np.zeros((7, 3))
        self.calibrated_baseline = False
        self.job = None
        self.last_calibration_message = ''
        self.filter_description = "6th-order Butterworth 400Hz anti-alias before downsampling when native Fs>1000; rectified branch filtered separately; 2nd-order Bessel 5Hz envelope at 1000Hz; causal phase delay not removed"
        self.fingerprint['filter_version'] = 'anti-alias-v1'
        self.reset_stream()

    def reset_stream(self):
        self.origin = None
        self.tails = [None] * 7
        self._aa_state = [None]*7
        self._aa_last = [None]*7
        self.zi = [None] * 7
        self.next_ms = [0] * 7
        self.watermarks = [-1] * 7
        self.last_seen = [None] * 7
        self.pending = {}
        self.last_emitted = -1
        self.rejected = [0] * 7
        self.discontinuities = [0] * 7
        self.job = None

    def _prefilter(self,ch,t,v):
        fs=float(self.channels[ch].get('sample_rate',RATE))
        rect=v.copy() if self.channels[ch]['is_rms'] else np.abs(v)
        if fs<=RATE:return v,rect
        sos=signal.butter(6,400,fs=fs,output='sos')
        out=np.empty_like(v);envelope=np.empty_like(rect)
        boundaries=np.r_[0,np.flatnonzero(np.diff(t)>50)+1,len(t)]
        state=self._aa_state[ch]
        if self._aa_last[ch] is None or t[0]-self._aa_last[ch]>50:state=None
        for a,b in zip(boundaries[:-1],boundaries[1:]):
            if a:state=None
            if state is None:state=(signal.sosfilt_zi(sos)*v[a],signal.sosfilt_zi(sos)*rect[a])
            out[a:b],z1=signal.sosfilt(sos,v[a:b],zi=state[0])
            envelope[a:b],z2=signal.sosfilt(sos,rect[a:b],zi=state[1]);state=(z1,z2)
        self._aa_state[ch]=state;self._aa_last[ch]=t[-1]
        return out,envelope

    def ingest(self, frames, now=None):
        now = time.perf_counter() if now is None else now
        clean = {}
        for ch, (timestamps, values) in frames.items():
            t, v = np.asarray(timestamps, dtype=float), np.asarray(values, dtype=float)
            if t.ndim != 1 or v.ndim != 1 or len(t) != len(v):
                raise ValueError('YT data/time lengths do not match')
            mask = np.isfinite(t) & np.isfinite(v) & (np.abs(v) <= 10.0)
            self.rejected[ch] += int(len(t) - mask.sum())
            t, v = t[mask], v[mask]
            order = np.argsort(t, kind='stable')
            t, v = t[order], v[order]
            if len(t):
                keep = np.r_[np.diff(t) > 0, True]
                t, v = t[keep], v[keep]
                clean[ch] = (t, v)
        if self.origin is None:
            if not clean:
                return []
            self.origin = min(t[0] for t, _ in clean.values())
        for ch, (t, v) in clean.items():
            t = (t - self.origin) * RATE
            tail = self.tails[ch]
            if tail is not None:
                # A real device clock reset requires a new session; never splice epochs.
                if t[-1] < tail[0] - 100:
                    raise ValueError('EMG device clock moved backwards; stop and reconnect')
                keep = t > tail[0] + 1e-8
                self.rejected[ch] += int(len(t) - keep.sum())
                t, v = t[keep], v[keep]
                if not len(t):
                    continue
            v, rect = self._prefilter(ch,t,v)
            if tail is not None:
                t, v, rect = np.r_[tail[0],t],np.r_[tail[1],v],np.r_[tail[2],rect]
            self.last_seen[ch] = now
            # Process each continuous segment separately, including segments before a gap.
            breaks = np.r_[0, np.flatnonzero(np.diff(t) > 50.0) + 1, len(t)]
            for seg in range(len(breaks) - 1):
                start, end = breaks[seg:seg+2]
                st, sv = t[start:end], v[start:end]
                if seg:
                    self.zi[ch] = None
                    self.discontinuities[ch] += 1
                if len(st) < 2:
                    continue
                lo = max(self.next_ms[ch], self.last_emitted + 1, math.ceil(st[0] - 1e-8))
                hi = math.floor(st[-1] + 1e-8)
                if hi < lo:
                    continue
                if hi - lo > 10000:
                    raise ValueError('EMG backlog exceeds 10 seconds; reconnect')
                labels = np.arange(lo, hi + 1, dtype=np.int64)
                raw = np.interp(labels, st, sv)
                source_values = rect[start:end]
                grid = np.interp(labels, st, source_values)
                zi = self.zi[ch]
                if zi is None:
                    zi = signal.lfilter_zi(self.b, self.a) * grid[0]
                envelope, self.zi[ch] = signal.lfilter(self.b, self.a, grid, zi=zi)
                envelope = np.maximum(envelope, 0)
                for label, r, e in zip(labels, raw, envelope):
                    row = self.pending.setdefault(int(label), {'seen': now, 'values': {}})
                    row['values'][ch] = (float(r), float(e))
                self.next_ms[ch] = hi + 1
                self.watermarks[ch] = hi
            self.tails[ch] = (float(t[-1]), float(v[-1]),float(rect[-1]))
        if len(self.pending) > 12000:
            raise ValueError('EMG alignment buffer overflow')
        return self.drain(now)

    def drain(self, now=None, force=False):
        now = time.perf_counter() if now is None else now
        active = [i for i, c in enumerate(self.channels) if c['present']]
        rows = []
        for label in sorted(self.pending):
            entry = self.pending[label]
            unresolved = [i for i in active if i not in entry['values'] and self.watermarks[i] < label]
            if unresolved and not force and now - entry['seen'] < .25:
                break
            raw, envelope, valid = np.zeros(7), np.zeros(7), np.zeros(7, dtype=bool)
            for i, pair in entry['values'].items():
                raw[i], envelope[i] = pair
                valid[i] = True
            self._collect_calibration(envelope, valid, now)
            corrected = np.maximum(envelope - self.baseline, 0)
            corrected[~valid] = 0
            denominator = self.mvc.max(axis=1)
            ratio = np.full(7, np.nan)
            calibrated = (denominator > 1e-9) & valid
            ratio[calibrated] = np.clip(corrected[calibrated] / denominator[calibrated], 0, 1)
            rows.append({'label_ms': label, 'raw_v': raw, 'uncalibrated_v': envelope,
                         'envelope_v': corrected, 'mvc_ratio': ratio, 'valid': valid,
                         'baseline_v': self.baseline.copy(), 'mvc_v': denominator.copy()})
            self.last_emitted = label
            del self.pending[label]
        self.finish_calibration(now)
        return rows

    def begin_calibration(self, kind, dof=0, duration=5., now=None):
        if self.job:
            raise ValueError('A calibration is already running')
        if kind not in ('baseline', 'mvc') or dof not in (0, 1, 2):
            raise ValueError('Invalid calibration request')
        if duration <= 0:
            raise ValueError('Calibration duration must be positive')
        now = time.perf_counter() if now is None else now
        self.job = {'kind': kind, 'dof': dof, 'start': now, 'end': now + duration,
                    'duration': duration, 'samples': [[] for _ in range(7)],
                    'after_label': max(self.pending, default=self.last_emitted)}
        self.last_calibration_message = f'{kind.upper()}: collecting {duration:g} s of fresh samples'

    def _collect_calibration(self, envelope, valid, now):
        job = self.job
        if not job or now > job['end']:
            return
        # Rows which were pending at the click are excluded, as are invalid slots.
        if self.last_emitted < job['after_label']:
            return
        for i in range(7):
            if valid[i]:
                value = envelope[i]
                if job['kind'] == 'mvc':
                    value = max(0., value - self.baseline[i])
                job['samples'][i].append(float(value))

    def finish_calibration(self, now=None):
        now = time.perf_counter() if now is None else now
        job = self.job
        if not job or now < job['end']:
            return
        self.job = None
        active = [i for i, c in enumerate(self.channels) if c['present']]
        minimum = int(job['duration'] * RATE * .8)
        if not active or any(len(job['samples'][i]) < minimum for i in active):
            self.last_calibration_message = 'Calibration rejected: insufficient fresh coverage; previous values retained'
            return
        values = np.zeros(7)
        for i in active:
            values[i] = (np.mean(job['samples'][i]) if job['kind'] == 'baseline'
                         else np.percentile(job['samples'][i], 95))
        if job['kind'] == 'mvc' and any(values[i] <= 1e-9 for i in active):
            self.last_calibration_message = 'MVC rejected: no usable contraction on one or more channels'
            return
        if job['kind'] == 'baseline':
            self.baseline = values
            self.calibrated_baseline = True
            self.mvc[:] = 0  # old MVC was measured with a different baseline
            self.last_calibration_message = 'Baseline applied; MVC cleared — collect MVC again'
        else:
            self.mvc[:, job['dof']] = values
            self.last_calibration_message = f'MVC movement {job["dof"] + 1} applied (95th percentile)'

    def calibration_dict(self):
        return {'schema': 1, 'units': 'V', 'fingerprint': self.fingerprint,
                'baseline_v': self.baseline.tolist(), 'mvc_v': self.mvc.tolist(),
                'baseline_measured': self.calibrated_baseline,
                'mvc_statistic': '95th_percentile', 'output_rate_hz': RATE,
                'filter': 'order2_bessel_5Hz_phase_normalized'}

    def load_calibration(self, record):
        if self.job:
            raise ValueError('Finish or cancel calibration first')
        if record.get('schema') != 1 or record.get('units') != 'V':
            raise ValueError('Unsupported calibration file or units')
        if record.get('fingerprint') != self.fingerprint:
            raise ValueError('Calibration sensor IDs, modes or source do not match this connection')
        b = np.asarray(record.get('baseline_v'), dtype=float)
        m = np.asarray(record.get('mvc_v'), dtype=float)
        if b.shape != (7,) or m.shape != (7, 3) or not np.isfinite(b).all() or not np.isfinite(m).all():
            raise ValueError('Invalid calibration dimensions or non-finite values')
        if (b < 0).any() or (m < 0).any():
            raise ValueError('Calibration values cannot be negative')
        self.baseline, self.mvc = b.copy(), m.copy()
        self.calibrated_baseline = bool(record.get('baseline_measured'))
        self.last_calibration_message = 'Matching baseline and MVC loaded together'
