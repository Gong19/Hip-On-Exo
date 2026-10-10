"""Append-only session/file associations; no sensor or network dependencies."""
import json
from pathlib import Path
import threading
import time
import uuid
from hipexo_recording_layout import context_stem, component


class SessionManifest:
    def __init__(self, directory, **metadata):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.path = self.directory / 'session_manifest.jsonl'
        self.session_id = uuid.uuid4().hex
        self.metadata = dict(metadata)
        self._events = []
        self._lock = threading.Lock()
        self.event('session_start', schema='hipexo-session/1', **metadata)

    def event(self, kind, **fields):
        event = dict(fields, kind=kind, session_id=self.session_id,
                     host_wall_ns=time.time_ns(), host_mono_ns=time.perf_counter_ns())
        line = json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n'
        with self._lock, self.path.open('a', encoding='utf-8') as stream:
            stream.write(line)
            self._events.append(event)

    def artifact(self, modality, path, **metadata):
        self.event('artifact', modality=modality, path=str(Path(path).resolve()), **metadata)

    def file_stem(self, modality):
        return context_stem(self.metadata.get('subject_id'),self.metadata.get('location'))+'__'+component(modality)

    def raw_references(self):
        with self._lock:
            return [dict(e) for e in self._events if e['kind']=='artifact' and
                    e.get('modality') in ('emg_remote_raw','emg_run_timing','lidar_raw','camera_sample','imu_reference')]


def camera_frame_timing(frame, received_wall_ns, received_mono_ns, stream_id):
    """Device timestamps retain their native domain; they are not assumed UTC."""
    result = dict(camera_stream_id=stream_id,
                  host_frame_received_wall_ns=received_wall_ns,
                  host_frame_received_mono_ns=received_mono_ns,
                  timestamp_semantics='host_frame_received',
                  device_frame_number=None, device_timestamp_ms=None,
                  device_timestamp_domain=None)
    for key, name in [('device_frame_number', 'get_frame_number'),
                      ('device_timestamp_ms', 'get_timestamp'),
                      ('device_timestamp_domain', 'get_frame_timestamp_domain')]:
        try:
            value = getattr(frame, name)()
            result[key] = str(value) if key.endswith('domain') else value
        except (RuntimeError, AttributeError):
            pass
    return result
