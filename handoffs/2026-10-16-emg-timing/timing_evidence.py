"""Host event timing only. No claim of hardware sample UTC or physical disk persistence."""
import copy
import math
import threading
from control_protocol import stamp


def milliseconds(later, earlier, offset_ns=0, same_host=False):
    if not isinstance(later, dict) or not isinstance(earlier, dict):
        return None
    key = 'mono_ns' if same_host else 'wall_ns'
    if type(later.get(key)) is not int or type(earlier.get(key)) is not int:
        return None
    return (later[key]-earlier[key]-offset_ns)/1e6


def summarize(run):
    events = run.get('timing_events', {})
    sync = run.get('clock_sync') or {}
    offset = sync.get('offset_ns')
    metrics = {}
    if type(offset) is int:
        click = run.get('ubuntu_request')
        pulse = run.get('gui_sync_pulse')
        for name, event in [('request_to_windows_receive_ms', run.get('command_received')),
                            ('request_to_gui_pulse_ms', pulse),
                            ('request_to_gate_release_ms', events.get('gate_released')),
                            ('request_to_first_frame_seen_ms', events.get('first_saving_frame')),
                            ('request_to_start_returned_ms', run.get('gui_start_returned')),
                            ('request_to_first_csv_write_ms', events.get('first_csv_write_returned'))]:
            metrics[name] = milliseconds(event, click, offset)
        metrics['collect_all_to_gui_pulse_ms'] = milliseconds(pulse, run.get('ubuntu_collect_all'), offset)
        metrics['send_to_windows_receive_ms'] = milliseconds(run.get('command_received'), run.get('client_send'), offset)
        receipt = run.get('start_client_receipt') or {}
        # Ubuntu event - (Windows event - Windows_minus_Ubuntu offset).
        metrics['gui_pulse_to_ubuntu_ack_receive_ms'] = milliseconds(receipt.get('received'), pulse, -offset)
        metrics['request_to_ubuntu_ack_receive_ms'] = milliseconds(receipt.get('received'), click, same_host=True)
        stop_sync = run.get('stop_clock_sync') or sync
        stop_offset = stop_sync.get('offset_ns')
        if type(stop_offset) is int:
            for name, event in [('stop_click_to_windows_receive_ms', run.get('stop_received')),
                                ('stop_click_to_sdk_stop_returned_ms', events.get('sdk_stop_returned')),
                                ('stop_click_to_save_complete_ms', run.get('stop_returned'))]:
                metrics[name] = milliseconds(event, run.get('ubuntu_stop_request'), stop_offset)
        metrics['offset_estimate_ms'] = offset/1e6
        rtt = sync.get('rtt_ns')
        metrics['minimum_rtt_ms'] = rtt/1e6 if type(rtt) is int else None
        metrics['rtt_half_reference_ms'] = rtt/2e6 if type(rtt) is int else None
    metrics['windows_receive_to_gui_begin_ms'] = milliseconds(run.get('gui_start_begin'), run.get('command_received'), same_host=True)
    metrics['windows_receive_to_stop_complete_ms'] = milliseconds(run.get('stop_returned'), run.get('stop_received'), same_host=True)
    metrics['csv_finalization_ms'] = milliseconds(events.get('file_save_stop_returned'), events.get('file_save_stop_begin'), same_host=True)
    return metrics


class TimingProbe:
    def __init__(self):
        self.lock = threading.Lock()
        self.active = False
        self.events = {}
        self.version = 0

    def begin(self):
        with self.lock:
            self.events = {}
            self.active = True
            self.version += 1

    def event(self, name, first=False, value=None):
        if not self.active:
            return
        with self.lock:
            if first and name in self.events:
                return
            self.events[name] = value or stamp()
            self.version += 1

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.events)


def install_probe(namespace):
    probe = TimingProbe()
    namespace['_emg_timing_probe'] = probe
    plot = namespace['PlottingManager']
    original_gate = plot.release_sync_transition

    def gate(manager):
        mono = original_gate(manager)
        observed = stamp()
        if type(mono) is int:
            observed['wall_ns'] += mono-observed['mono_ns']
            observed['mono_ns'] = mono
            observed['wall_mapping'] = 'nearby_host_clock_projection'
        probe.event('gate_released', value=observed)
        return mono
    plot.release_sync_transition = gate

    def instrument(cls, method, prefix):
        original = getattr(cls, method)
        def wrapper(obj, *args, **kwargs):
            probe.event(prefix+'_begin')
            result = original(obj, *args, **kwargs)
            probe.event(prefix+'_returned')
            return result
        setattr(cls, method, wrapper)
    instrument(plot, 'stop_streaming', 'processing_stop')
    instrument(namespace['TrignoBase'], 'Stop', 'sdk_stop')
    instrument(namespace['FileSaveManager'], 'stop', 'file_save_stop')
    saver = namespace['FileSaveManager']
    original_write = saver._save_matrix_batches_to_csv
    def write(obj, file_path, matrix_batches, *args, **kwargs):
        has_data = bool(matrix_batches) and bool(namespace['ENABLE_FILE_SAVING'])
        if has_data:
            probe.event('first_csv_write_begin', first=True)
        result = original_write(obj, file_path, matrix_batches, *args, **kwargs)
        if result and has_data:
            probe.event('first_csv_write_returned', first=True)
            probe.event('last_csv_write_returned')
        return result
    saver._save_matrix_batches_to_csv = write

    processor = namespace['EMGPreProcessor']
    original_frame = processor.process_data_frame
    def frame(obj, data_frame, time_frame, for_display_only=False, frame_meta=None):
        if probe.active and not for_display_only and 'first_saving_frame' not in probe.events:
            if data_frame and any(len(values) for values in data_frame):
                value = stamp()
                device_times = [float(t) for channel in time_frame for t in channel if math.isfinite(float(t))]
                value['device_first_s'] = min(device_times) if device_times else None
                value['device_last_s'] = max(device_times) if device_times else None
                value['meaning'] = 'First nonempty frame entering host saving-mode processor, not physical sample time'
                if isinstance(frame_meta, dict):
                    value['host_ready_perf_ns'] = frame_meta.get('host_ready_perf_ns')
                probe.event('first_saving_frame', first=True, value=value)
        return original_frame(obj, data_frame, time_frame, for_display_only, frame_meta)
    processor.process_data_frame = frame
    return probe
