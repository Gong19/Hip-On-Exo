"""Human-readable recording names, based on successfully written samples."""
from datetime import datetime
import json
from pathlib import Path
import uuid
import time

SENSOR_NAMES = {'motor':'Motor', 'imu':'IMU', 'force':'Force',
                'emg':'EMG', 'vision':'Camera', 'lidar':'Lidar'}
SENSOR_ORDER = tuple(SENSOR_NAMES.values())


def component(value):
    value = ''.join(c if c.isalnum() or c in '-_' else '_' for c in str(value or 'unknown'))
    result = ''
    for c in value.strip('_'):
        if len((result+c).encode('utf-8')) > 48:
            break
        result += c
    return result or 'unknown'


def context_stem(subject_id, location, when=None):
    when = when or datetime.now().astimezone()
    return f"{when:%Y%m%d_%H%M%S_%f}__{component(subject_id)}__{component(location)}"


def sensor_name(stream):
    return SENSOR_NAMES.get(stream.split('_')[0].lower(),component(stream))


def sensors_in(stats):
    found = {sensor_name(stream) for stream, item in stats.items()
             if item.get('valid_samples',0)>0}
    return [name for name in SENSOR_ORDER if name in found] + sorted(found-set(SENSOR_ORDER))


def batch_summary(stream, times, frames):
    """One compact descriptor per queue item; no second CSV parsing pass."""
    valid = sum(frame.get('valid',0)==1 for frame in frames) if stream.startswith('emg_') else len(frames)
    return dict(stream=stream,samples=len(frames),valid_samples=int(valid),
                first_t_ms=float(min(times)),last_t_ms=float(max(times)),
                fields=list(frames[0]),
                simulated=any(frame.get('simulated',0)==1 or frame.get('remote_simulated',0)==1 for frame in frames))


class RecordingBundle:
    def __init__(self, parent, metadata, session_id):
        self.started = datetime.now().astimezone()
        self.started_mono = time.perf_counter()
        self.stopped = None
        self.duration = None
        self.metadata = dict(metadata)
        self.session_id = session_id
        self.recording_id = uuid.uuid4().hex
        self.stem = context_stem(metadata.get('subject_id'),metadata.get('location'),self.started)
        self.stem += '__'+self.recording_id[:6]
        self.directory = Path(parent)/(self.stem+'__Recording')
        self.directory.mkdir(exist_ok=False)
        self.csv_path = self.directory/(self.stem+'__Recording__combined.csv')

    def mark_stopping(self):
        if self.stopped is None:
            self.stopped = datetime.now().astimezone()
            self.duration = time.perf_counter()-self.started_mono

    def finalize(self, stats, complete, error, references):
        sensors = sensors_in(stats)
        suffix = '-'.join(sensors) or 'NoValidData'
        if not complete:
            suffix += '__INCOMPLETE'
        name = self.stem+'__'+suffix
        final_dir = self.directory.parent/name
        final_csv = final_dir/(name+'__combined.csv')
        self.mark_stopping()
        ended = self.stopped
        pipeline = (self.directory/'pipeline_schema.json').exists()
        info = dict(recording_format='1000Hz cycles + native wide CSV + images' if pipeline else 'legacy long CSV',
                    schema='hipexo-recording/1',recording_id=self.recording_id,
                    session_id=self.session_id,subject_id=self.metadata.get('subject_id','unknown'),
                    location=self.metadata.get('location','unknown'),started_at=self.started.isoformat(),
                    stopped_at=ended.isoformat(),record_button_duration_s=self.duration,
                    sensors_with_valid_data=sensors,
                    sensors_without_valid_data=[s for s in SENSOR_ORDER if s not in sensors],
                    streams=stats,csv_path=str(final_csv.resolve()),csv_write_complete=complete,error=error,
                    raw_references_scope='Files from the same session; they may span a different interval. Referenced, not copied.',
                    session_raw_references=references,
                    local_files=sorted((final_csv.name if p==self.csv_path else str(p.relative_to(self.directory))) for p in self.directory.iterdir()),
                    notes=['EMG valid=0 placeholder rows do not count as a recorded EMG sensor.',
                           ('Camera image timestamps/paths are in camera_images.jsonl and cycles.jsonl; Lidar CSV fields are scan summaries.' if pipeline else 'Camera and Lidar fields in this CSV are summaries, not raw images/scans.'),
                           'CSV writer completion does not certify hardware completeness or Windows STOP_ACK.'])
        description = [
            '本次记录说明',f"受试者：{info['subject_id']}",f"地点：{info['location']}",
            f"开始：{info['started_at']}",f"停止：{info['stopped_at']}",
            '含有效数据的传感器：'+(', '.join(sensors) or '无'),
            '未记录到有效数据：'+', '.join(info['sensors_without_valid_data']),
            'CSV：'+final_csv.name,
            'CSV 写盘完成：'+str(complete), '错误：'+str(error or '无'),'',
            '通道与时间跨度（各传感器开始/停止时间可能不同）：']
        for stream, item in sorted(stats.items()):
            description.append(f"  {stream}: {item['samples']} 帧，{item['valid_samples']} 有效帧，"
                               f"时间跨度 {(item['last_t_ms']-item['first_t_ms'])/1000:.3f} 秒")
        description += ['', '同会话原始文件索引（引用原文件，未复制；可能覆盖不同时间范围）：']
        for ref in references:
            description.append(f"  {ref.get('modality')}: {ref.get('path')}")
        description += ['', ('主 CSV 为 1000Hz 同步宽表；native.csv 保留必要原始采样。cycles.jsonl 每包为 5×60ms。相机图像见 images/ 和 camera_images.jsonl；雷达原始扫描与 EMG 原始包见索引。' if pipeline else 'CSV 内的相机/雷达是识别或扫描摘要。原始深度样本、雷达扫描、EMG 原始包见索引。'),
                        'EMG 缺失槽的 valid=0 占位行不计为有效 EMG；CSV 写盘成功不等于全部硬件或 Windows 尾包已经验收。',
                        '完整字段名、通道统计、来源及路径见 recording_info.json。']
        (self.directory/'recording_info.json').write_text(json.dumps(info,ensure_ascii=False,indent=2),encoding='utf-8')
        (self.directory/'记录说明.txt').write_text('\n'.join(description)+'\n',encoding='utf-8')
        renamed = self.directory/final_csv.name
        if renamed.exists() or final_dir.exists():
            raise FileExistsError('Recording destination exists; original files retained')
        self.csv_path.rename(renamed)
        self.csv_path = renamed
        self.directory.rename(final_dir)
        self.directory,self.csv_path = final_dir,final_csv
        return str(final_csv)
