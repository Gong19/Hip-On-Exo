# Hip-On-Exo
The programme for Hip-On Exoskeleton

## 双机 EMG 交接

[2026-10-03 Windows 实测反馈与 Ubuntu 补丁（v4）](handoffs/2026-10-03-emg-v4/README.md)

## Ubuntu 同步采集优化

[2026-10-10：1000 Hz 对齐、300 ms 周期记录与有界内存](ubuntu/2026-10-10-sync/README.md)

包含代码、字段说明、模拟样例与验证报告。1000 Hz 为统一数据网格；真实采样率、插值与硬件限制见报告。

## Ubuntu 原生采集优化 V2

[2026-10-10 V2：双路 ADS8688 原生 1000 Hz、IMU 进程隔离与电机并行采集](ubuntu/2026-10-10-native-v2/README.md)

保留 V1；V2 的实测读取率、抖动、内存和未达标链路见版本验收报告。真实实验数据和凭证不上传。

## Ubuntu 原生采集优化 V3

[2026-10-10 V3：四 IMU 200 Hz、电机接近 1 kHz 零输出反馈及非阻塞 ADC 交接](ubuntu/2026-10-10-native-v3/README.md)

包含180秒完整界面联测结果。主动电机控制仍使用SDK原路径；实际频率、时间戳含义和限制见验收报告。

## Ubuntu V3.1 长时间采集

[995 Hz 电机监测目标与20分钟持续采集验收](ubuntu/2026-10-10-native-v3/docs/ENDURANCE_20MIN.md)。真实波形仅保存在本机。

## Ubuntu V3.2 稳定性修复

[950 Hz电机请求、调度时间表保持及IMU非阻塞交接](ubuntu/2026-10-10-native-v3/docs/STABILITY_950HZ.md)。包含修复前后的短时实测对比；不将其冒充新的20分钟验收。

## Ubuntu V3.3 默认1000 Hz

[默认电机请求恢复1000 Hz，保留调度及IMU交接修复](ubuntu/2026-10-10-native-v3/README.md)。按实际反馈时间戳重采样到1000 Hz，原生数据与质量标记继续保留。
