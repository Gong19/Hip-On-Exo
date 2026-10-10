# HiPExo native acquisition V3.1 — 2026-10-10

本版解决慢 I²C 总线上两只 IMU 的 200 Hz 读取，并新增独立的 C++ 电机零输出监测进程。1000 Hz 同步网格、60 ms × 5 的300 ms周期、有界队列和图像单独保存继续沿用V2。

## V3.1：995 Hz监测目标

默认零输出监测请求目标改为995 Hz，可用 `HIPEXO_MOTOR_NATIVE_HZ=990` 调整（接受100–1000）。统一处理网格仍为1000 Hz，按实际采样时间戳插值，不要求原生频率是整数分频，也不把插值当作新增硬件样本。Linux调度与USB交付会使实收频率偏离请求目标。

20分钟持续采集结果见 [ENDURANCE_20MIN.md](docs/ENDURANCE_20MIN.md)。

## V3历史180秒实测结果

180秒完整界面联测：**四IMU各200Hz；两路电机997.7/998.9Hz；两路力各1000Hz**。电机相较V2完整界面的约349/352Hz提高约2.8倍。863,014帧保存完成，迟到/乱序/记录队列溢出均0；两路电机发送数和有效反馈数分别一致，坏字节0。仍有Linux调度和USB交付抖动，详细间隔与未通过的中间试验见验收报告。

## 启动

关闭旧窗口后，原命令不变：

```bash
cd "/home/gong/Desktop/exo-control (copy)/python"
python3 hipexo_monitor19.py
```

其它机器需要本地 Unitree SDK，构建两个组件；不发布厂商二进制：

```bash
python3 build_motor_io.py --sdk-root '/path/to/unitree_sdk'
python3 build_motor_capture.py --sdk-root '/path/to/unitree_sdk'
```

当前构建脚本针对本机 ARM64 SDK。没有原生辅助程序时，界面继续使用 SDK 线程采集。`HIPEXO_MOTOR_NATIVE=0` 可回退。

## 四路 IMU

原先 I²C-7 配置400kHz，I²C-1配置100kHz。慢总线读完两只的必要寄存器约需5.2ms；合并事务和连续读取均未解决。这不是200Hz目标的数学问题，而是总线吞吐不足。

本机匹配 `i2c@c240000`、对应时钟ID49和100kHz设备树配置时，采集期间把 I2C2 **源时钟**从136MHz提高到其报告的支持上限204MHz。短测可读约277次/秒，正式仍按每只200Hz调度。停止采集恢复136MHz和原来的未锁定状态；不改启动配置，不重启，不提升CPU/GPU功耗或频率。

这是 NVIDIA BPMP 的运行时调试时钟覆盖，**不是把设备树改成400kHz**。SCL线频率未用示波器测量，不把204MHz写成总线速率。源时钟由整条总线共享；同总线还有FUSB301和INA3221，其厂家规格支持400kHz。该策略仅匹配已验证的本机配置，不自动推广到其它控制器/电脑。资料：[NVIDIA时钟文档](https://docs.nvidia.com/jetson/archives/r36.3/DeveloperGuide/SD/Clocks.html)、[FUSB301](https://www.onsemi.com/download/data-sheet/pdf/fusb301-d.pdf)、[INA3221](https://www.ti.com/lit/ds/symlink/ina3221.pdf)。

`HIPEXO_I2C_CLOCK_TUNING=0` 或 `HIPEXO_DISABLE_TUNING=1` 可禁用。提权/调优失败会在界面报告，不能据此假定仍达到200Hz。排他租约和落盘的原设置支持下次运行恢复；整个主进程被SIGKILL时无法当场执行清理。不同应用如另有时钟覆盖，本版拒绝覆盖其策略。

IMU统计的是主机有效寄存器读取率，设备内部解算和重复寄存器值的限制仍与V2一致。

## 电机监测

每条电机串口在独立C++进程中以默认目标995次/秒发送**所有位置/速度/刚度/阻尼/扭矩参数均为零**的监测请求，使用本机官方SDK编码和CRC校验。采集调度不再受Python界面执行锁影响。不会在延迟之后连续突发补发；发送之间至少保留750µs的主机间隔，以减少半双工碰撞。主机间隔不是线上时序测量。

有足够CPU核心时，两路进程分别绑定到当前可用核心列表末尾的两颗核；只设置本进程的亲和性和FIFO10调度，不隔离系统CPU。IPC以约20ms批量交接，应用待发队列上限256KiB，串口接收缓冲4096字节；过载明确失败并回退SDK路径。

**主动电机控制仍使用原来的SDK线程路径。** 从监测切换到控制时先停止原生进程，结束串口占用后再发送控制命令。停止时先停原生请求并收尾，再发原路径零输出。没有用未验证的异步链路替换带负载控制。

异步反馈没有设备序号用于可靠配对，因此：

- 原生监测数据时间戳为 `host_validated_frame`，即主机收到并校验一帧的时间；USB可能把多个反馈一起交给主机，不能假设它们物理采样严格间隔1ms。
- CSV `host_round_trip_ms` 在此模式下是NaN，不猜测或套用最近请求的发送时刻。同步SDK模式继续写实际主机往返耗时。
- 每条电机另存 `motor_<id>_transport_quality.json`，包含发送数、有效回复数、丢弃的坏字节、错过的完整调度周期、发送错误、IPC峰值及CPU亲和性。`final=true`时再比较发送/回复数；计数范围为采集进程生命周期，可能包含记录按钮按下之前/之后的时间。
- `source_metadata.jsonl`记录 `motor_transport` 和 `timestamp_basis`，能够辨别同一场内的回退或控制模式切换。主机有效回复率不是设备内部采样率证明。

## 力传感器交接修复

电机提速后，在180秒完整界面联测中发现ADC批次交接可能阻塞采集，曾出现43–62ms空档。本版把ADC发送改成非阻塞泵送，另设256KiB硬上限；界面短时忙碌时继续采样，恢复后按原时间戳交接。停止时有界排空，过载或排空失败明确报错，不无限累积。SPI转换与通道设置保持原实现。

## 验收和回滚

见 [VALIDATION.md](docs/VALIDATION.md) 和 [TESTING.md](docs/TESTING.md)。真实波形、私有配置及SDK二进制仅留本机，GitHub只有代码、文档和SIMULATED样例。V2备份在本机 `backup-before-native-v3-20261010-220958`；回退应在关闭界面后恢复备份代码，或先禁用上述两个新功能对照。
