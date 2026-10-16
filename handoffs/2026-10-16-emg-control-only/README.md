# EMG 改为 Windows 本地采集，Jetson 仅同步控制（2026-10-16）

用户最新要求：保留 Windows 原来的波形采集界面；Jetson 不接收、显示、滤波、重采样或保存 EMG 样本，只显示连接/采集状态，并可选择是否联动开始、停止。本方案取代此前向 Ubuntu 持续发送 EMG 数据的工作方式。

## 已完成与尚未完成

- 新增 Windows 联动启动器；在内存中给原 `AAA_EMG_RealTime_Collection.py` 安装控制入口，原文件和旧启动器不改。沿用本机固定令牌，不重新生成。
- 开启原波形程序的本地 CSV 保存、周期写入。**保存的是原程序已有的处理后 `envelope_data.csv`，不是旧桥接的原始 SDK JSONL，也不承诺包含未经处理的原始电压。** 采样模式、标定和波形仍在 Windows 操作。
- Jetson 替换为轻量状态面板；其 EMG 模块只依赖标准库和已有 PyQt5。不会构造 `EmgProcessor`、EMG 环形样本缓存或绘图对象。
- Ubuntu 统一周期记录移除 7 路 EMG 数据及质量列，共 42 列；其他设备记录保持原实现。主程序因其他传感器仍需 NumPy/绘图，不能宣称整个应用不再使用这些库。
- 5 项本机集成测试通过，另用**原波形程序的模拟源**完成开始、断线续采、重连、停止、真实 CSV 写盘验证：3207 数据行、513407 字节，另有表头/说明行。不是硬件实测。
- 尚未登录或修改 Jetson 实机，尚未做新版双机真实联测；公开上传不代表 Ubuntu 已安装。

## Windows 操作

1. 关闭旧 EMG 桥接服务和旧波形窗口，避免基站被两个程序占用。
2. 在本机 `outputs/emg_control_only` 中双击 **Start_EMG_Waveform_Sync.cmd**。这是原波形界面加“Jetson 同步”工具栏。默认真实硬件，不自动启动正式采集。
3. 像以前一样连接基站、唤醒传感器、扫描，核对波形和所选模式。保留工具栏“允许 Jetson 联动开始/停止”勾选。
4. Jetson 填本次 Windows 的 IP，**端口 8766**，沿用原固定令牌。旧流式桥接 8765 与新控制协议不通用。
5. 数据保存至本目录 `records/EMG_Session_时间/envelope_data.csv`；同步旁证为 `records/sync_运行UUID.json`。
6. 保持 Windows 窗口打开。停止后原界面会禁用开始按钮，下一轮按原界面流程重新扫描/准备，等 Jetson 显示就绪。

本启动器依赖现有 `outputs/start_emg_standalone.py`、`outputs/windows_emg_bridge/bridge_settings.py` 和本机原项目/运行环境；不能把这个单独目录移到任意 Windows 后就直接运行。GitHub 不包含 SDK、许可证、私有令牌和原项目。

## Ubuntu 安装与操作

先关闭正在运行的 Hip Exo 界面。此补丁针对仓库 `ubuntu/2026-10-10-native-v3/python` 在下述提交中的版本：

`6320378018f5b6d30d50eed722a78396e2ac5c60`

将这个交接目录带到 Ubuntu，在终端执行（把目标替换为实际项目 python 目录）：

```bash
python3 apply_ubuntu_control_only.py --target "/实际路径/python" --dry-run
python3 apply_ubuntu_control_only.py --target "/实际路径/python"
```

安装器先核验两个源文件 SHA256 和全部替换位置，再把将被覆盖的所有文件备份到目标目录的父目录，最后安装。任一基线不一致会停止，不会覆盖本机较新的代码。它修改 `hipexo_monitor.py`、`hipexo_pipeline.py`，新增 `hipexo_emg_control.py`、`control_protocol.py`。不改现有 `emg_config.json`；首次读取它的 IP/令牌，控制端口另用 8766，之后使用本机 `emg_control_private.json`。

用原命令 `python3 hipexo_monitor19.py` 启动。

1. EMG 页面输入 Windows 当前 IP、8766、原固定令牌，点击“连接 / 重连”。
2. 默认没有勾选联动：此时可以照常采集其他设备，不会随全局 Record 启动 Windows EMG。
3. 要联合采集时，勾选“联动 Windows EMG”。`Collect All` 只建立 EMG 控制连接，实际 EMG 保存随全局 **Record** 指令开始；其他设备仍按原 Collect All 流程启动。
4. 全局停止 Record/Stop All 会发送 EMG 停止指令（联动保持勾选时）；也可使用 EMG 页的单独“同步并开始 EMG”“停止 Windows EMG”。单独按钮不会启动其他传感器或电机。
5. 核对 EMG 页面确认 Windows 正在采集/已停止。Ubuntu 本地记录开始不等于 Windows 已成功开始，网络/扫描失败会在 EMG 页显示错误，其他数据继续保存。
6. Ubuntu 会话目录只追加小型 `emg_control_UUID.jsonl`，保留运行编号、时钟探测、指令及回执、两端文件路径；没有 EMG 样本文件。已有旧数据不会删除。

回滚时关闭两端程序，按安装器输出的备份位置恢复原 `hipexo_monitor.py`、`hipexo_pipeline.py`，以及备份中原先存在的同名模块；本次新加模块可保留，不再被导入。Windows 使用旧启动器即可恢复旧入口。

## 同步语义与故障处理

- `hipexo-emg-control/1` 使用 TCP JSON 小消息，单条最多 16 KiB；只允许 hello、ping、status、start、stop、mark，协议没有 data 消息。
- Windows 始终是 SDK 和本地数据的拥有者。网络线程只把开始/停止指令交给原 GUI 线程；不再另开 SDK 采集器。
- 平时约每 2 秒一次状态查询；开始前 7 次四时间戳探测，取最小 RTT；运行中每 60 秒补充 3 次探测，用于检查时钟变化。保存少量控制元数据是必要关联证据，并不把波形转移到 Jetson。
- 采用“软件同步指令 + 实际执行时间回执”，**不是硬件触发同步，也不是保证两台设备首个采样点同时出现**。全局 Record 先启动 Ubuntu 本地记录，再异步请求 Windows；会保留两者时间，不把网络 RTT/2 当设备内部延迟。
- `offset_ns = Windows UTC - Ubuntu UTC`。换算 Windows 主机事件到 Ubuntu UTC 可减去该偏移。Windows 旁证记录指令接收、GUI 同步脉冲、开始返回、停止返回及原 CSV 的设备时间基准。设备时间到物理采样 UTC 的固定延迟未知；不能直接把 CSV 第 0 行宣称为同步脉冲时刻。
- 原程序独立模式中的 `label_zero_perf_ns`、`actual_sync_start_perf_ns` 可能为 null，测试中也如此；保留原值，不伪造精度。长时漂移探测只留证据，不自动重写两台机器的时间轴。
- 断网、关闭 Jetson 或“仅断开”不会停止 Windows；界面显示状态未知。重连读取原运行编号后可停止。不要在超时后换新编号盲目重发开始。
- START/STOP 按运行编号防重复；Windows 停止 ACK 只在原 GUI 停止并完成保存流程后返回。开始 ACK 表示进入保存模式，不保证已有样本落盘；真实验收仍须看波形、保存行数和 CSV 增长。
- 同一 eduroam 名称不能证明新端口可达。如果连接失败，应先核对 Windows 当前 IP、8766 监听和防火墙；校园客户端隔离需改用允许互通的有线/实验网络或由 IT 开通。此版本未重新实测校园网络。

## 本轮依据与验证范围

1. 用户本轮明确选择“保留原来有波形的 EMG 采集界面”。
2. 本机 `AAA_EMG_RealTime_Collection.py`：检查 `on_start`、`on_stop`、`sync_pulse_signal`、`FileSaveManager` 及原保存开关；通过内存适配保持源文件不变。
3. [Ubuntu 已审阅基线提交](https://github.com/Gong19/Hip-On-Exo/commit/6320378018f5b6d30d50eed722a78396e2ac5c60)：核对 `hipexo_monitor.py` 的 Collect/Record 流程、`hipexo_emg.py` 的处理链、`hipexo_pipeline.py` 的固定列定义。安装器的基线文件 Git blob 哈希已与仓库返回值核对。
4. `test_control_only.py`：5 项回环集成测试（模拟 GUI、Qt worker、断线/重连、防重复、联动关闭、安装备份、实际记录器无 EMG 列）。
5. `validate_original_simulation.py`：本机原波形程序模拟运行，证实本地 CSV/同步旁证及断线续采；测试不会访问真实基站。

本轮属于已授权的软件架构改造，没有新增学术性能结论、延迟实测承诺或社交媒体引用。此前研究报告和参考文献仍保留；新模式的真实硬件、校园网络、长时间同步误差需要另做双机验收。
