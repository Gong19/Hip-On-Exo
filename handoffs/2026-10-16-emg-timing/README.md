# EMG 开始、停止和回执时间记录升级（2026-10-16）

此升级继续由 Windows 原波形程序采集、处理和保存 EMG；Jetson 仅收发状态/同步指令。新增逐次延迟数值和事件时间戳，便于事后审计和近似事件对齐。端口仍为 **8766**，保留原私有令牌，不重新生成。

## 部署：两端均需更新、重启

Windows 本机适配代码已更新。先正常停止并关闭正在运行的 EMG 窗口，再双击原来的 `Start_EMG_Waveform_Sync.cmd`。扫描并按原流程准备，保留“允许 Jetson 联动”勾选。正在运行的旧进程不会自动加载磁盘上的新代码。

Ubuntu 升级针对已安装控制面板的提交 **8af0b24fb80f074f970619b9702d3156f7203659**，目标为该项目 `ubuntu/2026-10-10-native-v3/python` 的实际运行副本。先关闭 Ubuntu 界面，在本交接目录执行：

```bash
python3 apply_ubuntu_timing_update.py --target "/实际项目/ubuntu/2026-10-10-native-v3/python" --dry-run
python3 apply_ubuntu_timing_update.py --target "/实际项目/ubuntu/2026-10-10-native-v3/python"
```

安装器先验证三个现有文件 SHA256，再备份至目标父目录的 `backup-before-emg-timing-*`，最后更新 `hipexo_monitor.py`、`hipexo_emg_control.py`、`control_protocol.py`。校验不一致时不覆盖任何文件，应先合并较新代码；不要绕过检查。回滚时关闭界面，恢复这三个备份文件。配置和令牌不改动。仍用原 `python3 hipexo_monitor19.py` 启动。

这个包是既有控制版的增量升级；Windows 入口依赖此前本机的独立启动辅助脚本、原始 AAA 程序、SDK 和环境，不能把本目录单独复制到另一台 Windows 就运行。Ubuntu 不需要 `timing_evidence.py` 或新的科学计算库。

## 按钮含义与验收

1. 连接 Windows，确认真实源、允许联动和 Windows 就绪。勾选全局联动时，**Collect All 建立连接；全局 Record 开始 Windows 保存**。也可点 EMG 页的单独“同步并开始 EMG”。本次保留既有按钮行为。
2. 开始后，Ubuntu 面板显示请求→Windows 软件开始、发送→Windows 接收、Windows 软件开始→Ubuntu 收到回执。Collect All 时间已记录时另显示它→Windows 开始的间隔，明确包含人为等待。
3. 停止并等待 Windows 确认保存成功，Ubuntu 面板显示停止点击→Windows 保存流程完成；Windows CSV 应增长、最终不再增长。
4. Windows `records/sync_UUID.json` 应有 `timing_evidence_version: 2`、`timing_events`、`timing_metrics_ms`、`start_client_receipt`、`stop_client_receipt`。Ubuntu 小型 `emg_control_UUID.jsonl` 应有 `start_receipt_saved` 和 `stop_receipt_saved`；核对双方 UUID 和 CSV 路径一致。
5. 本次代码已通过本机模拟链路验收，但还需上述短时双机真实验收。旧试采集缺失的时间点无法凭更新代码追补。

## 时间戳保存在哪里

保留原 `envelope_data.csv` 格式：其中时间是处理后的相对时间。每次联动实验在其父级 `records` 下写 `sync_UUID.json`，并通过 `windows_record_path` 指向 CSV。Ubuntu 只保存控制事件小日志，无 EMG 样本。新时间戳为整数纳秒 `wall_ns` 和同主机单调时钟 `mono_ns`；纳秒单位不代表纳秒精度。

| JSON 字段 | 精确含义 |
|---|---|
| `ubuntu_collect_all` | Collect All Qt 回调进入时；不是操作系统物理按键时刻 |
| `ubuntu_request`, `ubuntu_trigger` | Record/单独开始的回调时刻及触发来源；进入后台队列之前 |
| `client_send` | 客户端开始编码/发送 START 前的软件时刻；不是网卡发包时刻 |
| `command_received` | Windows 接收并解码 START 的软件时刻 |
| `gui_start_begin` | Windows 进入原程序开始处理之前 |
| `gui_sync_pulse` | 原程序发出软件同步脉冲；是软件开始标记 |
| `timing_events.gate_released` | 原程序解除保存模式同步门；`wall_ns` 由相邻系统/单调时钟投影 |
| `timing_events.first_saving_frame` | 首个非空帧进入保存模式预处理函数，附设备时间范围；不是首个物理样本采样时刻 |
| `gui_start_returned` | 原程序开始处理返回 |
| `timing_events.first_csv_write_begin/returned` | 首次 CSV 批次写入调用开始/成功返回 |
| `timing_events.last_csv_write_returned` | 最近一次 CSV 批次写入成功返回 |
| `start_client_receipt.received` | Ubuntu 解码开始回执的时刻；通过追加 MARK 保存回 Windows |
| `ubuntu_stop_request`, `stop_client_send`, `stop_received` | 停止回调、STOP 编码发送前、Windows 接收 STOP |
| `timing_events.processing_stop_begin/returned` | 原处理/排队停止流程的边界 |
| `timing_events.sdk_stop_begin/returned` | SDK Stop 调用边界；返回不等于传感器最后一个物理样本时间 |
| `timing_events.file_save_stop_begin/returned` | 保存管理器停止/收尾边界 |
| `stop_returned` | 原 GUI 停止和保存流程返回，需同时核对 `save_ok: true` |
| `stop_client_receipt.received` | Ubuntu 解码停止回执的时刻 |

只有经联动创建的运行才有完整跨机记录。Windows 手动停止可有本机收尾点，但没有对应的 Ubuntu 停止点击。网络中断若使 MARK 回传失败，Ubuntu 日志仍保留接收时间，Windows 缺失字段为 null。程序异常退出也可能留下不完整记录；不能把缺失当作零延迟。

## 具体数值与公式

Windows `timing_metrics_ms` 中按毫秒保存：

- `request_to_gui_pulse_ms`：触发回调→软件开始标记。
- `request_to_gate_release_ms`：触发回调→解除保存同步门。
- `request_to_first_frame_seen_ms`：触发回调→首帧进入主机处理。
- `request_to_windows_receive_ms`：回调→Windows 接收，包括排队、前置查询/时钟探测和发送。
- `send_to_windows_receive_ms`：发送前→接收解码，包含协议/系统处理与网络，不能等同纯链路时延。
- `gui_pulse_to_ubuntu_ack_receive_ms`：Windows 软件开始→Ubuntu 解码回执，包括 Windows 后续处理、返回和网络。
- `request_to_ubuntu_ack_receive_ms`：Ubuntu 请求→收到回执，使用 Ubuntu 自身单调时钟。
- `collect_all_to_gui_pulse_ms`：Collect All→软件开始，可能包含等到 Record 的人为间隔。
- `stop_click_to_windows_receive_ms`、`stop_click_to_sdk_stop_returned_ms`、`stop_click_to_save_complete_ms`：停止回调到各个停止阶段；停止前另做 3 次时钟探测，它们的耗时包含在点击总延迟中。
- `windows_receive_to_gui_begin_ms`、`windows_receive_to_stop_complete_ms`、`csv_finalization_ms`：同一 Windows 主机单调时钟差。

设估计偏差 `offset_ns = Windows − Ubuntu`，则跨机开始延迟为 `(Windows事件.wall_ns − offset_ns − Ubuntu请求.wall_ns) / 1e6`；反向回执为 `(Ubuntu收到.wall_ns − Windows开始.wall_ns + offset_ns) / 1e6`。停止使用停止前重新测量的 `stop_clock_sync`，缺失时回退到开始偏差。**不同机器的 mono_ns 绝不相减。**

偏差来自最小 RTT 的四时间戳探测。`minimum_rtt_ms` 和 `rtt_half_reference_ms` 作为质量参考；RTT/2 不是实测单向延迟、不是严格同步误差上界。无线非对称、调度、时钟漂移会影响估计。协议拒绝明显时钟跳变和不合法探测，但不校准设备内部时延。

## 对齐时不能越过的证据边界

软件同步标记、首帧进入处理、首个物理采样点和 CSV 写入完成是不同事件。本次增加前三类主机观测点中的软件标记及处理进入点，没有凭空创造硬件 UTC。原 CSV 的 `save_time_base`、处理/滤波/重采样/缓冲和 Delsys 无线链路决定额外偏移；原模式中的绝对样本锚点仍可能为空。

因此逐次软件延迟可用于事件级近似对齐和排查，不能把某一次毫秒值固定扣到所有实验，也不能直接把 CSV 第 0 行认定为 GUI 同步脉冲。要确定物理样本级误差，需要双方共同记录的硬件触发或独立标定。保存完成指应用写入/关闭完成，不保证存储介质已经物理持久化。

## 验证与修改依据

- 原有 9 项控制协议/Qt 面板测试通过；新增 4 项覆盖偏差方向、不同单调时钟原点、停止偏差更新、缺失值、空批次不冒充数据写入、安装备份和拒绝覆盖未知版本。
- 原 Windows 波形程序模拟采集整链路成功：最终复核 3193 数据行（另有 6 行表头/说明），验证开始、断线续采、重连、停止、CSV 和全部关键新增打点。模拟数值仅证明功能路径，不作为 eduroam 或真实设备性能结论。
- 依据是仓库 Ubuntu 提交 `8af0b24fb80f074f970619b9702d3156f7203659`、此前控制交接提交 `5534b3655df48f9479e8f51a8a599d529d71dd75`、本机原程序相关方法及本机测试。此次是代码与实测文件核验，没有引入新的网上延迟指标。
- 公开包仅含代码、文档和模拟验证摘要；真实 CSV、真实同步 JSON、信号图和凭证留在本机。
