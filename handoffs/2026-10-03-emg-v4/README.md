# Windows → Ubuntu EMG 最新交接（2026-10-03，v4）

这次真实跨电脑十秒测试的回执、Windows 修正和 Ubuntu 补丁集中在这里。Windows 桥接源码版本为 `20261003.4`，协议仍是 `hipexo-emg/1`。

## 先读结论

- 真实 ED / SID 57569，148.148148 Hz 厂商 RMS、100 ms 窗口。
- 两端原始记录 264 包、1496 个样本逐点相同，最终序号 263；原始记录可保留。
- Ubuntu 未收到成功 STOP_ACK；派生 CSV 只到 7.364 秒，不能当作完整十秒结果。
- 应用层逐包延迟中位约 631 ms、最大约 2.70 秒；起始 RTT/2 约 2.167 ms 不能代表实际显示延迟。
- 旧数据幅值单位未经通道 Unit 核实；不能仅凭 `values_v` 名称推断为伏特。

[完整中文核查回执](EMG_实测核查_20261003/实测核查与修复回执_中文.txt) · [逐包核对摘要](EMG_实测核查_20261003/双端原始记录核对.json) · [软件验证](EMG_实测核查_20261003/修复验证.json)

## Ubuntu 端处理顺序

1. 先备份当前实际工程和配置；当前修改以你本机代码为准。
2. 阅读 [STOP 补丁](EMG_实测核查_20261003/ubuntu_patch/hipexo_emg_remote.py.patch)：默认等待 20 秒，记录停止请求/失败，超时关闭旧连接，避免重复 STOP 与迟到 ACK 混淆。20 秒是可配置预算，不是设备性能承诺。
3. 合入 [标定单位补丁](EMG_实测核查_20261003/ubuntu_patch/hipexo_emg_core.py.patch)：单位/倍率加入标定 fingerprint，防止旧尺度的 baseline/MVC 被自动复用。
4. [原文件哈希](EMG_实测核查_20261003/ubuntu_patch/base_sha256.json) 用于确认补丁基线；若当前文件已变化，只合入对应函数差异，不整体覆盖新工作。目录内 `.py` 是修改后的参考副本，尚未部署到 Ubuntu。
5. 继续处理 worker 异常时的派生尾数据排空，以及网络接收被处理/CSV/绘图拖慢的问题；这两项尚未在本交接里修完。不得用隐藏 ERROR 或丢包来制造成功。
6. 重启 Windows 新版本，检查实际 SDK Unit/倍率；重新做十秒真实采集、成功 STOP 和 CSV 完整性核对，通过后再做十分钟测试。

## Windows 本机已准备的修改

[源码](windows_emg_bridge/) 按 `ChannelTrigno.Unit` 换算已知电压单位，同时保留未换算 `sdk_values`、单位和倍率；未知单位拒绝采集。新增 STOP 收取、SDK Stop 前后、排空和关闭时刻，以及 `.stop_receipt.json` 中的 ACK 发送结果。固定令牌仍由本机私有文件持久保存，公开仓库不含令牌。

23 项软件测试通过，另外验证了单位变化拒绝旧标定。没有做改动后的真实硬件复测。socket 写成功不等于接收端应用已收到 ACK。Windows 源码里的个人路径已替换为 `YOUR_WINDOWS_USER`，这是代码审阅快照；本机原启动器仍使用本机实际路径。

## 读取与后续回执

Ubuntu 可以直接访问本目录，也可以 `git pull` 后读取。建议优先阅读完整中文回执，再检查 `.patch` 差异。后续测试回执可新增到独立日期目录，写明版本、run_id、实际模式/单位、STOP_ACK 和原始/派生完整性；不要覆盖这次历史结论。

公开内容包含汇总指标、修复代码与补丁，不包含原始 JSONL 波形、CSV、设备 DLL、许可证、私有令牌或历史完整工作日志。研究原始证据保留在两台实验电脑。

官方依据：[Delsys Python 示例文档](https://github.com/delsys-inc/Example-Applications/blob/main/Python/README.md) 的 ChannelTrigno.Unit / PollYTData / Stop；详细引用及验证边界见完整回执。
