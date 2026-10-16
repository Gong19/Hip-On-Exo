# Ubuntu control-only 安装回执（2026-10-16）

## 已安装

已核验Windows交接提交5534b3655df48f9479e8f51a8a599d529d71dd75及包内SHA256清单。Ubuntu实际基线另含10月12日电机启动路径修复，因此逐项合并安装，保留该本机修复，没有整文件覆盖回旧版本。修改前完整备份受影响文件；软件标识native-v3.5-emg-control-only-20261016。

原启动入口 `python3 hipexo_monitor19.py` 不变。EMG工作线程/页面已替换成hipexo_emg_control，协议为hipexo-emg-control/1，端口8766；旧8765数据桥接不兼容。沿用本机原IP/令牌，新的私有配置emg_control_private.json权限600，未上传。

Ubuntu不创建EMG处理器、样本环或曲线，不接收/保存EMG样本；公共周期CSV已移除7路EMG的42列。其他传感器的采集及1000 Hz网格不变。只保留小型emg_control_UUID.jsonl：运行ID、同步探测、开始/停止请求回执、Windows及Ubuntu文件关联。历史EMG模块/旧数据保留用于回滚，不由正常入口调用。

## 本机补充

- 单独“同步并开始EMG”不依赖全局联动勾选；全局Record仍须显式勾选联动。
- 开始回执核验run_id与recording；停止回执核验stopped、recording=false及save_ok，避免把错误回执当成功。
- 开始前同步探测检测Ubuntu壁钟跳变，并拒绝Windows处理时间为负的探测。
- 页面展示Windows报告的保存行数和路径；状态报告不能替代读取Windows CSV验证数据内容。

## 操作

Windows关闭旧桥接，用Start_EMG_Waveform_Sync.cmd打开新窗口，准备传感器并启用“允许Jetson联动开始/停止”。Ubuntu在EMG页连接IP:8766，确认Windows就绪。单独开始/停止只操作Windows EMG；联合采集勾选联动、Collect All启动本地传感器、Record开始两端保存。Stop Rec向Windows发停止并停止本地记录；必须核对Windows停止及保存回执。

关闭Ubuntu或仅断开网络不会停止Windows，网络断开时状态未知；重连查看运行ID后可停止。取消联动后全局Record不再下发EMG启停。同步是软件指令/时间戳旁证，不是硬件触发。Windows保存原程序处理后的envelope_data.csv，未承诺保存原始SDK电压。此前Ubuntu的1000 Hz重采样不再参与；Windows CSV是否满足1000 Hz须由Windows时间列核验，不能把本次控制安装视为采样率验收。

## 验证与当前边界

交接包5项测试、Ubuntu补充4项测试通过。真实X11窗口预览检查通过：状态面板无波形；全局Record启停调用已连接到新worker；实际写出的CSV无EMG列；worker无样本环/处理器。单独导入新模块没有加载NumPy、SciPy、pyqtgraph或旧EMG处理器。整套界面因其他传感器仍会使用绘图库。

未启动任何真实传感器或电机进行本轮验证。当前配置Windows IP的8766连接超时，等待Windows新启动器/监听/防火墙状态确认；尚未进行新版真实双机联采，也没有测得新架构内存节省量。上次8765真实单路40秒测试属于旧数据桥接，不能计作本版本验收。

回滚：关闭界面，将本机backup-before-emg-control-*里的原文件恢复；Windows使用对应旧启动器。此次新增的模块没有被旧入口导入时可以保留。
