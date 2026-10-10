# 回归与性能测试

以下自动回归不连接真实硬件。先安装原项目 Qt/NumPy/SciPy/传感器 Python 依赖；SDK 需要在本机自行提供和构建，仓库不分发厂商库。

新采集、隔离、队列、停止排空和同步网格测试：

```bash
HIPEXO_DISABLE_TUNING=1 QT_QPA_PLATFORM=offscreen python3 -m unittest \
  test_realtime test_imu_capture test_motor_parallel test_writeback test_image_writer test_pipeline \
  test_force_capture test_recording_process test_pipeline_integration test_camera_isolation
```

旧界面、EMG、IMU参考姿态与断线状态回归需旧保存格式和旧 vision 模式：

```bash
HIPEXO_DISABLE_TUNING=1 HIPEXO_RECORDING_MODE=legacy HIPEXO_VISION_MODE=inference QT_QPA_PLATFORM=offscreen python3 - <<'PY'
import sys,unittest
sys.argv.append('--preview')
import hipexo_monitor
names=['test_combined_interface','test_emg_integration','test_emg_remote','test_imu_reference','test_imu_reference_ui','test_imu_status','test_recording_layout','test_timing_evidence']
r=unittest.TextTestRunner().run(unittest.defaultTestLoader.loadTestsFromNames(names))
sys.exit(not r.wasSuccessful())
PY
```

合成持续负载（无硬件、无网络；临时数据自动删除）：

```bash
mkdir -p evidence
python3 benchmark_pipeline.py --seconds 180 --isolated --images --image-noise --cycles-callback --output evidence/synthetic_180s.json
python3 generate_simulated_example.py --output examples/SIMULATED_300ms
python3 read_cycle.py examples/SIMULATED_300ms 0
```

实机基准必须先关闭界面及其它采集程序，使用本机 SDK 路径。会真实采集力、IMU、相机；只有显式 `--motors-zero` 才构造电机采集并发送零输出命令。它不会执行负载控制，不连接 Windows EMG 或启动 LiDAR。实测数据仅留本地 `evidence/live-acquisition/`，不可混入合成样例。

```bash
HIPEXO_SDK_LIB_DIR="/home/gong/Desktop/exo-control (copy)/lib" QT_QPA_PLATFORM=offscreen python3 benchmark_live_readonly.py --seconds 180 --motors-zero
```

`live_benchmark.json` 的 `quality.streams` 是完整记录统计；`rates` 来自显示环形缓存末段，不能替代整场统计。频率以实际时间戳计算，不能用1000Hz处理CSV的行数验收原生采样。

完整Qt页面重绘与实机采集联合测试（依次显示六个页面，电机仅零输出；需要先关闭其它硬件采集程序）：

```bash
HIPEXO_SDK_LIB_DIR="/home/gong/Desktop/exo-control (copy)/lib" QT_QPA_PLATFORM=offscreen python3 benchmark_gui_readonly.py --seconds 60 --motors-zero
```

`event_loop_iterations`是测试循环次数，不是显示帧率。offscreen测试包含Qt绘图，不能完全替代桌面显示服务器和真实相机画面的负载。
