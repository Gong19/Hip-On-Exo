# V3复现

先构建C++辅助程序（参见README）。新增测试全部用临时文件和伪终端，不打开真实电机：

```bash
HIPEXO_DISABLE_TUNING=1 python3 -m unittest -v test_i2c_clock test_motor_process test_motor_transition test_capture_ipc
```

包含时钟恢复、锁冲突、崩溃后恢复、拒绝其它时钟策略；伪电机验证每条请求全零、正确CRC、过滤损坏反馈、部分IPC拼包、主进程消失后退出。旧版27项采集/记录测试及57项界面回归也需通过。

实际完整界面联测：必须先关闭其它界面和串口采集程序。明确启用零输出监测，禁止在负载控制中运行此基准。

```bash
mkdir -p evidence
QT_QPA_PLATFORM=offscreen HIPEXO_SDK_LIB_DIR='/home/gong/Desktop/exo-control (copy)/lib' python3 benchmark_gui_readonly.py --seconds 180 --motors-zero
```

每30秒切换六个页面，真实采集四IMU、两路力、两路电机反馈；本轮不启用Windows EMG/LiDAR，未检测到相机。Qt offscreen包含绘图但不能完全代表实际桌面显示服务器和相机画面负载。

验收同时查看 `pipeline_quality.json` 与两份 `motor_*_transport_quality.json`：记录没有队列溢出不代表线上反馈没有丢失。用原生时间戳算频率，不用1000Hz处理表的行数证明硬件达到1000Hz。

V3全部38项采集/保存回归：

```bash
HIPEXO_DISABLE_TUNING=1 QT_QPA_PLATFORM=offscreen python3 -m unittest -v test_i2c_clock test_motor_process test_motor_transition test_capture_ipc test_realtime test_imu_capture test_motor_parallel test_writeback test_image_writer test_pipeline test_force_capture test_recording_process test_pipeline_integration test_camera_isolation
```

57项旧界面回归采用离线preview，禁用新硬件调优和原生辅助进程：

```bash
HIPEXO_DISABLE_TUNING=1 HIPEXO_MOTOR_NATIVE=0 HIPEXO_RECORDING_MODE=legacy HIPEXO_VISION_MODE=inference QT_QPA_PLATFORM=offscreen python3 - <<'PYTEST'
import sys,unittest
sys.argv.append('--preview')
import hipexo_monitor
names=['test_combined_interface','test_emg_integration','test_emg_remote','test_imu_reference','test_imu_reference_ui','test_imu_status','test_recording_layout','test_timing_evidence']
r=unittest.TextTestRunner().run(unittest.defaultTestLoader.loadTestsFromNames(names))
sys.exit(not r.wasSuccessful())
PYTEST
```
