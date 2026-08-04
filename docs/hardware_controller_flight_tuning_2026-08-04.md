# Hardware 外部控制器实飞反馈改进记录

## 基线证据

本轮修改基于实飞包 `/home/hnuter/下载/123` 和 PX4 ULog
`log_113_2026-8-4-11-59-46.ulg`。完整分析、原始文件、参数快照和图表保存在
`reports/hardware_flight_20260804_123/`。

ULog 确认实际飞行固件为：

```text
ver_sw        = e0958bbd93ec1346e42d28d01b8bcf434344699f
ver_sw_branch = codex/hnuter-180deg-tilt-margin
ver_hw        = CUAV_7_NANO
```

飞行时 MAIN8--11 使用 `800/1500/2200 us`，一级舵机范围为 `+/-185 deg`，没有
`HNTR_S2_GEAR` 参数。控制器 CSV 与 ULog 对齐后的姿态 RMSE 小于 `0.1 deg`，
最终 PWM、估计器和电池数据均不支持“输出饱和或状态估计异常”的解释。

## 合并实飞包修改

- 将本次实际飞行使用的 `allocator_force_y_sign=-1` 固化到默认实机配置。
- 二级物理关节限幅保持 `+/-90 deg`，归一化仍等效为 `theta / 90 deg`。
- 水平位置增益使用实飞值 `Kp=[3,3]`、`Kd=[2.1,2.1]`。
- 保留前部单电机和尾电机 `50 N` 推力上限、低油门起飞门控和 3 秒软启动。
- 默认实机调参文件改为 `config/hnuter_direct_hardware_tuning.json`。

## 固件配置隔离

默认配置针对本次实际飞过的旧固件：

```text
profile       = e0958bbd_800_2200
primary       = +/-185 deg
secondary     = joint +/-90 deg
servo PWM     = 800/1500/2200 us
```

`HNTR_S2_GEAR=2.0` 在这个外部配置中用于表达二级舵机轴角到输出关节角的比例，
不表示旧固件具有同名 PX4 参数。

新版 `3131ddd4` 使用独立配置：

```text
config/hnuter_direct_hardware_tuning_3131ddd4.json
primary       = +/-180 deg
secondary     = servo +/-180 deg, gear=2, joint +/-90 deg
servo PWM     = 500/1500/2500 us
```

切换固件时必须显式指定对应文件，不能只修改一两个角度参数。

## 姿态积分修复

实飞时 `Ki=[0,0,0]`，但积分状态仍在约 15 秒内达到三个轴限幅。现在修改为：

- Ki 为零的轴每个控制周期都清零，不再积累隐藏状态。
- 在线改变非零 Ki 时按新旧增益比例缩放积分状态，保持积分力矩连续。
- Ki 从零启用时从零状态开始，避免把历史饱和值瞬间变成控制力矩。
- 未饱和时正常积分；若本次积分增量继续把力矩推向饱和，则撤销该轴增量。
- XY 位置 Ki 为零时同样不再积累无效位置积分。

## 实飞反馈调参

Pitch bias 的内部力矩符号反推与实机观察冲突。以“参数越小尾部越低”的实机
方向为准，下一次短悬停从 `0.09` 小步增加到 `0.10`，不在空中在线修改。

姿态参数从实飞值调整为：

```text
KR        [1.8, 1.8, 3.8] -> [2.1, 2.1, 4.2]
Domega    [1.35,1.35,2.5] -> [1.4, 1.4, 2.6]
Ki        [0,0,0]          -> [0.15,0.18,0.50]
tau_limit [0.9,0.9,1.8]    -> 保持不变
```

遥控响应调整为：

```text
XY filter tau       0.35 s -> 0.25 s
Body-Y max accel    0.55   -> 0.70 m/s^2
Body-X max accel    1.00   -> 保持不变
max horizontal speed 0.60  -> 保持不变
```

基于原飞行姿态误差和角速度的控制律离线回放中，三个轴均未超过现有力矩限幅。
该回放不是闭环实飞预测，只用于排除明显过激的参数组合。

## 运行方式

本次实飞旧固件使用默认配置：

```bash
cd ~/px4_ws_ros2
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
source px4-venv/bin/activate
HNUTER_LOG_DIR=$PWD/hnuter_logs/hardware_recheck \
python3 hnuter_external_direct_controller_hardware.py
```

新版 `3131ddd4` 固件必须显式指定：

```bash
HNUTER_LOG_DIR=$PWD/hnuter_logs/hardware_3131ddd4 \
HNUTER_TUNING_FILE=$PWD/config/hnuter_direct_hardware_tuning_3131ddd4.json \
python3 hnuter_external_direct_controller_hardware.py
```

启动日志必须显示预期的 profile、PWM、一级角度和二级比例。程序仍不发送 Arm、
Disarm 或 Offboard 模式命令。

## 实机验证顺序

1. 拆桨检查 profile、MAIN8--11 中位/方向和四路关节动作。
2. 装桨后只进行低高度短悬停，先观察 `PITCH_BIAS=0.10` 的方向是否正确。
3. 检查姿态积分是否从零平滑增长、是否长期触及限幅以及力矩是否饱和。
4. 姿态确认后再分别测试前后和左右遥控移动。
5. 每次同时保存控制器 CSV、ROS 日志和 PX4 ULog，不同时修改 bias、姿态增益和
   遥控滤波。

本次验证只包含静态检查、单元测试和实飞日志控制律回放，尚未进行新参数实飞。
