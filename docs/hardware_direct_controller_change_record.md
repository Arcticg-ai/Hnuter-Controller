# Hnuter 实机直接外部控制器修改记录

本文记录 `hnuter_external_direct_controller_hardware.py` 截至控制器仓库提交
`082b6ce` 的实现状态。该文件在提交 `cb156f6` 中成为可独立运行的实机入口，
并在提交 `92006e5` 中完成最近一次实机安全与分配参数更新。

## 独立实机入口

- 不依赖导入其他本地控制器文件，保留姿态、位置、分配、遥控输入、日志和
  Offboard 会话状态机的完整实现。
- 程序不发送 Arm、Disarm 或模式切换命令，只持续发布 Offboard heartbeat。
- 只有 PX4 同时报告 Armed 和 Offboard 后才开始控制；退出 Offboard 或解除
  Armed 后立即结束当前执行会话。
- 程序启动后不自动爬升到固定高度，首次接管位置为当前目标位置。
- 正在执行自动任务时关闭再重新打开 Offboard，会从当前位置重新开始该任务。

## 遥控输入

- 优先读取 RC 来源的 `manual_control_setpoint`，拒绝 MAVLink 注入的同名消息。
- `rc_channels` 作为回退输入；RC 丢失或超时后速度与偏航指令回零并保持当前位置。
- Pitch 映射机体前后速度，Roll 映射机体左右速度，居中 Throttle 映射升降速度，
  Yaw 映射偏航角速度。
- 支持死区、指数、低通滤波、机体系 X/Y 独立速度与加速度限制，以及各通道方向
  环境变量覆盖。

## 实机起飞安全

- 首次 Armed + Offboard 后默认保持电机最小输出，必须先确认油门低位，再向上
  越过触发阈值才允许闭环输出。
- RC 数据中断后必须重新观察到低油门，防止高油门重连直接启动。
- 起飞许可后使用 smoothstep 对电机输出软启动，默认时间为 3 s。
- 完成一次飞行后短暂切换 Offboard 可恢复控制；落地并解除 Armed 后重新启用
  完整低油门起飞门控。
- 日志记录门控状态、许可状态和实时软启动比例。

相关环境变量：

```text
HNUTER_HARDWARE_TAKEOFF_GATE
HNUTER_HARDWARE_TAKEOFF_LOW
HNUTER_HARDWARE_TAKEOFF_TRIGGER
HNUTER_HARDWARE_SPOOL_RAMP_S
```

## 推力与尾电机补偿

- 前部每臂总推力上限为 100 N，单个前电机上限为 50 N，尾电机上限为 50 N。
- 电机归一化采用 `sqrt(thrust / max_thrust)`，与每个执行器的物理推力上限对应，
  不再使用固定 `1000 rad/s` 隐式上限。
- 尾电机支持 `HNTR_PITCH_BIAS`、`HNTR_TAIL_SIGN` 和 `HNTR_TAIL_COMP` 语义；
  Pitch bias 先作为归一化俯仰力矩加入，再按尾电机推力和力臂恢复物理力矩。
- 默认 `HNTR_PITCH_BIAS=0.09`，来自当时最近实飞日志保存参数；三项参数均写入
  启动参数快照和逐行诊断日志。

## 舵机映射历史与当前状态

该文档记录的早期实飞版本直接发布 `ActuatorServos.control[]`，不经过 PX4
内部 Hnuter control allocator。当时的历史映射为：

```text
一级 normalized = primary_joint_angle / 185 deg
二级 normalized = secondary_joint_angle / 180 deg
```

2026-08-12 当前四个实机入口统一适配无延迟固件 profile
`3131ddd4_500_2500_gear2`：

- `hnuter_external_direct_controller_hardware.py`、
  `hnuter_external_direct_ok_hardware.py` 和
  `hnuter_external_direct_drcda_hardware.py` 的四路倾转舵机输入都使用
  `500/1500/2500 us`。该范围不用于电机。
- 一级归一化为 `primary_joint_angle / 180 deg`。
- 二级归一化为
  `secondary_joint_angle * HNTR_S2_GEAR / 180 deg`。
- 默认 `HNTR_S2_GEAR=2.0`，二级物理关节限幅为 `±90 deg`。
- 二级关节 slew rate 改为舵机轴 rate 除以齿轮比。
- 三个直接分配入口分别加载独立的 hardware tuning JSON，启动时
  会记录 profile、PWM 范围、舵机轴角和减速比。
- 电机命令仍为 `ActuatorMotors.control` 归一化推力，使用独立的前电机/尾电机
  推力限幅和可逆设置，不读取 `servo_pwm_*_us`。
- `hnuter_external_controller_px4_position_hardware.py` 只发布 PX4 位置/速度
  setpoint，不发布电机或舵机命令；它记录同一 profile，PWM 映射由
  PX4 内部完成。

## 日志与验证边界

- 日志统一写入 `HNUTER_LOG_DIR` 下的 `hardware/` 目录，并保存启动参数快照。
- 当前测试覆盖 RC 映射、RC 失效、Offboard 任务重启、禁止 VehicleCommand 和
  未解锁 idle 输出。
- 代码没有舵机轴或输出关节的实测角度反馈。机械限位、回差、带载下垂和左右
  不同步只能通过拆桨台架与实飞日志继续核对。
