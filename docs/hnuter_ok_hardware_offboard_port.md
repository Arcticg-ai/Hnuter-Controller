# Hnuter OK 控制律实机 Offboard 移植

## 目标与边界

新控制器 `hnuter_external_direct_ok_hardware.py` 以远端 `hardware` 分支提交
`0c31ccb` 的实机框架为外壳，将 PX4 标签 `hnuter-ok-144bd9fe` 中经过日志 48
验证的控制器、控制分配器和实际参数移植到 ROS 2 Offboard 直接执行器路径。

这不是对旧固件的逐字复制。旧版本的舵机输出校准已经过时，以下硬件行为明确
保留当前 Hardware 版本：

- 程序不发送 Arm、Disarm 或模式切换命令。
- 仅在 PX4 同时报告 Armed 和 Offboard 后控制。
- Position 模式空中切入时捕获当前位置、航向及新鲜执行器输出，并平滑接管。
- Offboard 关闭再打开时，从当前位置重新开始被中断的任务。
- `ActuatorServos.control` 保持 `[-1, 1]`；PX4 使用 `500/1500/2500 us` 输出。
- 一级舵机轴范围为 `+/-180 deg`；二级采用 `HNTR_S2_GEAR=2.0`，物理关节
  范围为 `+/-90 deg`。

因此“OK”仅描述移植来源。这个新组合尚未经过实机验证，不能继承旧固件的 OK
结论。

## 控制器移植

位置环保持旧固件的级联结构，而不是现有直接位置 PD：

```text
v_sp = v_ff + K_pos * (p_sp - p)
a_unsat = a_ff + K_vel_p * (v_sp - v)
          + velocity_integral - K_vel_d * measured_acceleration
```

水平速度、垂直速度、水平加速度和垂直加速度分别限幅。速度积分使用旧代码相同的
饱和方向判定：当积分会继续推动已饱和输出时暂停该轴积分。水平向量采用范数限幅，
不是逐轴裁剪。

姿态环使用旧固件的矩阵几何误差：

```text
e_R = vee(0.5 * (R_des.T * R - R.T * R_des))
tau = -K_R * e_R - D_omega * e_omega - K_i * integral(e_R)
```

关闭后续版本的四元数误差、reduced-tilt 混合、大姿态 yaw scheduling 和陀螺补偿。
仅 Pitch 轴积分，积分状态限制为 `+/-1.5`。旧参数
`HNTR_ATT_ILIM_P=0.8 Nm` 在该组合下不会成为更紧限制，因为
`0.06 * 1.5 = 0.09 Nm`。

## 控制分配移植

保留旧 Hnuter 解析分配结构及符号约定：

- 每个前臂最大总推力：`170.96 N`。
- 尾电机最大双向推力：`85.48 N`。
- 前电机使用实飞 hover 锚定：悬停单电机推力映射到控制量 `0.50`。
- 推力指数：`0.50`。
- Pitch bias：`0.09`，按归一化尾部俯仰力矩加入。
- `HNTR_TAIL_COMP=0`，尾电机不跟随总升力补偿。

分配后仍使用当前 Hardware 的连续一级倾转分支、舵机速率限制、二级减速比和
Position-to-Offboard 执行器混合。

## 日志 48 参数

独立配置位于 `config/hnuter_direct_ok_hardware_tuning.json`，其中同时记录：

- 来源标签、完整提交哈希和参数快照文件名。
- `HNTR_POS_*`、`HNTR_VEL_*` 级联位置控制参数。
- `HNTR_ATT_*`、`HNTR_TAU_*` 姿态控制参数。
- `HNTR_MAX_*`、`HNTR_MOT_*` 分配与电机参数。
- 当前 `3131ddd4_500_2500_gear2` 舵机硬件映射。

运行时以原始 `HNTR_*` 键为优先来源，并映射到内部 `direct_*` 数组；配置中保留的
`direct_*` 值是便于日志和现有调参工具显示的镜像，不应与 `HNTR_*` 分别修改。

旧 OK 版本的 `800--2200 us`、一级 `+/-185 deg`、自动起飞保护计时、旧着陆低油门
逻辑和 PX4 内部 RateControl 路径没有移植。当前 Offboard 框架在空中接管，且直接
发布执行器命令。

## 运行

```bash
cd ~/px4_ws_ros2
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
source px4-venv/bin/activate

HNUTER_LOG_DIR=$PWD/hnuter_logs/hardware_ok_offboard \
HNUTER_TUNING_FILE=$PWD/config/hnuter_direct_ok_hardware_tuning.json \
python3 hnuter_external_direct_ok_hardware.py
```

## 验证顺序

1. 拆桨核对 MAIN8--11 为 `500/2500/1500 us`、四个舵机方向和机械端点。
2. 不解锁，确认程序不会主动 Arm 或切换 Offboard。
3. 拆桨解锁并切换 Offboard，检查五电机和四舵机接管方向及平滑混合。
4. 安装桨后先做系留或低高度短悬停，不启用大姿态任务。
5. 比较位置、姿态、角速度、力矩限幅、尾电机和倾转日志，再决定是否扩大包线。
