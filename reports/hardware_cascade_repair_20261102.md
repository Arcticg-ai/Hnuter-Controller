# DRCDA v2 实机外环与遥控响应修订

## 范围

依据2026-11-02第一次接管的机载日志和ULog，修订实机DRCDA v2。
不修改恒姿态升降配平、不修改接管力/力矩补偿、不新增纯延迟模型。
保持手动解锁、手动Offboard、切换位姿捕获、0.5秒过渡及直接电机/舵机输出。
之前远程修复的独立心跳时钟和取消重复vehicle_status依赖已同步至生成模板。
原始版本备份位于本地实验目录output/hardware_before_outer_loop_20261102.tar.gz。

## 实现

### 期望角速度

基类增加默认保持原行为的`_desired_body_rate_for_control`钩子，仅v2覆盖。
手动/解析Euler姿态参考使用完整ZYX Euler导数转换：

```text
omega_FLU = [roll_dot - yaw_dot*sin(pitch),
             pitch_dot*cos(roll) + yaw_dot*sin(roll)*cos(pitch),
             -pitch_dot*sin(roll) + yaw_dot*cos(roll)*cos(pitch)]
omega_des_FRD = diag(1,-1,-1) * omega_FLU
omega_error = omega_measured_FRD - R_measured.T * R_des * omega_des_FRD
```

参考角触限后使用实际参考变化率；偏航使用wrap后的实际参考增量，避免过零尖峰。
捕获矩阵保持阶段保留已有零速参考行为。姿态P/D/I及力矩限幅没有因本修复而扩大。

### 位置—速度串级

按照飞行固件`acff025af33f45e9f4b889ceb2686d6e5bc1d270`中的
`src/modules/mc_rate_control/HnuterControl.cpp`实现常规位置模式的结构：

```text
v_sp = velocity_reference + K_position * position_error
a_raw = acceleration_reference + K_velocity * (v_sp - v) + I_velocity - D_acceleration * a_measured
```

`HardwareVelocityCascade`持有积分和诊断状态，默认NED增益如下：

| 配置 | x/y/z |
|---|---|
| cascade_position_p_ned | 3.75 / 3.75 / 3.5 |
| cascade_velocity_p_ned | 9.05 / 9.05 / 4.0 |
| cascade_velocity_i_ned | 0.2 / 0.2 / 0.2 |
| cascade_acceleration_d_ned | 0.36 / 0.36 / 0.4 |
| cascade_integral_limit_ned | 1.5 / 1.5 / 2.5 |

速度约束为水平模长3m/s、向上1.5m/s、向下1m/s，实际摇杆速度上限仍每轴0.3m/s。
水平加速度保留原Offboard的3m/s²，不直接套用飞控6m/s²；竖直保留8m/s²上限，
并结合重力和前电机最大推力约束。它是串级结构和增益对齐，不是逐字复制固件的所有模式和限幅。

积分单位为加速度；按速度误差积分。请求超加速度范围时禁止向饱和方向继续积分，
水平积分按模长限幅，竖直独立限幅。接管/退出重置，0.5秒过渡期间冻结。
关闭父类旧位置误差积分，避免两个积分器重复作用；旧`direct_pos_*`字段仅保留兼容，
v2使用`cascade_*`字段调参。该抗饱和对应外环加速度约束，不等同于真实执行器扳手反馈。

加速度优先读取`vehicle_local_position.ax/ay/az`，保留NED坐标。
字段不可用时，使用新位置消息的速度差分和0.1秒滤波，不在每个姿态回调重复差分。
增加日志字段`v2_cascade_json`、`v2_acceleration_source`、`v2_desired_body_rate_frd_json`。

### 遥控参数

| 参数 | 旧 | 新 |
|---|---|---|
| 水平命令滤波 | 0.2s | 0.12s |
| 水平命令加速度 | 0.3m/s² | 0.6m/s² |
| 通用命令滤波 | 原代码默认0.25s | 显式0.12s |
| 横滚/俯仰参考变化率 | 5 / 5 deg/s | 8 / 6 deg/s |
| 水平摇杆速度 | 0.3 / 0.3m/s | 不变 |

通用滤波影响竖直、偏航和姿态速率命令。没有改变通道符号、死区、expo或姿态角范围。
参考位置软限距和连续性修复保留。

## 文件与使用

- 生成后的独立入口：`controllers/hardware/hnuter_external_direct_drcda_v2_hardware.py`。
- 本次实机专用配置：`config/hardware/hnuter_drcda_v2_hardware_20261102_cascade.json`。
- 默认配置：`config/hardware/hnuter_drcda_v2_hardware_tuning.json`。
- 源模板：`tools/templates/drcda_hardware_v2.py.in`。

实机专用配置从本次runtime快照迁移，保留`HNTR_PITCH_BIAS=0.09`及俯仰阻尼1.5等实机标定；
仓库通用默认值不应替代这一机体标定。旧JSON缺少新的必需字段将明确报错，不静默使用不同外环。

迁移其他机体配置时使用`tools/migrate_hardware_cascade_tuning.py SOURCE NEW_OUTPUT`，
工具只生成新文件，不覆盖源配置。需同时分发新独立入口和完整JSON，运行时显式指定`HNUTER_TUNING_FILE`。

## 验证边界

离线测试覆盖Euler/机体系角速度映射、手动俯仰速率进入实际力矩反馈、
串级公式、速度/加速度/积分限幅、抗饱和、切换重置、加速度来源、配置迁移及心跳修复。
理想点质量扰动抑制测试只验证积分算法，不是Gazebo、PX4或实机效果证明。
硬件回归使用模拟ROS发布器，实际并未连接飞控；真实ROS接收测试默认跳过。
最终运行`pytest tests/hardware -q`：233 passed、1 skipped；独立脚本构建一致性检查与`git diff --check`通过。

未运行Gazebo或实机飞行，未部署/重启远程控制器，未提交或推送Git。
串级外环反馈显著强于旧PD，必须把本版当作待验证候选，而不是已证明改善的飞行版本。
升力变化时的俯仰配平问题按用户要求留待后续，不能宣称本次修改消除了该耦合。
