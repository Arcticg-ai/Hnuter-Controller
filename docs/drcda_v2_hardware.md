# DRCDA 实机直接执行器 Offboard v2

日期：2026-10-13。新增文件，不覆盖原来的 hardware、旧硬件 DRCDA 或仿真入口。

## 文件与验证边界

- 运行入口：`controllers/hardware/hnuter_external_direct_drcda_v2_hardware.py`。
- 参数模板：`config/hardware/hnuter_drcda_v2_hardware_tuning.json`。
- 可维护的增量实现：`tools/templates/drcda_hardware_v2.py.in`。
- 固定的分配器源码快照：`tools/templates/drcda_hardware_v2_allocator.py.in`。
- 打包工具：`tools/build_hardware_drcda_v2.py`，生成可独立运行的单文件。
- 离线测试：`tests/hardware/test_drcda_v2_hardware.py`。

运行文件内嵌硬件控制器、RC 解析、分配器和完整默认参数，不需要 import 仓库内的其他 Python 文件。
仍需要 ROS 2、与固件匹配的 px4_msgs、NumPy；启用真实关节反馈才需要 sensor_msgs。
首次运行默认参数写入 `HNUTER_LOG_DIR/config/`。因此单独部署这个 Python 文件也能启动，
但内嵌参数是已有硬件版本的参考标定，不代表程序已读取或确认当前飞行器参数。

本次仅执行离线测试，ROS 发布器、订阅和定时器在测试中均被替换；没有连接、解锁或驱动实机，
没有修改固件、没有飞行验证。旧硬件入口保持原样。
文件名的 v2 表示此硬件封装版本，不等于 `controllers/experiments/drcda_v2` 仿真实验配置。

## 控制链路与算法

遥控输入 -> 机体系速度/姿态参考 -> 现有位置与姿态控制器 -> 期望扳手 ->
DRCDA -> 五个电机归一化输入及四个舵机归一化输入。

实际分配在本 Python 程序内完成，发送 `/fmu/in/actuator_motors` 和
`/fmu/in/actuator_servos`，`OffboardControlMode.direct_actuator=true`。
不向 PX4 发送位置、速度、姿态或力矩目标来让 PX4 代做控制分配。
保留原硬件代码的 `vehicle_thrust_setpoint` 着陆检测提示：它是总推力幅值提示，
不是控制指令链路；`thrust_and_torque=false`，不发送 torque setpoint。

分配器使用当前共享实现的有界 Gauss–Newton、move blocking、0.10 s 预测，
在 0.05/0.10 s 两个点优化，权重为 0.4/1.0，并包含近时刻扳手约束的软惩罚、
命令变化和偏好正则项。执行器角度、命令速率、电机推力和正反转限制仍生效。
近时刻项是加权软目标，不是保证扳手可实现的硬约束。

**本版默认为 no-preview 实机配置**：两个未来采样点均保持当前期望扳手。
没有将仿真 Full 的未来参考预测直接用于遥控飞行，也未移植 Gazebo SDF、磁场模型或关节 PID 标定。
真实 RC 未来输入未知，不能声称其是有真实关节反馈和未来轨迹预览的完整仿真 Full。

所有纯延迟均为零。默认舵机模型仅有角度/速率限制，增益为 1，时间常数为 0；
这是未知动态下的近似，不表示真实舵机无动态。提供实机阶跃辨识后，可填写
`hardware_servo_tau_positive_s/negative_s`，按 `[aL,bL,aR,bR]` 顺序，单位秒。
不能填 Gazebo 标定值冒充实机辨识。电机时间常数默认同样为 0，保留变化率限制。

保留原外环积分及其原有饱和处理；本版额外的分配残差 back-calculation 系数为 0，
不在首次硬件接入时启用尚未完成实机符号/时序验证的分配残差积分补偿。

## 硬件映射

| 项目 | 参考值/约定 |
| --- | --- |
| 质量 | 4.8 kg |
| 前臂推力参数 | HNTR_MAX_ARM_T=170.96；最终每电机上限还受悬停锚点映射可达范围约束 |
| 尾电机 | +12.78 / -6.04 N；正反指数 0.55 / 0.68 |
| 几何 | L1=0.33 m，L2=0.72 m，CG_Z=0 |
| 尾部偏置 | HNTR_PITCH_BIAS=0.07；沿用硬件版扳手偏置函数，仅应用一次 |
| 舵机 PWM | 500 / 1500 / 2500 us，仅用于舵机 |
| 一级舵机 | 轴角 ±180°，归一化命令 ±1 |
| 二级舵机 | 轴角 ±180°，减速比 2，物理关节角 ±90° |
| 舵机速率 | 轴角 4.7 rad/s；二级关节对应 2.35 rad/s |
| 电机控制量 | 前四个 [0,1]；可逆尾电机 [-1,1]，不是 500–2500 |
| PX4 电机顺序 | R1,R2,L1,L2,tail |
| PX4 舵机顺序 | aR,aL,bR,bL |

以上参数必须对照本次飞行器参数导出、推力标定和实际减速比核对。
`actuator_servos.control` 中仍发送 [-1,1]，不是直接填 500 或 2500；PWM 输出由固件驱动层完成。
前/尾电机沿用原硬件推力曲线，不能把舵机 PWM 范围用于电机。

## 接管、遥控和失效处理

- 默认监视，不发布心跳或执行器输入；必须显式设置 `HNUTER_HARDWARE_DRCDA_ENABLE=1` 才允许输出。
- 不发送 VehicleCommand，不自动 Arm/Disarm、不自动切 Offboard、不自动起飞。
- 先由飞手用 Position 起飞，确认稳定后手动切 Offboard；必须同时收到 Armed、Offboard、有效 RC 和新鲜估计状态。
- 接管要求新鲜的 PX4 电机与舵机命令快照，捕获当前位置/高度/航向，0.8 s 平滑混合；混合期间忽略遥控移动。
- 接管后同步的是实际发布的混合命令，不再让预测器传播未发布的原优化命令。
- 主杆按已有 RC 映射转为机体系水平速度、垂向速度、偏航率；AUX1/AUX2 为横滚/俯仰参考变化率。
- 默认水平速度上限各 0.3 m/s，姿态命令限幅 15°、变化率 5°/s；不是经过实机优化的性能参数。
- 高度以接管时为零，允许负的相对高度；松杆不会自动返回接管高度。此相对坐标不提供地面高度或防撞保证。
- 实机姿态使用 PX4 NED/FRD 四元数，不使用旧仿真中选择等效 180° 翻转坐标系的启发式处理。
- 状态超时：位置 0.30 s、姿态/角速度 0.15 s、状态/模式 1.5 s、RC 0.5 s；接管快照 0.30 s。
- 控制采样间隔超过 50 ms、估计器重置、非法数值、优化失败、超姿态/速度保护会锁存退出。
- 保护不依赖“比接管点高 0.15 m”，也不因处于姿态任务而绕过；默认总倾角保护 30°。
- 故障时停止外部执行器指令及心跳，交由飞手接管和固件 Offboard-loss failsafe；**不是主动发送电机归零或自动 Disarm**。
- 故障不会自动重启控制，恢复 Position/落地并排查后重新启动节点。普通人工关闭/打开 Offboard 则重新捕获当前位置。

必须预先确认固件的 Offboard 丢失超时与失效动作、遥控模式切换和失联处理。
停止心跳并不保证立即或无扰地切回 Position，具体行为由固件参数和状态决定。
应先拆桨检查，再台架、系留验证，不能把离线测试通过当成可自由飞行许可。

## 自动任务

默认禁用键盘轨迹，避免误触大范围任务。需要测试相对轨迹时单独设置
`HNUTER_HARDWARE_TASKS=1`：键 1 为矩形，2 为李萨如，3 为姿态。
应先核对程序中轨迹幅值、周期与场地，默认任务参数不构成安全飞行边界。
姿态模板已改为 10°、15 s 变化、5 s 稳定，不带仿真 90°/180°动作。
任务从启动时实测位置开始；关掉 Offboard 再打开，任务从新的当前位置重新开始，而不是恢复旧绝对坐标。
仍适用上述 30°总倾角保护，不要用于大姿态实验。

## 可选真实关节反馈

默认只有命令模型预测，PX4 `/fmu/out/actuator_servos` 只用于接管，**不是实测角度**。
只有另一个已标定驱动提供真实关节传感器数据时，才设置：

```bash
export HNUTER_HARDWARE_JOINT_TOPIC=/your_calibrated_joint_states
export HNUTER_HARDWARE_JOINT_NAMES='["aL", "bL", "aR", "bR"]'
```

该接口类型为 `sensor_msgs/msg/JointState`，名字必须替换成驱动实际名字。
`position` 必须是**减速后物理关节角，弧度，已经校准零位与正方向**，不能直接喂电位器值、PWM 或二级舵机轴角。
时间戳须为与本节点同步的 ROS 时钟，不是 PX4 启动微秒；接收时最大陈旧 0.15 s、最多超前 0.02 s。
名字缺失/重复、越界、无效时间戳或超时阻止接管；控制中发生则触发故障退出。
启用后每次分配先传播状态，再以实测四关节角覆盖初值。电机推力仍是模型估计，不是测力反馈。
仓库未提供这个真实传感器驱动，不能通过转发执行器命令伪造此接口。

## 运行

DDS Agent 必须与真实飞控连接，且 px4_msgs 与固件版本匹配。
需要现有硬件入口使用的 `/fmu/out/vehicle_local_position_v1`、`vehicle_attitude`、
`vehicle_angular_velocity`、`vehicle_status_v1`、`vehicle_control_mode`、
`actuator_motors`、`actuator_servos`，以及有效 RC 输入 `manual_control_setpoint` 或 `rc_channels`。
缺失时不会接管，状态输出会显示具体 gate 原因。禁止同时运行另一个 Offboard 控制器。
默认 DDS 仅本机发现；Agent 和控制器在同一伴随计算机运行即可。

先监视，不输出：

```bash
source /opt/ros/jazzy/setup.bash
source /home/hnuter/px4_ws_ros2/install/setup.bash
cd /home/hnuter/px4_ws_ros2_consolidated
export HNUTER_LOG_DIR="$PWD/hnuter_logs/hardware_drcda_v2/$(date +%Y%m%d_%H%M%S)"
export HNUTER_TUNING_FILE="$PWD/config/hardware/hnuter_drcda_v2_hardware_tuning.json"
HNUTER_HARDWARE_DRCDA_ENABLE=0 python3 controllers/hardware/hnuter_external_direct_drcda_v2_hardware.py
```

停止监视节点、核对参数与拆桨检查条件后，使用同一环境显式允许输出：

```bash
HNUTER_HARDWARE_DRCDA_ENABLE=1 python3 controllers/hardware/hnuter_external_direct_drcda_v2_hardware.py
```

设置为 1 不会自动解锁，仍须飞手/PX4 满足全部门控；但满足后会直接驱动电机舵机。
这不是建议立即进行自由飞行。配置在本次运行中冻结，修改后须退出 Offboard 并重启程序。
默认参数缺失时会生成完整模板；自定义参数文件不完整/非法则启动失败，不静默回退。

所有日志在 `HNUTER_LOG_DIR` 内，包括 ROS 日志、CSV 和 `config/drcda_v2.runtime.json`。
运行快照包含来源哈希、完整配置、最终分配器参数、反馈来源和输出开关。

## 构建与离线复查

```bash
python3 tools/build_hardware_drcda_v2.py
python3 tools/build_hardware_drcda_v2.py --check
source /opt/ros/jazzy/setup.bash
source /home/hnuter/px4_ws_ros2/install/setup.bash
PYTHONPATH="$PWD:$PYTHONPATH" python3 -m pytest tests/hardware tests/common/test_hnuter_drcda.py -q
```

生成器保留硬件版的几何与电机标定，嵌入已固定的分配器源码快照，并记录四个输入文件 SHA-256。
快照来自本次验证的共享分配器，包括有界优化与近时刻扳手项；运行时不会加载公共模块。
这样不必提交其他仿真修改，干净克隆也能重建相同的实机单文件。
后续修改公共分配器不会自动影响实机代码；需要人工审查并更新快照，再重新生成与运行测试。
修改增量模板后同样须重新生成，不能只修改生成文件而忘记源模板。
2026-10-13 执行以上测试：145 passed，3 条既有 IEBC 依赖弃用警告。
还将生成文件复制到 `/tmp` 单独加载，验证不依赖仓库目录深度和本地 Python 模块；没有启动 ROS 节点。
`build_hardware_drcda_v2.py --check` 与 `git diff --check` 均通过。
证据级别为离线单元/模拟消息回调，不是 Gazebo、台架或实机飞行。
