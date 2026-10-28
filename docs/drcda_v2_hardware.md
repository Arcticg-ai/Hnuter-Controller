# DRCDA 实机直接执行器 Offboard v2

## 2026-10-28 日志持久化不再阻塞接管

Gazebo 验证中发现，接管事件的同步 `fsync` 可以阻塞控制线程，令仍在接收的
新状态来不及被控制循环消费，进而触发时效故障。事件现先复制并序列化，再放入
512 条有界队列，由独立线程写盘、flush 和 fsync；控制线程不等待磁盘。
队列溢出/磁盘错误有明确告警，不伪装成记录成功，也不改变姿态/角速度时效限制。
正常退出会等待日志排空（最多 2 s）；强制断电或 `kill -9` 仍可能丢失未落盘事件。
CSV 诊断仍使用原有记录路径和频率，不把日志回显当作执行器实测反馈。

## 2026-10-27 遥控参考连续性修复

手动解锁、手动切 Offboard、切入时完整实测位姿参考和 0.5 s 接管混合不变。
本次没有新增姿态门槛，也不主动拉平；严重数据/数值故障处理沿用下文已批准策略。

原先的位置限距会把参考点投影回当前位置附近，再将投影差分成速度。
0.7 m 偏差、0.6 m 限距、15 ms 周期、0.0001 m/s 输入即可制造约 6.67 m/s
速度前馈。现在 `_advance_manual_position_reference` 不投影/重定位参考点：

- 只抑制使目标继续远离飞机的输入速度，保留指向飞机的人工输入；零输入不随飞机漂移改目标。
- 接近限距时按剩余距离减缓向外的输入，随后施加速度与加速度限制并积分位置。
- 位置、速度、加速度来自同一离散参考状态，不将几何校正误认为遥控运动。
- 水平速度/加速度模长限制来自已配置两轴 RC 上限的范数；垂直速度沿用 RC 上限，垂直参考加速度为 0.5 m/s²。
- 限距是柔性输入限制：飞机被外力拉走或参考正在减速时，距离可以暂时超限，不能通过跳目标伪造跟踪改善。
- 每次接管重置参考速度和诊断节流时间，CSV 增加 `v2_control_session_id`，避免短的第二次会话无日志。

实机前挂点钢绳的拉力/俯仰力矩尚未辨识。接管平衡补偿不等于测得的外扰，
本次不因偏航相关漂移就把全部力补偿改成机体系，也不删除原补偿产生新的接管跳变。
仿真调参与实机物理标定必须分开记录；固件侧 Position 接回修复需要相应固件，
仅更新机载 Python 文件不能改变飞控内部接回逻辑。

## 2026-10-27 当前修复策略

本节替代下面历史记录中的30°退出和“只停发等Offboard-loss”策略。
正常流程仍只由飞手手动解锁、手动切Offboard；程序不自动Arm、Disarm、切Offboard或起飞。
等待阶段持续发20 Hz `OffboardControlMode(direct_actuator=true)`，不发执行器命令。
Armed+Offboard后捕获**实测位置和完整姿态**，不要求姿态接近水平、不裁剪到普通遥控角度，
正常控制不因倾角大小或旧参考差异退出。保留真实执行器角度、速率、推力和力矩限幅。

接管同时由新鲜执行器命令快照重建名义扳手，计算它与首个新控制请求的差：
力差转换到世界系保存，力矩差作为本次会话机体系配平保存。其后反馈项照常叠加，
该配平不在0.5秒后强行消失；普通手动退出/重新切入时清空并重新捕获。
这不是测得的负载或CG参数，不能当作已完成实机标定。没有有效快照时仍沿用旧的无快照路径，
但事件/诊断中无法声明扳手连续；缺少实际执行器角反馈的边界没有改变。

0.5秒执行器混合、实测状态校正、0.1秒分配时域、有界求解、无纯延迟策略均保留。
当前RC零输入会保持捕获姿态，即便它超出普通手动倾转范围；不主动拉回水平。

只有关键状态丢失/非法、估计器坐标重置、非有限数值或分配计算失败才进入硬故障交接：

1. 立即经DDS `/fmu/in/vehicle_command`发送 `VEHICLE_CMD_DO_SET_MODE`，
   `param1=1, param2=3, param3=0`，即请求Position，不是MAVLink链路，不涉及飞控参数。
2. 请求期间不发布无效执行器，也不发送空闲/零电机命令；短暂继续心跳并每0.1秒重试。
   以新的实际模式/解锁反馈确认已经退出Offboard，单独ACK不能当作切换完成。
3. 若0.35秒仍无反馈确认，明确报错并继续Position请求，同时允许原有PX4 Offboard-loss兜底。
   不能无期限保活、把飞机留在冻结PWM的直接模式中。该超时仅适用于硬故障，不是正常姿态门槛。
4. 硬故障仍锁存，需要排查并重启程序；普通手动Position/Offboard切换不锁存故障。

这能消除“只有停发，没有主动请求交接”的程序缺陷，但**不能保证飞控一定接受Position**。
定位无效、链路断开、进程崩溃/`kill -9`时，机载程序无法完成握手，飞控自身failsafe仍决定输出。
尤其当次固件Position中的Hnuter旧倾转目标在Offboard期间仍保留；仅改机载程序不能保证
反向切回Position时固件内部姿态目标也自动无扰重置。本次不改固件或飞控参数，不能宣称双向飞行已验证。

本轮完成原版77项、修复版93项ROS实体全Mock的离线测试，以及固定初始状态的接管回放。
回放中尾电机不再从0.652下降至0.360，修复版保持0.652；这不是闭环动力学仿真或飞行验证。
本次不启动控制器，不自动运行任何DDS节点。

## 历史策略

2026-10-24 当前策略（替代以下旧接管策略）：启动后持续发送 Offboard 心跳，
人工 Armed + Offboard 触发控制；切换瞬间捕获实测位置和完整姿态作为参考，
默认 0.5 s 从已有执行器输出平滑过渡。普通关闭/重开 Offboard 会重新捕获位姿。
不再要求 PX4 `hntr_att` 目标、5°误差、普通遥控参考范围、模型扳手一致性或
参数快照匹配；执行器快照只用于存在时的平滑混合，缺失不拒绝启动控制。
参数读回只供诊断。没有实测位姿时不虚构参考，等待首个位姿；控制中的非法数据、
输入中断、求解失败和实际输出约束继续保护。程序不自动解锁、不切模式、不起飞。

以下日期记录保留历史诊断；涉及旧门槛、旧参考来源或“未部署”的描述仅代表当日状态。

**2026-10-22 历史状态：修复版已部署到机载主机，完成无飞控连接的 DDS 心跳验证；尚未完成修复后的实机飞行验证。**

2026-10-22：等待阶段心跳与接管门禁分离，采用原有实机入口的独立 20 Hz 定时器。
显式设置 `HNUTER_HARDWARE_DRCDA_ENABLE=1` 后，即使未连接飞控、尚无遥控或状态、
参数尚未核对、未解锁或未进入 Offboard，也持续发送 `OffboardControlMode`。
这只是 `direct_actuator=true` 的 DDS proof-of-life，不发送任何执行器或轨迹指令。
电机/舵机仍仅在 Armed + Offboard 以及接管检查通过后发布，不自动解锁或切模式。
若实际进入 Armed+Offboard 时无法接管，或接管后出现故障，锁存报错并停止心跳与
执行器指令，不用心跳长期掩盖控制失效。监视模式 `ENABLE=0` 不发送心跳或控制。
注意：心跳存在只代表外部程序在线，不表示接管条件已满足。缺少参数快照不再导致
程序启动失败，而是等待阶段持续心跳、执行器门禁保持关闭。

受限实机验证另有显式 `HNUTER_HARDWARE_BOUNDED_TEST=1` 选项，只用于飞手已确认
可靠龙门架/系留约束的试验：接受所选配置而不强制要求实时参数快照，运行记录仍明确
`firmware_parameters_verified=false`，不会把历史参数冒充今日读回；模型扳手差距仅记录
诊断，不阻断这类受限接管。默认模式不变。输出另需 `HNUTER_HARDWARE_DRCDA_ENABLE=1`。
这不是仅监听模式，会在人工 Armed+Offboard 且关键状态有效后直接驱动执行器。
受限模式中，“姿态跟踪已小于 5°”及“旧参考已在普通手动角度范围内”同样仅记录
诊断，不再用于阻断心跳/接管；真实倾角和参考仍不得超过物理安全倾角范围。
接续的 PX4 姿态目标若在普通 15°手动范围之外，不突然裁剪到 15°，只允许遥控
将它逐渐向范围内收回，避免接管后的下一周期再次产生参考跳变。
遥控失联、估计状态超时/重置、姿态保护、输入/输出有限值与范围检查、真实姿态目标
接续及其有效性检查均保留；自动任务默认禁用，不自动解锁、上锁或切模式。
受限试验仍不能当作自由飞行验证，切回 Position/kill 等独立人工处置由飞手负责。

本次本地硬件与公共分配器回归共 185 项通过，机载主机 v2 专项测试 73 项通过。
机载主机未连接飞控、雷达，实际控制器与独立只订阅 DDS 进程同时运行：
12.024 s 观察窗口收到 169 条心跳，发现完成后的频率为 20.001 Hz，最大间隔
51.87 ms；电机、舵机、轨迹、力、力矩和 VehicleCommand 消息均为 0 条。
试验使用 `ENABLE=1`、`BOUNDED_TEST=0`、无参数快照、自动任务关闭，验证了
等待阶段的心跳不依赖接管条件。验证结束后控制器和订阅进程均已停止。
本次不包含真实飞控切入 Offboard 或带桨接管验证。两台主机的单文件重建检查通过。

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
| 几何 | L1=0.33 m，L2=0.72 m，CG_X=0.013 m，CG_Z=0；来自 2026-10-16 ULog |
| 前电机悬停锚点 | HNTR_MOT_HOV=0.58，不再使用旧的 0.50 |
| 尾部补偿 | HNTR_PITCH_BIAS=0.07；和姿态相关重力补偿一起，在物理力矩限幅前加入 |
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

- 默认监视，不发布心跳或执行器输入；显式 `ENABLE=1` 持续发送心跳，人工 Armed+Offboard 接管。
- 不发送 VehicleCommand，不自动 Arm/Disarm、不自动切 Offboard、不自动起飞。
- 飞手控制解锁和模式开关；捕获切换时实测位置/高度及完整姿态作为参考。
- 已有有效执行器输出时进行 0.5 s 平滑混合；没有快照则使用分配器正常初始化，混合期间忽略遥控移动。
- 不依据旧姿态目标、姿态偏差、普通遥控角度范围、模型扳手差距或参数快照匹配拒绝接管。
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
可选 `actuator_motors`、`actuator_servos` 用于初始化过渡；遥控使用
`manual_control_setpoint` 或 `rc_channels`。`debug_vect/hntr_att` 仅保留历史诊断，
控制参考来自切换时的实测位姿。禁止同时运行另一个 Offboard 控制器。
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

停止监视节点后，使用同一环境显式允许人工模式开关触发输出：

```bash
HNUTER_HARDWARE_DRCDA_ENABLE=1 python3 controllers/hardware/hnuter_external_direct_drcda_v2_hardware.py
```

设置为 1 不会自动解锁，仍须飞手/PX4 手动切模式；Armed+Offboard 后直接驱动电机舵机。
可选参数读回 `HNUTER_HARDWARE_FIRMWARE_PARAMS` 仅供诊断，不阻止接管。
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

## 2026-10-16 接管故障修复

实机日志记录：进入 Offboard 后 28.21 ms，触发
`invalid or reset telemetry: motors`，CSV 只保留一帧控制结果。
问题所在是 DDS `/fmu/out/actuator_motors` 命令回传的处理，不是 MAVLink 控制消息。
网页调参服务读取参数使用 MAVLink，与本控制器的 DDS 直接控制链路分开。

修复前，执行器命令回传和位置、姿态等传感器状态共用严格的时间戳/有效值检查。
切入直接控制后，这些共享 uORB 话题也会回传本节点发布的命令，可能出现较旧时间戳或无效过渡输出；
不能把它们等同于电机传感器故障而立刻撤销控制。这次旧日志没有记录原始异常帧，
无法进一步区分是非有限值还是时间戳回退，不能声称已证实其中某一种。

当前修改：

- 接管前仍严格要求新鲜、有限、范围有效的 PX4 执行器快照。
- 接管时检查电机 `reversible_flags` 与本节点一致，避免同一数值被不同方式解释。
- 控制中的无效/重复/旧执行器命令回传不覆盖预测器、不触发关键状态故障；预测器仍使用实际发布的命令。
- 退出后重新接管需要新的 Position 执行器回传，不能复用上一会话的缓存。
- 保留位置、姿态、角速度、RC、解锁/模式状态的有效性与超时检查。
- 在 `events/drcda_v2.jsonl` 记录接管快照、前三次实际发布、被丢弃回传的原始值/时间戳/原因，以及故障前状态。

本次离线回归：154 passed，3 条既有依赖弃用警告；新增测试覆盖执行器时间戳回退、重复和 NaN
不会中断有效控制，以及坏快照仍阻止首次接管。没有通过再次实机飞行验证。

**停止指令和心跳并不等于 PX4 立即切回闭环 Position。** 本次控制撤销后约 0.73 s 的
控制台样本仍显示 Offboard，约 1.73 s 样本才显示 Position；具体退出原因、实际 PWM、执行器保持时间、
以及姿态估计是否正常，都仍需本次 ULog 核对。不能把单独修复回传误判当成已经证明飞行问题完全解决。
## 2026-10-16 Historical Hypothesis (Superseded Below)

The echo timestamp fix did not resolve the real flight problem. The next run
controlled for 1.78 s, then withdrew at the 30-degree attitude envelope.
Pitch changed from -3.85 to -28.40 deg ENU in the recorded 17 rows. Tail command
fell from 0.6043 to 0.4646 during handover. This is a failed flight test.

Local fixes (not deployed/restarted on the aircraft):

- Add `HNTR_CG_X/Z` attitude-dependent gravity trim and `HNTR_PITCH_BIAS` in
  physical FRD torque **before** saturation and integral saturation checks,
  matching local `HnuterControl.cpp`. Apply the tail sign only once afterward.
- Hold the optimizer solution separately from the actually published blended
  command. Non-solve callbacks no longer blend an already-blended command.
- Require a current-aircraft parameter readback before enabling output.
  `HNUTER_HARDWARE_DRCDA_ENABLE=1` alone is no longer sufficient.

`HNTR_CG_X=0.055` in the example configuration comes from the local firmware
source default, **not** a verified readback of the aircraft. Do not copy defaults
into a supposed readback. The example pitch limit remains 3 N m: its gravity
plus bias trim would already be 3.234 N m at level. The readback check rejects
that combination rather than silently increasing the torque limit.

The optional `HNUTER_HARDWARE_FIRMWARE_PARAMS` path accepts JSON containing a
parameter-name/value object, either at top level or under `parameters`; a list
of `{name, value}` records under `parameters` is also accepted. Values must come
from the current aircraft. Geometry, thrust maps, bias, rates, gear and attitude
P/D/I/torque limits must match the controller configuration. A readback match is
necessary but **not sufficient flight validation**: output functions, servo
direction/zero/travel, motor mapping and physical response still require
prop-off/bench/tethered verification. No automatic arm/disarm/mode changes were
added. No pure delay was added. Monitor mode does not need a readback file.

The current flight's ULog is still required to audit physical outputs, current
firmware/parameters and estimator status. Command-model residuals are not
measured wrench accuracy. Remote controller was stopped after the failure;
this local candidate has only offline validation, not Gazebo or flight proof.

## 2026-10-17 ULog 核对与本地修复

核对了龙门架目录的 7 个 ULog，其中 log_6/log_7 对应两次 Offboard 接管。
记录的固件为 `acff025af33f45e9f4b889ceb2686d6e5bc1d270`，不是简单假设当前本地 main。
没有切换或修改固件仓库；按该提交源码核对 DDS 和控制语义。

更正昨日假设：实际 CG-X 是 0.013 m，重力补偿约 0.61 N m，不是用源码默认
0.055 m 算出的约 2.58 N m。不能将数值碰巧接近命令缺口当作重心辨识。
第二次切入前 Hnuter 的俯仰目标约 -23.97° FRD，实际约 +3.86°，误差约 27.83°；
旧外部代码重置目标为 0°，使原有 P 项与配平命令大幅丢失。前电机悬停锚点
实际为 0.58，部署配置为 0.50，同归一化命令的推力估计比例约为 0.743。

当前配置按该次 ULog 对齐 CG-X、悬停锚点与姿态 P/D/I/力矩上限；这不是任意调大
增益。RC 速度/姿态限幅仍保守保留，位置环仍是现有外部 PD+I，未声称等同固件
级联位置/速度环。不同前级产生的扳手差异必须通过接管前一致性检查，不能隐藏。

新增 `/fmu/out/debug_vect` 订阅，仅接收 `name=hntr_att`，x/y/z 为固件 FRD
目标欧拉角（度），正确转换为 ENU/FLU 参考。有效期 0.15 s，切入时保留该目标；
退出后必须收到新的 PX4 目标才能再接管。acff025 固件已导出该 DDS 话题，不需改固件。

新增接管门禁：SO(3) 参考跟踪误差 <=5°；参考在本模式 RC 角度范围内；捕获
命令的模型扳手与新控制器请求逐轴差距不超过 Fx/Fy/Fz=[3,3,5] N、
Tx/Ty/Tz=[0.5,0.5,0.3] N m。它检查两侧控制条件的一致性，不是保证实机扳手
可实现的定理。这些门禁不改变 DRCDA 优化目标，不给 Direct 对比添加不同轨迹。

参数快照检查额外覆盖输出函数、舵机角范围、PWM 映射/反向位、CA_R_REV 与
THR_MDL_FAC；本配置针对日志中的具体接线，其他接线应单独审查，不能随便绕过。
ULog 导出的文件是**历史参数读回**，不是实时连接确认，不能自动认作今日实机配置。

已完成：171 项离线测试，3 项既有依赖警告；两次真实日志状态的完全离线回放都
在首条执行器指令前拒绝不合适的接管状态，发布次数为 0；稳定合成状态和非零参考
的接管、参考持续保持及混合路径通过回归。生成单文件与四个来源输入一致。
**没有启动实机或 Gazebo，没有部署或推送，不能据此声称修复后飞行稳定。**

ULog 未见这两个接管窗口发生估计器重置；内部姿态/角速度/分配使能均为 0，
没有内部控制器同时分配的证据。尾部及舵机驱动 PWM 与实际发布指令匹配。
第二次撤销后约 1 s 才触发 Offboard-loss（COM_OF_LOSS_T=1），期间最后命令保持。
没有擅自修改飞控超时、失效动作或发送自动模式命令。
# State receive/compute separation (2026-10-27)

The v2 entry now creates a dedicated `hnuter_drcda_v2_state_receiver` node and
single-threaded executor for position, attitude and angular velocity. Its
subscriptions use volatile, best-effort `KEEP_LAST(1)` and only replace entries
in a locked three-slot inbox. They never run the allocator or mutate controller
state. The main executor takes a coherent snapshot at most once per new attitude
publication; it updates position and rates before attitude triggers control.
Messages received during a solve remain available for the next snapshot instead
of creating a backlog of control jobs. Existing event/status subscriptions stay
on the control executor.

The initial software freshness bounds are 75 ms for attitude/rates, 200 ms for
position, 30 ms maximum rates/attitude sample skew and 100 ms position/attitude
sample skew. Prefer `timestamp_sample`; fall back to `timestamp` only when the
sample field is absent or zero. A temporarily skewed snapshot waits for a matching
latest sample. The age watchdog on the last used state continues to run, and an
expired state follows the existing Position-request fault path. The same check
runs before takeover and again before publishing a solved actuator command.
These are rejection bounds, not a declaration that those delays are stable for
flight or permission to increase gains.

Receive time is recorded in the receive executor and is not renewed when the
control executor later processes the message. Duplicates do not refresh it.
The minimum observed `receive_monotonic - PX4_publication_timestamp` estimates the
clock offset **plus minimum transport latency**. This detects extra delay and
sample-to-publication age without subtracting unrelated raw clocks. It cannot
measure a constant delay already present at startup or prove absolute one-way
latency; clock drift and PX4 restart require attention, and source timestamp
regression rejects further control until restart. This remains Python/ROS soft
real-time execution, not a hard real-time scheduling guarantee.

`v2_state_timing_json` in the diagnostic CSV records the source sample timestamps,
receive ages, age relative to the minimum transport baseline, publication time
and elapsed control time. Frozen control state is checked even if newer messages
have already arrived, so new receptions cannot make an old solve appear fresh.

Validation on NX: 103 offline tests passed, including delayed-but-newly-received
samples, repeated/reversed timestamps, skewed states and rejection of stale solve
results. One actual ROS/DDS executor test passed in private domain 173, verifying
that reception continues while control is blocked and that the next control
uses the newest snapshot. Publishers are mocked in the offline tests; the DDS
test runs monitor-only on its isolated domain. These tests do not validate gains,
actuator mechanics, absolute transport delay or flight stability.

```bash
python3 tools/build_hardware_drcda_v2.py --check
python3 -m pytest tests/hardware/test_drcda_v2_hardware.py \
  tests/hardware/test_balance_handover_repair.py tests/hardware/test_state_freshness.py -q
ROS_DOMAIN_ID=173 HNUTER_RUN_ISOLATED_ROS_TEST=1 \
  python3 -m pytest tests/hardware/test_state_receiver_ros.py -q
```

A subsequent 8-second live monitor-only check on NX accepted 1,225 coherent
attitude samples with rates/attitude sample skew from -10.545 to +9.046 ms.
Output stayed disabled. This was not an allocation-load or flight test.
