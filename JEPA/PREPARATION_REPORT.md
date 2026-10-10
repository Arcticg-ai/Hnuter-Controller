# 质量自校准：准备代码与受阻报告

本目录是第一阶段准备产物，**尚不是可运行的闭环自校准系统**。本次实际执行了环境诊断、SDF 质量/惯量审计、负载模型生成、SDF 校验和离线数学测试；**PX4/Gazebo 飞行 0 次**。没有操作硬件，没有启动控制节点，没有改动原控制器或 PX4 源码。JEPA、神经网络训练均未实现。

## 复现已执行检查

```bash
cd /home/hnuter/px4_ws_ros2
python3 JEPA/run_offline_checks.py
```

此脚本只运行准备检查，不会解锁飞行器。当前权限下预期返回码为 **2**，表示 SITL 受阻；不能把离线测试通过当成任务完成。结果写入 `JEPA/runs/offline_<时间>/`，最新目录记在 `JEPA/latest_offline_run.txt`。时间使用 Asia/Shanghai。

单独重现：

```bash
python3 JEPA/preflight.py
python3 JEPA/model_audit.py
python3 -m unittest discover -s JEPA -p test_offline.py -v
gz sdf -k JEPA/generated/zero_sensor_noise/hnuter_mass_cal_payload/model.sdf
```

不要将上述命令称为完整实验启动命令。隔离启动器、普通位置入口状态机、在线遥测和固件质量补偿尚未接入，不能据此直接执行 A/B/C/D。

## 当前阻塞及所需权限

1. 本会话创建 AF_INET UDP/TCP socket 均失败：`PermissionError(1, 'Operation not permitted')`，甚至尚未连接任何端点，仅尝试绑定 `127.0.0.1`。ROS/DDS、XRCE Agent、Gazebo Transport 无法在该限制下联调。
2. 会话文件权限仅允许 JEPA 所在 ROS 工作区、指定 visualization 目录和临时目录写入；指定 PX4 仓库处于只读范围。不能在原仓库修改 `HnuterControl`、写构建目录或使用默认 rootfs 写参数/ULog。

继续完整验证需要将该 PX4 仓库加入本会话可写工作区，并允许本机仿真所需进程间/回环通信。无需开放真实飞控网络、串口或硬件访问。没有使用 chmod、sudo、替代工具或提权绕过限制；没有自动审批拒绝事件。

## 本次确认的工程事实

| 项目 | 本次读取结果 |
|---|---|
| PX4 路径 | `/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl` |
| PX4 分支 / HEAD | `hnuter-tail-motor-bench-sitl-20260812` / `49b60a5d8775655c1e6a9be722f76e9e4ab2517b` |
| PX4 工作树 | 无已跟踪或未跟踪变化 |
| ROS 分支 / HEAD | `hardware` / `abeb52c13c49fb601401a44a30b343fad0183133` |
| 原有 ROS 修改 | 9 个已跟踪修改文件，以及此前未跟踪目录/视频；全部保留 |
| ROS / Gazebo | Jazzy 环境加载、rclpy/px4_msgs 导入通过；`gz sim --versions` 输出 8.11.0 |
| 构建身份 | `build_git_version.h` 中版本与 PX4 HEAD 一致；二进制 SHA256 已保存，尚未通过运行中的 `ver all` 确认 |
| AGENTS.md | 已检查祖先目录及涉及源码/JEPA 子树，未发现适用文件 |

诊断 JSON 包含 Git 状态、原有已跟踪修改的哈希、源码/二进制哈希、版本头、实际执行命令与错误。`*_existing_changes.diff` 是运行前已有修改的备份，不是本任务的改动。

## 模型审计

对所有 **10 个 link** 沿 `relative_to` frame graph 求变换，将每个 link 的惯性中心转换到模型坐标，旋转惯量后用平行轴定理合成。结果对应 **SDF 零关节位姿**，不是运动中固定不变的 CoM。

- 标称总质量：**4.49999998 kg**；base_link 单独为 3.0552478 kg。
- 标称组合 CoM（模型局部坐标，米）：`[-0.1049171401, 0.0010965586, -0.0040426450]`。
- 内置 0.9 kg 刚性负载放在该 CoM，尺寸 `0.12 × 0.12 × 0.08 m`，惯量对角为 `[0.00156, 0.00156, 0.00216] kg·m²`，满足正定及主惯量三角不等式。
- 加载后总质量：**5.39999998 kg**，此位姿下组合 CoM 不变。
- 标称组合主惯量：`[0.1262345492, 0.2469026946, 0.3264970403] kg·m²`；加载后：`[0.1277945502, 0.2484630817, 0.3286566522] kg·m²`。

完整分 link 数据与惯量矩阵见 `generated/mass_audit.json`。模型为独立 `hnuter_mass_cal_payload`，通过 fixed joint 连接 base_link；内部负载沿用原外壳碰撞体，不增加重叠碰撞体。mesh 使用当前机器源路径，换机器后应重新生成。

`native_noise/` 保留原 SDF 传感器噪声；`zero_sensor_noise/` 将模型内传感器 noise 的均值/标准差置零。所有 link 关闭 wind。后者仅是模型噪声设置，不保证整个仿真无噪声：世界、系统插件、估计器和传输仍待实际检查。两份文件均通过 `gz sdf -k`，但尚未装载进 Gazebo。

## 控制链路与输入审计

普通入口 `hnuter_external_controller_px4_position.py:481` 固定 `use_px4_position_takeoff=True`，`control_loop()` 的该分支发布位置轨迹并返回。不能因同文件也有直接执行器代码，就误判本模式使用外部力控制。

`MulticopterRateControl.cpp:167` 在 `CA_AIRFRAME=16` 且相应控制标志有效时调用 `HnuterControl::update()`。`HnuterControl.cpp:148` 从 `HNTR_MASS` 读取质量；第 319 行组合

```text
acc_unsaturated = acc_ff + velocity_p * vel_error + velocity_integral
                  - velocity_d * measured_acceleration
```

随后限制加速度，第 337–358 行执行积分更新和限幅，第 363 行通过 `F_world = mass * (acc_des - gravity_vec)` 计算合力。积分器是加速度量纲，但本周期 `acc_des` 使用更新积分之前的值，因此在线实现要在一致阶段重算，不能只在现有参数读取处换质量。

世界 `default.sdf` 的重力为 **9.8**，HnuterControl 内部使用 **9.81**。辨识器按世界的已知重力设置 9.8，无扰公式按实际控制实现使用 9.81；不能混用。这里没有修改既有控制器的重力或增益。

`GZMixingInterfaceESC.cpp:39–48,135` 将 `/.../command/motor_speed` 同时用于输出与“反馈”，直接填入 `esc_rpm`，既不是独立转速测量，也不能因为字段名 rpm 就按 rpm 单位换算。前四个 SDF 电机 `k=8.54858e-5`，转速时间常数 0.001/0.002 s；尾电机 `k=1.2777084e-5`，时间常数 0.05 s。舵机为 JointPositionController，不能把舵机命令直接当实际角度。

目前只实现从物理转速、实际推力轴和姿态旋转重建 `T_up` 的数学函数；**尚未提供这些独立遥测，也没有经验证的指令滞后/舵机状态观测器**。因此在线质量可辨识性尚未验证。

## 已有代码及测试边界

| 文件 | 实现与边界 |
|---|---|
| `preflight.py` | 只读诊断、Git/文件身份记录、socket 探针；不启动飞行 |
| `model_audit.py` | frame graph、整机惯量审计、独立负载模型生成；明确拒绝未支持的 SDF 结构 |
| `estimator.py` | 0.4 s 积分窗口、遗忘最小二乘、固定先验、物理范围、20 训练窗后稳定性检查、冻结候选并以 5 个独立后续窗口验收；异常间隔/不合格数据重置训练段 |
| `test_offline.py` | 10 项离线测试，包括 5 个合成速度噪声种子；这些不是要求的 5 次 SITL 复测 |
| `run_offline_checks.py` | 自动执行并保存原始诊断、测试、SDF 校验日志；生成解析参考图 |
| `experiment_plan.json` | A/B/C/D/C_shadow 组别、初步预算和未实现项；属于实验配置，不供估计器读取 |

估计器没有读取模型或真实质量的接口。但离线测试为验证公式而使用真值生成合成力输入，这不构成真实执行器重建证据。`eligible` 只是调用方契约，尚未接入实际离地、状态有效、姿态、饱和、时延等检测。

无扰切换目前仅有代数候选函数：在未饱和时保持切换前后合力一致；积分超限则拒绝，不裁剪。**尚无固件 m_eff、变化率限制、SITL 编译保护、消息输入和闭环瞬态日志。**没有修改 HNTR_MASS，没有高频写参数，没有改变 PID。

图 `planned_reference.png` 仅为 16 s、0.08 m、0.4 Hz、sin⁴ 包络的解析位置/速度/加速度，标题明确标为非飞行数据。当前窗口、噪声和阈值均待实测调试。

## 三个研究问题的当前答案

| 问题 | 当前结论 |
|---|---|
| 质量能否独立辨识？ | 数学与合成输入测试通过；真实执行器/速度遥测链未建立，SITL 未验证 |
| 补偿是否平稳？ | 未饱和公式的力连续性测试通过；固件未接入，闭环未验证 |
| 主动比被动值得吗？ | 未验证；合成悬停也能辨识，不能预先宣称主动更好 |

A/B/C/D/C_shadow、标称基线、加载后稳定性、实际质量误差/收敛时间、独立实测窗口残差、轨迹 RMSE/最大误差、更新瞬态、饱和率、校准失败率、安全退出次数全部 **N/A（未执行）**。不能把 N/A 写成零故障、零饱和或成功。

## 解除权限限制后的顺序

1. 验证独占实例和通信边界：拟使用独立 domain 73、XRCE UDP 28888、命名空间 `jepa_sitl`、独立 GZ_PARTITION、loopback；这些值是候选，未检查宿主占用。基类绝对 `/fmu/...` topic 必须显式 remap；Agent 的监听地址需实际约束，环境变量本身不是完整网络隔离证明。
2. 在每次独立 rootfs 中启动原固件/原模型，记录 `ver all`、实际参数与 ULog。先完成普通位置基线，不复用依赖 cube world 的 IEBC 入口。只关闭本次创建的 PID/process group。
3. 原模型基线通过后，加载生成的独立模型，旧控制器完成同样动作。失败则先报告稳定边界，不开启质量补偿。
4. 实现独立电机/关节遥测及对齐，先 C_shadow 辨识；通过后才接入固件内存 m_eff 和完整安全状态机。
5. 冻结公共评价轨迹和噪声/延迟配置，执行 5 组无噪声试验及每组 5 个随机种子，保留全部失败。控制增益固定，单独计算校准时间/误差/动作代价。

本报告不借用历史 SITL 结果作为本次实验成功证据。
