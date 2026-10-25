# 论文微分分配修复与无延迟仿真验证

日期：2026-10-11。本轮先验证论文 NDA 的 Hnuter 平台适配能否稳定悬停及低速跟踪，
再与 DRCDA、解析分配进行统一配置的连续换向测试。探索失败、资格测试和正式对比分开保存。
结果仅为当前仿真平台的证据，不构成实机飞行许可或对原论文平台性能的评价。

## 固件与版本

使用 `/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl` 的现有无延迟模型。
本轮开始时源码已是 `53266349b9c65001b72e8f2d437b7340c8c4756f`，
分支为 `codex/payload-selfcalibration-20261010`，并非上轮的 `49b60a5d`。
两者间变更在固件控制器的载荷参数接收与状态消息中，Gazebo SDF 未变；本轮不发送载荷辨识命令，
外部控制直接发布电机与舵机输入。本任务没有切换或修改固件源码。
每批 manifest 保存实际固件版本，正式运行逐次保存控制器源码及参数 SHA-256。
正式组还逐次读取 ULog 的 `ver_sw` 核实实际构建身份，保存 ULog 哈希与初始参数快照。

没有使用旧延迟仓库，也没有添加纯传输延迟。Gazebo 原有关节 PID、前电机 1/2 ms 与尾电机
50 ms 响应仍保留；“无纯延迟”不等于执行器瞬时响应。

## 核对发现与修复

### 连续反演的采样实现

论文式（14）的连续模型为 `qdot = (K*u-q)/tau`。原实现直接用
`u=(q+tau*qdot_desired)/K`，却在约 10 ms 的控制周期中保持命令不变。
当电机 tau=1 ms 时，要求 100 N/s 的变化，一周期实际平均变化只有约 10 N/s。
这不是论文公式错误，而是连续公式与离散执行方式不匹配。

现对每个采样周期使用相同执行器模型的保持输入映射：

```text
a = 1-exp(-dt/tau)
q_next = q + clip(a*(K*u-q), physical_rate_bounds*dt)
qdot_bar(u) = (q_next-q)/dt
```

由可行输入上下界映射得到平均速率边界，再按论文式（8）至（13）归一化、伪逆分配、
零空间投影和整向量统一缩放。最后通过单调映射反演得到可行输入。
未触及边界时，这等价于 `u=(q+dt*qdot_bar/a)/K`。
该实现是 NDA 的采样版本，不加入未来轨迹预见，也不调用 DRCDA 的多点优化目标。
这里的精确保持输入映射只对固定模型分支成立。当前正/负命令采用不同拟合时间常数，
过零处可能有小的速率跳变；例如关节状态 1 rad、步长 12 ms 时，两个一级关节的分支跳变
约 0.057/0.064 rad/s。当前有界二分反演不能因此被宣称为全域连续、全域精确反演。
该残留的分段模型近似没有在本轮更换，NDA 与 DRCDA 使用同一模型；这也是后续模型核对点。

### 悬停速度目标与双轴冗余

原零空间目标让各前旋翼推力趋向当前平均值，不能阻止它们一起增推力。
论文式（6）则使用固定悬停速度。现将前四旋翼的悬停速度目标转换到推力坐标；固定尾桨保留在主任务中：

```text
Fdot_preferred = 2*k_omega*sqrt(F)*(sqrt(F_hover)-sqrt(F))
```

仅修正这个目标仍不足以抑制本机的相反方向倾转自运动。Hnuter 每臂双轴倾转，
与论文单轴臂拓扑不同；两臂可以在保持近似相同机体扳手时逐渐向相反方向倾转。
因此采用论文式（4）/（12）允许的姿态次目标：`qdot_joint_preferred=-k_posture*q_joint`，
先转换到归一化坐标，再严格通过零空间投影加入，不直接叠加到主扳手任务。

最终 NDA 内部参数固定为 `k_j=12/s`、`k_omega=2/s`、`k_posture=2/s`。
这些是平台适配参数，不是宣称从原论文获得的数值。选择来自悬停/低速阶段，
正式快换向对比中不继续针对结果调参。

### 初始化与评价

各方法共同使用解析分配起飞、进入任务位置并稳定。NDA/DRCDA 在首段开始时接管，
以实测关节角和当前解析电机命令对应的推力初始化。接管后直到末段悬停结束均使用待测方法，
不能因轨迹结束而退回解析控制。该接管逻辑有回归测试。

这证明的是稳定初态后的悬停/跟踪能力，不证明 NDA 已独立完成起飞。
探索中的独立起飞失败仍保留，未从记录中删除。

到点判定也已修复：从日志读取实际相对轨迹终点，计算三维到点距离，不再硬编码世界坐标
`y=+/-1.2`。误差窗口包含完整运动、停留及末段悬停；所有算法使用同一判据。

## 对比条件

- 正式组为空旷场景，不生成门框。此前门框边界附近出现过角速度突变，可能有接触冲击；
  但去掉门框后旧 NDA 仍失稳，因此不能把所有失败归因于碰撞。
- 共享同一外环参数文件、质量/惯量、力矩限幅、轨迹、初始准备流程和物理模型。
- 正式组将附加模型残差回算增益设为零，三者均保留共同外环的积分限幅。
  原回算配置下的预试验单列，不混入正式统计。
- 前旋翼单个推力上限 25 N，同轴两桨合计 50 N；尾电机上下限采用同一仿真配置。
- 关节物理速率界 10 rad/s、舵机命令变化率界 50 rad/s；前/尾推力命令变化率界
  分别为 2500/3500 N/s。实验解析分配也显式施加该推力变化率界。
- 外环约 250 Hz；NDA/DRCDA 分配周期设为 10 ms，在该时钟下通常约 12 ms 更新一次；
  解析分配在外环频率更新。这一实现差异保留并披露，不宣称所有计算频率完全相同。
- NDA 与 DRCDA 共用关节实测反馈、同一原始阶跃拟合模型与模型估计的电机推力。
  旋翼转速尚未直接测量，功率模型没有引入。
- 正式运动为 A→B→A→C，每段 1.2 m，固定水平姿态，点间无停留，结束后悬停 5 s。
  T=2.0、1.5、1.1 s，各方法两次重复；第二次反转方法顺序。

“相同外环参数”不意味着不同闭环轨迹中的数值 wrench 完全相同。
NDA 的内部 jerk 增益与 DRCDA 的多点目标权重也不是同一个参数，不能强行相等。

## 实际结果

<!--RESULTS-->

数值范围为两次运行的最小至最大值，不是置信区间。正式轨迹通过标准为位置峰值不超过
0.75 m、SO(3) 峰值不超过 8 deg、滚转/俯仰不超过 5 deg、高度不低于 0.5 m，
并完成目标、保持解锁、没有安全切断、关节反馈覆盖率至少 95%。
悬停使用更严格的 0.15 m/3 deg 标准；低速跟踪使用 0.25 m/5 deg/滚俯3 deg 标准。

## 消融含义

| 方法 | 相对完整方法只改变什么 |
|---|---|
| DRCDA No preview | 同样 0.05/0.10 s 预测、模型、求解器和权重；未来参考改为当前参考 |
| DRCDA One step | 相对 No preview，将当前参考的目标压缩到 0.01 s，总跟踪权重仍为 1.5 |
| NDA No posture | 保留固定悬停旋翼速度目标，去掉关节姿态零空间目标 |
| NDA Continuous inverse | 保留姿态目标和其他参数，恢复连续时间速率边界及反演 |

前两个消融在正式运动中重复，后两个首先在悬停工况重复。
Full 与 No preview 用于检验未来参考；No preview 与 One step 用于检验当前参考条件下的预测时域。
Full 与 One step 同时改变预见参考和时域，不能把该差值全部归因于单一因素；本轮不是完整的二因素实验。
NDA 消融图中的完整方法曲线复用共同配置的悬停资格测试，不另计作新的运行。
零空间消融失败只说明该平台适配需要适当的冗余管理，不能宣称原论文 NDA 本身失败。
连续反演消融的飞行结果与单周期数值测试是两种不同层次的证据，不能互相替代。

## 与论文的边界

已核对本地 Cuniato 等 2026 年论文的式（6）至（14），恢复了归一化、偏置、
统一缩放和动态反演链条。主状态仍为四个关节角与五个旋翼推力，是 Hnuter 拓扑下的力坐标适配。
论文以关节角和旋翼转速为状态，因此不能称为逐变量、逐参数的完全复现。
姿态零空间目标属于文中允许的次目标选择，具体形式和增益为本机适配。
本轮没有式（15）至（19）的电机电功率参数与物理功率限制曲线，因此不是 PDA 完整复现。

本轮的有效比较对象是“通过悬停/低速资格验证的 NDA 平台适配”，而非之前不能稳定飞行的草稿。
两次重复不足以建立统计显著性、可靠性概率或全局稳定性结论；未测试的轨迹、扰动和实机不能外推。
NDA 本轮按悬停/低速资格选定内部增益，没有进行修复后快速工况的完整增益搜索。
因此快速换向失败限定于当前固定参数，不能解释为论文算法经充分整定后也必然失败，
也不能把这三档测试当作全局最小可行时间的测定。

Gazebo 电机力矩/滤波实现的辅助核对来源：
[Gazebo 官方 MulticopterMotorModel 源码](https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/src/systems/multicopter_motor_model/MulticopterMotorModel.cc)。
实际插件包含本地双向尾电机适配，不把上游源码替代为本地二进制一致性证明。

## 复现命令

在 `/home/hnuter/px4_ws_ros2_consolidated` 中执行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/hnuter/px4_ws_ros2/install/setup.bash
export OPENBLAS_NUM_THREADS=1
export HNUTER_NDA_GAIN=12
export HNUTER_NDA_NULL_GAIN=2
export HNUTER_NDA_POSTURE_GAIN=2
export HNUTER_NDA_INVERSION=sampled
export HNUTER_SWITCH_COMMON_PRELUDE=1
export HNUTER_SWITCH_COMMON_AW_GAIN=0
export HNUTER_DRCDA_REACTION_RATIO_M=0.016
unset HNUTER_SWITCH_JOINT_MODEL
/home/hnuter/px4_ws_ros2/px4-venv/bin/python \
  tools/experiments/run_fixed_attitude_switching.py \
  --output "$PWD/hnuter_logs/nda_manual_qualification" \
  --method paper_nda --duration 3 --span 0 --no-gate --dwell 0 \
  --post-hold 10 --position-peak-limit .15 --attitude-peak-limit-deg 3 \
  --roll-pitch-peak-limit-deg 3 --gui
```

低速验证改为 `--span .4 --duration 3`。对比选用 `paper_nda/full/direct/no_preview/no_horizon`。
`--output` 每次使用新目录，避免覆盖既有运行。无需另起 PX4 或 DDS Agent，驱动会启动并清理仿真。
不要把此实验入口、起飞流程或参数直接用于实机。

## 文件与验证

- `all_metrics.csv`、`all_results.json`：所有已完成记录的运行，包括探索失败。
- `data/`：原始诊断 CSV、有效参数、控制器与仿真控制台、原始结果。
- 正式组各运行的 `ulog_metadata.json`：实际固件身份、原 ULog 校验值与初始固件参数；`model.sdf` 保存本轮物理模型。
- `allocator_diagnostics.csv`：求解耗时分位数、NDA 缩放比例与关节运动诊断；不是硬实时调度保证。
- `figures/`：悬停、低速、三档对比、DRCDA 消融与 NDA 修复消融曲线。
- `locked_plan.json`：正式批次的固定参数及执行顺序。
- `incomplete/`、`incomplete_runs.json`：尚未形成结果的中断试跑，不计入通过率或完整运行数量。
- `run_manifests/`：版本、模型无延迟审计、判据和批次结果。
- `source_before.tar.gz`、`source_final.tar.gz`：修改前与最终代码快照。
- `SHA256SUMS.txt`：交付目录的校验清单。

代码没有推送 Git；固件源码没有由本任务修改。ULog 保留在固件日志目录，原始结果记录其路径；
交付目录主要归档本次分析使用的诊断 CSV，不把仅有路径声称为已复制 ULog。

复核归档后重绘曲线（不重新启动仿真）：

```bash
python3 analyze_nda_validation.py --runs data --output /tmp/nda_report_replot
```

上述命令在本报告目录运行；输出目录应是新的目录。`verification.json` 记录归档核验结果，
`test_results.xml` 是 60 项回归测试的结果，`this_turn.patch` 只包含本轮相对于开始时快照的实现修改。
