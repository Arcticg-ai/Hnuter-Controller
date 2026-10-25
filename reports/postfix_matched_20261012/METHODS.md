# 实现与指标口径

本文件只解释本轮冻结代码，不把之前版本的结果当作本轮证据。源文件路径以下均相对控制仓库根目录；确切源码及哈希见 `provenance/controller_source.tar.gz` 与 `run_manifest.json`。

## 实际执行链

`controllers.experiments.maneuver_revision` 创建多继承实验控制器，顺序为 RevisedManeuverMixin → ManeuverMixin → 指定分配控制器。外环仍是 `controllers/simulation/hnuter_external_direct_controller_debug.py` 的直接控制路径：位置反馈和期望加速度生成合力，四元数/大倾角误差、角速度阻尼、积分及陀螺项生成力矩。所有方法经过相同外环、映射和发布层，输出 `actuator_motors` / `actuator_servos`。

共同修订仅涉及已存在的接管/启停/几何和传感器修复，没有为 Full 单独削弱轨迹。磁场修复运行在固件 GZBridge；真值只用于事后审计，不参与外环。关节实测状态由 Gazebo 反馈进入分配器，飞机位置/姿态仍取 PX4 估计。

Direct 在 `FixedAttitudeDirectController.publish_direct_actuator_setpoint` 施加前电机力变化率限幅，`ManeuverMixin` 额外统一尾推力变化率 100 N/s；底层发布层施加舵机角度、速率和电机命令范围限制。因此本轮不是“Direct 不限速，而优化器限速”的对比。

## 方法与消融

|标签|实际实现|预测和目标|
|---|---|---|
|Direct|名义几何的解析力分解，经输出约束投影|当前期望，无关节动态前瞻|
|NDA adaptation|`fixed_attitude_switching/audit_allocator.py:MeasuredNDAAllocator`|实测状态、归一化微分分配、采样动态逆，无未来参考目标|
|Full DRCDA|`common/hnuter_drcda.py:DRCDAAllocator` + `drcda_closed_loop/governor.py:MeasuredDRCDAAllocator`|0.05/0.10 s 两点未来参考，0.01 s 近期目标，2 次 GN|
|No preview|与 Full 相同|两点目标均换成当前 wrench，保留预测传播和近期目标|
|One step|与 Full 相同|只在 0.01 s 追当前 wrench，权重 1.5；仍保留近期项，不是完全没有动态模型|
|No near-term|与 Full 相同|去掉 0.01 s 近期目标，保留两点未来参考|

`runtime_config.json` 是初始化配置；单步分支在 `_future_drcda_wrench_references` 中将运行 horizon 改为 0.01 s。因此单步初始化快照中的 0.1 s 不代表实际目标仍为 0.1 s，诊断 CSV 的 `drcda_prediction_h1_s/h2_s` 是运行时检查依据。

### Full 的预测与优化

每个周期先传播电机内部状态，然后用实测关节角覆盖舵机预测初态。电机力是模型状态，不能称为实测电机推力。四个舵机方向性一阶预测使用单位稳态增益和零纯延迟：

```text
a_i = 1 - exp(-Delta / tau_i)
q_next = q + a_i * (u_i - q)
s_next = (1 - a_i) * s + a_i
```

随后按当前分支施加速率及角度限制，同步修改灵敏度。四关节耦合力臂与力矩由 `SDFWrenchModel` 计算，不将关节后的旋翼中心固定在初始位置。

一个候选命令在整个预测段保持不变（move blocking），一步传播 Delta=0.01 s。利用 Phi 与灵敏度 S：

```text
q_H = Phi_H(q_measured, u)
B_H = G(q_H) S_H
```

目标为以下残差的加权平方和，加上命令增量和姿态偏好正则：

```text
sqrt(0.5) W (w_0.05(u) - w_d(k+0.05))
sqrt(1.0) W (w_0.10(u) - w_d(k+0.10))
W_near (w_0.01(u) - w_d(k))
sqrt(R_delta) ((u-u_previous)/command_scale)
sqrt(R_pref)  ((u-u_preferred)/command_scale)
```

`W=sqrt(wrench_weight)/wrench_scale`；`wrench_weight=[1.2,1.2,2,3,3,2.5]`，`wrench_scale=[80,80,100,12,12,12]`。近期项原始权重 `[0,0,0.5,3,3,1.5]`。归一化变量下构造 GN Hessian 和梯度，使用有界二次步，再做共同命令投影，最多 2 次迭代。完整对角正则数值在 runtime JSON。

**有 future_references 时不再叠加旧的终端 wrench-rate 残差**。日志仍有 jerk/reference-rate 字段，不意味着它参与当前两点目标。

李萨如未来位置/速度/加速度来自同一个相位时钟。未来 wrench 使用相同外环形式和短时状态外推；大姿态未来参考通过 `_future_attitude_wrench`，以当前角速度指数传播姿态并保留当前积分值。没有未来真实机体状态，也没有完整刚体最优控制。`ReachableWrenchContract` 提供预测见证与诊断，不改变公共轨迹；本轮共同残差反算增益为 0。

### 论文 NDA 的准确边界

本轮实现取论文归一化微分分配结构，使用 Hnuter 执行器力坐标。先由当前状态和命令变化率盒求单个采样周期可达执行器速率上下界，然后取中点 m 与半宽 d，构造 `A=G diag(d)`。

```text
j = 12 (w_desired - w_estimated)
z = pinv(A) (j-G m) + (I-pinv(A) A) z_preferred
rate = m + d * z/max(1,max(abs(z)))
```

零空间偏好包含关节回正与固定悬停转速偏好在力坐标下的等价方向。不是直接往主任务力矩加偏好。主增益 12、零空间增益 2、关节姿态增益 2。本轮使用 `sampled` 逆：将“给定命令在一个真实控制采样周期产生的平均状态变化率”作为动态函数，在可行命令盒里二分 24 次反演；避免连续逆在采样时间超过快速电机时间常数时失配。

该实现对论文结构做了双轴倾转平台及采样系统适配，使用经验推力/命令约束，**没有重现电压-转速辨识的功率约束或完整 PDA**。NDA 的失败不能推导成原论文完整方法失败；Full 优于它的某项结果也不能写成优于完整论文全部方法。

## 误差与统计

- 位置误差：控制日志中的 ENU/相对高度位置与同一时刻期望位置之差的欧氏范数。
- 控制 SO(3)：控制器日志的期望姿态到 EKF 姿态的旋转角；无欧拉角跨 ±180° 相减。
- 真值参考 SO(3)：将 ENU/FLU 期望转为 NED/FRD 后，计算其与 ULog 真值四元数的相对旋转角。转换为 `R_NF = R_NE R_EL R_LF`，`R_NE=[[0,1,0],[1,0,0],[0,0,-1]]`，`R_LF=diag(1,-1,-1)`。
- EKF/真值 SO(3)：`2 acos(abs(dot(q_EKF,q_GT)))`，计算前归一化、截断浮点域。它不是期望跟踪误差。
- CSV 与 ULog 以首次 armed 边沿对齐；真值姿态取不晚于样本的最近 ULog 值，绘图高度和 EKF/真值误差使用时间插值。存在约一个状态采样间隔对齐误差。
- RMS、分位数为实际记录样本统计（诊断约 50 Hz），不宣称独立样本总体统计。主表含全部任务段及约 5 s 终端观测。失败样本未裁去。
- 关节总变差为四关节相邻诊断样本角度绝对差之和；低频采样可能漏掉高频活动，不等同完整累计机械行程或能耗。
- 峰值保持段按实际 phase 标签分组；与真值低于 0.5 m 的时刻重叠或发生其后的段，图中置为 N/A，但原始数值仍保存在 phase_metrics.csv。

## 验证边界

18 次单次条件测试能提供实测对比和反例，不能提供统计显著性、稳定性证明、全局最优性或实机性能保证。本轮没有重新辨识全工况舵机动态，也没有凭模型扳手估计声称测得真实电机推力。高动态下剩余 EKF 偏差、大姿态方向不对称、单步反超 Full 都需在后续控制/预测审计中继续解释。
