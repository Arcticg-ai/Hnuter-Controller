# 理论、代码与版本导航

本文件导航到**本包源码**，不把历史建议自动当成已经正确实现的理论。

## 控制链与坐标

轨迹或 RC -> 位置/姿态外环 -> 期望扳手 w=[F,tau] -> 本地控制分配 ->
`/fmu/in/actuator_motors`、`/fmu/in/actuator_servos`。
控制实验使用 `OffboardControlMode.direct_actuator=true`，不是让 PX4 接收位置/角速度/力矩并代分配。
部分入口有 `vehicle_thrust_setpoint` 仅作着陆检测提示，不能把其存在误认为 thrust_and_torque 控制链。

位置常用 ENU、机体 FLU；PX4 姿态和控制模型涉及 NED/FRD。必须沿实际转换检查每条符号与电机顺序。
分配器状态一般为四关节角加五电机推力，共 9 维；电机力是模型估计，不是实测力。
硬件电机顺序 R1,R2,L1,L2,tail；舵机发布 aR,aL,bR,bL，内部模型常用 aL,bL,aR,bR，不能直接混用。

## 各版本不是同一个 Full

|入口/实现（相对 03_CONTROLLER_REPO）|用途与区别|
|---|---|
|`controllers/simulation/hnuter_external_direct_drcda.py`|v1 仿真外壳，使用 common 共享分配器；不能拿默认值代替最新实验配置|
|`controllers/common/hnuter_drcda.py`|DRCDAConfig、HnuterWrenchModel、DRCDAAllocator、早期 BasicDifferentialAllocator|
|`controllers/experiments/drcda_v2/allocator.py`|独立实验分配器与归一化分配工具；不是实机 v2 文件|
|`controllers/experiments/drcda_closed_loop/`|实测角初态覆盖、ReachableWrenchContract 与诊断/抗饱和|
|`controllers/experiments/fixed_attitude_switching/controller.py`|Direct、Full、No-preview、No-horizon、NDA 适配的公共任务包装与未来轨迹采样|
|`controllers/experiments/fixed_attitude_switching/audit_allocator.py`|最新论文 NDA 力坐标/采样动态逆适配，不是最早 Basic DA|
|`controllers/experiments/maneuver_revision.py`|最新磁场/舵机增益修复后，3D 李萨如和 15 s 大姿态匹配实验入口|
|`controllers/experiments/sdf_wrench_model.py`|关节化 SDF 几何，随倾转变化的旋翼位置/力臂|
|`controllers/hardware/hnuter_external_direct_drcda_hardware.py`|旧实机 DRCDA，保留作沿革|
|`controllers/hardware/hnuter_external_direct_drcda_v2_hardware.py`|最新独立单文件实机封装；默认 no-preview、未辨识硬件动态 tau=0|

## Full 的计算主线

保留实测状态修正、0.10 s 预测、单一候选命令的 move blocking、有界 Gauss-Newton。
最新匹配仿真通常在 0.05/0.10 s 取目标；具体权重读有效配置，不同版本曾用 0.4 或 0.5。

```text
a_i = 1 - exp(-Delta/tau_i)
q_next = q + a_i*(K_i*u_i - q)
s_next = (1-a_i)*s + a_i*K_i
q_H = Phi_H(q_measured, u)
B_H = G(q_H) S_H
```

位置/速率边界激活时按 active branch 修改状态和灵敏度。最新 Gazebo 修复令 K+=K-=1，
避免 q=u 平衡点仍被预测继续移动。方向时间常数保留，**纯延迟为零**。
这是一阶内部预测，不是往 Gazebo 物理模型再次插入一阶滞后；真实响应来自关节 PID 和动力学。

目标包含多点未来扳手、近时刻平衡项、命令增量及偏好正则：

```text
J = sum_j lambda_j ||W(w_hat_Hj(u) - w_d(k+Hj))||²
    + ||W_near(w_hat_0.01(u)-w_d(k))||²
    + ||u-u_previous||²_Rdelta + ||u-u_preferred||²_Rpref
```

未来位置/速度/加速度来自同一个轨迹时钟，再经公共控制律和短时状态外推生成未来扳手。
不是未来真实状态、完整机体 MPC，也没有保证全局最优。近期项是软惩罚，不是硬可达约束。
有 future_references 时不重复叠加旧的 terminal wrench-rate 残差；日志字段存在不等于参与目标。
实机遥控未来输入未知，默认两个点保持当前扳手，不能称为完整仿真预览算法。

## 论文微分分配的适配边界

`MeasuredNDAAllocator` 取论文式 7–14 的归一化微分分配结构，用本机力坐标：

```text
j = k_j (w_d - w_est)
A = G diag(d), m=(qdot_max+qdot_min)/2, d=(qdot_max-qdot_min)/2
z = pinv(A)(j-Gm) + (I-pinv(A)A)z_preferred
qdot = m + d*z/max(1,max(abs(z)))
```

零空间偏好包括关节回正与固定悬停转速在力坐标下的等价方向。
将连续逆直接用于 dt 远大于快速电机 tau 的系统曾导致采样失配；现采用同一保持输入映射的
采样平均速率，再在命令盒中二分逆。分段正反模型过零仍有近似，不应宣称全域精确连续逆。
这不是完整复现电压/电功率辨识约束 PDA；对比不得扩写成“优于论文全部方法”。

## 更完整的实现说明

- `../03_CONTROLLER_REPO/reports/postfix_matched_20261012/METHODS.md`
- `../03_CONTROLLER_REPO/reports/nda_baseline_validation_20261011/README.md`
- `../03_CONTROLLER_REPO/reports/physical_gate_fixed_attitude_20261002/predictive_upgrade/README.md`
- `../03_CONTROLLER_REPO/docs/drcda_v2_hardware.md`：开头是现行策略，后面日期段保留历史。

代码默认 horizon 曾为 0.18 s，实验和硬件覆盖为 0.10 s；必须读 runtime_config、诊断时域与有效参数，不能只看 dataclass 默认值。
