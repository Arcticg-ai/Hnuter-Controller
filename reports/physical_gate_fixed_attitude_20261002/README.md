# 实体窄门定姿快速侧移验证

## 实验目的

在无纯延迟模型的 Hnuter SITL 中，用可碰撞的中央窄门验证直接执行器控制与微分分配。飞机保持进入任务时的固定航向和水平姿态，执行三段 minimum-jerk 横移：

`A(0 m) -> B(+1.2 m) -> A(0 m) -> C(-1.2 m)`

单段时间分别为 `T=2.0 s` 和 `T=1.5 s`。三种方法使用相同外环、轨迹、固件和通过判据：Direct、Full DRCDA、No-horizon。

## 场景与硬约束

- Gazebo 门模型包含左右墙体和顶梁的 collision，不是只显示不碰撞的 visual。
- 开口宽 `2.30 m`，高度 `2.36 m`。
- 仿真机纵向碰撞包络约 `0.82 m`；净空指标还包含飞行时的横向耦合偏移。
- 地面黄、蓝、绿标记分别对应 A、B、C，便于视频核对任务顺序。
- 到点判据为目标中心 `0.20 m` 邻域；门平面保守净空必须不小于 `0.05 m`。
- SO(3) 姿态峰值不超过 `8 deg`，横滚/俯仰峰值不超过 `5 deg`，位置峰值不超过 `0.75 m`。
- Gazebo 实测关节角反馈覆盖率不低于 `95%`，任务结束后仍保持 armed，且不得触发 direct safety cutoff。

## 可复现身份

- 固件：`/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl`
- 固件提交：`49b60a5d8775655c1e6a9be722f76e9e4ab2517b`
- 固件分支：`hnuter-tail-motor-bench-sitl-20260812`
- 控制器基线：`f1c1cba70d90e8c3b5e5f2ef7fddfd089ba3d085` 加本报告对应的未提交实验修改
- 执行器模型：Gazebo 关节 PID 动态与实测关节角校正
- 纯延迟模型：未使用

## 软件结构与数据流

本实验不是为每种分配方法分别调一套控制器。三种正式比较方法共享完全相同的轨迹、位置环、姿态环、飞行器参数和安全限制，只替换控制分配层：

```text
minimum-jerk 位置/速度/加速度参考
                  |
                  v
       位置 PID + 加速度前馈
                  |
                  v
          期望机体系合力 Fd

固定姿态参考 ---> SO(3) 姿态控制 ---> 期望力矩 taud
                  |
                  v
       六维期望扳手 wd=[Fd, taud]
                  |
       +----------+-----------+
       |          |           |
     Direct    Full DRCDA   No-horizon
       |          |           |
       +----------+-----------+
                  |
                  v
  4 个倾转角命令 + 5 个电机推力命令
                  |
                  v
        PX4 actuator_* -> Gazebo
```

主要代码位置：

| 功能 | 文件 |
|---|---|
| 公共位置环、姿态环、Direct 解析分配 | `controllers/simulation/hnuter_external_direct_controller_debug.py` |
| 非线性扳手模型、Full DRCDA、Basic DA | `controllers/common/hnuter_drcda.py` |
| DRCDA 与公共控制器的接入、调参和抗饱和 | `controllers/simulation/hnuter_external_direct_drcda.py` |
| Gazebo 实测关节角读取 | `controllers/experiments/drcda_closed_loop/feedback.py` |
| 可达扳手契约与实测状态校正 | `controllers/experiments/drcda_closed_loop/governor.py` |
| A-B-A-C 轨迹和方法选择 | `controllers/experiments/fixed_attitude_switching/controller.py` |
| 实体门模型 | `controllers/experiments/fixed_attitude_switching/gate.sdf` |
| 启动、判定和数据归档 | `tools/experiments/run_fixed_attitude_switching.py` |

## 公共控制器实现

### 位置环

轨迹在 ENU 中生成，进入控制器后转换为 PX4 NED 坐标。位置环为带加速度前馈的逐轴 PID：

```text
ep = pd - p
ev = vd - v
ad = aff + Kp ep + Kd ev + Ki integral(ep)
Fd_world = m (ad - g_NED)
Fd_body = R_NED_FRD^T Fd_world
```

本次公共参数为：

- `Kp=[4.0, 3.5, 8.0]`
- `Kd=[3.8, 3.2, 4.0]`
- `Ki=[0.15, 0.12, 3.0]`
- 水平和垂直加速度上限均为 `8 m/s^2`
- 位置积分限幅为 `[0.7, 0.7, 2.0]`

所有方法使用同一个位置积分器。Full 和 No-horizon 额外用“请求扳手与可达扳手之差”做 back-calculation，避免分配饱和后位置积分继续积累。

### 姿态环

实验始终要求 `roll=0`、`pitch=0`，航向保持为进入任务时的航向。控制器使用连续四元数/SO(3) 姿态误差，并计算：

```text
tau_d = -KR * eR - Domega * eomega - Ki_R * integral(eR)
        + omega x J omega
```

本次参数为：

- `KR=[5.0, 5.5, 14.0]`
- `Domega=[3.0, 3.2, 5.9]`
- `Ki_R=[0.03, 0.03, 0.02]`
- 力矩限幅 `[2.5, 2.8, 5.0] N*m`

姿态误差较大时积分器不继续累积，而是按 `0.5 s` 时间尺度释放已有积分。三种方法收到完全相同的 `Fd_body` 和 `tau_d`，因此图中的差异主要来自分配器，而不是外环增益差异。

## 执行器与扳手模型

分配器状态/命令向量为：

```text
q = [alpha_L, beta_L, alpha_R, beta_R,
     f_L1, f_L2, f_R1, f_R2, f_tail]
```

前四项是左右两组二级倾转角，后五项是四个前部同轴电机和尾部双向电机的推力。每个前部旋翼方向为：

```text
d(alpha,beta) = [cos(beta)sin(alpha), -sin(beta), cos(beta)cos(alpha)]
```

非线性六维扳手由各旋翼推力、力臂矩和反扭矩共同得到：

```text
F(q)   = sum(fi * di)
tau(q) = sum(ri x fi*di + sigma_i*k_m*fi*di)
w(q)   = [F(q), tau(q)]
```

代码同时提供解析 Jacobian `G(q)=dw/dq`，单元测试中用中心差分检查该 Jacobian。四个倾转输入、五个推力、倾转角范围、推力范围和每周期命令变化量都在投影步骤中同时约束。

本次严格采用 no-delay 配置：

- 舵机纯延迟为 `0`
- 不使用旧的辨识一阶滞后
- 保留方向相关静态增益，用于输入到物理角度的映射
- 保留倾转命令速率限制 `[8, 4, 8, 4] rad/s`
- Gazebo 中实际动态来自 JointPositionController PID
- 前部电机内部预测时间常数约 `1/2 ms`，尾电机为 `50 ms`

因此这里验证的是“受限执行器状态和关节 PID 跟踪误差”，不是人为加入的通信纯延迟。

## 当前方法：Full DRCDA

### 实测状态校正

Full 不把上一条舵机命令当作真实关节角。`JointAngleFeedback` 订阅：

```text
/world/<world_name>/dynamic_pose/info
```

通过父子 link 相对四元数计算 `alpha_L, beta_L, alpha_R, beta_R`。零位只允许在解锁状态标定；开始控制后锁定。反馈年龄超过 `0.12 s` 时控制器 fail closed，拒绝继续输出 DRCDA 命令。每次分配前，预测器的四个角状态都会被 Gazebo 实测角覆盖。

### 短时域预测

Full 使用：

- 分配更新周期 `0.01 s`
- 预测步长 `0.01 s`
- 预测时域 `H=0.10 s`
- 一个候选命令在整个时域内保持不变，即 move blocking

预测器从实测倾转角和内部电机推力状态出发，逐步施加角度、推力和命令变化率限制，得到终端状态 `q_H(u)`、灵敏度 `dq_H/du` 和终端可达扳手 `w_H=w(q_H)`。

扳手变化参考由当前请求的低通差分和扳手误差反馈构成：

```text
wd_dot_ff = LPF((wd(k)-wd(k-1))/dt), tau=0.08 s
r_w = wd_dot_ff + 6.0 * (wd - w_est)
```

### 优化目标

每次最多进行两次 Gauss-Newton 迭代，最小化以下加权目标：

```text
J(u) = ||W^(1/2) (w_H(u)-wd)||^2
     + lambda_r ||W^(1/2) (w_H(u)-w_est-H*r_w)||^2
     + ||u-u_prev||^2_Rmove
     + ||u-u_pref||^2_Rpref
```

其中：

- 六维扳手权重为 `[1.2, 1.2, 2.0, 3.0, 3.0, 2.5]`
- 扳手变化率项权重 `lambda_r=0.06`
- `u_prev` 抑制每周期命令跳变
- `u_pref` 是 Direct 解析分配给出的首选构型，用于在冗余解之间保持连续性
- 每次迭代后都投影到角度、推力和每周期变化率约束内
- 线性系统异常或出现非有限值时退化到固定倾转角的电机最小二乘分配

### 可达扳手契约与抗饱和

每次求解后会构造 `ReachableWrenchContract`，检查：

1. 命令确实位于本周期可达的执行器变化率盒内。
2. `predicted_wrench` 必须能由一个明确的预测状态 witness 重新计算得到。
3. 根据 `w_H-w_est` 计算可达扳手变化率。
4. 使用实测偏航角速度和预测偏航力矩形成预测偏航角速度诊断量。

位置积分抗饱和使用 `w_reachable-w_requested`，而不是未约束的原始分配残差。

需要强调：当前契约已经约束和校验分配器输出，但还没有把未来可达扳手正式反向送给轨迹生成器，轨迹仍按预定时间推进。这也是高速情况下 Full 不能在所有指标上优于 No-horizon 的主要结构性限制。

## 对比方法实现

### Direct 解析分配

Direct 不进行数值优化，也不使用实测关节角预测。它将六维请求拆成左右倾转组和尾电机的解析中间量：

```text
u1 = Fx/2 - tau_z/(2*l1)
u4 = Fx/2 + tau_z/(2*l1)
f_tail = (tau_y - (r_z*Fx-r_x*Fz)) / (r_x+l2)
Fz_front = Fz - f_tail
u2 = Fz_front/2 + (tau_x+r_z*Fy)/(2*l1)
u5 = Fz_front/2 - (tau_x+r_z*Fy)/(2*l1)
u3 = u6 = -Fy/2
```

然后计算左右组总推力及倾转角：

```text
F_L = norm([u1,u2,u3])
F_R = norm([u4,u5,u6])
alpha_L = atan2(u1,u2), beta_L = asin(u3/F_L)
alpha_R = atan2(u4,u5), beta_R = asin(u6/F_R)
```

最后执行推力限幅、倾转角限幅和舵机命令 slew limit。Direct 的优点是确定性强、计算量小；缺点是先求瞬时几何解再限幅，限幅后的实际扳手不再等于请求扳手，也不能利用关节实测状态预判下一时刻的可达能力。

### Basic DA

Basic DA 保留论文式的局部微分分配结构，但只在当前名义命令点线性化：

```text
w_dot ~= G(q_k) q_dot
```

它用固定角速度/推力变化率归一化 Jacobian，解一个带正则项的线性最小二乘问题，再把归一化变化率裁剪到 `[-1,1]`。Basic DA：

- 不读取 Gazebo 实测关节角
- 令内部状态直接等于上一周期命令
- 不传播执行器动态
- 不计算有限时域可达集
- 不使用 `0.10 s` 终端扳手目标
- 每周期只做一次局部线性更新

当前 Basic DA 在这套飞行器和调参下经常在正式轨迹前失稳，尚不是健康基线。因此它保留在代码和失效分析中，但未纳入本报告六个正式性能排序案例；不能把它的起飞失败当作 Full 的算法优势。

### No-horizon 消融

No-horizon 与 Full 使用相同：

- 公共外环参数
- 非线性扳手模型和权重
- Gazebo 实测关节角反馈
- 角度、推力和变化率约束
- Direct 首选构型、抗饱和和安全逻辑

唯一核心改动是把预测时域从 `0.10 s` 缩短到一个离散步 `0.01 s`。调参文件加载后会再次应用该消融，避免配置中的 `drcda_horizon_s` 意外恢复 Full 时域。因此 Full 与 No-horizon 的差异可以主要归因于有限时域终端预测，而不是反馈或外环不同。

## 实验内容与执行过程

### 实体门和飞行空间

门位于 `y=0` 平面，开口沿世界 `x` 方向宽 `2.30 m`，有效高度 `2.36 m`。左右隔墙和顶梁都有 collision。开口尺寸看起来大于原方案中的 `0.8-1.0 m`，原因是当前 Hnuter 模型包含长尾梁，其纵向碰撞包络约为 `1.58 m`；低于该尺寸的门会在起飞或通过时直接与模型碰撞，不能代表控制算法失败。

净空计算采用比主体碰撞盒更保守的半长 `0.82 m` 和半高 `0.40 m`：

```text
clearance = min(
  opening_half_width - |x| - 0.82,
  z - lower_edge - 0.40,
  upper_edge - z - 0.40
)
```

### A-B-A-C 任务

任务点均为相对进入任务时的位置：

- `A=(x0, y0, 1.5 m)`，位于门中心
- `B=(x0, y0+1.2 m, 1.5 m)`
- `C=(x0, y0-1.2 m, 1.5 m)`

完整时序为：

1. 用 `2.5 s` minimum-jerk 从当前点进入 A。
2. 在 A 稳定 `1.5 s`。
3. 用时 `T` 执行 A 到 B。
4. 在 B 停留 `0.5 s`。
5. 用时 `T` 返回 B 到 A，并在 A 停留 `0.5 s`。
6. 用时 `T` 执行 A 到 C。
7. 到达 C 后保持 C，不再跳回起飞点。

每段使用五次 minimum-jerk：

```text
s=t/T
h(s)=10s^3-15s^4+6s^5
p_d=p_0+(p_f-p_0)h(s)
```

代码解析计算 `v_d` 和 `a_d`，直接送入公共位置环。正式比较使用 `T=2.0 s` 和 `T=1.5 s`；后者具有更高的加速度和换向需求，是本报告的动态边界案例。

### 自动运行顺序

运行器依次：

1. 检查目标固件不含动态延迟标记。
2. 启动 Micro XRCE-DDS Agent。
3. 启动指定无延迟 PX4/Gazebo 固件。
4. 通过 Gazebo create service 生成实体门。
5. 启动指定方法的控制器，发送 `o` 完成仿真 Offboard/Arm。
6. 发送按键 `2` 启动 A-B-A-C 任务。
7. 等待完成标记，保持 C 点并采集后段数据。
8. 保存控制器 CSV、PX4 ULog、有效调参、控制台日志和逐案例 `result.json`。

GUI 只影响是否显示 Gazebo 客户端；轨迹、控制参数和判据与 headless 批量实验一致。

### 评价指标与通过逻辑

位置、姿态、偏航角速度和关节误差只在 `a_to_b`、`b_to_a`、`a_to_c` 三个动态段统计。一个案例必须同时满足：

- A/B/C 顺序完成，B 和 C 进入目标中心 `0.20 m` 邻域
- 返回阶段进入门中心 A 的 `0.20 m` 邻域
- 门平面保守净空不小于 `0.05 m`
- 水平位置峰值误差不大于 `0.75 m`
- SO(3) 姿态峰值不大于 `8 deg`
- `max(|roll|,|pitch|)` 不大于 `5 deg`
- 最低高度不低于 `0.5 m`
- 关节反馈覆盖率不低于 `95%`
- 任务后仍 armed，且没有 direct safety cutoff

`position_error_peak` 与 `minimum_gate_clearance` 分别约束跟踪性能和实际几何安全，不能相互替代。通过门但位置误差过大、位置误差较小但擦碰门框，都会判为失败。

## 结果

六个案例均通过统一硬判据。

### T=1.5 s 动态边界

| 方法 | 位置 RMS | 位置峰值 | 姿态峰值 | 横滚/俯仰峰值 | 偏航角速度峰值 | 关节跟踪 RMS | 最小净空 |
|---|---:|---:|---:|---:|---:|---:|---:|
| Direct | 0.119 m | 0.227 m | 5.277 deg | 2.040 deg | 0.462 rad/s | 0.084 rad | 0.283 m |
| Full DRCDA | 0.114 m | 0.268 m | 2.285 deg | 2.180 deg | 0.108 rad/s | 0.088 rad | 0.162 m |
| No-horizon | 0.117 m | 0.259 m | 2.744 deg | 2.244 deg | 0.119 rad/s | 0.082 rad | 0.284 m |

Full 相比 Direct：位置 RMS 降低 `3.9%`，姿态峰值降低 `56.7%`，偏航角速度峰值降低 `76.7%`。

Full 相比 No-horizon：位置 RMS 降低 `2.5%`，姿态峰值降低 `16.7%`，偏航角速度峰值降低 `9.3%`；但位置峰值增加 `3.8%`，关节跟踪 RMS 增加约 `8.0%`，最小门净空也更小。

### 结论边界

这个实验比空场直线往返更能验证方法：它同时包含固定姿态、三次横向加减速和换向、实体碰撞门、窄通道净空、实测关节状态与任务完成约束。

结果支持以下有限结论：在更快的 `T=1.5 s` 工况，Full 的预测时域对总姿态和偏航扰动有小幅到明显收益，并略微改善位置 RMS。

结果不支持“Full 全指标优于 No-horizon”。尤其是位置峰值、关节跟踪和门净空仍有代价；`T=2.0 s` 时 No-horizon 与 Full 基本相当，部分指标更好。因此后续算法改进应直接使用未来参考扳手序列，并让可达扳手约束反向限制外环参考，而不是继续单独增加终端预测权重。

## 文件

- `01_sweep_metrics.png`：两档切换时间的误差与姿态指标
- `02_tracking_T1p5.png`：`T=1.5 s` 三方法轨迹和姿态跟踪
- `03_wrench_actuator_boundary.png`：Full 与 No-horizon 的可达扳手和实测关节响应
- `data/summary.csv`：六个案例统一指标
- `data/manifest.json`：固件身份、阈值和案例状态
- `data/runs/*/*/diagnostics.csv`：控制器逐采样诊断
- `data/runs/*/*/result.json`：逐案例判定

## 重跑

```bash
cd /home/hnuter/px4_ws_ros2_consolidated
source /opt/ros/jazzy/setup.bash
source /home/hnuter/px4_ws_ros2/install/setup.bash

python3 tools/experiments/run_fixed_attitude_switching.py \
  --firmware /home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl \
  --output /tmp/physical_gate_recheck \
  --duration 2.0 --duration 1.5 \
  --method direct --method full --method no_horizon

source /home/hnuter/px4_ws_ros2/px4-venv/bin/activate
python3 tools/experiments/analyze_fixed_attitude_switching.py \
  --input /tmp/physical_gate_recheck \
  --output reports/physical_gate_fixed_attitude_recheck
```
