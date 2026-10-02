# 固定姿态快速侧移验证报告

## 1. 实验目标

在 Hnuter 无延迟 SITL 中执行固定姿态 A-B-A 快速侧移，比较 Direct、Basic DA、Full DRCDA 和 No-horizon 四种分配方法，并扫描单程切换时间：

`T = 2.0, 1.5, 1.2, 1.0, 0.8, 0.6 s`

轨迹使用 minimum-jerk 五次多项式：

`p(s) = p0 + (p1-p0)(10s^3-15s^4+6s^5)`

最终验证版以进入任务时的位置为中心，使用固定 ENU Y 轴：

- A：`(x0, y0-1.2 m, 1.5 m)`
- B：`(x0, y0+1.2 m, 1.5 m)`
- 航向：进入任务时的航向，整个任务保持不变
- 流程：起点到 A、稳定、A 到 B、短暂停留、B 到 A

虚拟门框在 24/24 个案例中成功生成，只包含 visual，不包含 collision，因而不会用碰撞替控制器制造成功或失败。

## 2. 可复现身份

- 控制器基线提交：`f1c1cba70d90e8c3b5e5f2ef7fddfd089ba3d085`
- 固件目录：`/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl`
- 固件提交：`49b60a5d8775655c1e6a9be722f76e9e4ab2517b`
- 固件分支：`hnuter-tail-motor-bench-sitl-20260812`
- 验证策略：no-delay only
- 纯延迟模型：未使用
- 固件动态执行器延迟标记检查：未发现
- 公共调参：`config/simulation/no_delay_drcda_tuning.json`

Full 和 No-horizon 都使用 Gazebo 实际关节角反馈。两者唯一核心消融是预测时域：Full 为 `0.1 s`，No-horizon 退化为一个 `0.01 s` 预测步。Basic DA 是当前命令点处的一步微分分配，只使用固定变化率边界，不含关节反馈、时域预测或可达集预测。

## 3. 通过判据

只在 A 到 B 和 B 到 A 两个快速段判断：

- 水平位置峰值误差不超过 `0.75 m`
- SO(3) 总姿态峰值误差不超过 `8 deg`
- `max(|roll|, |pitch|)` 不超过 `5 deg`
- 最低高度不低于 `0.5 m`
- Gazebo 关节反馈覆盖率不低于 `95%`
- 任务后仍 armed，且未触发 direct safety cutoff

这里的最短有效时间只表示本次离散扫描网格内的最小通过点，不是连续意义上的精确临界时间。

## 4. 主要结果

| 方法 | 最短通过 T | T=2.0 s 位置峰值 | T=2.0 姿态峰值 | T=2.0 横滚/俯仰峰值 | 结论 |
|---|---:|---:|---:|---:|---|
| Direct | 无 | 0.257 m | 8.320 deg | 2.324 deg | 位置和倾角可接受，但固定航向误差使严格判据失败 |
| Basic DA | 无 | 无有效轨迹 | 无有效轨迹 | 无有效轨迹 | 多数案例在轨迹前失稳；T=1.0 s 启动后倾覆 |
| Full DRCDA | 2.0 s | 0.264 m | 3.099 deg | 2.214 deg | 慢速点稳定；T=1.5 s 在三个阈值附近越界 |
| No-horizon | 1.5 s | 0.255 m | 3.302 deg | 2.335 deg | 当前参数下通过边界优于 Full |

T=1.5 s 是最有区分度的边界案例：

| 方法 | 位置峰值 | SO(3) 峰值 | 横滚/俯仰峰值 | 判定 |
|---|---:|---:|---:|---|
| Direct | 0.468 m | 9.892 deg | 3.739 deg | 失败：航向/总姿态 |
| Full DRCDA | 0.822 m | 8.209 deg | 5.004 deg | 失败：三个阈值均轻微越界 |
| No-horizon | 0.621 m | 6.965 deg | 5.000 deg | 通过 |

T=1.2 s 时三种可运行方法全部失败。Direct 仍有较小的位置误差，但总姿态误差增至 `20.36 deg`；Full 和 No-horizon 将总姿态峰值分别控制在 `9.58 deg` 和 `8.43 deg`，代价是位置峰值分别达到 `1.55 m` 和 `1.18 m`。这表明当前可达性约束主要在位置跟踪和姿态保持之间重新分配不足的控制能力，并未把该任务变为可行。

## 5. 机理判断

当前 Full DRCDA 没有在该实验中稳定优于 No-horizon，不能据此宣称预测时域带来性能提升。核心原因不是纯延迟模型，因为本实验明确未使用它，而是当前预测结构仍有以下不足：

1. Full 只把一个候选命令保持到 `0.1 s` 终端，优化终端可达扳手。
2. 未来扳手由当前期望扳手的一阶差分和误差反馈外推，未直接使用已知 minimum-jerk 未来参考序列。
3. 在 A-B-A 反转附近，当前单点目标很快过时，较长时域可能形成相位超前、过冲和更大的关节跟踪误差。
4. 外环没有接受正式的可达位置/加速度约束，因此即使分配器知道当前扳手不可达，位置参考仍继续推进。

因此，下一步有理论意义的改进不是继续调一个权重，而是让 horizon 优化直接接收未来参考扳手序列，并由可达扳手/扳手变化率反向约束轨迹时间标定或外环加速度命令。当前结果已经提供了这个改动的反例基线。

Basic DA 的起飞失败说明当前这份 Basic 实现不是可用于性能排序的健康基线。它可以保留为失效案例，但在修复起飞与悬停稳定性之前，不应把其失败当作 Full DRCDA 优越性的证据。

## 6. 图与数据

- `01_sweep_metrics.png`：位置、总姿态、横滚/俯仰峰值随 T 的变化
- `02_tracking_T1p5.png`：T=1.5 s 边界案例跟踪曲线
- `03_wrench_actuator_T1p2.png`：请求/预测可达侧向力及命令/实测关节角
- `data/summary.csv`：24 个案例的统一指标
- `data/manifest.json`：阈值、固件身份、案例状态和源 ULog 路径
- `data/runs/*/*/diagnostics.csv`：每案例控制器诊断数据
- `data/runs/*/*/result.json`：每案例判定与证据路径

扳手图中的 `Kinematic m ay estimate` 使用仿真质量 `4.5 kg` 与速度反馈微分得到，只是实际侧向合力的运动学估计，不是六维力传感器测量。关节角则来自 Gazebo 实际 joint-state feedback。

## 7. 重跑命令

```bash
cd /home/hnuter/px4_ws_ros2_consolidated
source /opt/ros/jazzy/setup.bash
source /home/hnuter/px4_ws_ros2/install/setup.bash

python3 tools/experiments/run_fixed_attitude_switching.py \
  --firmware /home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl \
  --output /tmp/fixed_attitude_switching_recheck

python3 tools/experiments/analyze_fixed_attitude_switching.py \
  --input /tmp/fixed_attitude_switching_recheck \
  --output reports/fixed_attitude_switching_recheck
```

单独验证 Full、T=1.5 s：

```bash
python3 tools/experiments/run_fixed_attitude_switching.py \
  --firmware /home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl \
  --output /tmp/fixed_switch_full_T1p5 \
  --method full --duration 1.5
```
