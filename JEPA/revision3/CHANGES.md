# 本轮新增文件（全部位于 revision3）

| 文件 | 用途 |
|---|---|
| rigid_math.py | 坐标变换、完整惯量物理检查、关节构型运动学 |
| assembly.py | 离线全机组合 CoM/J/hrel 审计、两个独立负载 SDF 生成 |
| wrench.py | 实际关节/转速和固定标定的 O 点外部 wrench 重建 |
| integral_estimator.py | 五参数短窗角动量回归、信息子空间/物理/后续残差门控 |
| actions.py | C2 平滑分轴候选、信息选择及 fail-closed 状态机 |
| control_contract.py | 离线 CoM/J 增量消费者及零积分时的连续力矩 tracking 状态 |
| replay.py | 三个预选旧日志 shadow 回放；真值仅在独立评价部分使用 |
| offline_experiments.py | 已知参数、噪声、延迟、漏项、单轴合成试验及简化刚体 oracle |
| tests/ | 物理、可观测性、时序、内部动量、变换和更新连续性测试 |
| make_report.py / run_checks.py | 可复现报告/图和串行离线入口；门槛失败不启动飞行 |
| generated/ | 两个载荷模型、剥离质量惯量的传感几何、单独评价真值配置 |
| results/ | 全部日志、机器可读指标、传感回放数组、门槛、PNG/PDF/SVG |
| initial_state.json | 开始时两仓库分支、HEAD、脏文件和 revision2 脚本哈希 |

PX4 源码、本地普通位置入口、revision2 文件没有本轮变更。`changes.patch` 为上述新增源文件的可审阅差异；大体积结果另列 SHA256SUMS。
