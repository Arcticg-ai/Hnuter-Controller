# 实验目录与有效结论

本次仅打包，不重新执行飞行或仿真。以下数值来自本地现存报告，保留其条件与局限。

## 最新匹配批次：2026-10-12，18 次 SITL

入口 `controllers.experiments.maneuver_revision`；固件为 tail-sitl 无延迟，
包括磁场坐标、舵机单位稳态增益与着陆保护修复。每个条件一次，没有置信区间或显著性证明。

|任务|Direct 位置 RMS|NDA 适配|Full|No preview|One step|No near-term|
|---|---|---|---|---|---|---|
|3D 李萨如 7 s|0.394 m|0.283 m|0.223 m|0.257 m|0.254 m|0.222 m|
|3D 李萨如 5 s 压力|4.336 m|10.207 m|1.373 m|2.568 m|0.996 m|9.279 m|
|大姿态 +80/-80 roll、+180/-180 pitch|0.067 m|38.195 m|0.040 m|0.030 m|0.088 m|0.038 m|

5 s 的失稳组包括失高后数据，不能把这些全程 RMS 比值当作正常飞行效率提升。
7 s NDA/Full 和消融均全程留空，Direct 未满足精度；Full 较 Direct/NDA RMS 下降 43.4%/21.2%，
仅为这次单次运行差值。5 s 全部未达到原位置标准，Full 不能宣称解决激进飞行。
大姿态 Full 控制 SO(3) 峰值 13.02°，No preview 为 1.73°，说明当前未来参考并非总有利。
大姿态各方向 15 s 去程、5 s 保持、15 s 回程、5 s 水平稳定，不是快速倾转。

共同外环及约束见原报告。公平性仍有限：Direct 是名义解析几何，NDA/DRCDA 使用关节化 SDF；
分配刷新频率也不完全一致。不能声称完整同模型同算力预算或完整复现 PDA。

## 报告与图片

- 最新总报告：`../03_CONTROLLER_REPO/reports/postfix_matched_20261012/REPORT.md`
- 实现细节：同目录 `METHODS.md`；统计 `metrics.csv`、`phase_metrics.csv`。
- 美术风格图：同目录 `publication_figures/TRO/`、`SR/`，矢量 PDF/SVG、PNG。
- 原始数据：同目录 `data/<scenario>/repeat_1/<method>/`，保留 flight.ulg、参数、CSV、console、有效配置与审计。
- 第一次正横滚根因隔离：`reports/first_roll_diagnosis_20261012/REPORT.md`，磁场、估计器、SDF几何/关节增益诊断。
- NDA 资格与修复：`reports/nda_baseline_validation_20261011/README.md`，悬停、低速、连续换向及后续扩展/修订实验。
- 早期预测审计：`reports/allocation_audit_20261009/README.md`。
- 固定姿态 A-B-A 旧批次：`reports/fixed_attitude_switching_20260928/README.md`；其中 Full 仅在 T=2.0 s 通过，No-horizon 到 1.5 s；不是新算法已成功的证据。
- 多点预测升级：`reports/physical_gate_fixed_attitude_20261002/predictive_upgrade/README.md`。
- 旧 v1/v2 与实测闭环：`reports/drcda_v2_validation_20260906/`、`reports/drcda_closed_loop_validation_20260921/`。

上面未特别注明的 reports 路径均相对 `03_CONTROLLER_REPO`。历史报告不覆盖，保留后续更正。
特别是 2026-10-11 早期侧移组的解析尾电机速率曾为 3500 N/s，而优化组为 100 N/s；不能再称其完全同约束。

## 下次复核应优先看

1. q_measured -> Phi_H -> S_H 的分支、单位与采样时间；静态平衡点 q=u 应不被预测继续移动。
2. 未来扳手是否来自同一参考与一致状态外推，尤其大姿态方向不对称和高动态情况下的姿态预览。
3. 请求/可达/预测/真实状态分开，对齐预测发布时间 k 与实际 k+H，不用当前扳手误差替代预测误差。
4. SO(3) 控制误差与真值误差分别列出，保留 EKF 偏差、复位、失高和截止事件。
5. 同外环、实际运行后约束、更新频率、计算用时与初始化流程的公平性，不能只看配置文件是否同名。
