# 版本化报告与本地原始证据

2026-10-25 整理提交保留算法实现、文档、绘图/分析脚本、图表、汇总指标、
运行条件/版本记录以及历史失败和更正说明，不把原始飞行数据库混进源码提交。

本轮整理目录：

- `allocation_audit_20261009/`
- `nda_baseline_validation_20261011/` 及 extended/revised maneuvers
- `first_roll_diagnosis_20261012/`
- `postfix_matched_20261012/`
- `physical_gate_fixed_attitude_20261002/predictive_upgrade/detailed_figures/`

源码克隆不含新加入报告的原始 `data/`、ULog、原始 CSV、冻结源码压缩包等大体量证据。
图表和报告中引用它们是本地证据路径，不代表已上传 Git，也不代表只克隆文档即可重新计算。
汇总 `metrics.csv`、`phase_metrics.csv` 单独保留；完整数据留在原报告目录和私有迁移知识库。
`.gitignore` 仅阻止意外添加，不删除本地数据，也不改变以前已跟踪文件的状态。

知识库入口、哈希和使用边界见 `../docs/drcda_knowledge_base.md`。
旧报告的历史门槛/判断以其日期为界；后续更正与失败样本仍保留，不用最新版成功结论改写旧记录。
