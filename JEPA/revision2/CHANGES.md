# 第二轮文件清单与审阅入口

- PX4 `src/modules/mc_rate_control/HnuterControl.cpp`：SITL专用质量冻结分支，见 freeze_firmware.patch。第一轮hpp/msg改动保留，本轮未再修改。
- `sitl_position_probe.py`：原普通位置节点的子类；解锁前数据门禁、实际转速响应保护、两种明确标注的姿态参考、位置下降恢复和冻结请求。
- `actuator_sensors.py`：实际关节状态完整性、有限值和单调时间检查；原始关节角/速度记录与订阅诊断。
- `run_sitl.py`：独立结果路径；记录参考配置；提供未解锁门禁测试、缺失遥测负向测试、恢复注入和候选发现端口配置。
- `run_regression.py`：明确批次计划与总分母，所有完成/失败保存；无自动重试替换结果。
- `estimator.py`、`model_audit.py`、`test_offline.py`：沿用第一轮已验证核心，未改辨识窗、先验、限值或增益。
- `analyze_sitl.py`：沿用逐次离线指标，ULog及真值只进入评价。
- `summarize_revision.py`、`write_report.py`：分配置、完整窗口和部分窗口分别统计；全部失败保留。
- `audit_release.py`、`audit_landings.py`、`audit_transitions.py`、`diagnose_new_failures.py`、`audit_wrench_axes.py`：身份/时序/落地/瞬态/失效证据审计。
- `plot_revision.py`、`plot_outcomes.py`：新日志可视化。
- `original_v1/`：原八个脚本和哈希；`stage1_targeted_changes.patch`保留定向试验初版；`changes_from_v1.patch`为最终跨版本审阅差异，不应直接应用到原版脚本目录。
- `runs/`：78次独立启动原始证据；50次完整计划矩阵，其他为定向、消融、门禁及恢复测试。
- `results/CONCLUSIONS.md`：最终报告；`results_level_reference/`：水平参考矩阵独立汇总。

已构建并验证当前SITL固件，不能在当前已应用补丁的文件上重复应用。构建和运行命令见README.md。所有既有用户修改、原模型、原位置入口与第一轮脚本哈希核对均保留。
