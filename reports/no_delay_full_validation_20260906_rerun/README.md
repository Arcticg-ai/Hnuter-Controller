# 无延迟完整复跑结果

## 验证身份

- 验证策略：`no_delay_only`
- 控制器提交：`968df77b71953cb42dbbeb0f0868d32fbc06897f`
- PX4 固件提交：`49b60a5d8775655c1e6a9be722f76e9e4ab2517b`
- 禁用执行器插件检查：通过，`forbidden_actuator_tokens=[]`
- 自动化测试：`167 passed`
- SITL 工况：`8/8` 完成，`8/8` 保存 CSV 和 ULog

原始 CSV、ULog 和控制台日志保存在：

```text
/home/hnuter/px4_ws_ros2/hnuter_logs/no_delay_full_validation_20260906_rerun
```

## 汇总

| 工况 | 方法 | 流程完成 | 跟踪有效 | 位置 RMSE (m) | SO(3) RMSE (deg) |
| --- | --- | --- | --- | ---: | ---: |
| 高速 3D 李萨如 | Original direct | 是 | 是 | 0.5475 | 6.775 |
| 高速 3D 李萨如 | Paper NDA | 是 | 否 | 3.3438 | 132.001 |
| 高速 3D 李萨如 | DRCDA v1 | 是 | 是 | **0.4502** | **7.341** |
| 高速 3D 李萨如 | DRCDA v2 | 是 | 是 | 0.4620 | 8.471 |
| 大姿态悬停 | Original direct | 是 | 否 | 0.1624 | 22.813 |
| 大姿态悬停 | Paper NDA | 是 | 否 | 9.3199 | 146.170 |
| 大姿态悬停 | DRCDA v1 | 是 | 是 | **0.0923** | **5.135** |
| 大姿态悬停 | DRCDA v2 | 是 | 是 | 0.1023 | 5.236 |

`Paper NDA` 完成时间驱动任务不代表飞行有效：两项实验都发生明显姿态失稳和位置偏离。`Original direct` 在大姿态实验中的横滚保持误差未通过有效性门槛。

本轮再次支持继续使用 `DRCDA v1`。v2 没有获得可重复的性能优势，不替换当前推荐算法。所有结论只来自无延迟模型，不包含延迟分支、纯延迟或独立一阶执行器插件。

详细指标和分段保持结果见 [experiment_report_zh.md](experiment_report_zh.md)。
