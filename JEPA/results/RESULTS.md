# SITL 质量自校准实测汇总

全部正式组使用同一 5.39999998 kg 负载模型、固定增益、1 倍速、平滑起飞、16 秒校准预算和公共评价轨迹。
low 为原 SDF 传感器标准差的 1%；noisy 为原 SDF 噪声，加转速 2 rad/s、关节角 0.1°、竖直速度 0.01 m/s 的独立高斯噪声及 60 ms 对齐缓冲（low 为 30 ms）。严格零噪声因 PX4 恒值传感器检查失败，未伪称通过。

表内为均值 ± 样本标准差；性能统计只含完成公共评价阶段的样本，完整飞行成功数和校准验收数单独列出。质量误差含未接受时保留的先验，不能只挑成功估计。主位置误差使用 PX4 状态估计；Gazebo 真值以首个解锁样本做固定平移对齐后单独报告。

| 条件 | 组 | 全程成功 | 校准接受 | 评价 RMSE cm | 竖直 RMSE cm | 真值 RMSE cm | 质量误差 % | 验收时间 s |
|---|---|---|---|---|---|---|---|---|
| low | A | 1/1 | — | 2.637 ± 0.000 | 1.629 ± 0.000 | 3.714 ± 0.000 | N/A | N/A |
| low | B | 1/1 | 1/1 | 2.002 ± 0.000 | 1.118 ± 0.000 | 5.359 ± 0.000 | -0.081 ± 0.000 | 10.188 ± 0.000 |
| low | C | 1/1 | 1/1 | 2.381 ± 0.000 | 1.143 ± 0.000 | 7.056 ± 0.000 | -0.034 ± 0.000 | 10.232 ± 0.000 |
| low | D | 1/1 | — | 2.481 ± 0.000 | 1.740 ± 0.000 | 6.667 ± 0.000 | N/A | N/A |
| low | C_shadow | 1/1 | 1/1 | 3.262 ± 0.000 | 2.015 ± 0.000 | 7.572 ± 0.000 | 0.009 ± 0.000 | 10.004 ± 0.000 |
| noisy | A | 5/5 | — | 4.981 ± 1.197 | 4.094 ± 1.430 | 16.027 ± 5.307 | N/A | N/A |
| noisy | B | 4/5 | 3/5 | 3.420 ± 0.890 | 2.289 ± 1.297 | 9.358 ± 4.677 | -6.650 ± 9.144 | 10.189 ± 0.044 |
| noisy | C | 4/5 | 3/5 | 3.482 ± 0.343 | 2.324 ± 0.258 | 9.855 ± 3.206 | -6.748 ± 9.055 | 10.180 ± 0.028 |
| noisy | D | 5/5 | — | 3.940 ± 1.520 | 3.075 ± 1.758 | 11.722 ± 7.543 | N/A | N/A |
| noisy | C_shadow | 4/5 | 4/5 | 3.561 ± 0.853 | 2.512 ± 0.908 | 10.294 ± 6.103 | -3.341 ± 7.450 | 10.167 ± 0.114 |

A=固定标称；B=被动估计补偿；C=主动估计补偿；D=已知质量参考；C_shadow=主动估计、不补偿。
端到端成功率优先于条件 RMSE：某组中途退出的种子未进入评价，完成子集的较小误差不能推成该组总体更可靠。源结果 flight_result.json 中是采样均方，本文由原始 CSV 统一重算时间加权均方。
验收时间为有效独立验证窗口终点相对校准开始的时间，不包含另行记录的处理缓冲。质量未知时的“首次进入 2% 真值误差带”仅由离线评价程序计算。

## 配对差异

以下是 noisy 条件同种子前组减后组的平均 RMSE 差，正数代表后组误差较小。区间来自最多 5 个种子中可配对且完成评价的试验；配对数单列。bootstrap 仅作不确定性描述，不代表广泛统计结论。

| 指标 | 比较 | 配对数 | 平均差 cm | bootstrap 95% 区间 cm |
|---|---|---|---|---|
| position_rmse_m | A minus B | 4 | 1.277 | [0.470, 1.722] |
| position_rmse_m | A minus C | 4 | 1.215 | [0.362, 2.321] |
| position_rmse_m | A minus D | 5 | 1.041 | [-0.877, 2.920] |
| position_rmse_m | A minus C_shadow | 4 | 1.616 | [-0.081, 3.313] |
| position_rmse_m | B minus C | 4 | -0.062 | [-0.735, 0.742] |
| position_rmse_m | C_shadow minus C | 3 | 0.179 | [-0.591, 1.249] |
| vertical_rmse_m | A minus B | 4 | 1.472 | [0.735, 2.101] |
| vertical_rmse_m | A minus C | 4 | 1.437 | [0.362, 2.841] |
| vertical_rmse_m | A minus D | 5 | 1.018 | [-0.994, 2.930] |
| vertical_rmse_m | A minus C_shadow | 4 | 1.690 | [-0.412, 3.793] |
| vertical_rmse_m | B minus C | 4 | -0.035 | [-0.912, 1.347] |
| vertical_rmse_m | C_shadow minus C | 3 | 0.484 | [-0.097, 1.173] |
| truth_position_rmse_m | A minus B | 4 | 5.310 | [-2.586, 12.536] |
| truth_position_rmse_m | A minus C | 4 | 4.812 | [0.381, 9.243] |
| truth_position_rmse_m | A minus D | 5 | 4.305 | [-3.727, 12.337] |
| truth_position_rmse_m | A minus C_shadow | 4 | 5.798 | [-5.486, 14.099] |
| truth_position_rmse_m | B minus C | 4 | -0.498 | [-4.294, 4.756] |
| truth_position_rmse_m | C_shadow minus C | 3 | 1.137 | [-7.070, 11.026] |

## 校准成本、瞬态与失败

| 条件 | 组 | 校准 RMSE cm | 动作代价 m²/s | 质量绝对误差 % | 独立窗残差 % | 评价饱和 % | 校准失败率 | 安全退出 / 落地未确认 |
|---|---|---|---|---|---|---|---|---|
| low | A | 4.682 ± 0.000 | 0.000 ± 0.000 | N/A | N/A | 0.000 ± 0.000 | — | 0 / 0 |
| low | B | 3.881 ± 0.000 | 0.000 ± 0.000 | 0.081 ± 0.000 | 0.426 ± 0.000 | 0.000 ± 0.000 | 0.0% | 0 / 0 |
| low | C | 4.390 ± 0.000 | 0.090 ± 0.000 | 0.034 ± 0.000 | 0.289 ± 0.000 | 0.000 ± 0.000 | 0.0% | 0 / 0 |
| low | D | 5.099 ± 0.000 | 0.000 ± 0.000 | N/A | N/A | 0.000 ± 0.000 | — | 0 / 0 |
| low | C_shadow | 4.813 ± 0.000 | 0.090 ± 0.000 | 0.009 ± 0.000 | 0.928 ± 0.000 | 0.000 ± 0.000 | 0.0% | 0 / 0 |
| noisy | A | 4.982 ± 0.667 | 0.000 ± 0.000 | N/A | N/A | 0.000 ± 0.000 | — | 0 / 0 |
| noisy | B | 5.045 ± 1.448 | 0.000 ± 0.000 | 6.699 ± 9.099 | 0.544 ± 0.039 | 0.000 ± 0.000 | 40.0% | 1 / 1 |
| noisy | C | 4.849 ± 1.059 | 0.090 ± 0.000 | 6.754 ± 9.049 | 0.656 ± 0.197 | 0.000 ± 0.000 | 40.0% | 1 / 1 |
| noisy | D | 4.114 ± 0.693 | 0.000 ± 0.000 | N/A | N/A | 0.000 ± 0.000 | — | 0 / 0 |
| noisy | C_shadow | 4.630 ± 0.393 | 0.090 ± 0.000 | 3.421 ± 7.404 | 0.491 ± 0.126 | 0.000 ± 0.000 | 20.0% | 1 / 0 |

动作代价为整个校准段的参考速度平方积分，不是能耗。独立窗残差包含冻结候选之后的全部后续窗口；验收至少使用前五窗。

| 条件 | 组 | 转换代数力误差最大值 N | 公共 35–39 s 最大相邻力变化均值 N | 转换期位置峰值均值 cm | 离线推力平衡 RMS 均值 N |
|---|---|---|---|---|---|
| low | A | — | 5.142 ± 0.000 | N/A | 0.144 ± 0.000 |
| low | B | 7.63e-06 | 4.159 ± 0.000 | 3.702 ± 0.000 | 0.118 ± 0.000 |
| low | C | 7.63e-06 | 4.071 ± 0.000 | 4.656 ± 0.000 | 0.134 ± 0.000 |
| low | D | — | 1.836 ± 0.000 | N/A | 0.065 ± 0.000 |
| low | C_shadow | — | 4.210 ± 0.000 | N/A | 0.160 ± 0.000 |
| noisy | A | — | 4.918 ± 0.541 | N/A | 0.139 ± 0.043 |
| noisy | B | 7.63e-06 | 32.458 ± 64.131 | 5.987 ± 1.146 | 0.284 ± 0.254 |
| noisy | C | 7.63e-06 | 3.372 ± 1.879 | 6.261 ± 1.518 | 0.145 ± 0.029 |
| noisy | D | — | 2.642 ± 0.836 | N/A | 0.080 ± 0.004 |
| noisy | C_shadow | — | 4.733 ± 1.439 | N/A | 0.129 ± 0.031 |

代数力误差仅隔离质量/积分器转换；相邻力变化同时含正常反馈和轨迹变化，不能称为质量更新跳变量。公共 35–39 s 窗也保留安全退出后恢复过程，因此 B 组的大力变化包含未补偿的失败试验，不能解释为质量转换冲击。离线推力平衡使用评价真值质量与速度，含气动力，未送入估计器。
校准指标统计实际执行到的校准段：提前退出试验可能只有部分校准数据或没有校准数据。逐次时长、峰值和样本量在 metrics.json 中；不得把它们当成全部完整校准成功。

## 已接受估计的条件精度

| 条件 | 组 | 接受数 / 全部次数 | 仅接受子集质量绝对误差 % |
|---|---|---|---|
| low | B | 1/1 | 0.081 ± 0.000 |
| low | C | 1/1 | 0.034 ± 0.000 |
| low | C_shadow | 1/1 | 0.009 ± 0.000 |
| noisy | B | 3/5 | 0.055 ± 0.038 |
| noisy | C | 3/5 | 0.146 ± 0.116 |
| noisy | C_shadow | 4/5 | 0.110 ± 0.014 |

## 全部原始运行

- low / A / seed 0: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_low_A_s0_20261010_171426/metrics.json)，全程 PASS，原因 None。
- low / B / seed 0: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_low_B_s0_20261010_171426/metrics.json)，全程 PASS，原因 None。
- low / C / seed 0: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_low_C_s0_20261010_171426/metrics.json)，全程 PASS，原因 None。
- low / C_shadow / seed 0: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_low_C_shadow_s0_20261010_171607/metrics.json)，全程 PASS，原因 None。
- low / D / seed 0: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_low_D_s0_20261010_171607/metrics.json)，全程 PASS，原因 None。
- noisy / A / seed 0: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_A_s0_20261010_171956/metrics.json)，全程 PASS，原因 None。
- noisy / B / seed 0: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_B_s0_20261010_171956/metrics.json)，全程 PASS，原因 None。
- noisy / C / seed 0: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_C_s0_20261010_171956/metrics.json)，全程 PASS，原因 None。
- noisy / C_shadow / seed 0: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_C_shadow_s0_20261010_172142/metrics.json)，全程 PASS，原因 None。
- noisy / D / seed 0: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_D_s0_20261010_172142/metrics.json)，全程 PASS，原因 None。
- noisy / A / seed 1: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_A_s1_20261010_172141/metrics.json)，全程 PASS，原因 None。
- noisy / B / seed 1: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_B_s1_20261010_172325/metrics.json)，全程 PASS，原因 None。
- noisy / C / seed 1: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_C_s1_20261010_172326/metrics.json)，全程 PASS，原因 None。
- noisy / C_shadow / seed 1: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_C_shadow_s1_20261010_172507/metrics.json)，全程 PASS，原因 None。
- noisy / D / seed 1: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_D_s1_20261010_172322/metrics.json)，全程 PASS，原因 None。
- noisy / A / seed 2: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_A_s2_20261010_172507/metrics.json)，全程 PASS，原因 None。
- noisy / B / seed 2: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_B_s2_20261010_172505/metrics.json)，全程 PASS，原因 None。
- noisy / C / seed 2: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_C_s2_20261010_172650/metrics.json)，全程 PASS，原因 None。
- noisy / C_shadow / seed 2: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_C_shadow_s2_20261010_172648/metrics.json)，全程 PASS，原因 None。
- noisy / D / seed 2: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_D_s2_20261010_172650/metrics.json)，全程 PASS，原因 None。
- noisy / A / seed 3: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_A_s3_20261010_172838/metrics.json)，全程 PASS，原因 None。
- noisy / B / seed 3: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_B_s3_20261010_172837/metrics.json)，全程 PASS，原因 None。
- noisy / C / seed 3: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_C_s3_20261010_172837/metrics.json)，全程 PASS，原因 None。
- noisy / C_shadow / seed 3: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_C_shadow_s3_20261010_173019/metrics.json)，全程 FAIL，原因 tracking_velocity_or_contact_limit。
- noisy / D / seed 3: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_D_s3_20261010_173020/metrics.json)，全程 PASS，原因 None。
- noisy / A / seed 4: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_A_s4_20261010_173019/metrics.json)，全程 PASS，原因 None。
- noisy / B / seed 4: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_B_s4_20261010_173202/metrics.json)，全程 FAIL，原因 attitude_or_horizontal_limit。
- noisy / C / seed 4: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_C_s4_20261010_173110/metrics.json)，全程 FAIL，原因 attitude_or_horizontal_limit。
- noisy / C_shadow / seed 4: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_C_shadow_s4_20261010_173339/metrics.json)，全程 PASS，原因 None。
- noisy / D / seed 4: [日志与指标](/home/hnuter/px4_ws_ros2/JEPA/runs/sitl_noisy_D_s4_20261010_173201/metrics.json)，全程 PASS，原因 None。
