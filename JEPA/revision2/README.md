# 第二轮：门禁、姿态入口与异常恢复实际回归

本目录独立保存代码与新试验，不改写第一轮 `JEPA/runs`、`JEPA/results`。

当前修复：
- 解锁前等待完整关节角/实际转速、有效位置速度、姿态在同一仿真时间轴对齐。至少 25 个对齐样本，最近数据年龄 <0.2 s，连续就绪 3 s；45 s 不满足即未解锁失败。记录订阅返回值、首末时间、缺失字段、倒序和全部门禁判断。
- 普通位置消息仍通过原入口的继承节点发送。显式使用已有 Hnuter `TrajectorySetpoint.jerk[0:2]` 扩展指定水平 roll/pitch，避免继承 `mc_pos_control` 倾角。位置、速度、加速度、yaw 转换不变；这属于参考语义修复，不改 PID。`--attitude-reference legacy` 保留原 NaN 入口作为消融。
- 有有效状态的保护退出继续使用普通位置下降参考并保持 Offboard 心跳，冻结激励和质量更新。必须收到 landed 与 disarmed；35 s 超时失败。状态失效时仍请求 PX4 NAV_LAND，不能保证该路径已验证。
- 真实关节角、速度原始字段与重建推力一起保存；esc_rpm 未作为转速。

早期证据 `original_cause_evidence.json`：C_shadow/s3 的期望向上力约84 N而实际转速接近0，支持执行器命令链路故障，不能归因于负载过重。B/s4、C/s4真实倾角超过25度，目标倾角分别达45、34度；目标与原PX4姿态目标接近。共享发现端口仅是假设，默认保持 shared，不能宣称隔离已解决问题。

运行命令（仅独立本地 SITL，需要允许本地 UDP sockets）：

```bash
cd /home/hnuter/px4_ws_ros2
python3 JEPA/revision2/test_offline.py
python3 JEPA/revision2/run_regression.py --batch targeted --workers 3 --discovery shared
python3 JEPA/revision2/run_regression.py --batch ablation --workers 1 --discovery shared
python3 JEPA/revision2/run_regression.py --batch recovery --workers 2 --discovery shared
python3 JEPA/revision2/run_regression.py --batch matrix --workers 3 --discovery shared
```

每个 batch 的 `plan.json` 为总分母，`results.json` 保存全部尝试；单次目录有启动命令、源码/固件哈希、DDS/GZ隔离配置、原始 ULog、CSV、门禁日志、飞行结果与离线指标。未完成的启动不能从分母剔除。相同随机种子不保证并行调度、传感采样和EKF初始条件逐位相同；前后比较同时列出启动门禁/姿态参考/恢复方式差异。

## 冻结请求与最终矩阵

第二轮恢复注入发现，仅停发心跳仍让质量斜坡继续约 0.468 s。`freeze_firmware.patch` 增加 SITL 命令 31000 的 `param3=1` 分支：保持当前有效质量和积分器，仅撤销目标。位置节点进入 LAND 时立即发冻结请求并在恢复阶段重复。修复后更新中退出的日志只剩约一个控制周期的 0.0012 kg 变化，随后无后续更新采样；落地/上锁确认用时约 6.17 s。`param3=0` 为原质量请求。

新二进制 SHA-256：`4692fec677f8399be752bfcfa76df12779bfb0408779926608033455e3e933ad`。冻结的源码与构建身份见 `frozen_release.json`，构建日志 `build_freeze.log`。不改原 PID 与保护阈值。

水平参考只消除了目标姿态串入，不保证改善跟踪。旧入口三次消融也通过，且误差更小；水平参考矩阵出现 B/s0 水平漂移退出。因此保留两套完整、分别统计的矩阵，原入口配置作为判断性能收益的主要对照，不将水平参考的较弱表现用于制造优势：

```bash
python3 JEPA/revision2/run_regression.py --batch matrix --workers 3 --discovery shared --attitude-reference level
python3 JEPA/revision2/run_regression.py --batch matrix --workers 3 --discovery shared --attitude-reference legacy
python3 JEPA/revision2/summarize_revision.py
python3 JEPA/revision2/audit_transitions.py
python3 JEPA/revision2/audit_release.py
python3 JEPA/revision2/plot_revision.py
```

原姿态入口的正常实验仍继承原 `publish_px4_trajectory_setpoint`，新水平参考仅作为对照配置；两者进入恢复 LAND 后均显式水平姿态。原故障没有在三次同种子消融中确定性复现，因此“入口机制已确认”与“偶发故障根因已完全定位”必须分开表述。
