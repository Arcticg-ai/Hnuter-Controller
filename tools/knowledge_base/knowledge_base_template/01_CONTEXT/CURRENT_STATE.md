# 当前状态与验证边界

本文件根据打包时的本地源码和现存报告整理。远程状态最后证据为 2026-10-24，**本次未重新连接**。

## 最重要的版本区别

- 本地最新实机入口：`../03_CONTROLLER_REPO/controllers/hardware/hnuter_external_direct_drcda_v2_hardware.py`。
- 其 SHA256 应为 `7b9a09f54d15b15decaf80ec0ee4160962057ab4b62caa61ad2c674de9f4264e`，最终以 `09_PROVENANCE/package_metadata.json` 为准。
- 本地有已修改及未跟踪内容，不能仅凭 Git HEAD 判断实际源码；本包包括这些内容。
- 2026-10-22 已发布的 main 提交为 `9909590444408e2d80fe553b7ca74395753e4e66`，它不是本地最新实测位姿接管版；本次没有联网刷新远程分支，也没有推送。
- 实机 v2 是生成的单文件，内嵌固定分配器模板；不会在运行时 import 后来改动的仿真共享模块。

## 最新实机接管策略

1. 默认 `HNUTER_HARDWARE_DRCDA_ENABLE=0` 只监视，不发送心跳和执行器消息。
2. 显式 `ENABLE=1` 后，独立 20 Hz 定时器持续发送 `OffboardControlMode(direct_actuator=true)`，不等遥控、位姿、Arm 或 Offboard。等待阶段不发电机/舵机命令，避免模式切入死锁。
3. 飞手手动 Armed + Offboard 后捕获**切换时实测 ENU 位置和完整姿态**，默认 0.5 s smoothstep 混合过渡，不自动起飞，不发布 VehicleCommand。
4. 旧 PX4 `hntr_att` 参考、5°跟踪误差、普通 15°遥控目标范围、模型扳手一致性、参数快照匹配，不再是接管门槛。执行器快照可选，缺失走初始化，不拒绝接管。
5. 首个位姿是构造参考所需输入，不能虚构；运行中仍保留时效/有限值、估计器重置、数值失败及物理输出保护。现有总倾角保护为 30°。
6. 人工退出/重入 Offboard 重新捕获位姿并重置分配器；故障锁存不自动重启。停止心跳不等于立即切回 Position，取决于飞控 Offboard-loss 设置。

## 最近一次尝试并未证明解决乱飞

上次远程运行目录为：
`/home/adminpc/hnuter_ctrl/Hnuter-Controller-main/hnuter_logs/hardware_drcda_v2/measured_pose_20261024_fnErqP`。

已取回的最后一段控制日志在 unix `1792833485.2736` 报：
`DRCDA OUTPUT WITHDRAWN: tilt 30.1 deg exceeds hardware envelope`。
这是**估计器报告倾角**触发运行中保护，不是已取消的接管姿态目标门槛。
新事件文件/ULog 尚未取回，不能判断真实物理倾角、控制持续时间、实际输出条数或根因。
随后 SSH/SCP 报 No route to host；远程控制进程曾仍存在且故障锁存，不能据此宣称现在可用。

独立 DDS 观测器当时退出；本地发现 NumPy float32 列表无法直接 JSON 编码，
已在 `05_HARDWARE_EVIDENCE/remote_drcda_update_20261022/observe_live_dds.py` 改为 Python float。
该观测器修复**尚未远程部署**。下次先修复记录链，再回收日志，不要先继续调增益。

## 参数不能混用

|项|仓库默认|上次重启后飞控读回及单次运行配置|
|---|---|---|
|HNTR_PITCH_BIAS|0.07|0.09|
|俯仰阻尼 direct_Domega[1]|2.5|1.5|

其余标定参见硬件配置和原始参数。这里不是今日飞控读回。不要把旧参数 JSON 当成当前确认。
舵机输入 PWM 才是 500/1500/2500 us；DDS 舵机仍发 [-1,1]，不是直接填 PWM。
前电机归一化 [0,1]，尾部可逆 [-1,1]；二级舵机减速比 2，物理关节±90°。

## 已验证与未验证

- 历史精确源码回归：本地硬件/公共测试 189 项、远程 v2 77 项；证据在报告及 XML。
- 待机约 20 Hz 心跳及未激活时零执行器发布已验证；不是实机控制效果验证。
- 现有仿真 Full 有局部跟踪优势，但有单步/无预览反超的反例。
- 不支持“实机已稳定”“DRCDA 全面优于论文”“故障仅由期望不合适引起”的结论。

完整时间线：`../05_HARDWARE_EVIDENCE/remote_drcda_update_20261022/LIVE_RUN_20261024.md`。
