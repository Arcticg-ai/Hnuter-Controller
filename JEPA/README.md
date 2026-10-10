# PX4/Gazebo SITL 质量在线自校准

2026-10-10 在本机实际运行。普通位置入口由 `sitl_position_probe.py` 继承原 `hnuter_external_controller_px4_position.py`，原入口未改；实际闭环在 CA_AIRFRAME=16 的 HnuterControl 内更新内存有效质量。没有 JEPA、神经网络训练、IEBC、DRCDA 或直接执行器控制。

本轮结论：独立辨识和已接受更新的平滑性得到 SITL 支持；噪声组存在校准拒绝和安全退出，整套系统尚未通过可靠性验证，主动优势未得到证明。详见 [研究结论与失败边界](results/CONCLUSIONS.md)。

数值结果和全部正式运行见 [实测汇总](results/RESULTS.md)、[逐次指标](results/all_trials.csv)、[结果图](results/comparison.png)。[早期准备报告](PREPARATION_REPORT.md) 是权限开放前历史记录，其中“未启动/受阻”状态已经过期。本次通过用户授权的正式宿主执行审批运行，未使用 sudo/chmod。

## 重现命令

当前 PX4 补丁已经应用并构建。启动器自动启动并回收自有 DDS Agent、PX4、Gazebo 和位置节点。不要另开同实例的 Agent/PX4，不要将节点 remap 到其他飞控。

```bash
cd /home/hnuter/px4_ws_ros2
python3 -m unittest discover -s JEPA -p test_offline.py -v
python3 JEPA/model_audit.py
python3 JEPA/run_sitl.py --label manual_C --loaded --instrument --smooth-takeoff --group C --noise low_noise --seed 0
python3 JEPA/run_matrix.py --phase low --workers 3
python3 JEPA/run_matrix.py --phase noisy --workers 3
python3 JEPA/summarize_matrix.py
```

需要现有 Jazzy、Gazebo 8.11、MicroXRCEAgent、numpy、matplotlib、pyulog 和 Gazebo Python bindings。启动器自行加载 `/opt/ros/jazzy/setup.bash` 与本工作区 `install/setup.bash`。固件构建命令：

```bash
cd /home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl
make px4_sitl_default -j4
```

对未修改源码重现补丁，可在确认三处源码与备份一致后运行 `python3 /home/hnuter/px4_ws_ros2/JEPA/prepare_firmware.py --apply`；脚本检查源码锚点，拒绝重复应用或覆盖不同备份。当前仓库不要重复应用。见 [firmware.patch](firmware.patch)。这不是硬件部署流程。

启动器直接运行构建二进制及私有 etc/rootfs，采用 4051_gz_hnuter 启动配置，避免 make gz_hnuter 的共用 rootfs 冲突。每次 `launch.json` 记录实际命令、实例、命名空间及源码/二进制哈希；`px4.log` 含运行时 ver all 和参数，ULog 与 `ulog_parameters.json` 保存实际参数。

## 工程、通信和保存边界

- PX4：分支 `hnuter-tail-motor-bench-sitl-20260812`，HEAD `49b60a5d8775655c1e6a9be722f76e9e4ab2517b`。ROS：分支 `hardware`，HEAD `abeb52c13c49fb601401a44a30b343fad0183133`。未切分支、reset、pull、commit 或 push。
- PX4 只改 HnuterControl.hpp/.cpp 和 msg/HnuterControlStatus.msg；备份在 `firmware_original/`，最终副本在 `firmware_patched/`。ROS 新代码全部在 JEPA；原有九处已跟踪修改核对见 `preservation_check.json`。
- 实例 0/1/2 分别使用 XRCE UDP 28888/28889/28890、ROS domain 73/74/75；每次随机 DDS namespace 与 GZ_PARTITION。GZ_IP=127.0.0.1、FastDDS loopback whitelist；全部 /fmu topic 重映射到本次 namespace。私有启动禁用 MAVLink，不打开串口；PX4 只连接本机 Agent。
- Agent UDP 监听为通配地址；这是独立端口/namespace/本机 DDS 进程隔离，不声称建立网络 namespace 或 firewall。检查端口占用、PX4 实例锁和自有 PID；只清理本次进程组。未连接真实飞控。

## 模型与独立辨识

所有 10 个刚体经 frame graph、旋转惯量与平行轴定理审计：标称总质量 **4.49999998 kg**，零关节位姿 CoM `[-0.1049171401,0.0010965586,-0.0040426450] m`，不是 base_link 单独的 3.0552478 kg。

在该 CoM 加入固定 0.9 kg 内部箱形负载（0.12×0.12×0.08 m），惯量对角 `[0.00156,0.00156,0.00216] kg·m²`。总质量 **5.39999998 kg**，参考位姿 CoM 不变；主惯量由 `[0.1262345492,0.2469026946,0.3264970403]` 变为 `[0.1277945502,0.2484630817,0.3286566522] kg·m²`。未在线辨识惯量。独立模型及逐 link 审计在 `generated/`；原模型未覆盖，关闭风，无额外扰动力。

本地 esc_rpm 是电机命令回显，估计器不用它。JointStatePublisher 提供实际物理关节速度/角度；程序仅读取时间、axis position/velocity，忽略消息中的模型姿态、质量和力。由固定 motorConstant、旋向、slowdown=10 及实际倾转轴重建力，通过 PX4 姿态转 NED，负 z 分量为向上推力。配置 `actuator_geometry.json` 不含质量/惯量。该遥测能力仅在仿真中验证，不能假定实机具备。

世界 g=9.8；计算 `Q=∫T_up dt`、`Y=9.8Δt−Δv_z`、`Q=mY`。不使用控制期望力或由待估质量反算的力。真值质量/速度只供离线评价；推力平衡残差包含未建模气动力，并非直接测得的真值合力误差。

参数：0.4 s 不重叠窗、遗忘因子 0.98、4.5 kg 先验权重 1、范围 3.5–7 kg。至少 20 个训练窗，最后 8 窗标准差/质量 <1.5%；冻结候选，再以至少 5 个独立后续窗检验相对 RMS ≤3%。不靠内部协方差验收。

关闭 UXRCE wallclock translation，使用共同仿真时钟；速度时间之前的关节/姿态样本年龄 ≤25 ms。低噪声处理缓冲 30 ms，噪声组 60 ms，不用未来信息。约 20 ms 遥测实测存在 28–32 ms 间隔，初始 25 ms 最大间隔曾拒绝校准；保存数据重放比较 0.3/0.4/0.5 s 窗后统一采用 **45 ms 间隔上限**，见 `timing_window_audit.json`。原拒绝结果保留。

## 状态机、保护及补偿

共用状态机：有效状态等待 → 平滑起飞至 1.3 m（10 s minimum-jerk，阶段共 12 s）→ 悬停 6 s → 校准 16 s → 检查 6 s → 公共评价 16 s → 悬停 5 s → 降落。C/C_shadow 校准为 0.08 m、0.4 Hz 正弦乘 sin⁴ 包络，B 为悬停；公共轨迹为 0.15 m、0.3 Hz。位置/速度/加速度来自同一解析表达。

窗口要求解锁、自由飞行、位置/速度有效、误差 <0.3 m、倾角 <20°、速度 <1.2 m/s、电机绝对指令 <0.9。任务退出门限为倾角 25°、水平偏移 1 m、起飞后误差 0.5 m 或速度 1.5 m/s、执行器 ≥0.95 持续 0.3 s、状态失效/超时 0.5 s。触发后停止激励与质量心跳，请求仿真 NAV_LAND 并记失败，不能保证所有异常恢复成功。

正常降落采用 2 s 余弦速度过渡至 0.7 m/s 向下，持续下降参考直到原 land detector 自动上锁。地面接触后保留下降意图是该仿真落地判据所需，未强制上锁或降低落地门限。早期停止下降参考的版本出现落地超时，失败日志保留。

m_eff 初值取 HNTR_MASS；质量请求仅编译在 CONFIG_ARCH_BOARD_PX4_SITL 中。B/C 在检查阶段 1 s 后，以 10 Hz 发送 command 31000、source_system=73、param2=54123 的时间戳消息；超过 0.5 s 未收到心跳则冻结，不写持久参数。Commander 对自定义命令可能打印 unsupported，实际接收者是 HnuterControl 独立订阅；以 ULog 的 effective_mass 验证生效。

实际积分器是加速度量纲，F=m(acc_des−g)。在本周期积分更新之前变换同一状态的积分器并重算加速度，检查新旧加速度限幅、积分限幅、自由飞行/Offboard/有效状态和起飞保护。变化率 ≤0.3 kg/s（控制采样时钟），不能保持力连续则拒绝该步。已有重力力矩前馈也使用有效质量；不改 PID。

ULog publication timestamp 有重复/抖动，直接差分瞬时质量斜率可超过 0.3；另报整体转换平均斜率。代数质量变换力误差与含反馈动态的相邻 force_body 变化分开统计。

## 对照与未验证范围

| 组 | 校准 | 估计 | 补偿 | 初始质量 |
|---|---|---|---|---|
| A | 悬停 | 关 | 关 | 4.5 kg |
| B | 悬停 | 开 | 开 | 4.5 kg |
| C | 竖直激励 | 开 | 开 | 4.5 kg |
| D | 悬停 | 关 | 关 | 5.4 kg，仅参考 |
| C_shadow | 竖直激励 | 开 | 关 | 4.5 kg |

五组同样增益、负载、初始模型位姿、公共轨迹和 1 倍速。每种子用于 Gazebo 与附加遥测噪声；调度与轨迹使噪声调用顺序不同，配对不是完美共随机数。

**严格零噪声未通过**：恒值传感器触发原 PX4 数据不更新检查，不能正常解锁；没有关闭检查制造成功。正式低噪声使用原 SDF 标准差×0.01，不能称为无噪声验证。噪声组保留原 SDF 噪声，附加转速 2 rad/s、角度 0.1°、竖直速度 0.01 m/s 高斯噪声，处理缓冲增加 30 ms；未覆盖任意网络延迟、丢包或时钟漂移。

原始未改标称模型/控制器完整飞行通过；+20% 负载原快速起飞触发水平保护。因此正式全部组统一平滑起飞，先以旧质量无补偿完成校准动作，再接通补偿。不能宣称原启动策略可安全起飞任意负载。2 倍速诊断失败保留，正式评价全部 1 倍速。

指标覆盖完整校准和公共评价阶段，RMSE 时间积分加权，最大误差不筛样本；PX4 估计位置与 Gazebo 真值（首个解锁样本固定平移对齐）分开统计。动作代价 ∫v_ref²dt 是运动强度代理，不是能耗。五种子均值/标准差、配对 bootstrap 仅描述本场景；未验证更大负载、未知推力比例、未知竖直外力及真实硬件。

## 交付物

- 模型：`model_audit.py`、`generated/`。
- 在线链路：`actuator_sensors.py`、`estimator.py`、`sitl_position_probe.py`。
- 固件：`prepare_firmware.py`、`firmware.patch`、源码/二进制备份、实际构建日志。
- 自动化：`run_sitl.py`、`run_matrix.py`；分析：`analyze_sitl.py`、`summarize_matrix.py`。
- 验证：`test_offline.py`、`final_offline_tests.log`（十项离线测试，不能替代 SITL）。
- `runs/sitl_*/`：实际命令、Agent/PX4/节点日志、配置、ULog、原始 CSV、指标和飞行图。
- `results/`：正式汇总、图及诊断索引；`source_changes.patch`、`SHA256SUMS.txt`：源文件差异与校验和。

runs、生成模型和原二进制保留在本机，被 .gitignore 排除；未向远端提交。复制交付物必须包含这些目录，Git diff 不包含原始实验证据。
