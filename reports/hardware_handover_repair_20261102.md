# Position 起飞到 DRCDA Offboard：接管修复与无延迟 SITL 验证

## 结论

已在真实 PX4/Gazebo 无延迟仿真中复现一种与实机记录相符的失败链：
接收线程仍有新状态，但控制缓存未及时消费；切入 Offboard 时旧版以旧缓存判定
`missing/stale position`，未输出首帧就请求 Position。飞控记录随后出现尾电机配平
命令中断和快速抬头。修复版在同一调度故障注入条件下成功接管并完成后续任务。

这证明了该软件失败路径及修复效果，不等于已经确定实机当时缓存停更的唯一原因。
没有本次实机 ULog；仿真机体和电机标定也不是实机物理模型。没有修改远程机载文件、
重启实机控制器或修改实机飞控参数。代码的 Git 发布不代表已经部署到机载电脑。

## 修改范围

修改源：`tools/templates/drcda_hardware_v2.py.in`。
独立可运行文件：`controllers/hardware/hnuter_external_direct_drcda_v2_hardware.py`。

1. 位置、角速度、姿态分别消费最新有效消息，不再因跨话题采样时间偏差把整组状态挡住。
   30 ms / 100 ms 的跨话题偏差改为诊断信息，不作为接管否决条件。
2. Armed + Offboard 的模式边沿先消费接收队列，再捕获当前完整位姿。
   避免模式回调先于状态定时器时，对旧控制缓存作出错误判断。
3. 数据刷新与控制计算分开；刷新过程中不递归运行控制、也不使用半更新的状态。
4. 单个乱序/非法时间戳样本被丢弃并记录；后续该话题合法新消息到达后解除输入错误记录，
   不再让一次乱序永久阻塞所有状态更新。真实时钟重置不会被伪装成新鲜状态。
5. 故障后仍更新位置、姿态、角速度。新增每秒状态健康事件及故障时接收队列详情，
   不再把冻结姿态误当作实际反馈。

维持手动解锁、手动切模式、切入时完整姿态参考和已有 0.5 s 命令混合。
首帧沿用有效 Position 执行器快照，不新增姿态角、配平误差或参数一致性准入条件。
没有再次调增益，也没有改变分配器目标函数。

真实输入缺失/过期、非有限数值、计算失败等故障处理仍保留；不是不顾有效输入强发命令。
仅靠本次修复也不能保证在真正通信中断、无有效执行器快照时仍能无扰接管。

## 仿真条件

- 固件目录：`/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl`，未切分支/修改固件。
- `ver all`：`836a4602ce5916ed84cdf7e46c655678b19b099b`，PX4 1.16.0。
- 二进制 SHA256：`c5346e77f1026f3028da37e997e964828b584744742266a9993bf2172c491d9d`。
- SDF SHA256：`ce982dc1d1f18836714dff3a0551c883b8f7769c5e406efaa930535e2d746a80`。
- 不使用延迟固件，不注入执行器纯延迟；保留原生 Gazebo 电机动态和关节 PID。
- 只读冻结固件/模型，独立仅回环网络命名空间，DDS domain 174、端口 8894，不接触实机。
- 使用 hardware-v2 控制路径；适配的仅是 Gazebo 几何/电机映射、合成遥控入口和时钟。
  两版使用完全相同的适配参数，参数文件 SHA256 均为
  `ea8dde36a46b4fa8fcb575d77275c47dbbd0c0be34fcfb778bb70b87386c504e`。
- 旧版是本次实机运行版本 `f26853c`，不是更早的已飞版本。
  旧版源 SHA256：`00835b1d51c5634e744fb686cba899f66261166bc7c97d54c6cc14a5440e4cdc`；
  修复源 SHA256：`e2a8ab50bcd8dd68be7a720e77c4a3b75da2ab21220dd3c9598400c2d4ffa31c`。

## 实验流程和结果

仿真操作器先选择 Position，再解锁、以模拟油门 0.4 起飞，约 3 m 悬停后切入 Offboard。
飞控 ULog 确认起飞阶段 nav_state=2，没有使用自动起飞命令。
随后包括定点保持、偏航往返、横向移动往返、两次切回 Position 和两次重新进入 Offboard。

| 条件 | 旧版 | 修复版 |
| --- | --- | --- |
| 正常调度，完整任务 | 通过，3次接管 | 通过，3次接管 |
| 切换前暂停状态消费定时器350 ms，接收线程和物理模型不变 | 接管失败，未输出控制 | 通过，3次接管 |
| 相同旧版失败，推迟仿真降落请求2 s且保持模拟遥控中位 | 仍复现立起 | 此项为旧版因果核对，不重复修复版 |

暂停是显式的控制线程调度故障注入，不是舵机延迟模型，也不是声称实机发生了同样的350 ms暂停。
旧版失败记录保留为 INVALID/no samples，含原始故障和 ULog；不能把“无控制 CSV”当作试验没运行。

### 修复版接管指标

以下为接管后2秒，相对每次切入实际位姿的最大变化，不是整段轨迹 RMSE。

| 条件/接管次序 | 最大位置变化 | 最大 SO(3) 姿态变化 |
| --- | --- | --- |
| 正常/首次 | 0.01248 m | 0.547° |
| 正常/第二次 | 0.05511 m | 5.874° |
| 正常/第三次 | 0.01401 m | 1.462° |
| 消费暂停/首次 | 0.03705 m | 1.954° |
| 消费暂停/第二次 | 0.02068 m | 0.982° |
| 消费暂停/第三次 | 0.01247 m | 1.209° |

6次接管均记录首帧电机/舵机命令相对捕获快照的最大差值为0。
这证明的是命令首帧连续，不代表测得物理扳手完全连续。
正常组第二次接管仍有约5.9°姿态瞬态，不能称为零超调，也不能凭单次运行声称性能统计优势。

### 独立 ULog 核对：为何会立起

在不立即发送降落命令的失败对照中：

- t=0：飞控进入 Offboard；t=24 ms：返回 Position。
- 切换前尾电机归一化控制量约0.6873；切换后首批执行器日志出现0。
- t≈0.1 s：俯仰约3.88°；0.2 s：25.22°；0.3 s：55.89°；0.5 s：88.63°。
- 2 s内 SO(3) 姿态变化峰值约179.90°。后续还出现飞控自动模式变化，完整保留。
- 操作器在观察结束后才请求降落；ULog 的 LAND 模式在 t≈2.028 s 才出现。
  因此前0.5 s的抬头不是本轮降落命令造成的。

同样消费暂停条件下，修复版尾电机切换前后约0.686，t≈0.5 s俯仰约0.165°，
首次接管2 s内 ULog SO(3)变化峰值约1.95°，没有立即退出 Offboard。

执行器 ULog 采样较疏，报告中的控制量以源记录为准，不宣称精确到毫秒的断点。
`ulog_analysis/summary.json` 同时保存查询时刻和实际匹配的执行器采样时刻。
模式退出和配平命令中断在这里得到实测支持，但尚未穷尽飞控内部所有发布者的贡献。

## 测试与文件

- hardware + hardware_repair 离线回归：272 passed、1 skipped。
- 独立ROS网络、真实DDS接收线程专项：1 passed（上述跳过项另行执行）。
- 生成器一致性和 `git diff --check` 检查。
- 旧版/修复版仿真参数一致；本轮不引入增益变化来掩盖切换问题。

工作区根目录：`/home/hnuter/Documents/Codex/2026-10-11/drcda-gazebo`。

- 正常组：`output/handover_20261102/campaign`。
- 调度故障组：`output/handover_20261102_consumer_pause/campaign`。
- 排除降落干预的对照：`output/handover_20261102_fault_observation/campaign`。
- 每个 case 包含 `operator_events.jsonl`、`transitions.jsonl`、控制器事件、配置、结果和 `rootfs/log` ULog。
- 独立飞控曲线：`output/handover_20261102/ulog_analysis/pitch.png` 和 `tail_command.png`，另有PDF。
- 正常组独立 x/y/z、roll/pitch/yaw 期望实际图：`campaign/handover_analysis`。
- 复现入口：工作区 `prepare_handover_20261102.py`；支持 `--consumer-pause`、`--observe-fault`。
- 仿真执行器：仓库 `tools/experiments/hardware_repair_runner.py`，使用各 campaign 已冻结的 manifest。
- 分析：工作区 `analyze_handover_20261102.py`、`audit_handover_ulog_20261102.py`。

正常组：

```bash
source /opt/ros/jazzy/setup.bash
source /home/hnuter/px4_ws_ros2/install/setup.bash
cd /home/hnuter/px4_ws_ros2_consolidated
OPENBLAS_NUM_THREADS=1 python3 -m tools.experiments.hardware_repair_runner \
  --output /home/hnuter/Documents/Codex/2026-10-11/drcda-gazebo/output/handover_20261102/campaign \
  --execute --repeat 1 --group baseline --group ref_fix --disturbance none \
  --firmware-sha256 c5346e77f1026f3028da37e997e964828b584744742266a9993bf2172c491d9d
```

已完成的 case 会跳过；新增重复可提高 `--repeat`。调度故障对照需要使用对应 campaign，
旧版预期失败后工具会停止，修复版须另用 `--group ref_fix` 执行，不能删除失败记录换取通过。
