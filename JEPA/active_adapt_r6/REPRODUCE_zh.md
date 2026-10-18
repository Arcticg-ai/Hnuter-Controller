# 复现与文件说明

本目录为 R6；不覆盖 R5，旧的研究失败结果保持原样。原型冻结提交 7e43ab0；首条验收记录修复提交 4160322。后续完整结果提交见交付 receipt。

## 现有环境

- 已安装 Docker 镜像 `hnuter-adaptation:20261013`：ROS Jazzy、gz-sim8、CPU PyTorch 2.6.0；未安装额外软件。
- 冻结 PX4 工作树 `/home/hnuter/Documents/Codex/2026-10-13/task/jepa-px4-gps3-worktree`，提交与可执行文件 SHA 见 `runtime/FIRMWARE_MANIFEST.json`。不是原有脏工作区，也没有改主飞控逻辑。
- 运行需该本地镜像、PX4 构建及镜像内原 Controller 依赖；交付包没有捆绑数 GB 的镜像/固件。它可在本机复核，并非不依赖环境的独立飞行软件。
- 容器 network=none、无硬件设备映射，DDS 29888/ROS domain 83、独有 GZ partition。全局协作锁 `/tmp/hnuter-sitl-research.lock`，每次还拒绝已有 Docker 或 PX4/Gazebo 进程。

## 数据与划分

训练：R4 的 nominal/mass/CoM/inertia 四条完整轨迹，每条 495 个高度重叠窗口，共 1,980；验证：R4 combo 一条，495 窗口。`results/data/manifest.json` 提供路径及源文件 SHA。窗口数量不能替代独立轨迹数量。

新测试八条，参数组独立于训练。`runtime/MATRIX.json`、`runtime/results/experiment/cases.json` 和模型 SDF 定义全部固定条件。种子 0/2 分别配主动/常规激励；这些仿真噪声种子在旧研究中出现过，不能把本轮称全新噪声分布泛化。

原始飞行日志在 `runtime/results/experiment/runs/<case>/`，原始未解锁日志在 `runtime/results/unarmed/run/`。原始文件不进 Git/远程，也不捆绑进轻量交付包；清单列出本机路径和 SHA。低容量训练 NPZ、六个 checkpoint、原始逐窗误差 CSV、汇总与代码收入本地复现包；远程交付只含经过筛选的代码/模型/配置/小型结果。

## 只读复核

```bash
OPENBLAS_NUM_THREADS=1 python3 test_contract.py
OPENBLAS_NUM_THREADS=1 python3 runtime/test_bounded_port.py
python3 native_audit.py
python3 software_stress.py
python3 aggregate.py
```

`native_audit.py` 要求原始 ULog/轨迹仍在本地；`software_stress.py` 要求原始影子流。它对已记录输入添加合成标签脉冲，不能称真实风试验。`aggregate.py` 只需要已生成的每条 evaluation/native JSON。

离线重放某条（仿真全部停止后再运行，避免占资源）：

```bash
docker run --rm --network none --cpus 2 --memory 3g --user 1000:1000 \
  -e OPENBLAS_NUM_THREADS=1 -e PYTHONDONTWRITEBYTECODE=1 \
  -v "$PWD:/r6" -w /r6 --entrypoint /opt/venv/bin/python \
  hnuter-adaptation:20261013 evaluate.py s0_00_mass_active
```

动作时间约定见 MODEL_zh.md；evaluate 的真值读取只发生在飞行结束后。

## 重新训练或重新飞行

必须先复制到新的实验目录，保留本交付目录。脚本故意拒绝覆盖已有 data/models/run；不要为了复现删除本轮原始记录或停止标志。可在新的副本中把旧输出移到副本外的参考目录，留下代码、配置、训练 NPZ；然后在上述隔离容器里执行 `train.py`、`validate.py`。比较模型张量而不依赖 ZIP 序列化文件的偶然元信息。

重新飞行必须有新的登记、结果目录和冻结清单：复制配置/SDF到新 runtime/results/experiment，保留 `results/models`，执行 `freeze.py` 后本地提交；再 `runtime/run_bounded.py unarmed`，`runtime/check_unarmed.py`。只有检查通过才执行 MATRIX 的首槽和 `post_run.py`，之后 `campaign.py`。本轮脚本已经包含完整结果，会主动拒绝直接重跑。本交付不自动启动新试验。

首次冻结清单在 `results/first_run_frozen_sources.json`，后续 `runtime/FROZEN_SOURCES.json` 包含模型、标准化统计和 SDF。每条运行的 `source_snapshot/` 与 `launch.json` 记录实际源文件/固件/模型散列。`runtime/SOURCE_MANIFEST.json` 是继承的老插件来源清单，不是 R6 冻结表；不得用它替代本轮冻结证明。

## 已知限制

R6 无学习控制飞行；oracle 未重验，旧恢复失败未被改写。只有两类新环境，未新增惯量/舵机参数场景，未做真实风或飞行中阶跃。跨激励的配对是同噪声种子和准入统计匹配，不是同一 Gazebo 全状态分叉。所有三种方法的影子开/关对照共享同一实际轨迹，因此可用于预测比较，不能代表各自闭环控制轨迹。

## 诊断插件来源

`vendor_plugins/` 保存继承的 AuditedJointPositionController、BoundedForce、ClockAudit 源码及原提交/散列。另有 runtime/TruthRecorder.cc、seeded_server.cc。五项依赖已在现有镜像内重编译并完成动态加载或 RNG 探针检查，见 results/dependency_rebuild_receipt.json；重编译产物未替换本轮使用的二进制。可用 `vendor_plugins/rebuild.py <全新输出目录>` 复核，编译路径差异可能使二进制字节散列不同。
