# 迁移到远程主机

本包不自动连接/覆盖远程目录，也不启动控制输出。历史主机地址可能变化。

## 解压与校验

将 `.tar.gz` 与 `.sha256` 传到你选定的目录，再运行：

```bash
sha256sum -c hnuter_drcda_knowledge_base_20261025.tar.gz.sha256
tar -xzf hnuter_drcda_knowledge_base_20261025.tar.gz
cd hnuter_drcda_knowledge_base_20261025
python3 08_TOOLS/verify_bundle.py
```

若包文件名带后缀，使用实际文件名。源目录保留原样，不应解压覆盖旧控制仓库。

## ROS 与 Python

已知本机/历史机载系统使用 ROS 2 Jazzy。真实运行需要 rclpy、numpy、固件兼容 px4_msgs，
关节接口另需 sensor_msgs。离线分析通常还需 scipy、matplotlib、pyulog、pytest；参考
`03_CONTROLLER_REPO/requirements-px4-venv.txt`，不要将 ROS 的 rclpy 当作普通 pip 包随意安装。
虚拟环境不可直接跨架构复制；新建时建议 `python3 -m venv --system-site-packages px4-venv`。

本包 `07_DEPENDENCIES/ros2_sources/px4_msgs` 的 VehicleStatus.MESSAGE_VERSION=1，
OffboardControlMode 包含 direct_actuator。旧远程默认另一 overlay 的 v4 曾不匹配，
必须 source 与本次飞控消息定义一致的 overlay，不能仅看话题名字存在。

可在新建工作区构建所带消息源码（已有兼容 overlay 就复用，不重复安装）：

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/hnuter_msgs_migration_ws/src
cp -a 07_DEPENDENCIES/ros2_sources/px4_msgs ~/hnuter_msgs_migration_ws/src/
cd ~/hnuter_msgs_migration_ws
colcon build --packages-select px4_msgs
source install/local_setup.bash
python3 -c 'from px4_msgs.msg import VehicleStatus, OffboardControlMode; print(VehicleStatus.MESSAGE_VERSION, hasattr(OffboardControlMode(), "direct_actuator"))'
```

期望输出 `1 True`。这仅验证接口特征，不证明整个固件接口完全一致。
包同时保留 px4_ros_com 源码供需要时参考，不要求运行 hardware 时构建它。

## 创建独立运行副本

原源码中有本机绝对固件路径。离线/硬件副本无需本机仿真固件；使用：

```bash
python3 08_TOOLS/prepare_working_copy.py --output ~/hnuter_ctrl/drcda-debug-20261025
```

副本已有目录会拒绝覆盖。它不改算法，只替换已知本机路径，加入消息源码，生成路径修改记录。
要跑 SITL，额外指定**完整无延迟固件仓库**：

```bash
python3 08_TOOLS/prepare_working_copy.py \
  --output ~/hnuter_ctrl/drcda-sitl-20261025 \
  --firmware ~/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl
```

工具检查 Hnuter SDF 不包含已知纯延迟插件标志。`04_FIRMWARE` 只有相关源码快照与 dirty patch，
不能直接 make；完整仓库请依据 `09_PROVENANCE/tail_sitl_no_delay` 的 remote/HEAD 恢复并核对补丁。
不能将补丁无审查覆盖不同版本，不能把当前固件 HEAD 视为所有旧实验版本。

## 实机只监视入口

在 ROS 和匹配 overlay 已 source 的终端：

```bash
export DRCDA_WORKING_COPY=$HOME/hnuter_ctrl/drcda-debug-20261025
export PX4_MSGS_SETUP=$HOME/hnuter_msgs_migration_ws/install/local_setup.bash
bash 08_TOOLS/run_hardware_monitor.sh --check
bash 08_TOOLS/run_hardware_monitor.sh
```

该脚本强制 ENABLE=0、自动任务关闭，不发送心跳或执行器。需要持续 Offboard 心跳及人工接管时，
使用源码文档中的 ENABLE=1 命令，但它是**实际输出授权模式**，不能当作只读检查：

```bash
cd ~/hnuter_ctrl/drcda-debug-20261025
source /opt/ros/jazzy/setup.bash
source ~/hnuter_msgs_migration_ws/install/local_setup.bash
HNUTER_LOG_DIR="$PWD/hnuter_logs/manual_$(date +%Y%m%d_%H%M%S)" \
HNUTER_TUNING_FILE="$PWD/config/hardware/hnuter_drcda_v2_hardware_tuning.json" \
HNUTER_HARDWARE_DRCDA_ENABLE=1 HNUTER_HARDWARE_TASKS=0 \
/usr/bin/python3 controllers/hardware/hnuter_external_direct_drcda_v2_hardware.py
```

不自动 Arm/切 Offboard；飞手 Armed+Offboard 后会直接驱动电机舵机。该最新版本尚有未解释退出事件，
先完成拆桨/台架和受限验证，不把此命令当作自由飞行许可。运行前检查未遗留第二控制进程或第二 DDS Agent。
示例 JSON 是仓库默认，不是今日参数；上次单次运行的偏置 0.09、pitch D 1.5 也不能直接当今日值。

## 离线回归

```bash
cd ~/hnuter_ctrl/drcda-debug-20261025
source /opt/ros/jazzy/setup.bash
source ~/hnuter_msgs_migration_ws/install/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 HNUTER_LOG_DIR=/tmp/hnuter_kb_tests \
  PYTHONPATH="$PWD:$PYTHONPATH" /usr/bin/python3 -m pytest \
  tests/hardware/test_drcda_v2_hardware.py tests/common/test_hnuter_drcda.py -q -p no:cacheprovider
/usr/bin/python3 tools/build_hardware_drcda_v2.py --check
```

不要用 `PYTHONPATH=$PWD` 丢掉 ROS 路径。测试通过不是控制实效证明。
仿真入口参见 `tools/experiments/run_maneuver_revision.py --help`，可用 `--gui`，
显式 `--controller-python /usr/bin/python3`，输出指定新目录。它会自动启动仿真并执行任务，不能用于实机。

## 日志与网页

每次唯一 HNUTER_LOG_DIR 保存 CSV、events/drcda_v2.jsonl、config/runtime JSON 与 console；
同时留存 ULog 和真实参数。修复后的独立 DDS observer 源码在 `05_HARDWARE_EVIDENCE`，不是控制器必需依赖。
网页服务的令牌不随包分发。历史网页 JSON 修改不会被实机 v2 热加载，需在未接管时重新启动并确认有效配置。
网页 hntr_att 是固件目标，不等于新版 DRCDA 捕获参考，分析时以控制器日志为准。
