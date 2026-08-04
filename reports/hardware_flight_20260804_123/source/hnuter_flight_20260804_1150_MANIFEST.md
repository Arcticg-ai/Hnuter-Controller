# Hnuter Hardware Offboard Flight Package

打包时间：2026-08-04（Asia/Shanghai）

## 试飞主观结果

- Hardware Offboard 已能够正常飞行。
- 当前控制手感仍然偏“软”，后续需要继续增大或重新辨识控制参数。
- 悬停过程中存在持续姿态偏差。

## 主飞行记录

- 控制 CSV：`hnuter_direct_hardware_1785815437.csv`
- ROS 日志：`python3_12411_1785815437806.log`
- Armed + Offboard 记录时长：约 183.24 s
- 最大相对高度：约 1.342 m
- 实际 Roll：-9.34° 至 +2.26°，平均约 -4.10°
- 实际 Pitch：-18.47° 至 +1.77°，平均约 -10.95°
- 目标 Roll/Pitch：均为 0°
- 平均姿态误差角：约 13.27°

## 飞行时实际加载参数

以下数值来自主飞行 ROS 启动日志，而不是根据当前文件推测：

- `allocator_force_sign = [+1, -1]`
- `theta_limit_deg = 90`
- `direct_KR = [1.8, 1.8, 3.8]`
- `direct_Domega = [1.35, 1.35, 2.5]`
- `direct_tau_limit = [0.9, 0.9, 1.8]`
- `direct_pos_Kp_ned = [3.0, 3.0, 8.0]`
- `direct_pos_Kd_ned = [2.1, 2.1, 4.0]`
- `direct_pos_Ki_ned = [0.0, 0.0, 3.0]`
- `HNTR_PITCH_BIAS = 0.09`
- `HNTR_TAIL_SIGN = +1`
- `HNTR_TAIL_COMP = 0.0`

注意：这次能够正常飞行的记录实际使用 `allocator_force_y_sign=-1`。

## 包含内容

- `hnuter_external_direct_controller_hardware.py`：试飞后当前 hardware 控制代码。
- `config/hnuter_direct_tuning.json`：当前在线调参文件。
- `hnuter_tilt_system_id.yaml`：实机倾转系统辨识参数。
- `README_tilt_system_id.md`：倾转辨识说明。
- `hnuter_logs/external_control/`：本轮试飞的控制 CSV。
- `hnuter_logs/ros/`：对应 ROS 节点日志。

其中 `1785815437` 为本轮主要完整飞行；`1785815158`、`1785815326` 和
`1785815343` 为同一轮测试前的短会话，一并保留用于排查启动和模式切换。

工作区中未找到对应的 PX4 `.ulg`、ROS bag 或 MCAP，因此本压缩包不包含飞控
原生 ULog。姿态、位置、执行器命令和 PX4 执行器回读均已记录在控制 CSV 中。

## 来源与校验

- 远程 main 基准提交：`92006e525b1a9f02001b27faf0d0b747e4936cee`
- hardware SHA-256：`ff24ad29809d60714e8e9e41efbf715fe72cb655cb15658567596c0650257a98`
- tuning JSON SHA-256：`1e90043bba8411ab51044d1ff99b74fe4eee470ae471190559c3dc29cadcd033`
- 主飞行 CSV SHA-256：`7f08c41c7b5f26024d382db9b94af1ed1e056552f1aa4f55b234f23be137c895`
- 主 ROS 日志 SHA-256：`601a9bd4eb7424440b191e7255f21d3000a032b02480eaa376c08164488052d1`
