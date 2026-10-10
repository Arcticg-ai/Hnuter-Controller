# 复现与证据边界

在 `/home/hnuter/px4_ws_ros2` 执行。使用已有ROS2 Jazzy、Gazebo Sim8.11、MicroXRCEAgent；未安装组件或修改权限/安全设置。

## 测试与构建

```bash
python3 JEPA/revision3/articulated/test_spatial.py
python3 JEPA/revision3/articulated/test_consumer.py
mkdir -p JEPA/revision3/articulated/build
c++ -std=c++17 -shared -fPIC -O2 JEPA/revision3/articulated/TruthRecorder.cc -o JEPA/revision3/articulated/build/libJepaTruthRecorder.so $(pkg-config --cflags --libs gz-sim8)
```

PX4补丁基于c0d57b04的三个文件，当前实验分支已经应用，不要重复应用。prepare_spatial_firmware.py仅在显式备份后的对应旧源码上执行git apply检查后应用，不reset/stash。构建使用实验仓库 `make px4_sitl_default -j4`。初始消费者为b233b951，首次D必须使用这一历史实现才能复现未完成过渡；不能拿最终实现假称重现旧结果。恢复到新目录的方法见私有备份，不覆盖正在使用的用户工作树。

## 运行

每次独立rootfs、Gazebo分区、DDS namespace，禁用MAVLink、FastDDS仅回环。instance0、UDP28888、ROS_DOMAIN_ID73；严格串行，检查端口，不连接已有实例。launch.json给出实际命令、PID、二进制hash和配置，process_exit.json记录自有进程清理。

```bash
# 模型/时序诊断，固定标称控制器
python3 JEPA/revision3/articulated/run_diagnostic.py --label native_stamp_4ms --loaded --instrument --group A --noise low_noise --attitude-reference legacy --smooth-takeoff --seed 0

# 明确的负对照：2ms物理步、原250Hz IMU，不得用于消费者/估计入环
python3 JEPA/revision3/articulated/run_diagnostic.py --label native_stamp_2ms --loaded --instrument --group A --noise low_noise --attitude-reference legacy --smooth-takeoff --seed 0 --physics-step .002

# 偏置载荷 A / D；D已知参数只作为性能参考
python3 JEPA/revision3/articulated/run_diagnostic.py --label offset_A --loaded --instrument --group A --noise low_noise --attitude-reference legacy --smooth-takeoff --seed 0 --spatial-case offset_y
python3 JEPA/revision3/articulated/run_diagnostic.py --label offset_D --loaded --instrument --group D --noise low_noise --attitude-reference legacy --smooth-takeoff --seed 0 --spatial-case offset_y

# 目前整体收益门槛未过，B/C只能显式shadow
python3 JEPA/revision3/articulated/run_diagnostic.py --label offset_B_shadow --loaded --instrument --group B --noise low_noise --attitude-reference legacy --smooth-takeoff --seed 0 --spatial-case offset_y --shadow-only
python3 JEPA/revision3/articulated/run_diagnostic.py --label offset_C_shadow --loaded --instrument --group C --noise low_noise --attitude-reference legacy --smooth-takeoff --seed 0 --spatial-case offset_y --shadow-only
```

low_noise为SDF传感噪声标准差的原生1%，不是无噪声。异步通信调度不保证逐位重复，以历史日志绑定结果为准。

## 分析与可视化

```bash
python3 JEPA/revision3/articulated/analyze_balance.py <run-directory>
python3 JEPA/revision3/articulated/sensor_balance.py <run-directory> --discrete
python3 JEPA/revision3/articulated/analyze_consumer.py <spatial-run-directory>
python3 JEPA/revision3/articulated/make_closure_report.py
```

analyze_balance.py与sensor_balance.py都只离线评价。真正在线shadow为独立进程online_shadow.py，只读原生IMU/关节CSV、ROS态势NPZ与固定nominal_calibration.json，不读取loaded SDF、真值CSV、oracle请求。质量在此CoM/J阶段固定5.4kg，不联合拟合质量/推力比例。

校准16s分成独立训练/后续验证两半，另计异步计算时间。完整信息、物理性、独立残差、分半稳定性、独立零未知外扰条件和消费者范围都须满足。信息不足保持原模型，不能用先验补秩。积分要求每个4ms物理步的原生IMU与关节样本，缺失/不匹配即拒绝。

results/closure/all_attempts.json列出全部8次尝试，raw_evidence_manifest.json绑定CSV/ULog/配置及分析文件。flight_result.json沿用旧节点的mass字段在本阶段不使用；实际质量取ULog HNTR_MASS=5.4，空间参数取hnuter_control_status，在线辨识取online_shadow.json。B/C有效空间参数须始终为零。

Git只包含代码、校准、小型汇总和图；运行rootfs、ULog、大CSV、动态库、私有备份留本地。最早A/D未保存全部节点源码快照，这一限制保留；后续运行保存source_snapshot和额外源码hash。
