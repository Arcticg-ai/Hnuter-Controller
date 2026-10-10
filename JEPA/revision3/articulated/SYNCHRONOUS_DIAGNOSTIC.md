# 同步多刚体诊断检查点

2026-10-10，现有机型，普通位置入口，新增三次独立 SITL，均完成降落并解除解锁。原始资料位于本目录 `runs/`，只在本地保存；小型汇总在 `results/synchronous_diagnostic/`。

1. 4 ms / 固定标称控制器：94个全阶段0.4秒窗口，完整多刚体角动量与实际外力矩平衡 RMS 为 [0.000104,0.000131,0.000028] N·m。重建的几何、刚体速度、内部动量、外力矩与直接逐刚体记录一致，排除本轮坐标/施力点/slowdown双计错误。`ExternalWorldWrenchCmd`力矩以link原点为参考；`AddWorldForce`已补偿link自身COM偏置，不得再加一次。
2. 4 ms / 原始IMU与关节消息：95窗，前47窗拟合，后48窗独立验证。沿用原来的0.02 N·m门槛，无数据筛选或时移拟合。原始IMU角速度与物理真值误差约8.7e-6 rad/s。完整运动修正显著降低残差；进一步修正积分离散化后，后续窗口残差为 [0.000230,0.000155,0.000435] N·m。shadow接受，数据秩8，分半稳定性0.0134（门槛0.5）；CoM误差约0.2微米、对角惯量最大绝对误差0.000488 kg·m²。这里只是低噪声SITL记录离线shadow，不是硬件精度或在线闭环结果。
3. 2 ms物理步长、原机型250 Hz IMU：94窗，物理真值平衡更好，但传感器漏采交替物理步，插值不能恢复高频执行器响应。独立残差 [0.0404,0.0200,0.0975] N·m，shadow拒绝，秩7。此失败保留，不能把“步长减半”声称为同等传感链路的精度改善。

## 时序推导

Gazebo在步初按当前转速/姿态施力，物理更新后发布关节和IMU。原生IMU加速度在步末姿态中表达，等价于该步的原点速度增量。因而区间[k,k+1]使用τ[k]，加速度用 `R[k]ᵀ R[k+1] f[k+1]` 转回步初体坐标。关节构型修正也取步初，角动量取首末端点。所有数据须等到窗口终点到达后才可处理，不能作为无延迟控制反馈。仅适用于此已核实的仿真离散语义；硬件需要重新标定时间和采样模型。

梯形积分虽然后续残差也低于门槛，却产生约0.030 kg·m²的俯仰惯量误差；记录保留用于说明低预测残差不能单独证明参数正确。

GZBridge `imuCallback` 用回调时 `hrt_absolute_time()` 替代传感器消息时间戳；控制用角速度还有滤波。新增 `record_sensors.py` 只订阅带原生时间戳的IMU和关节消息，不订阅真值、不读载荷。`TruthRecorder.cc`完全独立，只记录，不施力，真值文件仅供离线评价。

## 控制消费者前置审计

现有分配器控制力矩到物理质心力矩的小信号映射不是单位矩阵：滚转/偏航比例约1.242/1.241。直接添加物理力矩会产生约24%的尺度误差。独立标称几何静态校准在限定邻域的100个样本中，局部逆映射最大算子误差0.147%；该静态结果仍不能代替实际执行器动态和oracle闭环验证。

## 复现

```bash
c++ -std=c++17 -shared -fPIC -O2 JEPA/revision3/articulated/TruthRecorder.cc -o JEPA/revision3/articulated/build/libJepaTruthRecorder.so $(pkg-config --cflags --libs gz-sim8)
python3 JEPA/revision3/articulated/run_diagnostic.py --label native_stamp_4ms --loaded --instrument --group A --noise low_noise --attitude-reference legacy --smooth-takeoff --seed 0
python3 JEPA/revision3/articulated/analyze_balance.py <run-directory>
python3 JEPA/revision3/articulated/sensor_balance.py <run-directory> --discrete
python3 JEPA/revision3/articulated/allocation_contract.py
```

安装无变更，原SDF/世界/控制增益未改；DDS仅回环、MAVLink关闭，独立Gazebo分区。所有自有进程已回收。未声称完成CoM/J闭环；未知恒定力矩偏置仍必须有独立约束，没有该约束的负对照被拒绝。

源码依据：[Gazebo电机](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.11.0/src/systems/multicopter_motor_model/MulticopterMotorModel.cc)、[施力点](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.11.0/src/Link.cc)、[物理更新](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.11.0/src/systems/physics/Physics.cc)、[IMU发布](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.11.0/src/systems/imu/Imu.cc)。
