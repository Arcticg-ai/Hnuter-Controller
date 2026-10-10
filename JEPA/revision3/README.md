> 后续现有机型完整倾转验证已执行：见 [articulated 完整报告](articulated/results/closure/REPORT.md)。下文及原 results 保留为第一阶段门槛失败的历史记录，不代表后续尚未运行。

# Revision3：CoM / 惯量辨识的有界第一步

**状态：离线验证已完成；模型门槛未通过，没有启动新 SITL、没有改动 PX4 固件或将估计值入环。**

- [报告](results/REPORT.md)、[可视化](results/revision3_summary.png)
- [模型推导和实际控制消费点](MODEL_AND_CONTROL.md)
- [八例受限串行计划与门槛](results/sitl_gate.json)
- [变更清单](CHANGES.md)

本目录不读取真值参数来生成在线候选。`integral_estimator.py` 接收经标定的实际传感器 wrench 和状态，提出 shadow 候选；`assembly.py`、`replay.py` 中的真值组装只作模型配置与离线评价。`control_contract.py` 是离线接口实现，尚未接入真实固件。

复现（使用已安装 Python/numpy/scipy/matplotlib/pyulog，无安装）：

```bash
cd /home/hnuter/px4_ws_ros2
python3 JEPA/revision3/run_checks.py
```

预期：所有离线阶段返回0，最终输出 `BLOCKED_BEFORE_LAUNCH`，进程返回2。这是有记录的模型/控制门槛拒绝，不是启动错误。整个脚本不创建 DDS、PX4、Gazebo 进程，不连接飞控。

单独复绘（不重新运行实验）：

```bash
python3 JEPA/revision3/make_report.py
```

固定回放三个旧日志：原位置入口 legacy A/seed0、C/seed0、失败 C/seed1。真实日志留在 `JEPA/revision2/runs`；本目录 `results/replay_summary.json` 保存路径及 SHA-256。不要把此回放写成 revision3 实飞成功。

本次只创建本目录；两仓库既有脏文件、普通位置入口、revision2 代码保持原哈希。没有修改增益或安全限值，没有清理用户进程、安装软件、操作硬件或发布代码。可视化包含合成与回放两个证据层次，均标注。
