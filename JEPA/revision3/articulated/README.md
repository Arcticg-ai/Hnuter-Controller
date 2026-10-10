# 现有机型完整倾转路线

本阶段已完成8次新增独立SITL；结论与图见 [完整报告](results/closure/REPORT.md) 和 [机器可读汇总](results/closure/SUMMARY.json)。

- 4ms/250Hz原始传感链路的完整角动量闭合通过。固定标称运动部件校准与未知负载分离，Gazebo真值仅离线评分。
- 修正物理步与IMU积分时序后，中心未知负载的离线shadow通过；2ms/250Hz欠采样失败保留。
- Hnuter实际消费者已接入并验证参数到达、限速、无扰跟踪和退出冻结。首次D过渡未完成；定点修订经源码几何网格核验后评价前bias归零。
- oracle姿态改善伴随位置RMSE增加，整体收益未证明。总体门槛保持关闭；在线B/C均仅shadow，因信息不足拒绝更新（5/8、2/8）。未放行估计补偿或第二负载闭环矩阵。

未完成5种子统计、硬件、JEPA或神经训练。最初Git检查点的时移探索仍在diagnose_timing.py和results/timing_diagnostic.json；没有被当作已标定在线延迟。历史低运动门槛失败记录也保留。

复现与文件索引：[REPRODUCE.md](REPRODUCE.md)。
