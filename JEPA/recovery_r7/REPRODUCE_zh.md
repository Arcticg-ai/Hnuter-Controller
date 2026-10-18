# R7复现与范围

研究目录JEPA/recovery_r7，R6基准目录见START.json。原始日志仅保留本机，未上传。review包包含代码、登记、摘要结果和哈希，完整本地包还包含分析时间序列与仿真插件；大体积原始ULog/JSONL/真值CSV由raw_manifest.json定位。R6冻结模型及其训练/复现包在此前交付中，不以新模型冒名替代。

依赖使用已安装hnuter-adaptation:20261013镜像、numpy/pandas/pyulog；PyTorch仅在该镜像内离线运行。PX4复用冻结jepa-px4-gps3-worktree，SHA及二进制见运行launch.json；不读旧任务会话存储。五个插件使用R6已审计构建，源和构建配方在R6 vendor_plugins目录，R7 FROZEN_SOURCES列出实际文件哈希。无需额外下载、无外部数据传输。

只读重分析：

    python3 audit_r6.py
    python3 analyze_baseline.py
    python3 -m unittest discover -s . -p test_gated_adapter.py -v

R7-B回放在镜像中将R6目录只读挂载/r6，R7挂载/r7，使用/opt/venv/bin/python /r7/replay_ablation.py，OPENBLAS_NUM_THREADS=1、OMP_NUM_THREADS=1。输出ablation_metrics.csv、ablation_timing.csv、ablation_audit.json；源哈希见offline_frozen_sources.json。原始未来动作仅用于回溯条件预测，不能冒充前瞻控制。

新采集应复制到新的结果目录并独立登记，不删除/覆盖本批runs、BATCH_COMPLETE、BATCH_STOPPED等门禁来重跑。批次运行run_bounded.py先unarmed，再check_unarmed.py，四条顺序见MATRIX.json，每条post_run.py检查；campaign.py用于第一条已通过后的剩余三条。原71–72s失败保留；91–92s结果单独列出。Safety/data通过不等于恢复性能通过。

当前门禁始终learning_control=false。没有本轮学习控制A/B、没有实机飞行、没有在线骨干训练。R7开发诊断不能外推到舵机时间常数变化、未知质量/CoM/惯量或持续外扰的自动适应成功。
