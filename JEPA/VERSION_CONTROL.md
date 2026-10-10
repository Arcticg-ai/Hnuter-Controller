# 实验版本保护边界

本实验以 Controller `hardware` 的 `abeb52c13c49fb601401a44a30b343fad0183133` 和 PX4 `hnuter-tail-motor-bench-sitl-20260812` 的 `49b60a5d8775655c1e6a9be722f76e9e4ab2517b` 为基线，在 `codex/payload-selfcalibration-20261010` 实验分支保存。原分支历史不移动，不合并main，不force push。

提交范围：JEPA代码、参数/模型、推导、图与小型结果汇总；PX4仅HnuterControl.cpp/.hpp和HnuterControlStatus.msg的SITL质量更新/冻结实现。revision3保留离线与未集成状态，不声称完成新闭环飞行。

排除：用户原有9个Controller脏文件、无关视频/实验目录、原始ULog与完整运行rootfs、缓存、二进制、私有备份。相关原始日志完整保留本地且有索引/哈希；克隆仓库不会自动获得这些大文件，复现回放需另取本地证据归档。

Git整理前已保存两仓库真实保护引用、HEAD完整可达历史bundle、staged/unstaged二进制patch、脏文件原文、完整JEPA现场及当前SITL二进制。它是整理前现场快照，不声称为整个研究开始前的完整快照。具体本地路径、提交SHA、推送验证与恢复步骤在交付的本地Git保护报告中。

PX4测试仓库origin是本机主PX4仓库；其origin对应GitHub的Arcticg-ai/PX4-Autopilot-Hnuter。保护操作不改变任何remote配置，实验分支逐级推送，主仓库工作区不切换。
