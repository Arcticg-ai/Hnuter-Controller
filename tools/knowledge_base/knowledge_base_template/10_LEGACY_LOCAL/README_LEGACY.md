# 旧工作区补充材料

这里保存原 `/home/hnuter/px4_ws_ros2` 中仍在的硬件 DRCDA/直接控制源码、配置和专项测试，
用于比较历史本地修改；它们不是 `03_CONTROLLER_REPO` 最新实机 v2 的运行替代品。

`historical_reports_NOT_CURRENT_EVIDENCE/` 是旧 5.4 GiB 日志树中提取的 Markdown/PDF 报告、
metrics/summary JSON 和 manifest，包含早期算法实现说明、评估失效更正与调参过程。
其中部分是**历史延迟模型实验**，仅用于追溯讨论，永远不再作为新的验证方式或当前算法有效性证据。
旧源码/历史配置即使仍有 delay 选项，也不构成启用授权。

本包完整保留现存 consolidated/reports 的数据，而旧工作区 5.4 GiB 原始日志/ULog及约6.6 GB视频
没有全量复制，避免与现有报告档案重复。`old_log_inventory.json` 明确列出每个旧日志文件是否打包。
因此这是调试知识库，不是整个主目录的无损备份；需要特定旧原始数据时应按清单再取。

旧报告中的绝对图片链接可能指向原日志树，未复制的旧 PNG 不会伪造成可用。
当前完整图片/原始数据优先使用 `03_CONTROLLER_REPO/reports/postfix_matched_20261012`。
