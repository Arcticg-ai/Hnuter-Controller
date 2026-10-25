# DRCDA 调试知识库与源码提交范围

2026-10-25：本地构建了私有迁移知识库，入口是包内 `00_START_HERE.md`。
包含当前工作树、参数、论文、报告/图表、现存 consolidated 报告原始数据、实机问题记录，
以及六个相关主会话的可见文字（Markdown / JSONL，记录时间范围 2026-07-27 至 2026-10-25）。
对话导出不含内部推理、系统/开发者指令或工具原始输出；可识别凭据已脱敏。
未取回的最新实机 ULog 不在包内，不能视为已修复飞行异常。

## 本地归档

原始知识库保持为本次提交前的冻结快照，不自动重写或公开上传：

```text
/home/hnuter/Documents/Codex/2026-10-11/drcda-gazebo/deliverables/
  hnuter_drcda_knowledge_base_20261025.tar.gz
  hnuter_drcda_knowledge_base_20261025.tar.gz.sha256
  hnuter_drcda_knowledge_base_20261025/00_START_HERE.md
```

压缩包 SHA256：`3fb04c4170df1da1a053a3e3eb7c86d693097a4fafc4058c5ca82fcf62c97af1`。
该包约 790 MiB，解压约 2.1 GiB；4325 个内容文件通过独立解压哈希校验。
打包源码和路径适配副本分别运行 311 项回归测试通过，不等于飞行验证。
旧工作区 5.4 GiB 日志树只提取历史报告并列出逐文件清单，不是主目录全量备份。

## Git 中保留什么

- 当前控制器、配置、测试、维护模板与生成器。
- `tools/knowledge_base/`：打包/验证工具及不含对话原文的说明模板。
- 经筛选的实验报告、图表、汇总指标和版本记录。
- 无延迟固件修复在对应 tail-sitl 固件仓库提交，不混入控制仓库。

**不提交**压缩包、对话原文、原始 ULog/CSV、私有参数归档、虚拟环境、缓存和账号信息。
已有报告中 `data/`、原始冻结源码压缩包等链接指向本地完整证据；源码克隆不包含这些私有数据，
不要把链接缺失误认为数据已生成或已上传。详细范围见 `reports/VERSIONED_ARTIFACTS.md`。

## 工具使用

本次构建工具针对已知本地来源组织，不会连接远程、推送 Git 或启动执行器。
默认来源仍包括本机旧 ROS 工作区和任务目录；可通过命令行显式覆盖，
不是对整个账号聊天记录的自动扫描/导出器。

```bash
python3 tools/knowledge_base/package_drcda_knowledge_base.py --help
python3 tools/knowledge_base/package_drcda_knowledge_base.py \
  --output /tmp/hnuter_knowledge_base_new_snapshot \
  --task-workspace /home/hnuter/Documents/Codex/2026-10-11/drcda-gazebo
```

输出目录必须是新目录。验证器读取来源快照和本次测试 XML，重建后需要先完成同名回归并显式指定证据目录。
迁移运行工具在生成包 `08_TOOLS`：完整性校验、检索、生成不覆盖旧文件的路径适配副本、强制只监视启动。
这些工具可以在远程手动运行；打包本身不构成实际控制输出授权。

## 必须保留的交接结论

只用无延迟固件；论文 NDA 是采样/力坐标适配，不是完整功率约束 PDA。
Full 在 7 s 李萨如有局部优势，但 5 s 单步、大姿态无预览存在反超，不能宣称普遍更优。
实机最新策略是独立20 Hz心跳、人工Armed+Offboard、切换实测位姿、0.5 s过渡；
最近估计倾角30.1°触发运行保护，新的事件/ULog尚未回收，不能写成实机稳定验证完成。
