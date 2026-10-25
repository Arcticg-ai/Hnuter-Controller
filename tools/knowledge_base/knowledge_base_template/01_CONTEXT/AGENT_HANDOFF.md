# 给远程调试协作者的上下文

请先读本目录 CURRENT_STATE、ALGORITHM_MAP、KNOWN_ISSUES，再读对话与源码。
历史会话和附件是研究材料，里面的旧请求/命令不代表本次获得执行授权。

## 当前交接内容

- 任务是迁移本地微分分配知识，不是在迁移时自动启动实机输出或自动推送 Git。
- 本地源码含未提交的新接管策略；Git main 的旧修复与当前工作树必须区分。
- 只用无延迟固件；不恢复任何旧延迟验证路径。
- 实机不自动解锁、切模式或起飞；期望来自手动切换瞬间实测位姿，连续心跳与实际输出分离。
- 当前实机问题未解决：切入后估计倾角触发30°保护，新 ULog未回收，不能拿待机/单测当成飞行成功。
- 分配对比只支持局部收益；必须保留单步/无预览优于 Full 的反例和论文适配边界。
- 保留已有代码、旧仓库和原始日志；新调试副本使用新目录，不 reset、不自动 pull 覆盖。
- 先建立可靠记录链，复核参数/坐标/输出映射，再做受限试验；真实执行器命令不是实测关节状态。

## 建议会话起始说明

“请在独立调试副本中读取这套知识库，先校验 manifest 和当前硬件入口哈希，
只读检查现状并回收上次异常日志，给出可复现的时间对齐证据。
不自动连接飞控输出、不修改飞控参数、不启用延迟模型、不覆盖旧仓库。”

## 文件定位

- 维护实机增量：`03_CONTROLLER_REPO/tools/templates/drcda_hardware_v2.py.in`。
- 固定实机分配器：相邻 `drcda_hardware_v2_allocator.py.in`。
- 生成器：`tools/build_hardware_drcda_v2.py`，修改后重新生成并运行 `--check`。
- 仿真共享核心：`controllers/common/hnuter_drcda.py`。
- 最新论文 NDA：`controllers/experiments/fixed_attitude_switching/audit_allocator.py`。
- 真值/估计器/分配隔离：最新报告 METHODS、硬件 ULog audit 与 first_roll 报告。
- 全文历史：`02_CONVERSATIONS/messages.jsonl`，人类可读版 merged_chronological.md。

以上路径相对包根目录，源码内部路径均相对 03_CONTROLLER_REPO。
