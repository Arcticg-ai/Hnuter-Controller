# 首条试验后的验收脚本修复

原始冻结提交 7e43ab0；原冻结清单保存在 results/first_run_frozen_sources.json。

1. collection_audit 的布尔值含 numpy.bool_，导致 post_run 无法 JSON 序列化。统一转为 Python bool；没有改变数值计算或阈值。
2. update_only_id_or_trim 原以预测区间起点 elapsed 判定更新窗口。在线代码实际在区间终点标签到达后更新。首条原始日志的 trim 起点 33.9–43.9 s，但实际更新终点 34.004–43.996 s；慢矩阵更新终点 51.1–55.9 s。验收改为按 end_us 连接原始 aligned_states 的真实更新 elapsed，仍严格使用 [34,44) 和 [48,56)，没有放宽窗口。

失败的验收结果和 BATCH_STOPPED 已移至 runtime/results/audit_repairs 保存。没有重飞、换种子、重训、改变模型或在线更新算法；只是修复验收脚本后重新读取同一份原始数据。

首条单次性能尚未通过登记的 10% 改善门槛；后续按原八槽预算继续，不按漂亮分数筛选。所有学习控制仍关闭。

完成八条后的导出复核另发现 CSV 使用 int(float_seconds*1e6) 可截断 1 微秒；仅派生 CSV 标记受影响，在线整数时间戳和所有数值预测不受影响。已直接使用源 t_us 重导出，全部标记恢复到 100000 微秒网格。旧脚本和导出文件哈希在 results/csv_timestamp_repair 保留，未修改飞行原始记录。
