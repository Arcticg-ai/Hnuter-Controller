"""Write evidence-bound final report after both complete matrices."""
import csv,json,hashlib
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent;O=R/'results'
rows=list(csv.DictReader((O/'all_attempts.csv').open()));formal=[r for r in rows if r['batch'].startswith('batch_matrix_')]
s=json.loads((O/'summary.json').read_text());audit=json.loads((O/'release_audit.json').read_text())
G=['A','B','C','D','C_shadow']
floats=lambda rr,k:[float(x[k]) for x in rr if x[k]]
fmt=lambda v,scale=1:f'{np.mean(v)*scale:.3f} ± {np.std(v,ddof=1)*scale:.3f}' if len(v)>1 else (f'{v[0]*scale:.3f}' if v else '无完整数据')
lines=['# 第二轮修复与实际 SITL 回归报告','','本轮实际修改代码、重新构建 PX4 SITL 并运行实验。旧数据保留；未连接真实硬件，未更改增益或放宽保护阈值。**质量独立辨识成立、有条件的无扰更新得到支持；总体收益与可靠性必须结合全部失败判断，不能仅用准确估计或成功子集作结论。**','','## 实际修改与证据','',
'- 解锁前门禁：完整实际关节速度/四个倾转角、有效位置速度和姿态对齐；连续3秒就绪才允许解锁。45秒不足则失败且保持未解锁。记录全部就绪判断、订阅结果、首末时间、缺失字段、时间倒序和发现列表。',
'- 解锁后额外检查命令与实际转速响应；命令持续非零、实际转速低于门限超过0.5秒则退出。正式矩阵实际捕获过一次此类失效，已恢复落地。该检查不是传感器或电机故障诊断器。',
'- 两种姿态参考分别统计：legacy保留原普通位置入口的NaN jerk；level使用固件已有jerk[0:2]横滚/俯仰扩展指定水平。未修改原入口。旧入口三次消融也通过，不能将原失稳宣称为已确定性复现或完全根治。',
'- 有效状态下退出继续维持普通位置下降参考与Offboard心跳，收到landed和disarmed才结束；35秒超时仍记失败。状态失效时的NAV_LAND回退尚未获得充分故障注入验证。',
'- 仅SITL固件增加显式冻结请求：command31000、param2=54123、param3=1撤销质量目标，保持当前m_eff和积分器。修复前停止心跳后仍更新0.468秒；修复后日志无后续更新采样，剩余质量变化约0.0012kg。两次最终注入恢复约5.47/6.17秒确认落地上锁。',
'- 差异：`changes_from_v1.patch`、`freeze_firmware.patch`；定向回归较早版本：`stage1_targeted_changes.patch`。固件/在线文件身份：`frozen_release.json`。本轮只再修改PX4的HnuterControl.cpp；第一轮的hpp/msg补丁保留。',
'', '## 总分母与试验分类','',f"本轮共 **{audit['total_launches']}** 份独立启动清单。每次实际命令、参数、模型、原始ULog/CSV、退出记录保存在 `runs/sitl_*/`。原矩阵未覆盖，没有偷偷重跑替换失败。",'',
'- 定向复测原问题种子：7/7完整飞行、落地上锁；包含原遥测缺失A/s3、B/s1、C/s3、D/s2和原退出B/s4、C/s4、C_shadow/s3。',
'- 旧入口消融：3/3完整飞行，说明单次同种子并不足以确认偶发根因。',
'- 启动试验：1次门禁调试加12次并发启动均READY_UNARMED；故意移除关节发布器的1次负向试验按预期超时且从未解锁。负向试验仍保留FAIL记录。',
'- 恢复注入：修复前2次、显式冻结后2次，四次均成功落地上锁；飞行状态保留FAIL/injected_recovery_test，不能算完整任务成功。',
'- 正式比较：水平参考25次、原入口25次，分别报告。前者出现原入口不一定出现的性能退化，故主要性能判断使用保留原入口的配置，不能挑选弱基线制造补偿优势。',
'','## 五组五种子结果','', '质量误差为已验收子集的绝对百分比均值；未验收与飞行退出另计。位置RMSE只统计完整16秒公共窗口，n单列；中途退出的实际片段误差保存在all_attempts.csv的partial_or_full_rmse中，不混入完整窗口均值。']
for ref in ['legacy','level']:
 rr=[r for r in formal if r['attitude_reference']==ref];assert len(rr)==25,(ref,len(rr))
 lines+=['',f'### {ref}：'+('原普通位置入口，主要对照' if ref=='legacy' else '显式水平参考，独立对照'),'', '| 组 | 完整飞行 | 质量验收 | 质量MAE %（验收n） | 估计位置RMSE cm（n） | 真值RMSE cm | 校准成本 m²/s |','|---|---:|---:|---|---|---|---:|']
 for g in G:
  a=[r for r in rr if r['group']==g];accepted=[r for r in a if r['accepted']=='True'];mv=[abs(x) for x in floats(accepted,'mass_error_pct')];v=floats(a,'rmse');gt=floats(a,'truth_rmse');cost=floats(a,'calibration_cost')
  mass_text=f'{np.mean(mv):.4f} ({len(mv)})' if mv else ('不适用' if g in ['A','D'] else '无验收')
  accepted_text='不适用' if g in ['A','D'] else f'{len(accepted)}/5'
  lines.append(f"| {g} | {sum(x['status']=='PASS' for x in a)}/5 | {accepted_text} | {mass_text} | {fmt(v,100)} ({len(v)}) | {fmt(gt,100)} | {np.mean(cost):.5f} |")
 lines.append(f"\n该配置全部尝试落地上锁：{sum(r['landed']=='True' and r['disarmed']=='True' for r in rr)}/25。")
lines+=['','## 三个研究问题','', '**独立辨识：** 实际Gazebo关节状态、固定k与slowdown、PX4姿态重建NED向上推力；不用期望力、esc_rpm回显或真值质量。0.4秒不重叠窗，20训练窗后冻结候选，再用5个独立后续窗验证。重建/估计源文件未引入质量真值字段；Gazebo真值只在离线脚本中使用。原生传感噪声外加转速2rad/s、角度0.1度、竖直速度0.01m/s；60ms缓冲是已知时间对齐处理延迟，不代表任意网络延迟鲁棒性。', '']
primary=[r for r in formal if r['attitude_reference']=='legacy'];acc=[r for r in primary if r['accepted']=='True'];errs=floats(acc,'mass_error_pct');times=floats(acc,'acceptance_s');res=floats(acc,'holdout')
lines += [f"原入口验收子集的质量绝对误差平均 {np.mean(np.abs(errs)):.4f}%（n={len(errs)}），验收时间 {fmt(times)} s，独立验证窗相对RMS {fmt(res,100)}%。必须同时查看各组验收率和失败率。",'', '**补偿：** ULog的effective_mass确认内存有效质量变化，未高频写HNTR_MASS。积分器为加速度量纲，转换在本周期积分前执行，检查积分/加速度限幅与力连续性，质量斜率0.3kg/s；范围3.5–7kg。限幅不允许连续时拒绝更新。SITL分配器使用固定推力标定，硬件分支的质量悬停锚点不在本次路径中。']
force=floats(primary,'force_algebraic');steps=floats(primary,'force_step');trans=floats(primary,'transient_error')
if force:lines.append(f"本配置已更新试验代数合力误差最大 {max(force):.3g} N；含正常反馈的相邻期望力变化最大 {max(steps):.3f} N，更新前后位置峰值范围 {min(trans)*100:.2f}–{max(trans)*100:.2f} cm。这三者不是同一个指标。实测关节重建力在共同更新窗的波动见transition_audit.json。")
lines+=['','**主动相比被动：** 依据相同配置的配对完整窗口，不能预先宣称运动是可辨识的必要条件。原入口配置的差值（正值有利于C）：','']
for c in s['paired']:
 if c['comparison'] in ['B minus C','C_shadow minus C'] and c['metric']=='rmse' and c['mean'] is not None:
  lo,hi=c['bootstrap95'];lines.append(f"- {c['comparison']}：{100*c['mean']:+.3f} cm，bootstrap 95%区间 [{100*lo:.3f}, {100*hi:.3f}] cm，配对n={len(c['seeds'])}。")
lines+=['','主动校准额外参考速度平方积分约0.090 m²/s；这只是动作强度代理，不是实测能耗。固定最小窗数限制了最快验收时刻，不能将约10秒的同样验收时间解释为算法效率的严格最优结论。', '', '## 失败、局限与未完成的根因定位','',
'- 原C_shadow/s3以及本轮level B/s1均出现非零命令回显、实际转速接近零，支持命令送达/插件接收链路异常。单凭这些记录尚不能区分发现、连接和插件内部状态。共享发现端口保持原配置；隔离端口仅提供候选开关，未冒充已验证修复。',
'- 原B/s4、C/s4的真实倾角超限，目标姿态包含原PX4位置控制器的倾角。显式水平参考可消除目标串入，但其正式矩阵出现两次补偿后的水平漂移；不能把参考变化称为普遍性能修复。两次漂移距最后质量更新约9.4/7.9秒，并非质量切换瞬间，仍计闭环失败。',
'- 门禁证明缺数据时不解锁，不证明通信永不失效；转速响应保护证明能检测一种失效，不证明整个传输故障已根治。',
'- 原入口正式矩阵C/s1在约25.36度触发保护；恢复中Gazebo的DART/ODE碰撞求解出现aabbBound断言并中止，仿真时钟停止，35秒内未确认落地/上锁。该项是实际恢复失败，不能用可控状态下的注入成功替代，也不能因仿真崩溃而剔除。',
'- 恢复注入使用有效状态和可控飞行，不等同于大姿态、定位丢失或持续执行器失效的全面恢复验证。没有降低原倾角25度、位置0.5m等门限，没有强制上锁制造成功。',
'- 同种子不保证Gazebo调度、DDS采样和EKF初始状态逐位相同。五种子与bootstrap只描述此场景；跨配置结果不作严格因果效应估计。',
'- 严格零噪声仍只有第一轮的STALE/未解锁失败证据；本轮噪声回归不能替代零噪声通过。未测试更大负载、未知推力比例、竖直外力、JEPA或真实硬件。',
'', '## 原始证据与复现','',
'- `results/RESULTS.md`、`all_attempts.csv`、`summary.json`：完整分母、同种子前后、指标与配对区间。',
'- `results/new_failures.json`：每个正式失败的最后参考、实际/期望力、命令回显、质量更新间隔与恢复时间。',
'- `results/release_audit.json`：用户修改保留、冻结源码、统一增益/固件、所有启动与失败、时间戳检查。',
'- `results/landing_truth_audit.json`：最终Gazebo真值高度/速度对照；`recovery_freeze_audit.json`：退出冻结边界。',
'- `runs/batch_*/plan.json`与`results.json`、`runs/sitl_*/launch.json`、原始ULog/CSV：可追溯实验目录。',
'- `README.md`有实际可复现命令；`results/comparison.png`与`outcomes.png`为新结果图；`results_level_reference`保留另一配置完整汇总。',
'', '尚不能把本轮称为“负载变化后控制性能已可靠恢复”。下一步应针对通信命令送达与姿态/平移控制耦合进一步诊断；现阶段没有依据宣称主动激励或JEPA具备已证实收益。']
fail_lines=['','## 本轮全部正式失败','','| 配置 | 组/种子 | 原因 | 落地 | 上锁 | 运行目录 |','|---|---|---|---|---|---|']
for r in formal:
 if r['status']!='PASS':fail_lines.append(f"| {r['attitude_reference']} | {r['group']}/{r['seed']} | {r['failure']} | {r['landed']} | {r['disarmed']} | {Path(r['run']).name} |")
lines+=fail_lines
(O/'CONCLUSIONS.md').write_text('\n'.join(lines)+'\n')
print('Wrote final report',O/'CONCLUSIONS.md')
