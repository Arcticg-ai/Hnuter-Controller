"""Rebuild reports/figures and fail-closed SITL plan from recorded evidence."""
import hashlib
import json
import os
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'results/mplcache'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager


def main():
    results=ROOT/'results'
    offline=json.loads((results/'offline_experiments.json').read_text())
    replay=json.loads((results/'replay_summary.json').read_text())
    truth=json.loads((ROOT/'generated/truth_for_evaluation_only.json').read_text())
    names=['固定组 A/s0','主动组 C/s0','主动失败 C/s1']
    gates=dict(unit_tests_pass=(results/'unit_tests.log').read_text().rstrip().endswith('OK'),
               physical_models_generated=True,
               identifiable_low_motion_windows=all(x['admissible_sensor_windows']>=17 for x in replay),
               recorded_wrench_model_residual=all(np.linalg.norm(x['evaluation_only']['active_only']['articulated_truth_residual_Nm']['rms_components'])<=.02 for x in replay[:2]),
               independent_torque_bias_bound_verified=False,
               actual_px4_oracle_consumer_verified=False,
               actual_allocator_margin_verified=False,
               revision3_recovery_verified=False)
    plan=dict(status='BLOCKED_BEFORE_LAUNCH',gates=gates,new_sitl_launches=0,max_initial_launches=8,
              evidence_boundary='Offline synthetic and replay only; no revision3 flight evidence.',
              schedule=[dict(case=case,group=group,status='NOT_ATTEMPTED_MODEL_GATE')
                        for case in ('offset_y','symmetric_inertia') for group in ('A','D','B','C')],
              proposed_evaluation=dict(duration_s=24,horizontal_amplitude_m=.04,attitude_amplitude_deg=1.,
                                       axes=['roll','pitch','yaw'],same_for_all_groups=True,implemented_in_flight=False),
              baseline_controller_gains_unchanged=True)
    (results/'sitl_gate.json').write_text(json.dumps(plan,indent=2)+'\n')
    font=next((f.fname for f in font_manager.fontManager.ttflist if 'Noto Sans CJK' in f.name),None)
    if font:
        prop=font_manager.FontProperties(fname=font);plt.rcParams['font.family']=prop.get_name()
    plt.rcParams['axes.unicode_minus']=False
    fig,axs=plt.subplots(2,2,figsize=(14,10),layout='constrained')
    scenarios=offline['estimation'];ax=axs[0,0]
    labels=['干净已知','小噪声','大噪声','未声明延迟','遗漏力矩偏置','仅滚转']
    colors=['#0284c7' if x['accepted'] else '#dc6262' for x in scenarios];colors[4]='#d97706'
    ax.bar(range(6),[x['rank'] for x in scenarios],color=colors)
    for i,x in enumerate(scenarios):ax.text(i,x['rank']+.1,'验收*' if i==4 else ('验收' if x['accepted'] else '拒绝'),ha='center',fontsize=10)
    ax.set(xticks=range(6),xticklabels=labels,ylim=(0,6.1),ylabel='数据支持的参数方向数',title='合成试验：秩不等于模型可信')
    ax.tick_params(axis='x',labelsize=9)
    ax.text(.02,.08,'* 0.1 N·m 漏项造成 CoM 偏差约 1.89 mm\n残差验收无法识别被破坏的零偏置假设',transform=ax.transAxes,fontsize=10,
            bbox=dict(facecolor='white',edgecolor='none',alpha=.92,pad=4))
    ax=axs[0,1];x=np.arange(3);width=.35
    fixed=[offline['oracle'][c]['fixed']['attitude_rmse_deg'] for c in ('nominal','offset_y','symmetric_inertia')]
    ideal=[offline['oracle'][c]['oracle']['attitude_rmse_deg'] for c in ('nominal','offset_y','symmetric_inertia')]
    ax.bar(x-width/2,fixed,width,label='固定模型',color='#94a3b8');ax.bar(x+width/2,ideal,width,label='已知参数 oracle',color='#0891b2')
    for i in range(3):
        ax.text(i-width/2,fixed[i]+.04,f'{fixed[i]:.3f}',ha='center',fontsize=10);ax.text(i+width/2,ideal[i]+.04,f'{ideal[i]:.3f}',ha='center',fontsize=10)
    ax.set(xticks=x,xticklabels=['标称','水平偏置','对称惯量变化'],ylabel='姿态误差 RMSE (°)',ylim=(0,3.5),title='独立刚体 oracle：不是 PX4/SITL 消费验证');ax.legend()
    ax=axs[1,0]
    for i,(field,label,col) in enumerate([('rotor_spin_equivalent_Nm','旋翼内部动量变化','#94a3b8'),('tilt_internal_equivalent_Nm','倾转内部动量变化','#d97706'),('external_center_torque_rms_Nm','重建质心外力矩','#0891b2')]):
        val=[q['evaluation_only']['active_only'][field]['rms_components'][1] for q in replay]
        ax.bar(x+(i-1)*.25,val,.25,label=label,color=col)
    ax.axhline(.01,ls='--',color='#dc2626',label='忽略项预算 0.01 N·m')
    ax.set(xticks=x,xticklabels=names,ylabel='俯仰 RMS (N·m)',title='旧 SITL 回放：仅活动阶段，排除恢复发散');ax.legend(fontsize=9)
    ax=axs[1,1]
    counts=[q['evaluation_only']['active_only']['count'] for q in replay]
    ax.bar(x,counts,color='#cbd5e1',label='活动阶段窗口')
    for i,c in enumerate(counts):ax.text(i,c+2,f'0 / {c} 可接受',ha='center')
    ax.set(xticks=x,xticklabels=names,ylim=(0,115),ylabel='约 0.4 s 窗口数',title='低运动近似门槛：全部拒绝，参数保持')
    fig.suptitle('Revision3 离线门槛验证：未启动新 SITL，未将估计值入环',fontsize=19)
    for ext in ('png','pdf','svg'):fig.savefig(results/f'revision3_summary.{ext}',dpi=180)
    plt.close(fig)
    body=['# Revision3 有界验证报告','','结论：完成模型、五参数 shadow 估计器、物理/可观测性测试、两个载荷模型、主动动作接口与离线控制消费契约。**真实日志回放未通过模型门槛；新增 SITL 0 次，固件未修改，估计值未入环。**','','## 已运行的证据','','- 单元测试日志：`unit_tests.log`；合成实验：`offline_experiments.json`；旧日志回放：`replay_summary.json`。','- 两仓库分支/HEAD 与既有修改保存于 `../initial_state.json`；没有覆盖 revision2 代码或失败日志。','- 旧日志来源固定为 legacy A/s0、成功 C/s0、失败 C/s1，各 ULog 路径与 SHA-256 已记录。回放不是新飞行。','','## 合成估计结果','','| 场景 | 数据秩 | 候选结果 | 原因 |','|---|---:|---|---|']
    for name,p in zip(labels,scenarios):body.append(f"| {name} | {p['rank']} | {'接受' if p['accepted'] else '保持先验'} | {p['reason']} |")
    clean=scenarios[0];noisy=scenarios[1]
    body+=['',f"干净数据最大惯量绝对误差 {max(abs(v) for v in clean['error'][2:]):.3g} kg·m²；小噪声最大惯量绝对误差 {max(abs(v) for v in noisy['error'][2:]):.3g} kg·m²。这是已知模型的合成回收测试。单轴只更新 rx/ry 与对应惯量，其他惯量保持先验。",'','遗漏 0.1 N·m 常值力矩使 ry 偏约 1.89 mm，仍通过残差门控。此测试故意违反声明的零偏置假设，展示结构性混淆；在线接口默认没有独立偏置界就拒绝，不把稳定残差当作该假设的证据。','','## 真实回放与模型适用性','','| 旧试验 | 活动窗 | 低运动可接受 | 倾转俯仰等效 RMS | 质心外力矩俯仰 RMS | 比例 | 刚体残差俯仰 RMS | 完整修正后 |','|---|---:|---:|---:|---:|---:|---:|---:|']
    for name,q in zip(names,replay):
        a=q['evaluation_only']['active_only'];internal=a['tilt_internal_equivalent_Nm']['rms_components'][1];external=a['external_center_torque_rms_Nm']['rms_components'][1]
        body.append(f"| {name} | {a['count']} | {q['admissible_sensor_windows']} | {internal:.5f} | {external:.5f} | {internal/external*100:.1f}% | {a['rigid_truth_residual_Nm']['rms_components'][1]:.5f} | {a['articulated_truth_residual_Nm']['rms_components'][1]:.5f} |")
    body+=['','表内力矩单位均为 N·m。内部动量端点差除以窗长，与同窗重建外力矩 RMS 比较；两者不是完全相同滤波带宽，但足以说明其相对误差预算不能被忽略。初次回放总257窗包含14个恢复/边界窗；正式活动统计为243窗，全部不满足低运动近似，未据此调松门槛。原初汇总保留在 `replay_summary_initial.json`。',
           '', '转子气动力使用慢速关节速度×10，角动量使用慢速关节速度与 SDF 惯量。正常两例旋翼变化项俯仰 RMS 约0.00028/0.00043 N·m，主要忽略项来自倾转。完整 q、q̇、r(q)、J(q)、hrel 修正仅离线使用真值评价，没有送给估计器；修正后残差反而增大，说明当前测量时序/滤波、构型或力矩模型仍需闭合验证，不能直接采用该修正入环。没有把问题单独归因于某一个未经证实的环节。',
           '', '## Oracle 与真实消费者边界','','| 合成刚体案例 | 固定模型姿态 RMSE (°) | 已知参数 RMSE (°) |','|---|---:|---:|']
    for c in ('nominal','offset_y','symmetric_inertia'):body.append(f"| {c} | {offline['oracle'][c]['fixed']['attitude_rmse_deg']:.5f} | {offline['oracle'][c]['oracle']['attitude_rmse_deg']:.5f} |")
    body+=['','该独立刚体仿真采用日志中的 KR/D 与零姿态积分增益、原力矩限幅；标称行为一致。偏置载荷在理想固定原点力矩接口下可改善，对称惯量载荷改善很小。它不含现有 PX4 分配器、实际舵机动态和位置闭环，**不是实际控制消费者已经验证**。',
           '', '源码审计定位了 Hnuter 姿态律、CG trim 和 allocator 的多个消费点：现有 CG 参数约定不同于待估 O→C，不能直接写参数或重复补偿。实际姿态 I=0，已实现有界独立 tracking bias 的离线连续过渡并测试，没有启用积分器。详见 `../MODEL_AND_CONTROL.md`。',
           '', '## 未验证与下一步决策','','8例串行计划已写入 `sitl_gate.json`，全部 `NOT_ATTEMPTED_MODEL_GATE`，不计为飞行失败或通过。未验证：新载荷起飞、实际 CoM/J 消费者、真实 oracle、shadow 后入环、主动分轴动作、公共姿态+位置评价及恢复。',
           '', '下一步需要选择模型路线：优先在现有机型上闭合包含倾转内部动量的模型与传感时序；或新建明确受约束的固定构型研究模型，并重新证明该构型仍支持所需姿态/位置控制。当前“低运动五参数刚体近似直接上现有机型”的门槛不成立。两条路线都不能通过放宽窗口或飞行保护门槛替代验证。',
           '', '## 文件与复现','','见 `../README.md`、`../CHANGES.md`。所有结果为本次离线运行生成；原始 ULog 保持在 revision2，SHA-256 索引可追溯。']
    (results/'REPORT.md').write_text('\n'.join(body)+'\n')
    print(json.dumps(plan,indent=2))


if __name__=='__main__':main()
