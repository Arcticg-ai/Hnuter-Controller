"""All attempts, separate gates, figures, and raw-evidence hashes for this stage."""
import hashlib
import json
import os
from pathlib import Path
import numpy as np
import pandas as pd
from pyulog import ULog
ROOT=Path(__file__).resolve().parent
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'runs/mpl_cache'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False,'font.size':10})


def main():
    out=ROOT/'results/closure';out.mkdir(exist_ok=True)
    records=[];hashes={}
    for run in sorted((ROOT/'runs').glob('sitl_*')):
        row=dict(run=run.name)
        for name in ('launch','flight_result','process_exit','consumer_analysis','sensor_balance_discrete','online_shadow','envelope_diagnostic'):
            p=run/(name+'.json')
            if p.exists():
                d=json.loads(p.read_text());row[name]={k:v for k,v in d.items() if k not in ('windows','checks')}
        records.append(row)
        for p in [*run.glob('*.csv'),*run.glob('*.json'),*run.glob('*.log'),*run.glob('*.npz'),*(run/'rootfs/log').rglob('*.ulg')]:
            h=hashlib.sha256()
            with p.open('rb') as f:
                for chunk in iter(lambda:f.read(8*1024*1024),b''):h.update(chunk)
            hashes[str(p.relative_to(ROOT))]=dict(sha256=h.hexdigest(),size_bytes=p.stat().st_size)
    (out/'all_attempts.json').write_text(json.dumps(records,indent=2)+'\n')
    (out/'raw_evidence_manifest.json').write_text(json.dumps(hashes,indent=2)+'\n')
    source=ROOT/'runs/sitl_native_stamp_4ms_20261010_211352'
    initial=json.loads((source/'sensor_balance.json').read_text())
    fixed=json.loads((source/'sensor_balance_discrete.json').read_text())
    bias=json.loads((ROOT/'results/bias_identifiability.json').read_text())
    consumers=[x for x in records if 'consumer_analysis' in x]
    labels=[];metrics=[]
    for row in consumers:
        name=row['run'];label='A 固定'
        if 'D_transition' in name:label='D 过渡复测'
        elif '_D_' in name:label='D 首次（未完成过渡）'
        elif '_B_' in name:label='B 被动 shadow'
        elif '_C_' in name:label='C 主动 shadow'
        labels.append(label);metrics.append(row['consumer_analysis'])
    fig,ax=plt.subplots(2,2,figsize=(13,8.2),layout='constrained')
    x=np.arange(3);width=.25
    for i,(key,title) in enumerate([('rigid_Nm','忽略运动部件'),('corrected_Nm','完整修正 + 梯形积分')]):
        ax[0,0].bar(x+(i-1)*width,initial['holdout'][key],width,label=title)
    ax[0,0].bar(x+width,fixed['holdout']['corrected_Nm'],width,label='完整修正 + 匹配物理步积分')
    ax[0,0].axhline(.02,color='#be123c',ls='--',lw=1,label='固定门槛 0.02 N·m')
    ax[0,0].set(xticks=x,xticklabels=['滚转','俯仰','偏航'],yscale='log',ylabel='后续窗口残差 RMS / N·m',title='动力学链路：同一未知中心负载，48 个后续窗口')
    ax[0,0].legend(fontsize=8)
    y=np.arange(len(labels));ax[0,1].barh(y-.18,[m['evaluation_attitude_rmse_deg'] for m in metrics],.35,color='#2563eb',label='姿态 RMSE / °')
    axb=ax[0,1].twiny();axb.barh(y+.18,[100*m['position_rmse_m'] for m in metrics],.35,color='#f97316',label='位置 RMSE / cm')
    ax[0,1].set(yticks=y,yticklabels=labels,xlabel='姿态 RMSE / °',title='同一偏置负载：姿态改善伴随位置代价');axb.set_xlabel('位置 RMSE / cm',color='#c2410c');ax[0,1].invert_yaxis()
    for label,runname in [('首次 D','sitl_offset_D_20261010_213142'),('过渡复测 D','sitl_offset_D_transition_recheck_20261010_213745')]:
        run=ROOT/'runs'/runname;u=ULog(str(next((run/'rootfs/log').rglob('*.ulg'))));d=u.get_dataset('hnuter_control_status').data;tr=pd.read_csv(run/'trajectory.csv');zero=tr.px4_time_s.iloc[0]-tr.t.iloc[0]
        tt=d['timestamp']*1e-6-zero;mask=(tt>30)&(tt<81)
        ax[1,0].plot(tt[mask],d['spatial_tracking_bias[0]'][mask],label=label,lw=1)
    ax[1,0].axvspan(60,76,color='#dbeafe',alpha=.7,label='共享评价阶段')
    ax[1,0].set(xlabel='起飞后时间 / s',ylabel='滚转跟踪偏置 / N·m',title='首次过渡失败保留；复测在评价前归零');ax[1,0].legend(fontsize=8)
    shadow=[m for m in metrics if 'online_shadow' in m]
    if shadow:
        for m in shadow:
            s=m['online_shadow'];ax[1,1].plot(np.arange(1,len(s.get('singular_values',[]))+1),s.get('singular_values',[]),'o-',label=m['group']+' shadow')
        ax[1,1].axhline(5,color='#be123c',ls='--',label='固定信息门槛 5')
    ax[1,1].set(xlabel='奇异值序号',ylabel='归一化奇异值',yscale='log',title='在线训练信息：不以先验补足缺失方向');ax[1,1].legend(fontsize=8)
    fig.suptitle('Revision3 有界验证：模型闭合通过，整体控制收益尚未通过',fontsize=16)
    for ext in ('png','pdf','svg'):fig.savefig(out/('closure_overview.'+ext),dpi=180)
    plt.close(fig)
    # Machine-readable decisions deliberately distinguish flight success from acceptance.
    summary=dict(new_sitl_attempts=len(records),completed_flights=sum(x.get('flight_result',{}).get('status')=='PASS' for x in records),
        dynamics_diagnostics=3,consumer_and_shadow_attempts=len(consumers),
        original_comparison_matrix_completed=False,reason_matrix_not_expanded='Oracle overall position-tracking benefit not demonstrated; B/C are shadow only and the second payload matrix is gated.',
        model_closure_pass_4ms=True,known_parameter_consumer_exercised=True,
        consumer_gate=json.loads((ROOT/'results/consumer_gate.json').read_text()),
        shadow={m['group']:m.get('online_shadow') for m in shadow},
        unknown_torque_bias_identifiable=False,augmented_information_rank=bias['retained_rank'],augmented_dimension=11,
        no_hardware=True,no_neural_training=True)
    (out/'SUMMARY.json').write_text(json.dumps(summary,indent=2)+'\n')
    table='\n'.join(f"| {l} | {m['evaluation_attitude_rmse_deg']:.3f} | {m['position_rmse_m']:.5f} | {m['max_position_error_m']:.5f} | {max(m['tracking_bias_max_evaluation_Nm']):.6f} |" for l,m in zip(labels,metrics))
    shadows='\n'.join(f"- {m['group']} shadow：接受={m['online_shadow']['accepted']}，原因={m['online_shadow']['reason']}，秩={m['online_shadow'].get('rank','N/A')}/8，训练/验证={m['online_shadow'].get('train_windows')}/{m['online_shadow'].get('holdout_windows')}，决策时刻={m['online_shadow'].get('decision_phase_elapsed_s','N/A')} s。" for m in shadow)
    calibration_table='\n'.join(f"| {m['group']} | {m['calibration']['duration_s']:.3f} | {m['calibration']['position_rmse_m']:.5f} | {m['calibration']['attitude_rmse_deg']:.3f} | {m['calibration']['vertical_reference_velocity_cost_m2_s']:.6f} | {m['calibration']['vertical_reference_acceleration_cost_m2_s3']:.6f} |" for m in shadow)
    report=f'''# Revision3 完整倾转模型闭合与消费者有界验证

本轮完成 **{len(records)} 次新增独立 SITL**：3次模型/时序诊断，A固定基线、首次D、1次D必要定点复测、B/C在线shadow各1次。所有结果保留；原两载荷×四组闭环矩阵**未完成且未放行**。这里的飞行PASS仅表示完成公共轨迹并正常降落，不代表辨识接受、总体性能改善或硬件安全。

![总览](closure_overview.png)

## 已验证的模型链路

采用真实现有机型、5个旋翼及所有运动刚体。独立ECM记录器逐4ms物理步记录各link世界姿态/速度和施加外力矩；与SDF kinematics、全惯量和内部动量模型相比，坐标、关节轴、角动量和重建力矩一致。94/95个完整窗口的角动量平衡残差约1e-4 N·m。气动力使用实际慢速关节速度×slowdown，机械动量只用慢速物理转速；转子反扭矩没有重复计入内部动量。

原GZBridge的IMU时间为回调时间，且控制角速度经过滤波。新增传感记录器仅订阅IMU和关节原生时间戳消息。完整倾转修正与匹配物理步积分后，独立48窗残差RMS为 **[0.000230,0.000155,0.000435] N·m**，固定门槛仍为0.02。原先仅用梯形积分虽过残差门槛，俯仰惯量仍偏约0.030 kg·m²；时序推导后降到约0.000488 kg·m²。

精度数字对应**未知0.9kg中心刚性负载，整机5.4kg**，不是标称4.5kg自验；先验仍使用标称CoM/J，质量在此CoM/J阶段为各组共有的已知固定量5.4kg。真实CoM和完整惯量尺度见 `../unknown_payload_scales.json`。前47窗训练，后48窗未见验证；shadow最大CoM误差约0.2微米、惯量张量Frobenius误差0.000532 kg·m²，只适用于已测低噪声SITL。偏置负载真实横向CoM相对标称移动−6.667mm，也已独立验证其完整模型残差，不能把中心负载结果替代偏置辨识结果。

2ms物理步/250Hz IMU的对照失败：交替物理步漏采，插值不能恢复高频响应，残差[0.0404,0.0200,0.0975] N·m。该失败未删除。后续消费者/在线估计启动前强制4ms物理步与250Hz IMU一致；关节逐步发布，在线缺任一步即拒绝。未宣称跨频率泛化。

## 实际消费者与性能代价

额外力矩在HnuterControl最终限幅之前进入。标称CG trim、allocator CG和PID增益保持原值；补偿仅使用相对标称的CoM/J增量，避免重复质心补偿。无持久参数写入。独立tracking bias处理实际Ki=0的无扰更新；物理范围、变化率、时间戳/heartbeat、力矩余量和显式冻结均有效。

真实比例差来自SDF前桨横向力臂约0.410m，而HNTR_L1=0.330m；不是通过飞行拟合一个经验系数。`allocation_contract.py`复现现有allocator源码，再用独立标称几何前向wrench求局部Jacobian，并转换到标称CoM参考点。初始2N横向力校准域未覆盖正常飞行，导致跟踪偏置无法消退；首次结果保留。对实际需求覆盖的729点几何网格重新核验，最大逆映射算子误差0.595%，保持同一Jacobian/增益；控制接口校准域扩至横向6N，保留原0.02 N·m残差门槛和硬力矩/参数速率限制。该有限网格是工程验证，不是全局可实现性证明。

各组固定相同5.4kg质量、相同控制增益、相同评价轨迹。评价阶段为起飞后60–76s，含竖直、6cm水平及分轴2°平滑姿态参考。roll/pitch导数来自同一解析参考但现有入口不消费其角速度前馈；未更换直接执行器入口。

| 组别 | 姿态RMSE ° | 位置RMSE m | 最大位置误差 m | 评价期跟踪偏置峰值 N·m |
|---|---:|---:|---:|---:|
{table}

首次D：参数到达，但评价期tracking bias仍为0.16164 N·m；全记录更新拒绝比例45.51%，主要由CHECK期横向力越过初始2N校准域引起，评价含未完成过渡。定点复测：评价前bias归零，无扰变换误差≤1.12e-8 N·m，参数最大变化率约0.00200002m/s和0.01000001kg·m²/s（浮点误差级），进入降落后参数跨度0。更新拒绝比例降至4.39%，并非零拒绝。总控制力矩仍会随正常反馈变化；记录的第一更新邻近完整力矩差约0.00459 N·m，不把无扰变换误差冒充总力矩完全恒定。

**姿态改善伴随位置代价：** A→复测D姿态RMSE约6.598→4.058°，位置RMSE约0.03645→0.05459m。单次对照不能给出统计因果结论，但不足以证明整体轨迹跟踪恢复，因此整体收益门槛保持关闭，估计补偿未放行。没有调弱基线、改增益或筛掉坏片段。

## 在线shadow与主动运动

偏置负载未知CoM/J，B被动悬停、C使用三轴各0.75°、0.4/0.5/0.6Hz的平滑多正弦，并包含原竖直校准参考；所有轴出现在训练与验证两半。解析位置/速度/加速度一致，动作导数与理论力矩界已检查。B/C估计进程仅收到IMU、关节、ROS姿态/速度和独立标称运动部件校准；不读取真实载荷SDF、真值文件或oracle请求。calibration为16s，另计异步计算时间，45s前未完成则拒绝。

{shadows}

B/C均禁用补偿，ULog已核实有效CoM/J增量始终为零。它们是在线shadow对照，不能标为已完成B/C闭环补偿。主动动作本轮是否值得，只能依据其信息、拒绝原因和代价评估，不能预先宣称必须运动才可辨识。

| 组别 | 校准记录时长 s | 校准位置RMSE m | 校准姿态RMSE ° | ∫参考竖直速度²dt m²/s | ∫参考竖直加速度²dt m²/s³ |
|---|---:|---:|---:|---:|---:|
{calibration_table}

上述动作代价是竖直参考的运动指标，不是电能消耗，也未包含姿态动作成本。B/C完整八参数接受失败为2/2；8次飞行均正常降落并上锁，安全退出0次，5次消费者评价的力矩饱和比例均为0。飞行完成不能替代辨识接受。

恒定未知力矩偏置仍有混淆：加入三轴常值偏置后11参数模型只保留9个信息方向；静态悬停的水平一阶矩+三轴偏置5参数仅秩3。无独立偏置界的负对照明确拒绝。本轮零未知外扰假设来自受控SITL配置，不推广到硬件。

## 已做、未做、失败

- 已做：完整多刚体闭合、转子/关节轴及力矩重建、原始时间戳链路、离散积分修正、未知载荷离线shadow、固件实际参数消费及平滑过渡/冻结、在线B/C shadow。
- 失败/限制：2ms欠采样、首次D未完成过渡、oracle位置收益未证明、shadow的信息不足（详见逐组结果）；每次均保留原始日志。
- 未做：估计值闭环补偿、第二种对称大惯量载荷四组实飞、5随机种子统计、真实硬件、JEPA或神经训练。原第一阶段mass结果仍在revision2，本报告不将其历史结果冒充本轮验证。

原始CSV、ULog、私有模型及启动命令在 `../../runs/<run>/`；`all_attempts.json`记录所有尝试，`raw_evidence_manifest.json`绑定原始文件哈希。源码/二进制身份以每次launch.json为准：最早A/D消费者脚本未完整快照，已标注这一复现限制；之后运行保存source_snapshot。固件初次b233b951与随后校准域修订分别保留Git历史，二进制哈希单独记录，不将未提交构建假称为干净commit产物。

复现入口见 `../../SYNCHRONOUS_DIAGNOSTIC.md` 和 `../../run_diagnostic.py --help`。估计补偿入口受 `../consumer_gate.json` 阻止；B/C仅可显式 `--shadow-only`。所有运行关闭MAVLink、DDS仅回环、独立namespace/Gazebo分区，结束只回收本次进程。
'''
    (out/'REPORT.md').write_text(report)
    info=json.loads((ROOT/'results/shadow_information_diagnostic.json').read_text())
    add='''
## 参数维数、信息子空间与动作边界

最初五参数为质心处rx、ry、Jcxx、Jcyy、Jczz，假定固定构型与已知积惯量。完整运动模型以固定base原点O积分；未知CoM经过平行轴变换会改变O处积惯量，不能继续将它们固定。线性未知量因此改为 **hx、hy、JOxx、JOyy、JOzz、JOxy、JOxz、JOyz** 八个；h=m·r，质量与参考hz固定，标称运动部件的Δh(q)、ΔJO(q)、hrel,O为已知校准修正。估计后通过平行轴定理恢复完整Jc并检查物理性。

每个不重叠窗口约0.4s。数据矩阵采用列尺度S=diag(5.4×0.002,5.4×0.002,0.05,0.05,0.05,0.05,0.05,0.05)，角冲量噪声尺度σ=0.004 N·m·s，遗忘系数0.995；对加权AS/σ做SVD，保留奇异值≥5且≥最大奇异值/100的方向。数据秩不计先验；先验只作0.01弱正则。独立后半窗预测残差、分半信息/稳定性及物理检查仍须通过。

**当前spatial_estimator要求八方向全部满足才提出更新；尚未实现部分方向更新。** B秩5、C秩2并不等于全部CoM/J不可辨识。数据支持投影对角线中，B的hx/hy约0.99999，JOxx约0.99985，JOxy/JOxz约0.986/0.990，而JOyy、JOzz、JOyz弱；C的hx/hy约0.99996，惯量列均接近0。它们是归一化参数空间投影，不是参数置信概率；在质量/外扰假设成立时，水平一阶矩仍有强信息。完整数值见../shadow_information_diagnostic.json。

实际C是**预先固定**的三轴各0.75°、sin⁴包络、0.4/0.5/0.6Hz多正弦，未读取缺失方向在线选轴。旧actions.py的离线候选/门控接口不等于已完成信息增益在线动作优化。本轮没有该能力声明。

B/C都完成校准且逐窗拒绝数为0；B有40窗，C有39窗并留下0.396s不完整尾窗。C整体拒绝不是因为大量窗口被筛除。惯量信息取决于窗口首末R·Eᵢ·ω变化：B的JOxx/JOxy/JOxz归一化列范数约5.86/5.97/6.31，C约2.23/2.66/2.84。训练端点世界角速度变化RMS的三轴向量范数B约0.1167rad/s、C约0.0603rad/s，反映本次C实际响应给出的积分惯量信息更弱。没有证据将原因单独归结为整周期抵消；0.4s窗口也不是这些参考频率的完整周期。

校准参考通道确实不同：B保留普通位置入口的legacy姿态来源并悬停；C增加显式roll/pitch jerk扩展、yaw参考和竖直动作。这种响应差异与主动动作设计本身不可分，本轮没有隔离哪一机制导致秩下降，不能据此断言主动学习普遍无效。所有组的公共评价仍采用同一解析轨迹和60–76s窗口；调度/初始估计噪声不同，各组仅一次，D另有一次必要定点复测，未作统计显著性声明。

## 最小下一步（本轮不执行）

先在固定增益和相同公共轨迹下，定位oracle增加位置误差的平移/姿态耦合及实际力矩映射剩余误差；整体位置收益未证明前不放行估计补偿。辨识方面可另行实现经过独立验证的部分子空间更新，先利用已观测水平一阶矩；随后设计同一参考语义下的信息动作，验证弱惯量方向、动作代价及拒绝行为。第二负载和随机种子矩阵仍待前置门槛。
'''
    with (out/'REPORT.md').open('a') as f:f.write(add)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
