"""Evidence-bound aggregate report; never used by the online estimator."""
import csv
import json
import os
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
GROUPS=['A','B','C','D','C_shadow']


def statistics(values):
    values=[float(v) for v in values if v is not None and np.isfinite(v)]
    if not values:return dict(n=0,mean=None,std=None,min=None,max=None)
    return dict(n=len(values),mean=float(np.mean(values)),std=float(np.std(values,ddof=1)) if len(values)>1 else 0.,min=min(values),max=max(values))


def main():
    out=ROOT/'results';out.mkdir(exist_ok=True)
    rows=[];all_runs=[]
    for phase in ['low','noisy']:
        pointer=ROOT/f'latest_matrix_{phase}.txt'
        if not pointer.exists():continue
        matrix=Path(pointer.read_text().strip())
        entries=json.loads((matrix/'results.json').read_text())
        for entry in entries:
            run=Path(entry['run']);metrics=json.loads((run/'metrics.json').read_text()) if (run/'metrics.json').exists() else {}
            flight=entry.get('flight',{})
            evaluation=metrics.get('phases',{}).get('EVALUATION',{})
            cal=metrics.get('phases',{}).get('CALIBRATION',{})
            mass=metrics.get('mass',{});comp=metrics.get('compensation',{})
            estimate=flight.get('mass_estimate_kg')
            mass_error=100*(estimate/metrics.get('truth_mass_kg_evaluation_only',5.39999998)-1) if estimate is not None else None
            eval_complete=evaluation.get('duration_s',0)>15.9
            row=dict(phase=phase,group=entry['group'],seed=entry['seed'],run=str(run),flight_pass=flight.get('status')=='PASS',
                     failure=flight.get('failure'),evaluation_complete=eval_complete,
                     landing_confirmed=flight.get('landed') is True and flight.get('disarmed') is True,
                     sensor_samples=flight.get('sensor_samples'),mass_reason=flight.get('mass_reason'),
                     estimate_kg=estimate,mass_error_percent=mass_error,
                     mass_abs_error_percent=abs(mass_error) if mass_error is not None else None,
                     mass_accepted=flight.get('mass_accepted'),acceptance_time_s=mass.get('acceptance_time_s'),
                     holdout_relative_rms=mass.get('holdout_relative_rms'),
                     position_rmse_m=evaluation.get('position_rmse_m') if eval_complete else None,
                     vertical_rmse_m=evaluation.get('vertical_rmse_m') if eval_complete else None,
                     truth_position_rmse_m=evaluation.get('truth_position_rmse_m') if eval_complete else None,
                     position_max_m=evaluation.get('position_max_m'),calibration_rmse_m=cal.get('position_rmse_m'),
                     calibration_max_m=cal.get('position_max_m'),
                     calibration_cost_m2_s=cal.get('reference_velocity_energy_m2_s'),
                     saturation_fraction=evaluation.get('saturation_fraction'),
                     calibration_saturation_fraction=cal.get('saturation_fraction'),
                     effective_mass_kg=comp.get('final_effective_mass_kg'),
                     update_force_error_N=comp.get('max_algebraic_update_force_error_N'),
                     transition_force_step_N=comp.get('common_transition_max_force_step_N'),
                     transition_position_peak_m=comp.get('transient_position_max_m'),
                     thrust_balance_rms_N=metrics.get('offline_thrust_balance',{}).get('rms_residual_N'))
            rows.append(row);all_runs.append(dict(entry=entry,metrics=metrics))
    if not rows:raise RuntimeError('No completed matrix')
    with (out/'all_trials.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    aggregates=[]
    for phase in ['low','noisy']:
        for group in GROUPS:
            selected=[r for r in rows if r['phase']==phase and r['group']==group]
            if not selected:continue
            agg=dict(phase=phase,group=group,n=len(selected),flight_passes=sum(r['flight_pass'] for r in selected),
                     evaluation_complete=sum(r['evaluation_complete'] for r in selected),
                     calibration_accepts=sum(r['mass_accepted'] is True for r in selected) if group in ['B','C','C_shadow'] else None,
                     landing_unconfirmed=sum(not r['landing_confirmed'] for r in selected),
                     no_actuator_sensor_runs=sum(r['sensor_samples']==0 for r in selected),
                     safety_exits=sum(r['failure'] in ['attitude_or_horizontal_limit','tracking_velocity_or_contact_limit','sustained_saturation','invalid_or_stale_state'] for r in selected))
            agg['calibration_failure_fraction']=1-agg['calibration_accepts']/agg['n'] if agg['calibration_accepts'] is not None else None
            agg['accepted_only_mass_abs_error_percent']=statistics([r['mass_abs_error_percent'] for r in selected if r['mass_accepted'] is True])
            for field in ['position_rmse_m','vertical_rmse_m','truth_position_rmse_m','position_max_m','mass_error_percent','mass_abs_error_percent','acceptance_time_s','holdout_relative_rms','calibration_rmse_m','calibration_max_m','calibration_cost_m2_s','saturation_fraction','calibration_saturation_fraction','update_force_error_N','transition_force_step_N','transition_position_peak_m','thrust_balance_rms_N']:
                agg[field]=statistics([r[field] for r in selected])
            aggregates.append(agg)
    contrasts=[]
    for metric in ['position_rmse_m','vertical_rmse_m','truth_position_rmse_m']:
        for baseline,treatment in [('A','B'),('A','C'),('A','D'),('A','C_shadow'),('B','C'),('C_shadow','C')]:
            deltas=[]
            for seed in range(5):
                a=next((r for r in rows if r['phase']=='noisy' and r['group']==baseline and r['seed']==seed),None)
                b=next((r for r in rows if r['phase']=='noisy' and r['group']==treatment and r['seed']==seed),None)
                if a and b and a[metric] is not None and b[metric] is not None:deltas.append(a[metric]-b[metric])
            if deltas:
                rng=np.random.default_rng(20261010);boot=np.mean(rng.choice(deltas,(10000,len(deltas)),replace=True),axis=1)
                contrasts.append(dict(metric=metric,comparison=baseline+' minus '+treatment,positive_means_improvement=True,
                                      paired_seed_count=len(deltas),mean_difference_m=float(np.mean(deltas)),
                                      bootstrap_95pct_interval_m=np.percentile(boot,[2.5,97.5]).tolist()))
    (out/'summary.json').write_text(json.dumps(dict(aggregates=aggregates,paired_contrasts=contrasts),indent=2)+'\n')
    def fmt(stat,scale=1):
        return 'N/A' if stat['n']==0 else f"{stat['mean']*scale:.3f} ± {stat['std']*scale:.3f}"
    lines=['# SITL 质量自校准实测汇总','',
           '全部正式组使用同一 5.39999998 kg 负载模型、固定增益、1 倍速、平滑起飞、16 秒校准预算和公共评价轨迹。',
           'low 为原 SDF 传感器标准差的 1%；noisy 为原 SDF 噪声，加转速 2 rad/s、关节角 0.1°、竖直速度 0.01 m/s 的独立高斯噪声及 60 ms 对齐缓冲（low 为 30 ms）。严格零噪声因 PX4 恒值传感器检查失败，未伪称通过。','',
           '表内为均值 ± 样本标准差；性能统计只含完成公共评价阶段的样本，完整飞行成功数和校准验收数单独列出。质量误差含未接受时保留的先验，不能只挑成功估计。主位置误差使用 PX4 状态估计；Gazebo 真值以首个解锁样本做固定平移对齐后单独报告。','',
           '| 条件 | 组 | 全程成功 | 校准接受 | 评价 RMSE cm | 竖直 RMSE cm | 真值 RMSE cm | 质量误差 % | 验收时间 s |','|---|---|---|---|---|---|---|---|---|']
    for a in aggregates:
        accept='—' if a['calibration_accepts'] is None else f"{a['calibration_accepts']}/{a['n']}"
        lines.append(f"| {a['phase']} | {a['group']} | {a['flight_passes']}/{a['n']} | {accept} | {fmt(a['position_rmse_m'],100)} | {fmt(a['vertical_rmse_m'],100)} | {fmt(a['truth_position_rmse_m'],100)} | {fmt(a['mass_error_percent'])} | {fmt(a['acceptance_time_s'])} |")
    lines+=['','A=固定标称；B=被动估计补偿；C=主动估计补偿；D=已知质量参考；C_shadow=主动估计、不补偿。',
            '端到端成功率优先于条件 RMSE：某组中途退出的种子未进入评价，完成子集的较小误差不能推成该组总体更可靠。源结果 flight_result.json 中是采样均方，本文由原始 CSV 统一重算时间加权均方。',
            '验收时间为有效独立验证窗口终点相对校准开始的时间，不包含另行记录的处理缓冲。质量未知时的“首次进入 2% 真值误差带”仅由离线评价程序计算。','',
            '## 配对差异','',
            '以下是 noisy 条件同种子前组减后组的平均 RMSE 差，正数代表后组误差较小。区间来自最多 5 个种子中可配对且完成评价的试验；配对数单列。bootstrap 仅作不确定性描述，不代表广泛统计结论。','',
            '| 指标 | 比较 | 配对数 | 平均差 cm | bootstrap 95% 区间 cm |','|---|---|---|---|---|']
    for c in contrasts:
        lo,hi=c['bootstrap_95pct_interval_m'];lines.append(f"| {c['metric']} | {c['comparison']} | {c['paired_seed_count']} | {c['mean_difference_m']*100:.3f} | [{lo*100:.3f}, {hi*100:.3f}] |")
    lines+=['','## 校准成本、瞬态与失败','',
            '| 条件 | 组 | 校准 RMSE cm | 动作代价 m²/s | 质量绝对误差 % | 独立窗残差 % | 评价饱和 % | 校准失败率 | 安全退出 / 落地未确认 |',
            '|---|---|---|---|---|---|---|---|---|']
    for a in aggregates:
        failure='—' if a['calibration_failure_fraction'] is None else f"{100*a['calibration_failure_fraction']:.1f}%"
        lines.append(f"| {a['phase']} | {a['group']} | {fmt(a['calibration_rmse_m'],100)} | {fmt(a['calibration_cost_m2_s'])} | {fmt(a['mass_abs_error_percent'])} | {fmt(a['holdout_relative_rms'],100)} | {fmt(a['saturation_fraction'],100)} | {failure} | {a['safety_exits']} / {a['landing_unconfirmed']} |")
    lines+=['','动作代价为整个校准段的参考速度平方积分，不是能耗。独立窗残差包含冻结候选之后的全部后续窗口；验收至少使用前五窗。','',
            '| 条件 | 组 | 转换代数力误差最大值 N | 公共 35–39 s 最大相邻力变化均值 N | 转换期位置峰值均值 cm | 离线推力平衡 RMS 均值 N |',
            '|---|---|---|---|---|---|']
    for a in aggregates:
        err=a['update_force_error_N']['max'];err='—' if err is None else f'{err:.3g}'
        lines.append(f"| {a['phase']} | {a['group']} | {err} | {fmt(a['transition_force_step_N'])} | {fmt(a['transition_position_peak_m'],100)} | {fmt(a['thrust_balance_rms_N'])} |")
    lines+=['','代数力误差仅隔离质量/积分器转换；相邻力变化同时含正常反馈和轨迹变化，不能称为质量更新跳变量。公共 35–39 s 窗也保留安全退出后恢复过程，因此 B 组的大力变化包含未补偿的失败试验，不能解释为质量转换冲击。离线推力平衡使用评价真值质量与速度，含气动力，未送入估计器。',
            '校准指标统计实际执行到的校准段：提前退出试验可能只有部分校准数据或没有校准数据。逐次时长、峰值和样本量在 metrics.json 中；不得把它们当成全部完整校准成功。']
    lines+=['','## 已接受估计的条件精度','', '| 条件 | 组 | 接受数 / 全部次数 | 仅接受子集质量绝对误差 % |','|---|---|---|---|']
    for a in aggregates:
        if a['calibration_accepts'] is not None:
            lines.append(f"| {a['phase']} | {a['group']} | {a['calibration_accepts']}/{a['n']} | {fmt(a['accepted_only_mass_abs_error_percent'])} |")
    lines+=['','## 全部原始运行','']
    for r in sorted(rows,key=lambda r:(r['phase'],r['seed'],r['group'])):
        lines.append(f"- {r['phase']} / {r['group']} / seed {r['seed']}: [日志与指标]({r['run']}/metrics.json)，全程 {'PASS' if r['flight_pass'] else 'FAIL'}，原因 {r['failure']}。")
    (out/'RESULTS.md').write_text('\n'.join(lines)+'\n')
    os.environ['MPLCONFIGDIR']=str(out/'mpl_cache')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    for ax,(field,title,scale) in zip(axes.flat,[('position_rmse_m','Evaluation: estimated position RMSE (cm)',100),('truth_position_rmse_m','Evaluation: aligned truth position RMSE (cm)',100),('mass_error_percent','Signed mass estimation error (%)',1),('calibration_rmse_m','Calibration position RMSE (cm)',100)]):
        for i,group in enumerate(GROUPS):
            values=[r[field]*scale for r in rows if r['phase']=='noisy' and r['group']==group and r[field] is not None]
            if values:
                ax.scatter(i+np.linspace(-.08,.08,len(values)),values,s=28,color=f'C{i}')
                ax.errorbar(i,np.mean(values),yerr=np.std(values,ddof=1) if len(values)>1 else 0,fmt='_',color='black',capsize=5)
        ax.set_xticks(range(len(GROUPS)),GROUPS);ax.set_title(title);ax.grid(axis='y',alpha=.25)
    fig.suptitle('PX4/Gazebo SITL — 5 attempted seeds per group, fixed gains, +20% payload\nEvaluation: completed flights only; failed calibrations retain nominal mass')
    fig.tight_layout();fig.savefig(out/'comparison.png',dpi=160);plt.close(fig)
    print(json.dumps(aggregates,indent=2))


if __name__=='__main__':main()
