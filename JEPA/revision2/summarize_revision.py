"""All-denominator report; formal comparison is only the new matrix batch."""
import csv,json,hashlib
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent
G=['A','B','C','D','C_shadow']
def main():
 out=R/'results';out.mkdir(exist_ok=True)
 batches=[];trials=[]
 for p in sorted((R/'runs').glob('batch_*')):
  plan=json.loads((p/'plan.json').read_text());rs=json.loads((p/'results.json').read_text()) if (p/'results.json').exists() else []
  batches.append(dict(path=str(p),planned=len(plan['jobs']),finished=len(rs),passed=sum(r.get('flight',{}).get('status')=='PASS' for r in rs),ready_unarmed=sum(r.get('flight',{}).get('status')=='READY_UNARMED' for r in rs),failures=sum(r.get('flight',{}).get('status') not in ('PASS','READY_UNARMED') for r in rs)))
  for r in rs:
   f=r.get('flight',{});mp=Path(r['run'])/'metrics.json' if r['run'] else None;m=json.loads(mp.read_text()) if mp and mp.exists() else {};e=m.get('phases',{}).get('EVALUATION',{});c=m.get('phases',{}).get('CALIBRATION',{});ef=e if e.get('duration_s',0)>=15.9 else {}
   trials.append(dict(batch=p.name,attitude_reference=f.get('attitude_reference'),group=r['group'],seed=r['seed'],status=f.get('status','LAUNCH_FAILURE'),failure=f.get('failure'),landed=f.get('landed'),disarmed=f.get('disarmed'),ready_s=f.get('ready_at_wall_s'),sensor_samples=f.get('sensor_samples'),accepted=f.get('mass_accepted'),mass=f.get('mass_estimate_kg'),mass_error_pct=m.get('mass',{}).get('error_percent'),acceptance_s=m.get('mass',{}).get('acceptance_time_s'),holdout=m.get('mass',{}).get('holdout_relative_rms'),evaluation_duration_s=e.get('duration_s'),partial_or_full_rmse=e.get('position_rmse_m'),rmse=ef.get('position_rmse_m'),truth_rmse=ef.get('truth_position_rmse_m'),max_error=e.get('position_max_m'),saturation=e.get('saturation_fraction'),calibration_rmse=c.get('position_rmse_m'),calibration_cost=c.get('reference_velocity_energy_m2_s'),force_algebraic=m.get('compensation',{}).get('max_algebraic_update_force_error_N'),force_step=m.get('compensation',{}).get('max_adjacent_logged_force_change_during_update_N'),transient_error=m.get('compensation',{}).get('transient_position_max_m'),run=r['run']))
 if trials:
  with (out/'all_attempts.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(trials[0]));w.writeheader();w.writerows(trials)
 all_formal=[x for x in trials if x['batch'].startswith('batch_matrix_')]
 primary='legacy' if any(x['attitude_reference']=='legacy' for x in all_formal) else 'level'
 formal=[x for x in all_formal if x['attitude_reference']==primary]
 groups=[];contrasts=[]
 for g in G:
  rr=[r for r in formal if r['group']==g];s=dict(group=g,attempts=len(rr),flight_pass=sum(r['status']=='PASS' for r in rr),accepted=sum(bool(r['accepted']) for r in rr),confirmed_landed_disarmed=sum(bool(r['landed'] and r['disarmed']) for r in rr))
  for metric in ['rmse','truth_rmse','max_error','mass_error_pct','acceptance_s','holdout','calibration_rmse','calibration_cost','saturation','force_step','transient_error']:
   vals=[r[metric] for r in rr if r[metric] is not None]
   s[metric]=dict(n=len(vals),mean=float(np.mean(vals)) if vals else None,std=float(np.std(vals,ddof=1)) if len(vals)>1 else None)
  groups.append(s)
 for ga,gb in [('B','C'),('C_shadow','C'),('A','B'),('A','C')]:
  for metric in ['rmse','truth_rmse']:
   a={r['seed']:r[metric] for r in formal if r['group']==ga and r[metric] is not None};b={r['seed']:r[metric] for r in formal if r['group']==gb and r[metric] is not None};seeds=sorted(a.keys()&b.keys());v=np.array([a[s]-b[s] for s in seeds]);boot=np.mean(np.random.default_rng(41).choice(v,(10000,len(v))),axis=1) if len(v) else []
   contrasts.append(dict(comparison=ga+' minus '+gb,metric=metric,seeds=seeds,mean=float(np.mean(v)) if len(v) else None,bootstrap95=np.quantile(boot,[.025,.975]).tolist() if len(v) else None))
 summary=dict(primary_attitude_reference=primary,batches=batches,formal=groups,paired=contrasts,other_reference_trials=[x for x in all_formal if x['attitude_reference']!=primary])
 (out/'summary.json').write_text(json.dumps(summary,indent=2))
 old=list(csv.DictReader((R.parent/'results/all_trials.csv').open()));lines=['# 第二轮实际回归结果','','所有启动尝试保留；正式比较仅使用新 matrix，定向/消融/恢复测试单列。','', '| 批次 | 计划 | 已结束 | 飞行成功 | 未解锁就绪 | 失败 |','|---|---:|---:|---:|---:|---:|']
 for b in batches:lines.append(f"| {Path(b['path']).name} | {b['planned']} | {b['finished']} | {b['passed']} | {b['ready_unarmed']} | {b['failures']} |")
 lines+=['','## 原问题种子前后对照','','参考语义、门禁与恢复方式不同，随机种子相同；并行调度使初始估计与消息时序不保证逐位一致。','', '| 组/种子 | 原飞行 | 原遥测样本 | 新飞行 | 新遥测样本 | 新质量验收 | 新落地/上锁 |','|---|---|---:|---|---:|---|---|']
 for r in trials:
  if not r['batch'].startswith('batch_targeted_'):continue
  orig=next(x for x in old if x['phase']=='noisy' and x['group']==r['group'] and int(x['seed'])==r['seed'])
  lines.append(f"| {r['group']}/{r['seed']} | {orig['flight_pass']} | {orig['sensor_samples']} | {r['status']} | {r['sensor_samples']} | {r['accepted']} | {r['landed']}/{r['disarmed']} |")
 lines+=['','## 同口径正式矩阵：'+primary,'','RMSE只对完成评价阶段者计算，n单列。失败不填0。质量精度不能代替成功率。','', '| 组 | 飞行成功/尝试 | 验收 | 估计位置 RMSE cm (n) | 真值 RMSE cm (n) | 校准动作成本 m²/s |','|---|---|---:|---|---|---:|']
 fmt=lambda x:f"{100*x['mean']:.3f} ({x['n']})" if x['mean'] is not None else '未完成'
 for g in groups:lines.append(f"| {g['group']} | {g['flight_pass']}/{g['attempts']} | {g['accepted']} | {fmt(g['rmse'])} | {fmt(g['truth_rmse'])} | {g['calibration_cost']['mean']} |")
 lines+=['','完整逐次结果：`all_attempts.csv`。配对小样本bootstrap区间：`summary.json`。失败与尚未验证项见 `CONCLUSIONS.md`（完成后生成）。']
 (out/'RESULTS.md').write_text('\n'.join(lines)+'\n');print(json.dumps(batches,indent=2))
if __name__=='__main__':main()
