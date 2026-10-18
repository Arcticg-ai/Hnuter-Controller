from pathlib import Path
import json,numpy as np,pandas as pd
R=Path(__file__).resolve().parent
def main():
    matrix=json.loads((R/'runtime/MATRIX.json').read_text());meta={c['id']:c for c in matrix['cases']};allrows=[];runs=[]
    for name in matrix['order']:
        p=R/'results'/f'{name}_evaluation.json'
        if not p.exists():continue
        r=json.loads(p.read_text());runs.append(r)
        for row in r['metrics']:
            row.update(family=meta[name]['family'],simulation_seed=meta[name]['seed'],excitation=meta[name]['excitation'],method_group=row['method'].split('_')[0]);allrows.append(row)
    df=pd.DataFrame(allrows);df.to_csv(R/'results/all_metrics.csv',index=False);groups=['family','excitation','method_group','update','phase','horizon_s','target'];agg=[]
    for keys,g in df.groupby(groups):
        row=dict(zip(groups,keys));row.update(velocity_rmse_m_s=float(np.sqrt(np.mean(g.velocity_rmse_m_s**2))),omega_rmse_rad_s=float(np.sqrt(np.mean(g.omega_rmse_rad_s**2))),simulation_runs=int(g.run.nunique()),model_seeds=int(g.method.nunique()));agg.append(row)
    a=pd.DataFrame(agg);a.to_csv(R/'results/aggregate_metrics.csv',index=False);gates=[]
    for method in ['physics','supervised','jepa']:
        for family in ['mass','com']:
            g=a[(a.method_group==method)&(a.family==family)&(a.phase=='independent_task')&(a.horizon_s==.1)&(a.target=='estimator')]
            if len(g)!=4:continue
            active_on=g[(g.excitation=='active')&(g['update']=='on')].iloc[0];active_off=g[(g.excitation=='active')&(g['update']=='off')].iloc[0];conv=g[(g.excitation=='conventional')&(g['update']=='on')].iloc[0];ratios={};checks={}
            for metric in ['velocity_rmse_m_s','omega_rmse_rad_s']:
                ratios[metric+'_on_over_off']=float(active_on[metric]/active_off[metric]);ratios[metric+'_active_over_conventional']=float(active_on[metric]/conv[metric]);checks[metric+'_update_benefit_10pct']=ratios[metric+'_on_over_off']<=.9;checks[metric+'_excitation_benefit_5pct']=ratios[metric+'_active_over_conventional']<=.95
                worst=[]
                sub=df[(df.method_group==method)&(df.family==family)&(df.excitation=='active')&(df.phase=='independent_task')&(df.horizon_s==.1)&(df.target=='estimator')]
                for seed,ss in sub.groupby('simulation_seed'):
                    on=ss[ss['update']=='on'][metric].to_numpy();off=ss[ss['update']=='off'][metric].to_numpy();worst.append(float(np.sqrt(np.mean(on**2)/np.mean(off**2))))
                ratios[metric+'_worst_seed_on_over_off']=max(worst);checks[metric+'_each_seed_no_worse_5pct']=max(worst)<=1.05
            gates.append(dict(method=method,family=family,checks=checks,ratios=ratios,passed=all(checks.values())))
    outcomes=[json.loads((R/'runtime/results'/f"{r['run']}_outcome.json").read_text()) for r in runs];native=[json.loads((R/'results'/f"{r['run']}_native.json").read_text()) for r in runs if (R/'results'/f"{r['run']}_native.json").exists()];jepa=[g for g in gates if g['method']=='jepa'];result=dict(completed_runs=len(runs),planned_runs=8,safety_data_pass=bool(len(runs)==8 and all(o['continue_batch'] for o in outcomes)),prediction_gates=gates,jepa_prediction_release=bool(len(jepa)==2 and all(g['passed'] for g in jepa)),learned_control_release=False,old_oracle_recovery_gate_resolved=False,active_control_runs=0,method='RMS aggregation of per-flight/per-model-seed RMSE; model seeds share each physical flight and are not independent trials',max_latency_p95_ms=max(o['details']['collection']['details']['latency_p95_ms'] for o in outcomes),max_core_p95_ms=max(o['details']['collection']['details']['core_p95_ms'] for o in outcomes),max_replay_difference=max(r['online_offline_max_difference'] for r in runs),new_final_hover_failures=[n['run'] for n in native if not n['new_task_final_hover']['stable_final_1s']]);(R/'results/decision.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(10,6.5),layout='constrained');colors=['#567896','#d28a45','#4b9378'];methods=['physics','supervised','jepa']
    for col,family in enumerate(['mass','com']):
        for row,metric in enumerate(['velocity_rmse_m_s','omega_rmse_rad_s']):
            ax=axes[row,col];g=a[(a.family==family)&(a.phase=='independent_task')&(a.horizon_s==.1)&(a.target=='estimator')]
            for j,(exc,update) in enumerate([('conventional','off'),('conventional','on'),('active','off'),('active','on')]):
                heights=[float(g[(g.method_group==m)&(g.excitation==exc)&(g['update']==update)][metric].iloc[0]) for m in methods];ax.bar(np.arange(3)+(j-1.5)*.19,heights,width=.18,label=exc+' / '+update,alpha=.5+.15*j)
            ax.set_xticks(range(3),methods);ax.set_title(family+' | '+('velocity (m/s)' if row==0 else 'angular rate (rad/s)'));ax.set_ylim(bottom=0);ax.grid(axis='y',alpha=.2)
    axes[0,0].legend(fontsize=7);fig.suptitle('R6: independent task, 100 ms executed-action-conditioned prediction\nFrozen backbone; same 24-coefficient adapter; 2 simulation seeds per group');fig.savefig(R/'results/comparison.png',dpi=160);fig.savefig(R/'results/comparison.pdf');plt.close(fig)
if __name__=='__main__':main()
