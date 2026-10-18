from pathlib import Path
import json,numpy as np,pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parent;b=json.loads((R/'results/baseline_summary.json').read_text())
fig,axes=plt.subplots(4,1,figsize=(10,10),sharex=True,constrained_layout=True)
for ax,row in zip(axes,b['rows']):
 z=np.load(R/'results'/f"{row['run']}_timeseries.npz");t=z['elapsed'];m=(t>=34)&(t<=92);g=z['truth_elapsed'];gm=(g>=34)&(g<=92)
 ax.plot(t[m],np.linalg.norm(z['estimated_error_enu'][m],axis=1)*100,label='estimated position error',lw=.9);ax.plot(g[gm],np.linalg.norm(z['truth_error_enu'][gm],axis=1)*100,label='noise-free command error',lw=.9,alpha=.8);ax.axhline(row['position_bound_m']*100,color='black',ls='--',label='registered position bound');ax.axvspan(71,72,color='red',alpha=.12);ax.axvspan(91,92,color='green',alpha=.15);ax.set_title(row['run']);ax.set_ylabel('error (cm)');ax.grid(alpha=.2)
axes[0].legend(ncol=3,fontsize=8);axes[-1].set_xlabel('seconds after flight reference');fig.savefig(R/'results/baseline_recovery.png',dpi=150);fig.savefig(R/'results/baseline_recovery.pdf');plt.close(fig)
d=pd.read_csv(R/'results/ablation_metrics.csv');d=d[(d.phase=='task')&(d.target=='estimator')&(d.horizon_s==.1)].copy();d['family']=d.method.str.split('_').str[0];a=d.groupby(['family','kind'])[['velocity_rmse','omega_rmse']].mean();fig,axes=plt.subplots(1,2,figsize=(10,4),constrained_layout=True)
for ax,metric,unit in zip(axes,['velocity_rmse','omega_rmse'],['m/s','rad/s']):
 x=np.arange(3)
 for j,kind in enumerate(['stopped','original','gated']):ax.bar(x+(j-1)*.24,[a.loc[(fam,kind),metric] for fam in ['physics','supervised','jepa']],width=.24,label=kind)
 ax.set_xticks(x,['physics','supervised','JEPA-style']);ax.set_ylabel(f'{metric} ({unit})');ax.grid(axis='y',alpha=.2)
axes[0].legend();fig.suptitle('100 ms completed-interval prediction; mean trajectory RMSE');fig.savefig(R/'results/ablation.png',dpi=150);fig.savefig(R/'results/ablation.pdf')
