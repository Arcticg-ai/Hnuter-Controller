"""Post-run consumption, saturation, tracking and final-hover checks. No control."""
from pathlib import Path
import sys,json
import numpy as np,pandas as pd
R=Path(__file__).resolve().parent;sys.path.insert(0,str(R/'runtime'))
import evaluate_control as inherited
from init_gate import compare
def one(name):
    inherited.WINDOWS={'whole':(48,68),'identification':(48,56),'independent_task':(58,68)}
    x=inherited.evaluate(name);x['model_inference_not_run']=False;x['learned_control_enabled']=False;x['old_recovery_window_not_comparable_to_R2_oracle']=x.pop('recovery');p=R/'runtime/results/experiment/runs'/name;tr=pd.read_csv(p/'trajectory.csv');ep=json.loads((p/'diagnostic_epoch.json').read_text())['epoch'];t=tr.px4_time_s.to_numpy()-ep;err=tr[['x','y','z_up']].to_numpy()-tr[['x_ref','y_ref','z_ref']].to_numpy();pre=(t>=45)&(t<48);bound=max(1.1*np.sqrt(np.mean(np.sum(err[pre]**2,1))),.02)
    obs=[json.loads(s) for s in (p/'deployable_observations.jsonl').read_text().splitlines()];ot=np.array([s['elapsed'] for s in obs]);speed=np.linalg.norm([s['velocity_enu'] for s in obs],axis=1);final=(t>=71)&(t<72);vf=np.interp(t[final],ot,speed);x['new_task_final_hover']=dict(window=[71,72],position_bound_m=float(bound),position_max_m=float(np.linalg.norm(err[final],axis=1).max()),speed_max_m_s=float(vf.max()),stable_final_1s=bool(np.all(np.linalg.norm(err[final],axis=1)<=bound) and np.all(vf<=.05)),not_a_retest_of_old_oracle=True)
    path=R/'results'/f'{name}_native.json';x['checks']['learned_control_disabled']=x['checks'].pop('no_learning');path.write_text(json.dumps(x,indent=2));(R/'runtime/results'/f'{name}_control.json').write_text(json.dumps(x,indent=2));print(name,'consumer',x['contract_passed'],'final',x['new_task_final_hover']['stable_final_1s']);return x
def main():
    matrix=json.loads((R/'runtime/MATRIX.json').read_text());done=[]
    for name in matrix['order']:
        if not (R/'runtime/results'/f'{name}_outcome.json').exists():continue
        done.append(name);one(name)
    pairs=[]
    for name in done:
        if not name.endswith('_conventional'):continue
        active=name.replace('_conventional','_active');root=R/'runtime/results/experiment/runs';a=json.loads((root/active/'initialization_admission.json').read_text());b=json.loads((root/name/'initialization_admission.json').read_text());pairs.append(dict(active=active,conventional=name,comparison=compare(b['summary'],a['summary']),same_seed_not_exact_state_branching=True))
    (R/'results/initialization_pairs.json').write_text(json.dumps(pairs,indent=2))
if __name__=='__main__':main()
