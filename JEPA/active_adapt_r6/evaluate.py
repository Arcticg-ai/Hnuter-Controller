"""Post-flight evaluation only. Gazebo truth is isolated to this module."""
from pathlib import Path
import json,argparse,hashlib
import numpy as np,pandas as pd,torch
from scipy.spatial.transform import Rotation
from aligned import rollout,basis
from online import Engine
R=Path(__file__).resolve().parent;H=np.array([.1,.2,.4])
def loadjsonl(p):return [json.loads(s) for s in p.read_text().splitlines()]
def rmse(e):return float(np.sqrt(np.mean(np.sum(e*e,axis=-1))))
def evaluate(name):
    p=R/'runtime/results/experiment/runs'/name;eng=Engine();states=loadjsonl(p/'aligned_states.jsonl');online=loadjsonl(p/'online_predictions.jsonl');lookup={(x['t_us'],x['method']):x for x in online};x=np.array([s['x'] for s in states]);tt=np.array([s['t_us'] for s in states]);records=[];index=[]
    for i in range(15,len(states)-4):
        s=states[i]
        if not 48<=s['elapsed']<67.5 or tt[i+4]-tt[i-15]!=1900000 or (tt[i],'physics') not in lookup:continue
        acts=np.concatenate([states[i+k]['actions_from_previous'] for k in range(1,5)]);physical=rollout(x[i],acts);index.append(i);records.append((x[i-15:i+1],acts,x[i,:6],np.array([states[i+k]['state'] for k in [1,2,4]]),physical))
    assert records;xp,acts,current,target,physical=map(np.array,zip(*records));xn=(torch.tensor(xp,dtype=torch.float32)-eng.mean)/eng.std;un=torch.tensor(acts,dtype=torch.float32);t=tt[index]*1e-6;elapsed=np.array([states[i]['elapsed'] for i in index]);physrate=(physical-current[:,None,:])/H[None,:,None];phi=np.array([[basis(rate) for rate in row] for row in physrate]);phi=(phi-eng.pm)/eng.ps
    z=pd.read_csv(p/'truth_diagnostic.csv',usecols=['t','stage','base_link_vx','base_link_vy','base_link_vz','base_link_wx','base_link_wy','base_link_wz','base_link_qw','base_link_qx','base_link_qy','base_link_qz']);z=z[z.stage==1];rot=Rotation.from_quat(z[['base_link_qx','base_link_qy','base_link_qz','base_link_qw']].to_numpy());truth=np.c_[z[['base_link_vx','base_link_vy','base_link_vz']].to_numpy(),rot.inv().apply(z[['base_link_wx','base_link_wy','base_link_wz']].to_numpy())];truth_target=np.stack([np.stack([np.interp(t+h,z.t,truth[:,j]) for j in range(6)],-1) for h in H],1);truth_start=np.stack([np.interp(t,z.t,truth[:,j]) for j in range(6)],-1);metric=[];raw=[];parity=0.;adaptation=[]
    for name_,m in eng.models.items():
        snapshots=[lookup[(int(tt[i]),name_)] for i in index];w=np.array([a['W_before'] for a in snapshots]);bias=np.array([a['bias_before'] for a in snapshots]);phi0=np.array([a['phi0_before'] for a in snapshots]);correction=np.einsum('nij,nhj->nhi',w,phi-phi0[:,None,:])*eng.ys;correction=np.clip(correction,[-.3]*3+[-1]*3,[.3]*3+[1]*3)
        with torch.inference_mode():res=np.zeros_like(physical) if m is None else m(xn,un).numpy()*eng.ys+eng.ym
        for update in ['off','on']:
            pred=physical+H[None,:,None]*(res+bias[:,None,:]+(correction if update=='on' else 0));original=np.array([a['pred_'+update] for a in snapshots]);parity=max(parity,float(abs(pred[:,0]-original).max()))
            for phase,mask in [('identification',(elapsed>=48)&(elapsed<55.5)),('independent_task',(elapsed>=58)&(elapsed<67.5))]:
                for hi,h in enumerate(H):
                    for kind,tgt in [('estimator',target),('truth_absolute',truth_target),('truth_increment',truth_target-truth_start[:,None,:]+current[:,None,:])]:
                        err=pred[:,hi]-tgt[:,hi];metric.append(dict(run=name,method=name_,update=update,phase=phase,horizon_s=float(h),target=kind,n=int(sum(mask)),velocity_rmse_m_s=rmse(err[mask,:3]),omega_rmse_rad_s=rmse(err[mask,3:])))
            for i in range(len(t)):
                for hi,h in enumerate(H):raw.append(dict(run=name,method=name_,update=update,t_us=int(tt[index[i]]),elapsed=elapsed[i],horizon_s=float(h),v_squared=float(sum((pred[i,hi,:3]-target[i,hi,:3])**2)),w_squared=float(sum((pred[i,hi,3:]-target[i,hi,3:])**2)),truth_v_squared=float(sum((pred[i,hi,:3]-truth_target[i,hi,:3])**2)),truth_w_squared=float(sum((pred[i,hi,3:]-truth_target[i,hi,3:])**2))))
        rows=[r for r in online if r['method']==name_ and 48<=r['elapsed']<56];times=np.array([r['elapsed'] for r in rows]);v=np.array([r['target'] for r in rows]);on=np.array([r['pred_on'] for r in rows]);off=np.array([r['pred_off'] for r in rows]);good=[]
        for j,time_ in enumerate(times):
            mask=(times>time_-1)&(times<=time_);ratios=[rmse((on-v)[mask,sl])/max(rmse((off-v)[mask,sl]),1e-12) for sl in [slice(0,3),slice(3,6)]];good.append(mask.sum()>=9 and max(ratios)<=.9)
        first=None;since=None
        for time_,ok in zip(times,good):
            if not ok:since=None
            elif since is None:since=time_
            elif time_-since>=2:first=float(since-48);break
        adaptation.append(dict(method=name_,first_sustained_2s_improvement_s=first,right_censored_s=8 if first is None else None,samples_to_first=None if first is None else int(np.sum(times<=48+first)),definition='both velocity and omega trailing1s error improve10percent, sustained2s; ID-window only'))
    assert parity<1e-5,parity
    tr=pd.read_csv(p/'trajectory.csv');epoch=json.loads((p/'diagnostic_epoch.json').read_text())['epoch'];te=tr.px4_time_s.to_numpy()-epoch;pe=tr[['x','y','z_up']].to_numpy()-tr[['x_ref','y_ref','z_ref']].to_numpy();control={}
    for phase,lo,hi in [('pre',45,48),('id',48,56),('task',58,68),('final_hover',68,72)]:
        mask=(te>=lo)&(te<hi);control[phase]=dict(position_rmse_m=rmse(pe[mask]),position_peak_m=float(np.linalg.norm(pe[mask],axis=1).max()),tilt_peak_deg=float(tr.tilt_deg[mask].max()),motor_max=float(tr.motor_max[mask].max()),samples=int(sum(mask)))
    from runtime.excitation_r6 import reference
    mode='conventional' if name.endswith('conventional') else 'active';ts=np.linspace(48,56,8001);angles=np.array([reference(t,mode)[0] for t in ts]);energy=float(np.trapz(np.sum(angles**2,axis=1),ts));coverage=json.loads((p/'online_summary.json').read_text());out=dict(run=name,metrics=metric,adaptation=adaptation,control=control,commanded_angle_energy_rad2_s=energy,online_offline_max_difference=parity,online=coverage,measured_truth_target_gap=dict(velocity_m_s=rmse((target-truth_target)[:,:,:3]),omega_rad_s=rmse((target-truth_target)[:,:,3:])),prediction_semantics='completed-interval executed-action conditioned; not prospective',truth_evaluation_only=True)
    (R/'results'/f'{name}_evaluation.json').write_text(json.dumps(out,indent=2));pd.DataFrame(raw).to_csv(R/'results'/f'{name}_predictions.csv',index=False);print(json.dumps(dict(run=name,parity=parity,control=control,adaptation=adaptation),indent=2));return out
if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('run');evaluate(a.parse_args().run)
