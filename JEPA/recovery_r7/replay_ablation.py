"""Finite R7-B replay: recorded causal targets, frozen model weights, no control."""
from pathlib import Path
import os,sys,json,time
import numpy as np,pandas as pd,torch
from scipy.spatial.transform import Rotation
R=Path(__file__).resolve().parent;OLD=Path(os.environ.get('R6_ROOT','/r6'));sys.path.insert(0,str(OLD))
from adapter import Adapter
from online import Engine
from aligned import rollout,basis
from gated_adapter import GatedAdapter
H=np.array([.1,.2,.4]);START=time.monotonic()
def read(p):return [json.loads(x) for x in p.read_text().splitlines()]
def rmse(e):return float(np.sqrt(np.mean(np.sum(e*e,axis=-1))))
def main():
 eng=Engine();scale=eng.norm['adapter_scale'];metrics=[];audit=[];stress=[];information=[];latents=[];timings=[]
 for folder in sorted((OLD/'runtime/results/experiment/runs').iterdir()):
  if not (folder/'online_summary.json').exists():continue
  assert time.monotonic()-START<120,'registered_offline_budget'
  states=read(folder/'aligned_states.jsonl');logs=read(folder/'online_predictions.jsonl');end={x['t_us']:x['elapsed'] for x in states};state={x['t_us']:np.array(x['state']) for x in states};lookup={(x['t_us'],x['method']):x for x in logs};snap={};adapters={};parity=0.
  for method in eng.models:
   adapters[method]={k:cls(scale) for k,cls in [('original',Adapter),('gated',GatedAdapter),('original_pulse',Adapter),('gated_pulse',GatedAdapter)]};records=[x for x in logs if x['method']==method];counts={}
   for row in records:
    phi=np.array(row['phi']);error=(np.array(row['target'])-row['pred_off'])/.1+np.array(row['bias_before']);elapsed=end[row['end_us']];pulse=np.zeros(6)
    if 51<elapsed<53:pulse[0]=.3*np.sin(np.pi*(elapsed-51)/2)**2
    snap[(row['t_us'],method)]=(adapters[method]['gated'].W.copy(),adapters[method]['gated'].bias.copy(),adapters[method]['gated'].phi0.copy())
    for kind,a in adapters[method].items():
     if kind=='original':
      pred=state[row['t_us']]+.1*((np.array(row['pred_off'])-state[row['t_us']])/.1-np.array(row['bias_before'])+a.predict(phi));parity=max(parity,float(abs(pred-row['pred_on']).max()))
     then=time.perf_counter_ns();a.update(phi,error+(pulse if kind.endswith('pulse') else 0),elapsed,row['valid']);cost=(time.perf_counter_ns()-then)*1e-6
     if 48<=elapsed<56:timings.append(dict(run=folder.name,method=method,kind=kind,ms=cost))
   original=adapters[method]['original'];gated=adapters[method]['gated'];expected=json.loads((folder/'online_summary.json').read_text())['adapters'][method];assert abs(original.W-np.array(expected['W'])).max()<1e-8
   audit.append(dict(run=folder.name,method=method,original_updates=original.updates,gated_updates=gated.updates,gated_rank=gated.rank,gate=gated.last_gate,gated_reasons=gated.reasons))
   for kind in ['original','gated']:
    a=adapters[method][kind];b=adapters[method][kind+'_pulse'];changes=[np.linalg.norm(b.predict(np.array(x['phi']))[:3]-a.predict(np.array(x['phi']))[:3]) for x in records if 58<=x['elapsed']<68]
    stress.append(dict(run=folder.name,method=method,kind=kind,W_difference=float(np.linalg.norm(b.W-a.W)),task_correction_rms_difference=float(np.sqrt(np.mean(np.square(changes)))),synthetic_label_pulse_not_wind=True))
  assert parity<1e-8,parity
  phi=np.array([x['phi'] for x in logs if x['method']=='physics' and 48<=end[x['end_us']]<56]);phi-=phi.mean(0);eig,vec=np.linalg.eigh(phi.T@phi/len(phi));delta=np.zeros((6,4));delta[0]=vec[:,-1]*.2;parameter_response=phi@delta.T;disturbance_response=np.array(parameter_response,copy=True)
  information.append(dict(run=folder.name,regressor_eigenvalues=eig.tolist(),regressor_rank=int(sum(eig>=.05)),unstructured_W_identifiable_dimensions=int(6*sum(eig>=.05)),unstructured_W_total_dimensions=24,parameter_or_correlated_disturbance_observation_difference=float(abs(parameter_response-disturbance_response).max()),constructed_disturbance_normalized_rms=float(np.sqrt(np.mean(parameter_response**2))),parity_max=parity))
  x=np.array([s['x'] for s in states]);tt=np.array([s['t_us'] for s in states]);indices=[];records=[]
  for i in range(15,len(states)-4):
   s=states[i]
   if not 48<=s['elapsed']<67.5 or tt[i+4]-tt[i-15]!=1900000 or (int(tt[i]),'physics') not in lookup:continue
   acts=np.concatenate([states[i+k]['actions_from_previous'] for k in range(1,5)]);indices.append(i);records.append((x[i-15:i+1],acts,x[i,:6],np.array([states[i+k]['state'] for k in [1,2,4]]),rollout(x[i],acts)))
  xp,acts,current,target,physical=map(np.array,zip(*records));xn=(torch.tensor(xp,dtype=torch.float32)-eng.mean)/eng.std;un=torch.tensor(acts,dtype=torch.float32);t=tt[indices]*1e-6;elapsed=np.array([states[i]['elapsed'] for i in indices]);phi=np.array([[basis(rate) for rate in row] for row in (physical-current[:,None,:])/H[None,:,None]]);phi=(phi-eng.pm)/eng.ps
  z=pd.read_csv(folder/'truth_diagnostic.csv',usecols=['t','stage','base_link_vx','base_link_vy','base_link_vz','base_link_wx','base_link_wy','base_link_wz','base_link_qw','base_link_qx','base_link_qy','base_link_qz']);z=z[z.stage==1];rot=Rotation.from_quat(z[['base_link_qx','base_link_qy','base_link_qz','base_link_qw']].to_numpy());truth=np.c_[z[['base_link_vx','base_link_vy','base_link_vz']].to_numpy(),rot.inv().apply(z[['base_link_wx','base_link_wy','base_link_wz']].to_numpy())];truth_target=np.stack([np.stack([np.interp(t+h,z.t,truth[:,j]) for j in range(6)],-1) for h in H],1)
  for method,model in eng.models.items():
   rows=[lookup[(int(tt[i]),method)] for i in indices];oldw=np.array([r['W_before'] for r in rows]);bias=np.array([r['bias_before'] for r in rows]);center=np.array([r['phi0_before'] for r in rows]);gatew=np.array([snap[(int(tt[i]),method)][0] for i in indices]);assert np.max(abs(bias-np.array([snap[(int(tt[i]),method)][1] for i in indices])))<1e-9
   with torch.inference_mode():
    res=np.zeros_like(physical) if model is None else model(xn,un).numpy()*eng.ys+eng.ym
    if model is not None:
     latent=model.encoder(xn)[0][:,-1].numpy();ev=np.maximum(np.linalg.eigvalsh(np.cov(latent.T)),0);prob=ev/max(ev.sum(),1e-12);latents.append(dict(run=folder.name,method=method,latent_mean_std=float(np.std(latent,axis=0).mean()),covariance_eigenvalues=ev.tolist(),effective_rank=float(np.exp(-np.sum(prob*np.log(prob+1e-30)))),trace=float(ev.sum())))
   for kind,W in [('stopped',np.zeros_like(oldw)),('original',oldw),('gated',gatew)]:
    corr=np.clip(np.einsum('nij,nhj->nhi',W,phi-center[:,None,:])*eng.ys,[-.3]*3+[-1]*3,[.3]*3+[1]*3);pred=physical+H[None,:,None]*(res+bias[:,None,:]+corr)
    if kind=='original':assert abs(pred[:,0]-np.array([r['pred_on'] for r in rows])).max()<1e-5
    for phase,mask in [('id',(elapsed>=48)&(elapsed<55.5)),('task',(elapsed>=58)&(elapsed<67.5))]:
     for target_name,tgt in [('estimator',target),('truth_absolute',truth_target)]:
      for j,h in enumerate(H):
       e=pred[:,j]-tgt[:,j];metrics.append(dict(run=folder.name,method=method,kind=kind,phase=phase,target=target_name,horizon_s=float(h),window_count=int(mask.sum()),velocity_rmse=rmse(e[mask,:3]),omega_rmse=rmse(e[mask,3:])))
  print(folder.name,'replay complete',flush=True)
 df=pd.DataFrame(metrics);df.to_csv(R/'results/ablation_metrics.csv',index=False);tf=pd.DataFrame(timings);time_summary=tf.groupby(['method','kind']).ms.agg(['count','mean','max',lambda x:x.quantile(.95)]).reset_index();time_summary.columns=['method','kind','count','mean_ms','max_ms','p95_ms'];time_summary.to_csv(R/'results/ablation_timing.csv',index=False)
 summary=dict(wall_s=time.monotonic()-START,registered_budget_s=120,independent_flights=8,unique_physical_groups=2,new_network_training_steps=0,frozen_models=eng.summary()['frozen_backbone_files_unchanged'],audits=audit,stress=stress,information=information,latent_diagnostics=latents,control_enabled=False,prediction_semantics='completed-interval executed-action conditioned, retrospective; not prospective known-action forecast')
 assert summary['wall_s']<120 and summary['frozen_models'];(R/'results/ablation_audit.json').write_text(json.dumps(summary,indent=2));print('wall',summary['wall_s'])
if __name__=='__main__':main()
