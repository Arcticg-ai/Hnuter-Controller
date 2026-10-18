"""Offline initialization statistics. No truth enters the flight/control input."""
from pathlib import Path
import json
import numpy as np,pandas as pd
from pyulog import ULog
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parent

def stats(t,x):
 t=np.asarray(t);x=np.asarray(x);tc=t-t.mean();slope=(tc[:,None]*x).sum(0)/(tc*tc).sum();return dict(mean=x.mean(0).tolist(),std=x.std(0).tolist(),rms=np.sqrt(np.mean(x*x,0)).tolist(),peak_to_peak=np.ptp(x,axis=0).tolist(),linear_slope_per_s=slope.tolist(),last_half_second_minus_first_half_second=(x[t>=47.5].mean(0)-x[t<45.5].mean(0)).tolist())
def main():
 out={};series={}
 for p in sorted((R/'results/experiment/runs').glob('*')):
  if not (p/'diagnostic_epoch.json').exists():out[p.name]={'not_available':'no flight epoch'};continue
  ep=json.loads((p/'diagnostic_epoch.json').read_text())['epoch'];tr=pd.read_csv(p/'trajectory.csv');t=tr.px4_time_s.to_numpy()-ep;m=(t>=45)&(t<48)
  if m.sum()<2:out[p.name]={'not_available':'no complete pre-window'};continue
  tele=[json.loads(x) for x in (p/'init_telemetry.jsonl').read_text().splitlines()];abs_t=np.array([x['t'] for x in tele]);tm=(abs_t>=ep+45)&(abs_t<ep+48);abs_t=abs_t[tm];t=abs_t-ep;err=np.array([x['error'] for x in tele])[tm];ref=np.stack([np.interp(abs_t,tr.px4_time_s,tr[k]) for k in ['x_ref','y_ref','z_ref']],1);est=ref+err
  chunks=[]
  for z in pd.read_csv(p/'truth_diagnostic.csv',usecols=['t','stage','base_link_px','base_link_py','base_link_pz'],chunksize=25000):chunks.append(z[(z.stage==1)&(z.t>=ep+45)&(z.t<ep+48)])
  g=pd.concat(chunks);gt=g.t.to_numpy()-ep;gp=g[['base_link_px','base_link_py','base_link_pz']].to_numpy();at=np.stack([np.interp(t,gt,gp[:,i]) for i in range(3)],1);bias=est-at
  u=ULog(str(next((p/'rootfs/log').rglob('*.ulg'))));d=u.get_dataset('hnuter_translation_diag').data;abs_dt=d['diag_control_timestamp']*1e-6;dt=abs_dt-ep;dm=(abs_dt>=ep+45)&(abs_dt<ep+48)
  def vec(k):return np.stack([d[f'{k}[{i}]'] for i in range(3)],1).astype(float)
  local=u.get_dataset('vehicle_local_position').data;lm=(local['timestamp']*1e-6>=ep+45)&(local['timestamp']*1e-6<ep+48);resets={k:int(np.ptp(local[k][lm])) for k in ['xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter','heading_reset_counter']}
  online=json.loads((p/'initialization_admission.json').read_text());rec=dict(epoch=ep,position_error_enu=stats(t,err),estimated_position_enu=stats(t,est),true_base_position_enu=stats(gt,gp),est_minus_base_offset_enu=stats(t,bias),velocity_ned=stats(dt[dm],vec('diag_velocity')[dm]),integral_ned=stats(dt[dm],vec('diag_integral_initial')[dm]),reset_counter_changes=resets,online_admission=online,note='Raw estimator-base offset includes origins/z0. Report drift without per-run translation cancellation. Truth is diagnostic only.')
  rec['joints']={}
  for n in ['rj2','lj2','rj1','lj1']:
   z=pd.read_csv(p/f'servo_{n}.csv');jt=z.sim_ns.to_numpy()*1e-9-ep;jm=(jt>=45)&(jt<48);rec['joints'][n]=dict(angle_deg=stats(jt[jm],np.rad2deg(z.post_q_rad.to_numpy()[jm,None])),rate_deg_s=stats(jt[jm],np.rad2deg(z.post_qd_rad_s.to_numpy()[jm,None])))
  rec['online_mean_replay_max_abs_error']=max(float(np.max(abs(np.array(online['summary'][k])-np.array(rec[j]['mean'])))) for k,j in [('position_error_mean_enu','position_error_enu'),('velocity_mean_ned','velocity_ned'),('integral_mean_ned','integral_ned')]);assert rec['online_mean_replay_max_abs_error']<1e-12
  out[p.name]=rec;series[p.name]=(t,err,bias-bias[0],dt[dm],vec('diag_integral_initial')[dm]);np.savez_compressed(R/'results'/f'{p.name}_pre_diagnostic.npz',t=t,error_enu=err,est_minus_base_offset_enu=bias,true_base_enu=at,diag_t=dt[dm],integral_ned=vec('diag_integral_initial')[dm])
 (R/'results/initialization_diagnosis.json').write_text(json.dumps(out,indent=2,allow_nan=False))
 if series:
  fig,ax=plt.subplots(3,1,figsize=(10,10),sharex=True)
  for n,(t,e,b,dt,iv) in series.items():
   ax[0].plot(t,e[:,1]*100,label=n);ax[1].plot(t,b[:,1]*100,label=n);ax[2].plot(dt,iv[:,0],label=n)
  for a,l in zip(ax,['Estimated Y error (cm)','Estimator-base Y offset change (cm)','NED X integral (m/s2)']):a.set_ylabel(l);a.grid(alpha=.2);a.legend()
  ax[-1].set_xlabel('Elapsed physical time (s)');fig.tight_layout();fig.savefig(R/'results/initialization_window.png',dpi=140);plt.close(fig)
 print(json.dumps({n:{'error_mean':v.get('position_error_enu',{}).get('mean'),'bias_change':v.get('est_minus_base_offset_enu',{}).get('last_half_second_minus_first_half_second'),'online':v.get('online_admission',{}).get('checks')} for n,v in out.items()},indent=2))
if __name__=='__main__':main()
