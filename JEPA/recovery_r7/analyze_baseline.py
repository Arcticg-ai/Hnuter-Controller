"""R7-A offline diagnostics. Truth is never available to control or fitting."""
from pathlib import Path
import sys,json,numpy as np,pandas as pd
from pyulog import ULog
from audit_r6 import project
R=Path(__file__).resolve().parent;sys.path.insert(0,str(R/'runtime'))
import evaluate_control as inherited

def one(name):
 p=R/'runtime/results/experiment/runs'/name;ep=json.loads((p/'diagnostic_epoch.json').read_text())['epoch'];tr=pd.read_csv(p/'trajectory.csv');t=tr.px4_time_s.to_numpy()-ep;e=tr[['x','y','z_up']].to_numpy()-tr[['x_ref','y_ref','z_ref']].to_numpy();en=np.linalg.norm(e,axis=1);pre=(t>=45)&(t<48);bound=max(1.1*float(np.sqrt(np.mean(en[pre]**2))),.02)
 obs=[json.loads(s) for s in (p/'deployable_observations.jsonl').read_text().splitlines()];ot=np.array([x['elapsed'] for x in obs]);speed=np.interp(t,ot,np.linalg.norm([x['velocity_enu'] for x in obs],axis=1))
 u=ULog(str(next((p/'rootfs/log').rglob('*.ulg'))),message_name_filter_list=['vehicle_local_position']);lp=u.get_dataset('vehicle_local_position').data;lt=lp['timestamp_sample']*1e-6-ep;gps=pd.read_csv(p/'gps_noise_audit.csv');gt=gps.source_ns.to_numpy()*1e-9-ep;idx=np.maximum(np.searchsorted(lt,gt,side='right')-1,0);true=np.c_[project(gps.lat_in.to_numpy(),gps.lon_in.to_numpy(),lp['ref_lat'][idx],lp['ref_lon'][idx]),gps.alt_in.to_numpy()-lp['ref_alt'][idx]]
 commands=[json.loads(s) for s in (p/'commands_causal.jsonl').read_text().splitlines()];ct=np.array([x['px4_message_timestamp_us'] for x in commands])*1e-6-ep;ci=np.clip(np.searchsorted(ct,gt,side='right')-1,0,len(ct)-1);sp=np.array([x['actual_position_ned'] for x in commands])[:,[1,0,2]]*[1,1,-1];true_error=true-sp[ci];truth_norm=np.linalg.norm(true_error,axis=1)
 windows={}
 for label,lo,hi in [('pre',45,48),('id',48,56),('task',58,68),('old_final',71,72),('extended_final',91,92)]:
  m=(t>=lo)&(t<hi);g=(gt>=lo)&(gt<hi);windows[label]=dict(window=[lo,hi],n=int(m.sum()),estimated_rmse_m=float(np.sqrt(np.mean(en[m]**2))),estimated_peak_m=float(en[m].max()),speed_peak_m_s=float(speed[m].max()),passed=bool(np.all(en[m]<=bound) and np.all(speed[m]<=.05)),truth_rmse_m=float(np.sqrt(np.mean(truth_norm[g]**2))),truth_peak_m=float(truth_norm[g].max()),true_error_mean_enu_m=true_error[g].mean(0).tolist())
 def first_recovery(b):
  for start in np.arange(68,90.0001,.1):
   m=(t>=start)&(t<start+2)
   if m.sum()>=80 and np.all(en[m]<=b) and np.all(speed[m]<=.05):return float(round(start-68,2))
  return None
 negative=[]
 for start in range(34,47):
  m=(t>=start)&(t<start+1);negative.append(dict(start=start,passed=bool(np.all(en[m]<=bound) and np.all(speed[m]<=.05)),peak_m=float(en[m].max()),speed_peak_m_s=float(speed[m].max())))
 inherited.WINDOWS={'whole':(48,68),'id':(48,56),'task':(58,68)};native=inherited.evaluate(name)
 np.savez_compressed(R/'results'/f'{name}_timeseries.npz',elapsed=t,estimated_error_enu=e,speed=speed,truth_elapsed=gt,truth_error_enu=true_error)
 result=dict(run=name,gps_delay_ms=float(u.initial_parameters['EKF2_GPS_DELAY']),position_bound_m=bound,speed_bound_m_s=.05,windows=windows,first_continuous_2s_recovery_s=first_recovery(bound),censor_s=24,negative_windows=negative,negative_pass_count=sum(x['passed'] for x in negative),native=native,flight=json.loads((p/'flight_result.json').read_text()),estimator_resets={k:int(np.ptp(lp[k][(lt>=45)&(lt<92)])) for k in ['xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter','heading_reset_counter']})
 (R/'results'/f'{name}_baseline.json').write_text(json.dumps(result,indent=2));return result

def main():
 order=json.loads((R/'runtime/MATRIX.json').read_text())['order'];rows=[one(n) for n in order if (R/'runtime/results'/f'{n}_outcome.json').exists()];pairs=[]
 for kind in ['hover','active']:
  p=[x for x in rows if kind in x['run']]
  if len(p)!=2:continue
  b=p[0]['position_bound_m'];common=[]
  for row in p:
   z=np.load(R/'results'/f"{row['run']}_timeseries.npz");t=z['elapsed'];en=np.linalg.norm(z['estimated_error_enu'],axis=1);v=z['speed'];common.append(dict(run=row['run'],bound_m=b,old_pass=bool(np.all(en[(t>=71)&(t<72)]<=b) and np.all(v[(t>=71)&(t<72)]<=.05)),extended_pass=bool(np.all(en[(t>=91)&(t<92)]<=b) and np.all(v[(t>=91)&(t<92)]<=.05))))
  pairs.append(dict(kind=kind,common_d110_bound=common,truth_old_rmse_change_pct=100*(p[1]['windows']['old_final']['truth_rmse_m']/p[0]['windows']['old_final']['truth_rmse_m']-1),truth_extended_rmse_change_pct=100*(p[1]['windows']['extended_final']['truth_rmse_m']/p[0]['windows']['extended_final']['truth_rmse_m']-1)))
 (R/'results/baseline_summary.json').write_text(json.dumps(dict(rows=rows,pairs=pairs),indent=2));print(json.dumps([dict(run=x['run'],bound=x['position_bound_m'],negative=x['negative_pass_count'],old=x['windows']['old_final'],extended=x['windows']['extended_final'],recovery=x['first_continuous_2s_recovery_s']) for x in rows],indent=2))
if __name__=='__main__':main()
