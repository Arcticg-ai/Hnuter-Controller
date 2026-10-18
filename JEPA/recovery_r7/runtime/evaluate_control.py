"""Frozen native-sample diagnostic evaluation. Truth never enters online inference."""
from pathlib import Path
import json,sys
import numpy as np,pandas as pd
from pyulog import ULog
from timestamp_contract import match_payloads
R=Path(__file__).resolve().parent
WINDOWS={'whole':(48,68),'rise':(50,54),'fall':(54,58),'recovery':(58,68),'late':(60,68)}
T=np.array([[0,1,0],[1,0,0],[0,0,-1.]])
def vec(d,k):return np.stack([d[f'{k}[{i}]'] for i in range(3)],1).astype(float)
def rmse(x):return float(np.sqrt(np.mean(np.sum(x*x,axis=1))))
def peak(x):return float(np.max(np.linalg.norm(x,axis=1)))
def sat(t,x):
 dt=np.diff(t,append=t[-1]+np.median(np.diff(t)));cur=longest=0.
 for bit,dd in zip(x,dt):cur=cur+dd if bit else 0.;longest=max(cur,longest)
 return dict(fraction=float(np.mean(x)),longest_s=float(longest))
def evaluate(name):
 p=R/'results/experiment/runs'/name;arm=name.split('_')[1];sch=json.loads((p/'aligned_schedule.json').read_text());ep=sch['reference_epoch'];tr=pd.read_csv(p/'trajectory.csv');tt=tr.px4_time_s.to_numpy()-ep;pe=tr[['x','y','z_up']].to_numpy()-tr[['x_ref','y_ref','z_ref']].to_numpy();assert np.all(np.diff(tt)>=0)
 u=ULog(str(next((p/'rootfs/log').rglob('*.ulg'))));d=u.get_dataset('hnuter_translation_diag').data;dt=d['diag_control_timestamp'].astype(float)*1e-6-ep;dm=(dt>=48)&(dt<68)
 ff=vec(d,'diag_acc_ff');pv=vec(d,'diag_velocity_p')*vec(d,'diag_velocity_error');iv=vec(d,'diag_integral_initial');dv=-vec(d,'diag_velocity_d')*vec(d,'diag_measured_acc');unsat=vec(d,'diag_acc_unsat_initial');acc=vec(d,'diag_acc_des');after=vec(d,'diag_integral_after');ig=np.array([u.initial_parameters['HNTR_VEL_I_XY']]*2+[u.initial_parameters['HNTR_VEL_I_Z']]);expected_i=vec(d,'diag_integral_force')+ig*vec(d,'diag_velocity_error')*d['diag_dt'][:,None]
 cs=[json.loads(x) for x in (p/'commands_causal.jsonl').read_text().splitlines()];ct=np.array([x['px4_message_timestamp_us'] for x in cs])*1e-6-ep;cf=np.array([x['actual_acceleration_ned'] for x in cs]);ap=np.array([x['applied_ff_enu'] for x in cs]);original=np.array([x['original_acceleration_enu'] for x in cs]);final=np.array([x['final_acceleration_enu'] for x in cs]);cm=(ct>=48)&(ct<68);late=(ct>=60)&(ct<68)
 exact,errors,amb=match_payloads(cs,d['diag_setpoint_timestamp'],ff,dm);ages=(d['diag_control_timestamp'].astype(float)-d['diag_setpoint_timestamp'].astype(float))*1e-6
 qt=np.array([x.get('residual_query_timestamp_us',x['px4_message_timestamp_us']) for x in cs])*1e-6-ep
 ideal=np.zeros_like(ap);m=(qt>50)&(qt<58)
 if arm[1]=='1':ideal[m,1]=-.6*np.sin(np.pi*(qt[m]-50)/8)**2/sch['mass_kg']
 ef=pd.read_csv(p/'external_force_truth.csv');et=ef.sim_t.to_numpy()-ep;expected=np.where((et>50)&(et<58),.6*np.sin(np.pi*(et-50)/8)**2,0.) if arm[0]=='1' else np.zeros(len(et))
 # Rate audit in actual command receipt sequence, only forward source steps. Repeats must hold.
 steps=np.diff(qt);delta=np.linalg.norm(np.diff(ap,axis=0),axis=1);normal=np.array([not c.get('limiter',{}).get('emergency_zero',False) for c in cs]);forward=(steps>0)&normal[1:];same=(steps==0)&normal[1:]
 checks=dict(query_clock_instrumented=all('residual_query_timestamp_us' in c for c in cs),query_clock_monotonic=bool(np.all(steps>=0)),limiter_instrumented=all('limiter' in c for c in cs),force_waveform=float(np.max(abs(ef.force_y_N-expected)))<1e-9,ff_waveform=float(abs(ap-ideal)[cm].max())<1e-7,enu_ned=float(abs(cf-final@T)[cm].max())<1e-7,port_sum=float(abs(final-original-ap)[cm].max())<1e-7,consumer_payload_coverage=bool(exact[dm].all()),consumer_payload_error=bool(errors and max(errors)<1e-7),control_sum=float(abs(ff+pv+iv+dv-unsat)[dm].max())<1e-5,integral_update=float(abs(after-expected_i)[dm].max())<1e-5,consumer_age=bool(ages[dm].min()>=0 and ages[dm].max()<=.05),late_ff_zero=not bool(np.any(ap[late])),output_bound=bool(np.max(np.linalg.norm(ap,axis=1))<=.300000001),slew=bool(np.all(delta[forward]<=.2*steps[forward]+1e-10)),same_time_hold=bool(np.all(delta[same]<1e-12)),no_learning=not any(c['learned_control_enabled'] for c in cs),no_active_fallback=not any(c.get('limiter',{}).get('emergency_zero',False) for c,t in zip(cs,ct) if 48<=t<68))
 status=u.get_dataset('hnuter_control_status').data;st=status['timestamp']*1e-6-ep;ae=vec(status,'attitude_error');tor=vec(status,'torque_command');limits=np.array([u.initial_parameters['HNTR_TAU_'+a] for a in 'RPY']);tor_sat=np.any(abs(tor)>=.999*limits,axis=1)
 ca=u.get_dataset('control_allocator_status').data;cat=ca['timestamp']*1e-6-ep;ca_sat=np.any(np.stack([ca[f'actuator_saturation[{i}]'] for i in range(9)],1)!=0,axis=1)
 angular=u.get_dataset('vehicle_angular_velocity').data;at=angular['timestamp']*1e-6-ep;av=vec(angular,'xyz')
 obs=[json.loads(x) for x in (p/'deployable_observations.jsonl').read_text().splitlines()];ot=np.array([x['elapsed'] for x in obs]);mot=np.array([x['motors_normalized'] for x in obs]);ser=np.array([x['servos_normalized'] for x in obs]);phases={}
 truth=[]
 for ch in pd.read_csv(p/'truth_diagnostic.csv',usecols=['t','stage','base_link_px','base_link_py','base_link_pz'],chunksize=25000):truth.append(ch[(ch.stage==1)&(ch.t>=ep+45)&(ch.t<ep+68)])
 truth=pd.concat(truth);gt=truth.t.to_numpy()-ep;gp=truth[['base_link_px','base_link_py','base_link_pz']].to_numpy();gp-=gp[(gt>=45)&(gt<48)].mean(0)
 for phase,(lo,hi) in WINDOWS.items():
  tm=(tt>=lo)&(tt<hi);sm=(st>=lo)&(st<hi);am=(at>=lo)&(at<hi);dd=(dt>=lo)&(dt<hi);cc=(cat>=lo)&(cat<hi);oo=(ot>=lo)&(ot<hi);gg=(gt>=lo)&(gt<hi)
  assert tm.sum()>100 and np.ptp(tt[tm])>=hi-lo-.02
  phases[phase]=dict(position_rmse_m=rmse(pe[tm]),position_peak_m=peak(pe[tm]),attitude_rmse_deg=float(np.rad2deg(rmse(ae[sm]))),attitude_peak_deg=float(np.rad2deg(peak(ae[sm]))),angular_rate_rms_rad_s=rmse(av[am]),angular_rate_peak_rad_s=peak(av[am]),integral_mean_ned=iv[dd].mean(0).tolist(),consumed_ff_mean_ned=ff[dd].mean(0).tolist(),physical_base_displacement_rms_m=rmse(gp[gg]),physical_base_displacement_peak_m=peak(gp[gg]),saturation=dict(torque=sat(st[sm],tor_sat[sm]),allocator=sat(cat[cc],ca_sat[cc]),acceleration=sat(dt[dd],d['diag_acc_saturation_mask'][dd]!=0),motor_sampled=sat(ot[oo],np.any(mot[oo]>=.999,axis=1)),servo_sampled=sat(ot[oo],np.any(abs(ser[oo])>=.999,axis=1))))
 pre=(tt>=45)&(tt<48);bound=max(1.1*rmse(pe[pre]),.02);speed=np.interp(tt,dt,np.linalg.norm(vec(d,'diag_velocity'),axis=1));ok=(np.linalg.norm(pe,axis=1)<=bound)&(speed<=.05);start=None;first=None;lastbad=None
 for t,good in zip(tt,ok):
  if not 58<=t<68:continue
  if not good:start=None;lastbad=float(t)
  elif start is None:start=t
  elif t-start>=1 and first is None:first=float(start-58)
 recovery=dict(first_continuous_1s_s=first,last_out_of_band_s=None if lastbad is None else lastbad-58,stable_final_1s=lastbad is None or lastbad<=67,position_bound_m=bound,right_censored_s=10 if first is None else None)
 details=dict(consumer_age_s=dict(min=float(ages[dm].min()),p99=float(np.quantile(ages[dm],.99)),max=float(ages[dm].max())),payload_membership_error=max(errors),same_stamp_distinct_payload_groups=amb,source_clock_backsteps=int(np.sum(steps<0)),control_sum_max_error=float(abs(ff+pv+iv+dv-unsat)[dm].max()),integral_update_max_error=float(abs(after-expected_i)[dm].max()),waveform_max_error=float(abs(ap-ideal)[cm].max()),slew_max=float(np.max(delta[forward]/steps[forward])),fallback_count=sum(c.get('limiter',{}).get('emergency_zero',False) for c in cs),limited_count=sum(c.get('limiter',{}).get('limited',False) for c in cs))
 out=dict(run=name,contract_passed=all(checks.values()),checks=checks,details=details,phases=phases,recovery=recovery,physical_displacement_is_not_absolute_tracking_error=True,model_inference_not_run=True);(R/'results'/f'{name}_control.json').write_text(json.dumps(out,indent=2,allow_nan=False));return out

def batch():
 order=json.loads((R/'MATRIX.json').read_text())['order'];runs={n:json.loads((R/'results'/f'{n}_control.json').read_text()) for n in order if (R/'results'/f'{n}_control.json').exists()};pairs={}
 for seed in [2,3]:
  a,b=f's{seed}_10',f's{seed}_11'
  if a not in runs or b not in runs:continue
  checks={};ratios={}
  for phase in ['whole','recovery','late']:
   for metric,lim in [('position_rmse_m',1.05),('position_peak_m',1.10),('attitude_rmse_deg',1.05),('attitude_peak_deg',1.10)]:
    key=phase+'_'+metric;ratio=runs[b]['phases'][phase][metric]/max(runs[a]['phases'][phase][metric],1e-12);ratios[key]=ratio;checks[key]=ratio<=lim
   for kind in ['torque','allocator','acceleration','motor_sampled','servo_sampled']:
    for statistic in ['fraction','longest_s']:checks[f'{phase}_{kind}_{statistic}']=runs[b]['phases'][phase]['saturation'][kind][statistic]<=runs[a]['phases'][phase]['saturation'][kind][statistic]+1e-12
  checks['recovery']=runs[b]['recovery']['first_continuous_1s_s'] is not None
  checks['stable_final_1s']=runs[b]['recovery']['stable_final_1s']
  pairs[str(seed)]=dict(checks=checks,ratios=ratios,nondegradation_pass=all(checks.values()),position_benefit_5pct=ratios['whole_position_rmse_m']<=.95)
 complete=len(runs)==8 and (R/'BATCH_COMPLETE.json').exists();passed=complete and all(r['contract_passed'] for r in runs.values()) and len(pairs)==2 and all(p['nondegradation_pass'] for p in pairs.values());out=dict(complete=complete,runs=runs,pairs=pairs,known_ff_nondegradation_pass=passed,repeatable_benefit=passed and all(p['position_benefit_5pct'] for p in pairs.values()),learned_control_release=False,limitation='Development diagnostics only; approximate initial match is not full-state branching; independent held-out load tests remain sealed.');(R/'results/control_benefit.json').write_text(json.dumps(out,indent=2));print(json.dumps({k:v for k,v in out.items() if k!='runs'},indent=2))
if __name__=='__main__':
 if sys.argv[1]=='batch':batch()
 else:print(json.dumps(evaluate(sys.argv[1]),indent=2))
