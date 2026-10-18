"""Read actual native closure and admission; never turn a failed pair into PASS."""
from pathlib import Path
import json,sys,hashlib
import numpy as np,pandas as pd
from pyulog import ULog
from servo_data_gate import audit
R=Path(__file__).resolve().parent

def main(name):
 p=R/'results/experiment/runs'/name;h=json.loads((R/'results'/f'{name}_host_result.json').read_text());result=json.loads((p/'flight_result.json').read_text());arm=name.split('_')[1];c={};details={}
 def read(n):return json.loads((p/n).read_text()) if (p/n).exists() else None
 admission=read('initialization_admission.json');start=read('physical_start.json');consumed=read('initialization_consumed.json');pre=read('preintervention_servo_admission.json');runtime=read('runtime_stability.json');exitrecord=read('process_exit.json')
 receipt=read('seed_receipt.json');c['effective_seed']=bool(receipt and receipt['requested_seed']==int(name[1]) and receipt['effective_seed_before_server']==int(name[1]) and receipt['set_seed_called'] and receipt['draws_consumed_by_audit']==0)
 from gps_audit import audit as gps_audit
 details['gps_noise_audit']=gps_audit(p,int(name[1]));c['gps_private_rng']=details['gps_noise_audit']['passed']
 c['px4_normal_exit']=bool(exitrecord and exitrecord['processes']['px4']==0);c['no_forced_cleanup']=bool(exitrecord and not any(e['action'] in ['SIGKILL_owned_group','shutdown_wait'] for e in exitrecord['events']))
 c['container_closed']=bool(not h.get('final_state',{}).get('Running',True))
 c['async_log_complete']=bool(read('async_log_result.json') and read('async_log_result.json')['passed']);c['resources']=not h.get('guard_stop',False) and not h.get('soft_stop_requested',False) and not h.get('final_state',{}).get('OOMKilled',True)
 c['physical_start']=bool(start and start['action']=='release' and 30<=start['physical_s']<=30.25 and min(start['counts'].values())>=100 and start['aligned_count']>=100)
 c['native_pre']=bool(pre and pre['passed']);c['runtime_pre']=bool(runtime and runtime['passed']);c['initialization']=bool(admission and admission['passed'] and consumed and consumed['elapsed']<48.8)
 try:
  log=next((p/'rootfs/log').rglob('*.ulg'));u=ULog(str(log));vs=u.get_dataset('vehicle_status').data;ld=u.get_dataset('vehicle_land_detected').data;c['native_landed_disarmed']=bool(vs['arming_state'][-1]!=2 and ld['landed'][-1]);details['last_native_status']=dict(arming=int(vs['arming_state'][-1]),landed=bool(ld['landed'][-1]))
  try:
   d=u.get_dataset('hnuter_translation_diag').data;c['native_finite']=all(np.isfinite(d[k]).all() for k in d if d[k].dtype.kind=='f');c['constant_mass']=bool(np.max(abs(d['effective_mass']-5.4))<1e-5)
  except Exception as e:c['native_finite']=False;details['diagnostic_error']=repr(e)
 except Exception as e:c['native_landed_disarmed']=False;details['native_error']=repr(e)
 c['normal_application_exit']=bool(result['status']=='PASS' and result['landed'] and result['disarmed'] and exitrecord and exitrecord.get('flight_success_confirmed'))
 try:
  sa=audit(p);c['servo_data']=bool(sa['passed']);details['servo_audit']=sa
  for n in ['rj2','lj2','rj1','lj1']:
   z=pd.read_csv(p/f'servo_{n}.csv');c['physical_bounds_'+n]=bool(abs(z.post_qd_rad_s).max()<=10 and abs(z.post_q_rad).max()<=(np.deg2rad(185) if n.endswith('2') else np.pi))
  ef=pd.read_csv(p/'external_force_truth.csv');sch=read('aligned_schedule.json');t=ef.sim_t.to_numpy();expected=np.where((t>sch['start'])&(t<sch['end']),.6*np.sin(np.pi*(t-sch['start'])/(sch['end']-sch['start']))**2,0.) if arm[0]=='1' else np.zeros(len(t));c['registered_external_force']=bool(np.max(abs(ef.force_y_N-expected))<1e-9)
  commands=[json.loads(x) for x in (p/'commands_causal.jsonl').read_text().splitlines()];c['learning_off']=all(not x['learned_control_enabled'] for x in commands);c['bounded_ff']=all(np.linalg.norm(x['applied_ff_enu'])<=.300000001 for x in commands);c['zero_ff_when_disabled']=arm[1]=='1' or all(not np.any(x['applied_ff_enu']) for x in commands)
  ctr=[json.loads(x) for x in (p/'consumer.jsonl').read_text().splitlines()];c['zero_compensation']=bool(ctr and all(not x['mass_updated'] and not np.any(x['dr']) and not np.any(x['dj']) and not np.any(x['correction']) for x in ctr))
 except Exception as e:c['zero_and_data_audit']=False;details['audit_error']=repr(e)
 from collection_audit import audit as collection_audit
 details['collection']=collection_audit(p);c.update(details['collection']['checks'])
 passed=all(c.values());out=dict(run=name,seed=int(name[1]),continue_batch=passed,checks=c,failed_checks=[k for k,v in c.items() if not v],flight_result=result,initialization=admission,physical_start=start,details=details,closure_distinct_from_admission=True)
 (R/'results'/f'{name}_outcome.json').write_text(json.dumps(out,indent=2))
 if passed and arm=='00':
  (R/'anchors').mkdir(exist_ok=True);(R/'anchors'/f'{name}.json').write_text(json.dumps(dict(run=name,seed=int(name[1]),eligible_anchor=True,summary=admission['summary'],source_admission_sha256=hashlib.sha256((p/'initialization_admission.json').read_bytes()).hexdigest()),indent=2))
 if not passed:(R/'BATCH_STOPPED.json').write_text(json.dumps(dict(run=name,reason=result.get('failure') or 'post_run_gate',failed_checks=out['failed_checks'],no_repeats=True,remaining_slots_not_run=[x for x in json.loads((R/'MATRIX.json').read_text())['order'] if not (R/'results/experiment/runs'/x).exists()]),indent=2))
 elif name==json.loads((R/'MATRIX.json').read_text())['order'][-1]:(R/'BATCH_COMPLETE.json').write_text(json.dumps(dict(passed=True,scope='safety/data/initialization completed; performance gate decided separately; no full-state equivalence'),indent=2))
 print(json.dumps(dict(run=name,continue_batch=passed,checks=c,init=admission),indent=2));return passed
if __name__=='__main__':raise SystemExit(0 if main(sys.argv[1]) else 1)
