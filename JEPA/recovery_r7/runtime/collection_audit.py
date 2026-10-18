"""R7 baseline-only data contract; performance failures are recorded separately."""
import json,numpy as np,pandas as pd
from pyulog import ULog
from excitation_r6 import reference
def audit(p):
 c={};d={}
 try:
  case=next(x for x in json.loads((p.parents[1]/'cases.json').read_text()) if x['id']==p.name)
  cmds=[json.loads(s) for s in (p/'commands_causal.jsonl').read_text().splitlines()]
  active=[x for x in cmds if x['phase']=='EVALUATION' and 49<x['elapsed']<67]
  errors=[float(np.max(abs(np.array(x['actual_jerk'][:2])-reference(x['elapsed'],case['excitation'])[0][:2]))) for x in active]
  c['excitation_published']=len(active)>100 and max(errors)<1e-6;d['excitation_error']=max(errors)
  launch=json.loads((p/'launch.json').read_text());c['no_adaptation_worker']='adaptation' not in launch['commands'] and not (p/'online_predictions.jsonl').exists()
  u=ULog(str(next((p/'rootfs/log').rglob('*.ulg'))),message_name_filter_list=['vehicle_status'])
  d['actual_gps_delay_ms']=float(u.initial_parameters['EKF2_GPS_DELAY']);c['requested_gps_delay']=d['actual_gps_delay_ms']==case['gps_delay_ms'];d['gps_delay_records']=[list(x) for x in u.changed_parameters if x[1]=='EKF2_GPS_DELAY'];c['gps_delay_unchanged']=all(x[2]==case['gps_delay_ms'] for x in u.changed_parameters if x[1]=='EKF2_GPS_DELAY')
  c['full_extended_hover']=max(x['elapsed'] for x in cmds if x['phase']=='FINAL_HOVER')>91.95
  z=pd.read_csv(p/'truth_diagnostic.csv',usecols=['t','dt','iteration','stage']);z=z[z.stage==1]
  c['truth_100hz_1ms_physics']=bool(len(z)>9000 and np.max(abs(z.dt-.001))<1e-12 and np.all(z.iteration%10==0) and np.max(np.diff(z.t))<.011)
  receipts=[json.loads(s) for s in (p/'adapt_receipts.jsonl').read_text().splitlines()]
  d['callback_intervals_p99_s']={k:float(np.quantile(np.diff([x['t_us'] for x in receipts if x['key']==k])*1e-6,.99)) for k in ['motor','servo','position','attitude','rate']}
  c['actuator_callback_coverage']=all(d['callback_intervals_p99_s'][k]<.03 for k in ['motor','servo'])
 except Exception as e:c['collection_exception_free']=False;d['error']=repr(e)
 return dict(checks={k:bool(v) for k,v in c.items()},details=d,passed=all(c.values()))
