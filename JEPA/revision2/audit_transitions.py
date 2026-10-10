"""Offline common-window force/transient metrics; not used by estimator."""
import csv,json
from pathlib import Path
import numpy as np
from pyulog import ULog
R=Path(__file__).resolve().parent

def main():
 paths=sorted((R/'runs').glob('sitl_matrix_*'));out=[]
 for p in paths:
  if not (p/'flight_result.json').exists() or not (p/'trajectory.csv').exists():continue
  tr=list(csv.DictReader((p/'trajectory.csv').open()));t0=float(tr[0]['px4_time_s'])
  sensors=list(csv.DictReader((p/'actuator_sensor.csv').open())) if (p/'actuator_sensor.csv').exists() else []
  m=json.loads((p/'metrics.json').read_text()) if (p/'metrics.json').exists() else {}
  r=dict(run=p.name,group=m.get('flight',{}).get('group'),compensation=m.get('compensation'))
  for phase,lo,hi in [('pre',32,35),('update',35,39),('post',39,42)]:
   ss=[x for x in sensors if lo<=float(x['t'])-t0<=hi];v=np.array([float(x['thrust_up']) for x in ss]);tt=np.array([float(x['t']) for x in ss]);dd=np.abs(np.diff(v));valid=np.diff(tt)<.05
   rr=[x for x in tr if lo<=float(x['t'])<=hi]
   r[phase]=dict(samples=len(ss),thrust_mean_N=float(np.mean(v)) if len(v) else None,thrust_std_N=float(np.std(v)) if len(v) else None,actual_reconstructed_thrust_step_p95_N=float(np.quantile(dd[valid],.95)) if np.any(valid) else None,position_error_peak_m=max((float(x['error']) for x in rr),default=None))
  out.append(r)
 (R/'results/transition_audit.json').write_text(json.dumps(out,indent=2));print('Audited',len(out),'actual runs')
if __name__=='__main__':main()
