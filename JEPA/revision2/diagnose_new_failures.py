"""Independent offline failure descriptions; no automatic retries or controller tuning."""
import csv,json
from pathlib import Path
import numpy as np
from pyulog import ULog
R=Path(__file__).resolve().parent
out=[]
for p in sorted((R/'runs').glob('sitl_matrix_*')):
 f=p/'flight_result.json'
 if not f.exists():continue
 result=json.loads(f.read_text())
 if result['status']=='PASS':continue
 r=dict(run=str(p),result=result)
 if (p/'trajectory.csv').exists():
  rows=list(csv.DictReader((p/'trajectory.csv').open()));r['last_reference']=rows[-1];end=float(rows[-1]['px4_time_s']);start=float(rows[0]['px4_time_s'])
  u=ULog(str(next((p/'rootfs/log').rglob('*.ulg'))));h=u.get_dataset('hnuter_control_status').data;t=h['timestamp']*1e-6;mask=(t>=end-1)&(t<=end)
  r['desired_force_last_second_mean_N']=[float(np.mean(h[f'force_body[{i}]'][mask])) for i in range(3)]
  r['effective_mass_at_exit']=float(np.interp(end,t,h['effective_mass']));ut=t[h['mass_updated'].astype(bool)&(t<=end)];r['seconds_since_last_mass_update']=float(end-max(ut)) if len(ut) else None
  g=u.get_dataset('vehicle_local_position_groundtruth').data;gt=g['timestamp']*1e-6;r['true_displacement_m']=[float(np.interp(end,gt,g[k])-np.interp(start,gt,g[k])) for k in ['x','y','z']]
  if (p/'actuator_sensor.csv').exists():
   ss=[x for x in csv.DictReader((p/'actuator_sensor.csv').open()) if end-1<=float(x['t'])<=end]
   r['actual_rotor_speed_mean_rad_s']=[float(np.mean([float(x[f'omega{i}']) for x in ss])) for i in range(5)] if ss else None
   r['measured_thrust_up_mean_N']=float(np.mean([float(x['thrust_up']) for x in ss])) if ss else None
  for obj in [x for x in u.data_list if x.name=='esc_status']:
   d=obj.data;m=(d['timestamp']*1e-6>=end-1)&(d['timestamp']*1e-6<=end)
   if np.any(m):r['esc_command_echo_mean_NOT_measured_rpm']=[float(np.mean(d[f'esc[{i}].esc_rpm'][m])) for i in range(5)]
 if (p/'recovery.json').exists():
  rec=json.loads((p/'recovery.json').read_text());r['recovery_wall_s']=rec[-1]['wall_s'] if rec else None
 out.append(r)
(R/'results/new_failures.json').write_text(json.dumps(out,indent=2))
print('Recorded',len(out),'formal failures')
