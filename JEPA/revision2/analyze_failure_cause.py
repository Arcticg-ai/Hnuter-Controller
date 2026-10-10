"""Offline diagnostics only: correlate attitude fallback and applied-force reconstruction."""
import json,csv
from pathlib import Path
import numpy as np
from pyulog import ULog
R=Path(__file__).resolve().parent
out=[]
for case in json.loads((R/'original_failure_audit.json').read_text()):
 p=R.parent/'runs'/case['case'];u=ULog(str(next((p/'rootfs/log').rglob('*.ulg'))))
 start=float(case['last_reference']['px4_time_s'])-float(case['last_reference']['t']);end=float(case['last_reference']['px4_time_s'])
 h=u.get_dataset('hnuter_control_status').data;a=u.get_dataset('vehicle_attitude_setpoint').data
 t=h['timestamp']*1e-6;sel=(t>max(start,end-5))&(t<=end);t=t[sel]
 q=np.array([h[f'attitude_setpoint_q[{i}]'][sel] for i in range(4)]).T
 indices=np.searchsorted(a['timestamp']*1e-6,t,side='right')-1
 qa=np.array([a[f'q_d[{i}]'][indices] for i in range(4)]).T
 err=np.degrees(2*np.arccos(np.clip(abs(np.sum(q*qa,axis=1)),0,1)))
 r=dict(case=case['case'],fallback_difference_median_deg=float(np.median(err)),fallback_difference_max_deg=float(max(err)),desired_tilt_max_deg=float(np.degrees(np.arccos(np.clip(1-2*(q[:,1]**2+q[:,2]**2),-1,1))).max()),expected_force_up_mean_N=float(np.mean(-h['force_body[2]'][sel])))
 with (p/'actuator_sensor.csv').open() as f:rows=[x for x in csv.DictReader(f) if max(start,end-5)<float(x['t'])<=end]
 if rows:
  r['sensor_force_up_mean_N']=float(np.mean([float(x['thrust_up']) for x in rows]));r['sensor_rotor_omega_mean']=[float(np.mean([float(x[f'omega{i}']) for x in rows])) for i in range(5)]
 for name in ['vehicle_land_detected','actuator_motors']:
  d=u.get_dataset(name).data;mask=(d['timestamp']*1e-6>max(start,end-5))&(d['timestamp']*1e-6<=end)
  keys=['landed','ground_contact','maybe_landed'] if name=='vehicle_land_detected' else [f'control[{i}]' for i in range(5)]
  r[name]={k:float(np.mean(d[k][mask])) for k in keys}
 out.append(r)
(R/'original_cause_evidence.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
