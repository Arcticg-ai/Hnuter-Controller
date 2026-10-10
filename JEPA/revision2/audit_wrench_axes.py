"""OFFLINE ONLY: compare full reconstructed wrench direction with requested force."""
import json,csv,threading
from pathlib import Path
from collections import deque
import numpy as np
from pyulog import ULog
from actuator_sensors import JointSensors
R=Path(__file__).resolve().parent
out=[]
for name in ['sitl_matrix_1_B_s0_20261010_190011','sitl_matrix_12_C_s2_20261010_190527','sitl_matrix_7_C_s1_20261010_191649']:
 p=R/'runs'/name;u=ULog(str(next((p/'rootfs/log').rglob('*.ulg'))));d=u.get_dataset('hnuter_control_status').data;a=u.get_dataset('vehicle_attitude').data;gt=u.get_dataset('vehicle_local_position_groundtruth').data
 tr=list(csv.DictReader((p/'trajectory.csv').open()));end=float(tr[-1]['px4_time_s']);start=float(tr[0]['px4_time_s'])
 sr=[x for x in csv.DictReader((p/'actuator_sensor.csv').open()) if start+35<float(x['t'])<end]
 fake=object.__new__(JointSensors);fake.geometry=json.loads((p/'actuator_geometry.json').read_text());fake.lock=threading.Lock();ff=[];fw=[];ts=[]
 for x in sr:
  t=float(x['sensor_t']);joints={n:(float(x[n+'_angle']),float(x[n+'_velocity'])) for n in fake.geometry['joints']};fake.samples=deque([(t,joints)])
  _,om,axes,k=fake.at(t);body=np.sum((k*om*abs(om))[:,None]*axes,axis=0)
  idx=np.searchsorted(a['timestamp']*1e-6,t,side='right')-1;w,xx,y,z=[a[f'q[{i}]'][idx] for i in range(4)]
  rot=np.array([[1-2*(y*y+z*z),2*(xx*y-w*z),2*(xx*z+w*y)],[2*(xx*y+w*z),1-2*(xx*xx+z*z),2*(y*z-w*xx)],[2*(xx*z-w*y),2*(y*z+w*xx),1-2*(xx*xx+y*y)]])
  ff.append(body);fw.append(rot@body);ts.append(t)
 ff=np.array(ff);fw=np.array(fw);ts=np.array(ts);expected=np.array([np.interp(ts,d['timestamp']*1e-6,d[f'force_body[{i}]']) for i in range(3)]).T
 # Integrated force predicts delta velocity; g only in NED z.
 dv=np.array([np.interp(ts[-1],gt['timestamp']*1e-6,gt[k])-np.interp(ts[0],gt['timestamp']*1e-6,gt[k]) for k in ['vx','vy','vz']]);pred=np.trapz(fw,ts,axis=0)/5.39999998+np.array([0,0,9.8])*(ts[-1]-ts[0])
 out.append(dict(run=name,window_s=[float(ts[0]-start),float(ts[-1]-start)],mean_actual_body_N=np.mean(ff,axis=0).tolist(),mean_requested_body_N=np.mean(expected,axis=0).tolist(),body_component_correlation=[float(np.corrcoef(ff[:,i],expected[:,i])[0,1]) for i in range(3)],body_rms_difference_N=np.sqrt(np.mean((ff-expected)**2,axis=0)).tolist(),true_delta_v_ned=dv.tolist(),predicted_delta_v_from_joint_force=pred.tolist()))
(R/'results/wrench_axes_audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
