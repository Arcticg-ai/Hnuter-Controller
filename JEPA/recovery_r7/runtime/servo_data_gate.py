from pathlib import Path
import json,numpy as np,pandas as pd

def audit(run):
 result={};passed=True
 for n,kp,kd,limit in [('rj2',50,5,1.57),('lj2',50,5,1.57),('rj1',80,6,np.pi),('lj1',80,6,np.pi)]:
  d=pd.read_csv(run/f'servo_{n}.csv');force=np.clip(-kp*d.error_rad-kd*d.derivative_rad_s,-limit,limit);dt=np.diff(d.sim_ns.values);active=d.consumed_sequence>0
  checks=dict(samples=len(d)>1000,finite=bool(np.isfinite(d.values).all()),physics_1ms=bool((d.dt_ns==1_000_000).all()),clock_1ms=bool((dt==1_000_000).all()),command_sequence_monotonic=bool((np.diff(d.consumed_sequence)>=0).all()),received_before_consumption=bool((d.receive_monotonic_ns[active]<=d.consume_monotonic_ns[active]).all()),target_error=bool(abs(d.pre_q_rad-d.target_rad-d.error_rad).max()<1e-10),pid_formula=bool(abs(force-d.force_Nm).max()<1e-9),sim_velocity_bound=bool(abs(d.post_qd_rad_s).max()<10.01))
  result[n]=dict(passed=all(checks.values()),checks=checks,samples=len(d),sim_range_s=[float(d.sim_ns.iloc[0]*1e-9),float(d.sim_ns.iloc[-1]*1e-9)],force_formula_max_error=float(abs(force-d.force_Nm).max()),rate_peak_deg_s=float(abs(np.rad2deg(d.post_qd_rad_s)).max()),target_change_count=int((abs(d.target_delta_rad)>1e-9).sum()),receipt_to_consumption_max_wall_ms=float(((d.consume_monotonic_ns[active]-d.receive_monotonic_ns[active])*1e-6).max()));passed &= result[n]['passed']
 return dict(passed=bool(passed),joints=result)
if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser();p.add_argument('name');a=p.parse_args();R=Path(__file__).resolve().parent;v=audit(R/'results/experiment/runs'/a.name);(R/'results'/f'{a.name}_servo_data.json').write_text(json.dumps(v,indent=2));print(json.dumps(v,indent=2));assert v['passed']
