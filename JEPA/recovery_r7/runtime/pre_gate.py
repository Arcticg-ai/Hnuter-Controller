from pathlib import Path
import numpy as np
from analyze_servo import analyze_window
NAMES=['rj2','lj2','rj1','lj1']
def assess(frames,epoch):
 result={}
 for n in NAMES:
  d=frames[n];q=d[(d.sim_ns>=round((epoch+45)*1e9))&(d.sim_ns<round((epoch+48)*1e9))]
  if len(q)<2 or not np.isfinite(q.values).all():
   result[n]=dict(passed=False,checks={'finite_complete_window':False},diagnosis={'samples':len(q)},tracking_budget_deg=None);continue
  kp,kd,lim=(50,5,1.57) if n.endswith('2') else (80,6,np.pi)
  r=analyze_window(d,45,48,epoch,n);budget=2*(.37 if n.endswith('2') else .36)+np.rad2deg(.1/kp)
  checks=dict(r['diagnostic_checks']);checks.update(complete=len(q)==3000,window_start=bool(len(q) and q.sim_ns.iloc[0]==round((epoch+45)*1e9)),window_end=bool(len(q) and q.sim_ns.iloc[-1]==round((epoch+48)*1e9)-1_000_000),dt_1ms=bool((q.dt_ns==1_000_000).all()),target_not_future=bool((q.target_change_sim_ns<=q.sim_ns).all()),receipt_before_consume=bool((q.receive_monotonic_ns<=q.consume_monotonic_ns).all()),kinematic=bool(abs((q.post_q_rad-q.pre_q_rad)/.001-q.post_qd_rad_s).max()<1e-7),clock_1ms=bool((np.diff(q.sim_ns)==1_000_000).all()),finite=bool(np.isfinite(q.values).all()),target_error=bool(abs(q.pre_q_rad-q.target_rad-q.error_rad).max()<1e-10),formula=bool(abs(np.clip(-kp*q.error_rad-kd*q.derivative_rad_s,-lim,lim)-q.force_Nm).max()<1e-9),tracking_budget=bool(r['tracking_peak_deg']<=budget),physical_rate_bound=bool(abs(q.post_qd_rad_s).max()<=10),angle_bound=bool(abs(q.post_q_rad).max()<=(np.deg2rad(185) if n.endswith('2') else np.pi)))
  result[n]=dict(passed=all(checks.values()),checks=checks,tracking_budget_deg=float(budget),diagnosis=r)
 return dict(passed=all(x['passed'] for x in result.values()),window=[45,48],joints=result)
