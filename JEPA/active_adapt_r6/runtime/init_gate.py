"""Pure, offline-testable experiment admission. Never used as a control residual."""
import numpy as np
LIMITS={'position_error_mean_enu':.015,'velocity_mean_ned':.025,'integral_mean_ned':.03,'joint_angle_mean_deg':2.,'joint_rate_mean_deg_s':5.}
class PhysicalStart:
 def __init__(self):self.since=None;self.last=None;self.terminal=None
 def step(self,physical_s,ready,counts,aligned):
  if self.terminal:return self.terminal
  if self.last is not None and physical_s<self.last:self.terminal='clock_reversal';return self.terminal
  self.last=physical_s
  if not ready:self.since=None
  elif self.since is None:self.since=physical_s
  if physical_s<30:return 'wait'
  if physical_s>30.25:self.terminal='physical_start_deadline'
  elif not (ready and self.since is not None and physical_s-self.since>=3 and min(counts.values(),default=0)>=100 and aligned>=100):self.terminal='physical_start_not_ready'
  else:self.terminal='release'
  return self.terminal

def compare(summary,anchor):
 checks={}
 if anchor is None:return dict(passed=True,role='anchor_candidate',checks={},paired=False)
 for k,limit in LIMITS.items():
  a=np.asarray(summary[k],dtype=float);b=np.asarray(anchor[k],dtype=float)
  if a.shape!=b.shape or not np.isfinite(a).all() or not np.isfinite(b).all():raise ValueError('bad_pair_data')
  delta=float(abs(a-b).max() if k.startswith('joint') else np.linalg.norm(a-b));checks[k]=dict(difference=delta,limit=limit,passed=delta<=limit)
 return dict(passed=all(c['passed'] for c in checks.values()),role='paired_member',checks=checks,paired=True)

def summarize(telemetry,diagnostic,joints,epoch):
 # All filters operate on source physical timestamps, never array position or future values.
 def select(t,x):
  t=np.asarray(t,dtype=float);x=np.asarray(x,dtype=float);m=(t>=epoch+45)&(t<epoch+48);ts=t[m];ys=x[m]
  if len(ts)<2 or not np.isfinite(ys).all() or np.any(np.diff(ts)<=0) or ts[-1]-ts[0]<2.9 or np.diff(ts).max()>.0120001 or ts[0]>epoch+45.012 or ts[-1]<epoch+47.988:raise ValueError('incomplete_or_invalid_pre_window')
  return ts,ys
 pt,pe=select(telemetry['t'],telemetry['error']);dt,di=select(diagnostic['t'],np.c_[diagnostic['velocity'],diagnostic['integral']]);jt,ji=select(joints['t'],np.c_[joints['angle_deg'],joints['rate_deg_s']])
 return dict(position_error_mean_enu=pe.mean(0).tolist(),velocity_mean_ned=di[:,:3].mean(0).tolist(),integral_mean_ned=di[:,3:].mean(0).tolist(),joint_angle_mean_deg=ji[:,:4].mean(0).tolist(),joint_rate_mean_deg_s=ji[:,4:].mean(0).tolist(),source_ranges={k:[float(t[0]),float(t[-1]),int(len(t))] for k,t in [('pose',pt),('diag',dt),('joints',jt)]})

def finalize_time(result,completed_elapsed):
 result=dict(result,completed_elapsed=float(completed_elapsed))
 if not 48<=completed_elapsed<48.8:result.update(passed=False,reason='pair_report_deadline')
 return result
