"""Registered command-conditioned diagnosis; does not replace old absolute-rate gate."""
from pathlib import Path
import json,numpy as np,pandas as pd
from scipy.signal import periodogram,welch
R=Path(__file__).resolve().parent

def analyze_window(d,lo,hi,epoch,joint):
 q=d[(d.sim_ns>=round((epoch+lo)*1e9))&(d.sim_ns<round((epoch+hi)*1e9))].copy();q['rate']=np.rad2deg(q.post_qd_rad_s);q['error_deg']=np.rad2deg(q.post_q_rad-q.target_rad);q['age_ms']=(q.sim_ns-q.target_change_sim_ns)*1e-6
 all_changes=d[abs(d.target_delta_rad)>1e-9];mask=abs(q.rate.values)>20;indices=q.index.values;events=[];start=None
 for k,x in enumerate(np.r_[mask,False]):
  if x and start is None:start=k
  if not x and start is not None:
   ids=indices[start:k];peak=int(q.loc[ids].rate.abs().idxmax());row=d.loc[peak];prior=all_changes[all_changes.sim_ns<=row.sim_ns];c=prior.iloc[-1] if len(prior) else None;delta=float(c.target_delta_rad) if c is not None else 0.;lag=(row.sim_ns-c.sim_ns)*1e-9 if c is not None else 999.
   events.append(dict(start_elapsed_s=float(d.loc[ids[0]].sim_ns*1e-9-epoch),duration_s=len(ids)*.001,peak_deg_s=float(np.rad2deg(row.post_qd_rad_s)),peak_elapsed_s=float(row.sim_ns*1e-9-epoch),target_delta_deg=float(np.rad2deg(delta)),target_change_lag_s=float(lag),same_direction=bool(np.sign(row.post_qd_rad_s)==np.sign(delta)),pre_error_deg=float(np.rad2deg(row.error_rad)),post_error_deg=float(np.rad2deg(row.post_q_rad-row.target_rad)),error_magnitude_reduced=bool(abs(row.post_q_rad-row.target_rad)<abs(row.error_rad)),force_Nm=float(row.force_Nm),derivative_rad_s=float(row.derivative_rad_s),command_conditioned=bool(lag<=.001000001 and np.sign(row.post_qd_rad_s)==np.sign(delta))));start=None
 segments=[];steady_frames=[];longest_fixed=None
 groups=(abs(q.target_delta_rad)>1e-9).cumsum()
 for _,seg in q.groupby(groups):
  if len(seg)<100:continue
  if longest_fixed is None or len(seg)>len(longest_fixed):longest_fixed=seg.copy()
  tail=seg.iloc[40:];v=tail.rate.values;f,power=periodogram(v,fs=1000,detrend='constant');h=f>=20;hf=float(np.sqrt(np.trapz(power[h],f[h]))) if h.any() else 0
  segments.append(dict(start_elapsed_s=float(seg.sim_ns.iloc[0]*1e-9-epoch),duration_s=len(seg)*.001,tail_duration_s=len(tail)*.001,discard_initial_s=.040,target_deg=float(np.rad2deg(seg.target_rad.iloc[0])),tail_rate_rms_deg_s=float(np.sqrt(np.mean(v*v))),tail_high20_rms_deg_s=hf,tail_error_max_deg=float(abs(tail.error_deg).max())));steady_frames.append(tail)
 spectrum=None
 if longest_fixed is not None and len(longest_fixed)>80:
  sv=longest_fixed.rate.values[40:];sf,sp=welch(sv,fs=1000,nperseg=min(256,len(sv)));spectrum=dict(segment_duration_s=len(longest_fixed)*.001,discarded_initial_s=.040,frequency_resolution_hz=float(sf[1]-sf[0]),frequency_hz=sf.tolist(),rate_psd=sp.tolist(),peak_hz=float(sf[np.argmax(sp[1:])+1]),rate_rms_deg_s=float(np.sqrt(np.mean(sv*sv))))
 steady=pd.concat(steady_frames) if steady_frames else q.iloc[:0];late=q[q.age_ms>=40];limit=1.57 if joint.endswith('2') else np.pi;longest=cur=0;prevsign=0
 prev_ns=None
 for stamp,force in zip(late.sim_ns.to_numpy(),late.force_Nm.to_numpy()):
  if prev_ns is None or stamp-prev_ns!=1_000_000:cur=0;prevsign=0
  sign=np.sign(force);sat=abs(force)>=limit*.999;cur=cur+.001 if sat and sign!=prevsign else (.001 if sat else 0);longest=max(longest,cur);prevsign=sign;prev_ns=stamp
 rms=float(np.sqrt(np.mean(steady.rate**2))) if len(steady) else None;high=float(np.sqrt(np.mean([s['tail_high20_rms_deg_s']**2 for s in segments]))) if segments else None;frac=float(np.mean([e['command_conditioned'] for e in events])) if events else None
 checks=dict(enough_fixed_target_evidence=len(steady)>=200,steady_rate=bool(rms is not None and rms<=5 and all(s['tail_rate_rms_deg_s']<=5 for s in segments)),steady_high_frequency=bool(high is not None and high<=2 and all(s['tail_high20_rms_deg_s']<=2 for s in segments)),no_late_alternating_saturation=longest<=.01000001,peaks_command_locked=all(e['command_conditioned'] for e in events) if len(events)<20 else frac>=.95,short_events=all(e['duration_s']<=.00500001 for e in events),physical_rate_bound=bool(abs(q.post_qd_rad_s).max()<=10.01))
 return dict(window_s=[lo,hi],samples=len(q),rate_rms_deg_s=float(np.sqrt(np.mean(q.rate**2))),rate_peak_deg_s=float(abs(q.rate).max()),tracking_rms_deg=float(np.sqrt(np.mean(q.error_deg**2))),tracking_peak_deg=float(abs(q.error_deg).max()),tracking_min_deg=float(q.error_deg.min()),tracking_max_deg=float(q.error_deg.max()),longest_fixed_spectrum=spectrum,steady_tail_samples=len(steady),steady_segments=segments,steady_rate_rms_deg_s=rms,steady_high20_rms_deg_s=high,late_alternating_saturation_longest_s=longest,event_count=len(events),command_conditioned_fraction=frac,events=events,old_absolute20_pass=bool(abs(q.rate).max()<=20),diagnostic_checks=checks,diagnostic_pass=all(checks.values()))

if __name__=='__main__':
 p=R/'results/experiment/runs/00_s0';tr=pd.read_csv(p/'trajectory.csv');epoch=float(np.median(tr.px4_time_s-tr.t));out=dict(reference_epoch=epoch,windows={})
 for label,lo,hi in [('fixed_reference',45,48),('reference_switch',50,60)]:
  if tr.t.max()<hi-.1:out['windows'][label]=dict(available=False,reason='flight stopped before complete window');continue
  out['windows'][label]=dict(available=True,joints={})
  for n in ['rj2','lj2','rj1','lj1']:out['windows'][label]['joints'][n]=analyze_window(pd.read_csv(p/f'servo_{n}.csv'),lo,hi,epoch,n)
 (R/'results/servo_diagnosis.json').write_text(json.dumps(out,indent=2));print(json.dumps({k:({n:{a:r[a] for a in ['rate_peak_deg_s','event_count','command_conditioned_fraction','steady_tail_samples','steady_rate_rms_deg_s','steady_high20_rms_deg_s','tracking_peak_deg','diagnostic_pass']} for n,r in w['joints'].items()} if w.get('available') else w) for k,w in out['windows'].items()},indent=2))
