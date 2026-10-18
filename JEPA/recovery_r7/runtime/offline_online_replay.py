"""Execute actual online gate against causal-prefix saved logs, no simulation."""
from pathlib import Path
import json,tempfile,sys,types,runpy,os,time
sys.path.append(str(Path(__file__).resolve().parent.parent/'revision20'))
import numpy as np,pandas as pd
from pyulog import ULog
R=Path(__file__).resolve().parent;src=R.parent/'revision17/results/experiment/runs/00_s1';ep=json.loads((src/'diagnostic_epoch.json').read_text())['epoch'];reports=[]
with tempfile.TemporaryDirectory(prefix='jepa-collector-replay-') as td:
 root=Path(td);log=next((src/'rootfs/log').rglob('*.ulg'));raw=log.read_bytes();partial=root/'partial.ulg';low=0;high=len(raw)
 # Binary search the shortest file prefix containing diagnostic time >= epoch+48.05.
 for _ in range(24):
  mid=(low+high)//2;partial.write_bytes(raw[:mid])
  try:
   u=ULog(str(partial),message_name_filter_list=['hnuter_translation_diag']);end=float(u.get_dataset('hnuter_translation_diag').data['diag_control_timestamp'][-1])*1e-6
  except Exception:end=-1
  if end>=ep+48.05:high=mid
  else:low=mid+1
 partial.write_bytes(raw[:high]);u=ULog(str(partial),message_name_filter_list=['hnuter_translation_diag']);actual_end=u.get_dataset('hnuter_translation_diag').data['diag_control_timestamp'][-1]*1e-6;assert ep+48.05<=actual_end<ep+48.1
 for label,offset in [('anchor',None),('identical_pair',0.),('mismatch',.016),('late_clock',0.)]:
  dest=root/label;dest.mkdir();(dest/'rootfs/log').mkdir(parents=True);(dest/'rootfs/log/partial.ulg').symlink_to(partial);(dest/'diagnostic_epoch.json').write_text(json.dumps(dict(epoch=ep)))
  for n in ['rj2','lj2','rj1','lj1']:
   f=pd.read_csv(src/f'servo_{n}.csv');f=f[(f.sim_ns>=round((ep+42)*1e9))&(f.sim_ns<=round((ep+48.05)*1e9))];f.to_csv(dest/f'servo_{n}.csv',index=False)
  tr=pd.read_csv(src/'trajectory.csv');t=tr.px4_time_s.to_numpy();mask=(t>=ep+45)&(t<ep+48);e=tr[['x','y','z_up']].to_numpy()-tr[['x_ref','y_ref','z_ref']].to_numpy();(dest/'init_telemetry.jsonl').write_text(''.join(json.dumps(dict(t=float(tt),error=ee.tolist()))+'\n' for tt,ee in zip(t[mask],e[mask])))
  class Clock:
   ns=round((ep+48.1)*1e9)
   def after(self,end):return dict(ns=round((ep+(48.9 if label=='late_clock' else 48.1))*1e9),wall_ns=end+1,fresh=True)
   def sample(self):return dict(ns=self.ns,fresh=True)
   def close(self):pass
  sys.modules['clock_receiver']=types.SimpleNamespace(ClockReceiver=Clock);sys.argv=['live_pre_gate.py',str(dest)];os.environ['JEPA_SEED']='1';os.environ.pop('R20_ANCHOR',None)
  if offset is not None:
   anchor=dict(summary=dict(reports[0]['summary']),eligible_anchor=True,seed=1);anchor['summary']['position_error_mean_enu']=list(anchor['summary']['position_error_mean_enu']);anchor['summary']['position_error_mean_enu'][0]+=offset;ap=root/'anchor.json';ap.write_text(json.dumps(anchor));os.environ['R20_ANCHOR']=str(ap)
  start=time.monotonic();runpy.run_path(str(R/'live_pre_gate.py'),run_name='__main__');rr=json.loads((dest/'initialization_admission.json').read_text());rr['replay_wall_s']=time.monotonic()-start;rr['prefix_last_diag_elapsed']=float(actual_end-ep);rr['label']=label;reports.append(rr)
  assert rr['passed']==(label not in ('mismatch','late_clock')),rr
  assert all(v[1]<ep+48 for v in rr['summary']['source_ranges'].values())
(R/'results/offline_online_replay.json').write_text(json.dumps(reports,indent=2));print('PASS actual online prefix replay',[(r['label'],r['passed'],r['replay_wall_s']) for r in reports])
