"""Causal 45-48s admission, reading diagnostics only; deadline by physical clock."""
from pathlib import Path
from collections import deque
import sys,time,json,os,hashlib
sys.path.append(str(Path(__file__).resolve().parent.parent/'revision20'))
import numpy as np,pandas as pd
from pyulog import ULog
from pre_gate import assess,NAMES
from report_serialization import atomic_write
from clock_receiver import ClockReceiver
from init_gate import summarize,compare,finalize_time
def main():
 out=Path(sys.argv[1]);files={};headers={};pending={};rows={n:deque(maxlen=6000) for n in NAMES};started=time.monotonic();clock=ClockReceiver();native=None;last_error=None;attempts=0
 try:
  anchor_path=Path(os.environ['R20_ANCHOR']) if os.environ.get('R20_ANCHOR') else None
  anchor=None
  if anchor_path:
   anchor=json.loads(anchor_path.read_text());assert anchor['seed']==int(os.environ['JEPA_SEED']) and anchor['eligible_anchor']
  while time.monotonic()-started<180:
   epoch_file=out/'diagnostic_epoch.json';epoch=json.loads(epoch_file.read_text())['epoch'] if epoch_file.exists() else None
   for n in NAMES:
    p=out/f'servo_{n}.csv'
    if n not in files:
     if not p.exists():continue
     files[n]=p.open();headers[n]=files[n].readline().strip().split(',');pending[n]=''
     if len(headers[n])!=16:files[n].close();del files[n];continue
    pending[n]+=files[n].read();lines=pending[n].split('\n');pending[n]=lines.pop()
    for line in lines:
     if not line:continue
     vals=line.split(',')
     if len(vals)!=len(headers[n]):raise RuntimeError('broken diagnostic row')
     rows[n].append({k:float(v) for k,v in zip(headers[n],vals)})
   clock_state=clock.sample()
   if epoch is None or not clock_state['fresh']:time.sleep(.01);continue
   elapsed=clock_state['ns']*1e-9-epoch
   if elapsed>=48.8:
    atomic_write(out/'initialization_admission.json',dict(passed=False,reason='pair_report_deadline',completed_elapsed=elapsed,last_error=last_error,attempts=attempts));break
   if not all(rows[n] and rows[n][-1]['sim_ns']>=round((epoch+48)*1e9)-1_000_000 for n in NAMES):time.sleep(.01);continue
   if native is None:
    before=time.monotonic_ns();native=assess({n:pd.DataFrame(rows[n]) for n in NAMES},epoch);native.update(epoch=epoch,analysis_wall_s=(time.monotonic_ns()-before)*1e-9,completed_monotonic_ns=time.monotonic_ns(),latest_native_ns={n:int(rows[n][-1]['sim_ns']) for n in NAMES});atomic_write(out/'preintervention_servo_admission.json',native)
    if not native['passed']:
     atomic_write(out/'initialization_admission.json',dict(passed=False,reason='native_servo',completed_elapsed=clock.ns*1e-9-epoch));break
   if elapsed<48:time.sleep(.005);continue
   try:
    before=time.monotonic_ns();attempts+=1
    text=(out/'init_telemetry.jsonl').read_text();tele=[json.loads(x) for x in text.split('\n')[:-1] if x]
    log=next((out/'rootfs/log').rglob('*.ulg'));u=ULog(str(log),message_name_filter_list=['hnuter_translation_diag']);d=u.get_dataset('hnuter_translation_diag').data
    def v(k):return np.stack([d[f'{k}[{i}]'] for i in range(3)],1)
    diag=dict(t=d['diag_control_timestamp']*1e-6,velocity=v('diag_velocity'),integral=v('diag_integral_initial'))
    dfs=[pd.DataFrame(rows[n]) for n in NAMES];dfs=[z[(z.sim_ns>=round((epoch+45)*1e9))&(z.sim_ns<round((epoch+48)*1e9))] for z in dfs];t=dfs[0].sim_ns.to_numpy()*1e-9
    for z in dfs[1:]:assert np.array_equal(dfs[0].sim_ns,z.sim_ns),'unaligned_native_joints'
    joints=dict(t=t,angle_deg=np.rad2deg(np.stack([z.post_q_rad.to_numpy() for z in dfs],1)),rate_deg_s=np.rad2deg(np.stack([z.post_qd_rad_s.to_numpy() for z in dfs],1)))
    summary=summarize(dict(t=[x['t'] for x in tele],error=[x['error'] for x in tele]),diag,joints,epoch)
    decision=compare(summary,anchor['summary'] if anchor else None);decision.update(summary=summary,epoch=epoch,reason='paired' if decision['passed'] else 'pair_mismatch',attempts=attempts,last_error=last_error,analysis_wall_s=(time.monotonic_ns()-before)*1e-9,completed_monotonic_ns=time.monotonic_ns(),seed=int(os.environ['JEPA_SEED']),anchor_sha256=hashlib.sha256(anchor_path.read_bytes()).hexdigest() if anchor_path else None)
    fresh_clock=clock.after(time.monotonic_ns());decision['final_clock_receipt']=fresh_clock;decision['completed_monotonic_ns']=time.monotonic_ns();decision=finalize_time(decision,fresh_clock['ns']*1e-9-epoch);atomic_write(out/'initialization_admission.json',decision);print(json.dumps(decision),flush=True);break
   except (Exception,) as e:last_error=repr(e);time.sleep(.015)
  else:raise RuntimeError('preintervention_gate_timeout')
 finally:clock.close()
if __name__=='__main__':main()
