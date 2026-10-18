"""R4 bounded serial data collection, all parameter compensation disabled."""
from pathlib import Path
import json,subprocess,time,shutil,argparse,hashlib,fcntl
lock=open('/tmp/hnuter-sitl-research.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
R=Path(__file__).resolve().parent;M=json.loads((R/'MATRIX.json').read_text());ORDER=M['order'];OWNER='jepa-active-adapt-r6'
p=argparse.ArgumentParser();p.add_argument('slot',choices=['unarmed']+ORDER);a=p.parse_args();unarmed=a.slot=='unarmed';slot=ORDER[0] if unarmed else a.slot;seed=next(c['seed'] for c in M['cases'] if c['id']==slot)
out=R/('results/unarmed/run' if unarmed else 'results/experiment/runs/'+slot)
assert not out.exists()
assert not (R/'BATCH_STOPPED.json').exists() and not (R/'BATCH_COMPLETE.json').exists()
if not unarmed:
 assert json.loads((R/'results/unarmed_check.json').read_text())['passed']
 for prior in ORDER[:ORDER.index(slot)]:assert json.loads((R/'results'/f'{prior}_outcome.json').read_text())['continue_batch']
for path,digest in json.loads((R/'FROZEN_SOURCES.json').read_text()).items():assert hashlib.sha256((R.parent/path).read_bytes()).hexdigest()==digest,path
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
assert not ({'px4','seeded_server','gz'} & set(subprocess.check_output(['ps','-eo','comm='],text=True).split())), 'other_simulation_process_active'
assert shutil.disk_usage(R).free>8.3*2**30
spent=sum((json.loads(f.read_text())['end_monotonic_ns']-json.loads(f.read_text())['start_monotonic_ns'])/1e9 for f in (R/'results').glob('s*_host_result.json'));assert unarmed or spent+180<=1440
name=OWNER+'-'+a.slot.replace('_','-');cap=80 if unarmed else 180
cmd=['systemd-inhibit','--what=sleep','--mode=block','--who=JEPA-R6','--why=Bounded baseline data collection; release on exit','docker','run','--init','--name',name,'--label','org.openai.jepa.task='+OWNER,'--network','none','--shm-size','512m','--cpus','4','--memory','4g','--pids-limit','512','--user','1000:1000','-e','R12_ARM=00','-v',str(R)+':/work','-v',str(R.parent)+':/r6:ro','-v','/home/hnuter/Documents/Codex/2026-10-13/task/jepa-px4-gps3-worktree:/opt/px4:ro','--entrypoint','/bin/bash','hnuter-adaptation:20261013','/work/container_entry.sh','launch.py','/work/results/experiment',slot,'baseline','--seed',str(seed),'--out','/work/'+str(out.relative_to(R))]
if unarmed:cmd+=['--startup-only']
r=dict(slot=a.slot,command=cmd,start_wall_ns=time.time_ns(),start_monotonic_ns=time.monotonic_ns());samples=[];start=time.monotonic();proc=None
try:
 with (R/'results'/f'{a.slot}_runner.log').open('x') as f:
  proc=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT)
  while proc.poll() is None:
   elapsed=time.monotonic()-start;free=shutil.disk_usage(R).free
   raw=sum(x.stat().st_size for folder in [R/'results/experiment/runs',R/'results/unarmed'] for x in folder.rglob('*') if x.is_file());current=sum(x.stat().st_size for x in out.rglob('*') if x.is_file()) if out.exists() else 0;samples.append(dict(elapsed_s=elapsed,free_bytes=free,raw_bytes=raw,run_bytes=current))
   if not unarmed and elapsed>150 and out.exists() and not (out/'request_stop.json').exists():
    (out/'request_stop.json').write_text(json.dumps(dict(reason='run_budget_land')));r['soft_stop_requested']=True
   if free<8.3*2**30 or raw>4*2**30 or current>.6*2**30 or elapsed>cap:r['guard_stop']=True;break
   time.sleep(1)
  if proc.poll() is not None:r['returncode']=proc.returncode
finally:
 q=subprocess.run(['docker','inspect',name],capture_output=True,text=True)
 if q.returncode==0:
  item=json.loads(q.stdout)[0];assert item['Config']['Labels']['org.openai.jepa.task']==OWNER
  if item['State']['Running']:subprocess.run(['docker','stop','--time','10',name],timeout=20)
  item=json.loads(subprocess.check_output(['docker','inspect',name],text=True))[0];r['final_state']=item['State'];assert not item['State']['Running'];r['retained_exited_container']=True
 if proc:proc.wait(timeout=10)
 r.update(end_wall_ns=time.time_ns(),end_monotonic_ns=time.monotonic_ns(),samples=samples);(R/'results'/f'{a.slot}_host_result.json').write_text(json.dumps(r,indent=2))
print(json.dumps({k:v for k,v in r.items() if k not in ['command','samples']},indent=2));raise SystemExit(0 if r.get('returncode')==0 else 1)
