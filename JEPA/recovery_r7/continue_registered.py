"""Resume only unrun slots after a documented read-only audit bug repair."""
from pathlib import Path
import json,subprocess,time
R=Path(__file__).resolve().parent;rt=R/'runtime';order=json.loads((rt/'MATRIX.json').read_text())['order'];completed=[]
for name in order:
 out=rt/'results'/f'{name}_outcome.json'
 if out.exists():assert json.loads(out.read_text())['continue_batch'];completed.append(name)
 else:break
assert completed==order[:2]
for name in order[2:]:
 assert not (rt/'results/experiment/runs'/name).exists()
 (R/'results/campaign_status.json').write_text(json.dumps(dict(stage='running_after_documented_audit_fix',run=name,completed=completed),indent=2))
 for cmd,kind,cap in [(['python3',str(rt/'run_bounded.py'),name],'run',210),(['python3',str(rt/'post_run.py'),name],'audit',90)]:
  with (rt/'results'/f'{name}_{kind}_supervisor.log').open('x') as f:q=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=cap)
  if q.returncode:
   (R/'results/campaign_status.json').write_text(json.dumps(dict(stage='stopped',run=name,failed_step=kind,completed=completed,returncode=q.returncode),indent=2));raise SystemExit(q.returncode)
 completed.append(name);print(name,'safety/data PASS',flush=True)
(R/'results/campaign_status.json').write_text(json.dumps(dict(stage='completed',completed=completed,audit_fix='startup duplicate parameter record; no repeated flight'),indent=2))
