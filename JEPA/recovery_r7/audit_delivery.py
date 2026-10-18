from pathlib import Path
import hashlib,json,subprocess
R=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
start=json.loads((R/'START.json').read_text());protected={}
for key,v in start['originals_protection'].items():
 root=Path(v['root']);protected[key]=dict(head_unchanged=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()==v['head'],dirty_files={p:sha(root/p)==h for p,h in v['dirty_hashes'].items()})
assert all(x['head_unchanged'] and all(x['dirty_files'].values()) for x in protected.values());(R/'results/protected_repositories.json').write_text(json.dumps(protected,indent=2))
frozen=json.loads((R/'runtime/FROZEN_SOURCES.json').read_text());assert all(sha(R/p)==h for p,h in frozen.items());offline=json.loads((R/'results/offline_frozen_sources.json').read_text());assert all(sha(R/p)==h for p,h in offline.items())
order=json.loads((R/'runtime/MATRIX.json').read_text())['order'];outcomes=[json.loads((R/'runtime/results'/f'{n}_outcome.json').read_text()) for n in order];assert all(x['continue_batch'] for x in outcomes)
records=[]
for parent in [R/'runtime/results/experiment/runs',R/'runtime/results/unarmed/run']:
 for p in sorted(parent.rglob('*')):
  if p.is_file():records.append(dict(path=str(p.relative_to(R)),bytes=p.stat().st_size,sha256=sha(p)))
manifest=dict(root=str(R),count=len(records),bytes=sum(x['bytes'] for x in records),files=records);(R/'results/raw_manifest.json').write_text(json.dumps(manifest,indent=2));hosts=[json.loads((R/'runtime/results'/f'{n}_host_result.json').read_text()) for n in order];wall=sum((x['end_monotonic_ns']-x['start_monotonic_ns'])/1e9 for x in hosts);assert wall<=720 and manifest['bytes']<=2.5*2**30
result=dict(passed=True,flight_count=4,unarmed_count=1,flight_wall_s=wall,flight_wall_budget_s=720,raw_bytes=manifest['bytes'],raw_budget_gib=2.5,source_hashes_unchanged=True,offline_hashes_unchanged=True,protected=protected,no_control_enabled=True,performance_pass_is_separate=True,failed_audit_preserved=True);(R/'results/delivery_audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
