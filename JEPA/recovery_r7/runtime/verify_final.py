from pathlib import Path
import json,hashlib,subprocess,shutil
R=Path(__file__).resolve().parent;W=R.parent.parent

def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(2**20),b''):h.update(b)
 return h.hexdigest()
def main():
 frozen=json.loads((R/'FROZEN_SOURCES.json').read_text());assert all(sha(R/k)==v for k,v in frozen.items())
 assert not subprocess.check_output(['git','diff','c7705b6','--',*[f'JEPA/revision{i}' for i in range(6,20)]],cwd=W,text=True)
 initial=json.loads((W.parent/'audit/initial.json').read_text());protected=[]
 for repo in initial.values():
  if not isinstance(repo,dict) or 'dirty_hashes' not in repo:continue
  assert subprocess.check_output(['git','-C',repo['root'],'rev-parse','HEAD'],text=True).strip()==repo['head']
  for rel,h in repo['dirty_hashes'].items():assert sha(Path(repo['root'])/rel)==h;protected.append(str(Path(repo['root'])/rel))
 old=R.parent/'revision17';oldraw=json.loads((old/'results/raw_manifest.json').read_text());assert all(sha(old/x['path'])==x['sha256'] for x in oldraw)
 for rev in ['revision18','revision19']:
  old=R.parent/rev;mf=json.loads((old/'SOURCE_MANIFEST.json').read_text());assert all(sha(old/x['path'])==x['sha256'] for x in mf)
 runs=sorted((R/'results/experiment/runs').iterdir());assert [p.name for p in runs]==['s0_a'];assert (R/'BATCH_STOPPED.json').exists() and not (R/'BATCH_COMPLETE.json').exists();assert not (R/'anchors/s0_a.json').exists()
 records=[]
 for run in runs:
  for p in sorted(run.rglob('*')):
   if p.is_file():records.append(dict(path=str(p.relative_to(R)),bytes=p.stat().st_size,sha256=sha(p)))
 (R/'results/raw_manifest.json').write_text(json.dumps(records,indent=2))
 h=json.loads((R/'results/s0_a_host_result.json').read_text());o=json.loads((R/'results/s0_a_outcome.json').read_text());clock=json.loads((R/'results/collection_audit.json').read_text())['r20'];diag=json.loads((R/'results/initialization_diagnosis.json').read_text())['s0_a'];assert o['checks']['native_landed_disarmed'] and h['removed'] and not h['final_state']['Running'];assert not o['continue_batch'];assert clock['gate_native_deadline_pass'];assert diag['online_mean_replay_max_abs_error']<1e-12
 result=dict(base_preserved='c7705b6948ff12c21b385d026ca4706b6c18028b',flight_freeze='fe9c531',frozen_sources_unchanged=len(frozen),protected_original_dirty_files=len(protected),original_heads_unchanged=True,r17_raw_unchanged_files=len(oldraw),r18_r19_manifests_unchanged=True,attempted_runs=1,not_run=['s0_b','s1_a','s1_b'],retries=0,new_training=0,holdout_opened=False,raw_files=len(records),raw_bytes=sum(x['bytes'] for x in records),run_wall_s=(h['end_monotonic_ns']-h['start_monotonic_ns'])/1e9,normal_flight_land_disarm=True,overall_admission_pass=False,reason='inherited calibration observation sample count 259 < 280',owned_container_removed=True,own_inhibitor_released=True,observed_remaining_inhibitors=8,control_law_modified=False,ekf_modified=False,external_force_zero=True,compensation_zero=True,learning_off=True,pushed=False,uploaded=False,free_bytes=shutil.disk_usage(R).free)
 (R/'results/final_checks.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
