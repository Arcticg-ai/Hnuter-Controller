"""Verify preservation, fixed experimental identity, timing and complete denominators."""
import hashlib,json,subprocess,csv
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent;P=R.parent;PX4=Path('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 out=R/'results';out.mkdir(exist_ok=True);initial=json.loads((P/'runs/preflight_20261010_162752/preflight.json').read_text())
 preserved={n:sha(P.parent/n)==v for n,v in initial['preexisting_modified_file_sha256'].items()}
 preservation=dict(user_dirty_files=preserved,position_entry=sha(P.parent/'hnuter_external_controller_px4_position.py')==initial['input_sha256'][str(P.parent/'hnuter_external_controller_px4_position.py')],original_model=sha(PX4/'Tools/simulation/gz/models/hnuter/model.sdf')==initial['input_sha256'][str(PX4/'Tools/simulation/gz/models/hnuter/model.sdf')])
 assert all(preserved.values()) and preservation['position_entry'] and preservation['original_model']
 frozen=json.loads((R/'frozen_release.json').read_text());current={p:sha(p)==s for p,s in frozen['files'].items()};assert all(current.values())
 launches=[];all_failures=[];formal=[];gains=[];sensor_gaps=[];ready=[]
 for p in sorted((R/'runs').glob('sitl_*')):
  lp=p/'launch.json'
  if not lp.exists():continue
  l=json.loads(lp.read_text());f=json.loads((p/'flight_result.json').read_text()) if (p/'flight_result.json').exists() else {}
  row=dict(run=str(p),status=f.get('status','INCOMPLETE'),failure=f.get('failure'),cleanup=(p/'process_exit.json').exists(),experiment=l['experiment'],binary_sha256=l['binary_sha256'])
  launches.append(row)
  if row['status'] not in ['PASS','READY_UNARMED']:all_failures.append(row)
  if p.name.startswith('sitl_matrix_'):
   formal.append((p,l,f))
   if (p/'ulog_parameters.json').exists():
    params=json.loads((p/'ulog_parameters.json').read_text());gains.append({k:v for k,v in params.items() if k.startswith('HNTR_') and k!='HNTR_MASS'})
   if (p/'actuator_sensor.csv').exists():
    rr=list(csv.DictReader((p/'actuator_sensor.csv').open()));ages=[float(x['t'])-float(x['sensor_t']) for x in rr]
    sensor_gaps.append(dict(run=str(p),n=len(rr),min_age=min(ages),max_age=max(ages),future_count=sum(x<-.000001 for x in ages),stale_count=sum(x>.025001 for x in ages)))
   if (p/'readiness.json').exists():
    rr=json.loads((p/'readiness.json').read_text());yes=[x for x in rr if x['ready']]
    ready.append(dict(run=str(p),ready_at=f.get('ready_at_wall_s'),first_ready= yes[0]['wall_s'] if yes else None,last_ready=yes[-1]['wall_s'] if yes else None,ever_armed=f.get('samples',0)>0))
  geo=p/'actuator_geometry.json'
  if geo.exists():assert not any('"'+k+'"' in geo.read_text() for k in ['mass','inertia','truth','force'])
 ids={name:sorted(set(l['controller_sources_sha256'][name] for p,l,f in formal)) for name in ['run_sitl.py','sitl_position_probe.py','actuator_sensors.py','estimator.py']};binaries=sorted(set(l['binary_sha256'] for p,l,f in formal))
 identity=dict(runs=len(formal),binaries=binaries,online_sources=ids,identical_gains=all(x==gains[0] for x in gains) if gains else None)
 if formal:assert len(binaries)==1 and all(len(x)==1 for x in ids.values()) and identity['identical_gains']
 report=dict(preservation=preservation,frozen_files_unchanged=current,identity=identity,total_launches=len(launches),launches=launches,all_failures_including_injected=all_failures,sensor_timestamp_checks=sensor_gaps,prearm=ready)
 (out/'release_audit.json').write_text(json.dumps(report,indent=2));assert all(x['future_count']==0 and x['stale_count']==0 for x in sensor_gaps)
 print(json.dumps(dict(total_launches=len(launches),formal=len(formal),all_cleanup=all(x['cleanup'] for x in launches),identity=identity,preservation='PASS'),indent=2))
if __name__=='__main__':main()
