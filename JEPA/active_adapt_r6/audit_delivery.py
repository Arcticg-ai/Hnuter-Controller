from pathlib import Path
import hashlib,json,subprocess
import numpy as np,pandas as pd
R=Path(__file__).resolve().parent
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        while b:=f.read(1024*1024):h.update(b)
    return h.hexdigest()
def main():
    matrix=json.loads((R/'runtime/MATRIX.json').read_text());checks={};timing=[]
    for name in matrix['order']:
        p=R/'runtime/results/experiment/runs'/name;summary=json.loads((p/'online_summary.json').read_text());records=[json.loads(s) for s in (p/'online_predictions.jsonl').read_text().splitlines()];outcome=json.loads((R/'runtime/results'/f'{name}_outcome.json').read_text());checks[name+'_safety_data']=outcome['continue_batch'];checks[name+'_backbone']=summary['frozen_backbone_files_unchanged'];checks[name+'_no_control']=all(not x['control_enabled'] for x in records)
        for method,a in summary['adapters'].items():
            final=[x for x in records if x['method']==method and x['elapsed']>=56];checks[name+'_'+method+'_frozen_task']=all(np.array_equal(x['W_before'],a['W']) and np.array_equal(x['bias_before'],a['bias']) and np.array_equal(x['phi0_before'],a['phi0']) for x in final)
        timing.append(dict(run=name,all_seven_models_plus_common_preprocessing_p95_ms=float(np.quantile([x['all_models_interval_ms'] for x in records if x['method']=='physics' and x['elapsed']>=18],.95)),per_model_core_p95_ms=float(np.quantile([x['core_ms'] for x in records if x['elapsed']>=18],.95))))
        csv=pd.read_csv(R/'results'/f'{name}_predictions.csv',usecols=['t_us']);checks[name+'_integer_grid_export']=bool(np.all(csv.t_us%100000==0))
    snapshot=json.loads((R/'START_SNAPSHOT.json').read_text());protected={}
    for name,repo in snapshot['originals'].items():
        root=Path(repo['root']);protected[name]=dict(head_unchanged=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()==repo['head'],dirty_files_unchanged=all(sha(root/f)==h for f,h in repo['dirty_hashes'].items()),count=len(repo['dirty_hashes']));checks[name+'_protected']=protected[name]['head_unchanged'] and protected[name]['dirty_files_unchanged']
    frozen=json.loads((R/'runtime/FROZEN_SOURCES.json').read_text());changed=[f for f,h in frozen.items() if sha(R/f)!=h];checks['only_documented_postflight_analysis_changes']=set(changed)<={'AMENDMENTS_zh.md','evaluate.py'}
    raw=[]
    for root in [R/'runtime/results/unarmed/run',R/'runtime/results/experiment/runs']:
        for p in sorted(root.rglob('*')):
            if p.is_file():raw.append(dict(path=str(p.relative_to(R)),bytes=p.stat().st_size,sha256=sha(p)))
    manifest=dict(root=str(R),files=raw,file_count=len(raw),total_bytes=sum(x['bytes'] for x in raw),raw_uploaded=False);(R/'results/raw_manifest.json').write_text(json.dumps(manifest,indent=2));(R/'results/pipeline_latency.json').write_text(json.dumps(timing,indent=2));result=dict(passed=all(checks.values()),checks=checks,protected=protected,postflight_changed_frozen_files=changed,raw_files=len(raw),raw_bytes=manifest['total_bytes']);(R/'results/delivery_audit.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='checks'},indent=2));assert result['passed']
if __name__=='__main__':main()
