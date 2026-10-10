"""Read-only repository/run audit plus evidence manifests in JEPA."""
import difflib
import hashlib
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parent
PX4=Path('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl')


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1048576),b''):h.update(block)
    return h.hexdigest()


def main():
    initial=json.loads((ROOT/'runs/preflight_20261010_162752/preflight.json').read_text())
    preserved={name:sha(ROOT.parent/name)==value for name,value in initial['preexisting_modified_file_sha256'].items()}
    git=lambda repo,*args:subprocess.check_output(['git',*args],cwd=repo,text=True).strip()
    expected=['src/modules/mc_rate_control/HnuterControl.hpp','src/modules/mc_rate_control/HnuterControl.cpp','msg/HnuterControlStatus.msg']
    repos={name:dict(branch=git(repo,'branch','--show-current'),head=git(repo,'rev-parse','HEAD'),status=git(repo,'status','--short')) for name,repo in [('controller',ROOT.parent),('px4',PX4)]}
    check=dict(preexisting_modified_files_unchanged=preserved,repos=repos,
               original_position_entry_unchanged=sha(ROOT.parent/'hnuter_external_controller_px4_position.py')==initial['input_sha256'][str(ROOT.parent/'hnuter_external_controller_px4_position.py')],
               original_sdf_unchanged=sha(PX4/'Tools/simulation/gz/models/hnuter/model.sdf')==initial['input_sha256'][str(PX4/'Tools/simulation/gz/models/hnuter/model.sdf')],
               firmware_copies_match={name:sha(PX4/name)==sha(ROOT/'firmware_patched'/name) for name in expected},
               px4_only_expected_tracked_modifications=set(git(PX4,'diff','--name-only').splitlines())==set(expected))
    (ROOT/'preservation_check.json').write_text(json.dumps(check,indent=2)+'\n')
    assert all(preserved.values()) and check['original_position_entry_unchanged'] and check['original_sdf_unchanged']
    assert all(check['firmware_copies_match'].values()) and check['px4_only_expected_tracked_modifications']
    for name in repos:
        assert repos[name]['head']==initial['repos'][name]['head']['stdout'].strip()
        assert repos[name]['branch']==initial['repos'][name]['branch']['stdout'].strip()
    formal=set();entries=[]
    for phase in ['low','noisy']:
        pointer=ROOT/f'latest_matrix_{phase}.txt'
        if pointer.exists():
            matrix=Path(pointer.read_text().strip())
            for r in json.loads((matrix/'results.json').read_text()):formal.add(r['run']);entries.append(r)
    online=['run_sitl.py','sitl_position_probe.py','actuator_sensors.py','estimator.py']
    identities={name:set() for name in online};binaries=set();gains=[];cleanup=[]
    for entry in entries:
        run=Path(entry['run']);launch=json.loads((run/'launch.json').read_text());binaries.add(launch['binary_sha256'])
        for name in online:identities[name].add(launch['controller_sources_sha256'][name])
        params=json.loads((run/'ulog_parameters.json').read_text())
        gains.append({k:v for k,v in params.items() if k.startswith('HNTR_') and k!='HNTR_MASS'})
        cleanup.append((run/'process_exit.json').exists())
    evidence=dict(formal_runs=len(entries),binary_sha256=sorted(binaries),online_source_sha256={k:sorted(v) for k,v in identities.items()},
                  fixed_hnuter_parameters_except_mass=all(g==gains[0] for g in gains),all_have_cleanup_records=all(cleanup))
    assert len(binaries)==1 and all(len(s)==1 for s in identities.values())
    assert evidence['fixed_hnuter_parameters_except_mass'] and evidence['all_have_cleanup_records']
    out=ROOT/'results';out.mkdir(exist_ok=True)
    (out/'identity_audit.json').write_text(json.dumps(evidence,indent=2)+'\n')
    lines=['# 全部诊断与先行试验','', '正式矩阵之外的所有本次运行均保留。失败不是正式对比中的成功样本。', '', '| 运行 | 状态 | 原因 | 原始记录 |','|---|---|---|---|']
    for run in sorted((ROOT/'runs').glob('sitl_*')):
        if str(run) in formal:continue
        f=run/'flight_result.json';r=json.loads(f.read_text()) if f.exists() else {}
        status=r.get('status','INCOMPLETE/STARTUP FAILED');reason=r.get('failure') or r.get('mass_reason') or 'see logs'
        lines.append(f'| {run.name} | {status} | {reason} | [controller log]({run}/controller.log), [PX4 log]({run}/px4.log) |')
    (out/'DIAGNOSTICS.md').write_text('\n'.join(lines)+'\n')
    # This patch deliberately contains only the task's text source/docs, not raw
    # experiment logs, pre-existing user modifications, or binary build outputs.
    patches=[]
    for path in sorted(ROOT.iterdir()):
        if path.suffix not in ['.py','.md','.json'] or path.name in ['preservation_check.json']:continue
        patches.extend(difflib.unified_diff([],path.read_text().splitlines(True),fromfile='/dev/null',tofile='b/JEPA/'+path.name))
    (ROOT/'source_changes.patch').write_text(''.join(patches))
    paths=[p for p in ROOT.rglob('*') if p.is_file() and p.name!='SHA256SUMS.txt' and not {'__pycache__','mpl_cache'}.intersection(p.parts)]
    (ROOT/'SHA256SUMS.txt').write_text(''.join(f'{sha(p)}  {p.relative_to(ROOT)}\n' for p in sorted(paths)))
    print(json.dumps(dict(preservation='PASS',identity=evidence,hashed_files=len(paths)),indent=2))


if __name__=='__main__':main()
