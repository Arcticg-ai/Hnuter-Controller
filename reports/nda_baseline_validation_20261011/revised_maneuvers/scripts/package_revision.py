"""Mechanically collect measured evidence and verify matched runtime settings."""
import difflib
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile

import pandas as pd

BASE = Path(__file__).resolve().parent
ROOT = Path('/home/hnuter/px4_ws_ros2_consolidated')
FW = Path('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl')
PARENT = ROOT/'reports/nda_baseline_validation_20261011'
DEST = PARENT/'revised_maneuvers'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n')


def git(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args], text=True)


def main():
    DEST.mkdir(exist_ok=False)
    shutil.copytree(BASE/'replay_check', DEST, dirs_exist_ok=True)
    shutil.copy2(BASE/'REPORT.md', DEST/'README.md')
    shutil.copy2(BASE/'METHODS.md', DEST/'METHODS.md')
    batches = ['landing_only', 'pilot_raw', 'pilot_smooth', 'matched',
               'geometry_pilot', 'geometry_strong_pilot', 'unit_gain_pilot', 'isotropic_pilot']
    counts = {'formal': len(pd.read_csv(DEST/'metrics.csv'))}
    for name in batches+['final_qualification']:
        source = BASE/(name+'_report')
        target = DEST/'qualification' if name=='final_qualification' else DEST/'exploratory'/name
        shutil.copytree(source, target)
        metrics = pd.read_csv(target/'metrics.csv')
        counts[name] = len(metrics)
        for scenario, group in metrics.groupby('scenario'):
            methods = set(group.method)
            for f in (target/'figures').glob(scenario+'*.png'):
                if ('_ablation' in f.name and not methods.intersection({'full','no_preview','no_horizon','no_nearfield'})) or ('_four_joints' in f.name and not {'full','no_nearfield'}<=methods):
                    f.unlink()
    prov = DEST/'provenance'
    prov.mkdir()
    for name in ['controller_before.tar.gz','source_pilots.tar.gz','source_pre_geometry.tar.gz',
                 'source_geometry_pilot.tar.gz','MulticopterLandDetector.cpp.before',
                 'tests_geometry.xml','test_landing_guard.cpp','incomplete_inventory.json',
                 'live_geometry_pilot.json']:
        shutil.copy2(BASE/name, prov/name)
    for name in ['README.md','SHA256SUMS.txt']:
        shutil.copy2(PARENT/name, prov/('parent_'+name+'.before'))
    shutil.copytree(BASE/'pilot_raw', prov/'incomplete_startup')
    early = BASE/'matched/attitude_80_180'
    if early.exists():
        shutil.copytree(early, prov/'incomplete_attitude_before_geometry')
    scripts = DEST/'scripts'
    scripts.mkdir()
    for name in ['analyze_revision.py','package_revision.py','audit_live_geometry.py',
                 'run_qualification.py','revision_qualification.py','run_isotropic.py',
                 'unit_gain_revision.py','isotropic_revision.py']:
        shutil.copy2(BASE/name, scripts/name)

    roots = ['controllers','config','tools/experiments','tests']
    files = sorted({p for folder in roots for p in (ROOT/folder).rglob('*')
                    if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'})
    with tarfile.open(prov/'controller_final.tar.gz', 'w:gz') as out:
        for p in files:
            out.add(p, arcname=str(p.relative_to(ROOT)))
    with tarfile.open(BASE/'controller_before.tar.gz') as before:
        old = {m.name: before.extractfile(m).read() for m in before.getmembers()
               if m.isfile() and '__pycache__' not in m.name and not m.name.endswith('.pyc')}
    current = {str(p.relative_to(ROOT)):p.read_bytes() for p in files}
    # The initial backup covered tests/experiments, not all tests.
    eligible = lambda s: not s.startswith('tests/') or s.startswith('tests/experiments/')
    changed = sorted(s for s in set(old)|set(current) if eligible(s) and old.get(s)!=current.get(s))
    patch = []
    for name in changed:
        patch.extend(difflib.unified_diff(old.get(name,b'').decode().splitlines(True),
                     current.get(name,b'').decode().splitlines(True),
                     fromfile='before/'+name,tofile='after/'+name))
    (prov/'controller_this_turn.patch').write_text(''.join(patch))
    write_json(prov/'changed_files_this_turn.json', changed)
    (prov/'firmware_tracked.patch').write_text(git(FW,'diff'))
    manifest = json.loads((DEST/'run_manifest.json').read_text())
    for name in manifest['firmware_sha256']:
        assert digest(FW/name)==manifest['firmware_sha256'][name], name
        if name.startswith('build/'):
            continue
        target = prov/'firmware'/name
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(FW/name,target)
    write_json(prov/'repositories.json', {str(r):dict(commit=git(r,'rev-parse','HEAD').strip(),
                branch=git(r,'branch','--show-current').strip(),status=git(r,'status','--short')) for r in [ROOT,FW]})

    cases = [json.loads(p.read_text()) for p in sorted((DEST/'data').rglob('result.json'))]
    configs = [json.loads(p.read_text()) for p in sorted((DEST/'data').rglob('runtime_config.json'))]
    audits = [json.loads(p.read_text()) for p in sorted((DEST/'data').rglob('ulog_metadata.json'))]
    assert len(cases)==len(configs)==len(audits)==18
    for c in configs:
        assert c['outer']==configs[0]['outer']
        assert c['shared_force_command_rate_n_s']==configs[0]['shared_force_command_rate_n_s']
        assert c['geometry']==configs[0]['geometry']
    non_direct = [c['allocator'] for c in configs if c.get('allocator')]
    keys = ['servo_state_limit_rad','servo_command_limit_rad','servo_command_rate_rad_s',
            'servo_rate_positive_rad_s','servo_rate_negative_rad_s','thrust_min_n','thrust_max_n',
            'thrust_command_rate_n_s','servo_delay_positive_s','servo_delay_negative_s','antiwindup_gain']
    for c in non_direct:
        assert all(c[k]==non_direct[0][k] for k in keys)
        assert all(v==0 for k in ['servo_delay_positive_s','servo_delay_negative_s'] for v in c[k])
    assert all(c['constraints']==cases[0]['constraints'] and not c['pure_delay_model_used'] for c in cases)
    assert all(c['source_sha256']==cases[0]['source_sha256'] for c in cases)
    assert all(digest(ROOT/p)==h for p,h in cases[0]['source_sha256'].items())
    assert all(a['direct_actuator_all'] and not any(a['forbidden_offboard_flags'].values())
               and not any(a['internal_control_flags_in_offboard'].values()) for a in audits)
    assert sum(a['elevated_landing_disarms'] for a in audits)==0
    one_step = []
    for p in (DEST/'data').rglob('no_horizon/diagnostic.csv'):
        d = pd.read_csv(p,low_memory=False)
        d = d[~d.switch_phase.isin(['idle','ingress','settle_a'])]
        horizons = sorted(d.drcda_prediction_h1_s.dropna().unique().tolist())
        assert horizons==[0.01], (p,horizons)
        one_step.append(dict(path=str(p.relative_to(DEST)),h1_s=horizons))
    assert sum(counts.values())==55, counts
    write_json(DEST/'verification.json', dict(evaluated_run_counts=counts,total=55,
        incomplete_excluded=4,formal_same_outer=True,formal_same_constraints=True,
        formal_same_controller_sources=True,current_sources_match_formal=True,
        firmware_and_sdf_match_formal=True,all_formal_direct_actuator=True,
        no_internal_px4_control_or_allocation=True,elevated_landing_disarms_formal=0,
        one_step_observed_horizons=one_step,source_change_list=changed,
        limitations=['one repetition per formal condition','not identical allocation update frequency',
                    'raw ULogs retained at source; reduced data archived','hardware not qualified']))
    print(json.dumps(counts,indent=2))
    print(DEST)


if __name__=='__main__':
    main()
