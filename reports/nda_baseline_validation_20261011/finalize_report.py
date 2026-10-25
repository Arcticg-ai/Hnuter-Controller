"""Verify and package the completed no-delay validation, without rerunning it."""
import difflib
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
from pyulog import ULog

BASE = Path(__file__).resolve().parent
OUT = BASE/'report'
REPO = Path('/home/hnuter/px4_ws_ros2_consolidated')
FIRMWARE = Path('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl')
records = json.loads((OUT/'all_results.json').read_text())
expected = {'qualification_hover_common': 2, 'qualification_low_common': 2,
            'comparison_common': 30, 'nda_ablations': 4, 'qualification_hover_long': 1}
formal = [r for r in records if r['cohort'] in expected]
for cohort, count in expected.items():
    assert sum(r['cohort'] == cohort for r in formal) == count, cohort
assert len({r['tuning_sha256'] for r in formal}) == 1
assert len({json.dumps(r['controller_source_sha256'], sort_keys=True) for r in formal}) == 1
for r in records:
    if 'archived_csv' in r:
        assert hashlib.sha256((OUT/r['archived_csv']).read_bytes()).hexdigest() == r['csv_sha256']
for r in formal:
    assert r['pure_delay_model_used'] is False and r['common_prelude'] is True
    assert not r['gate']['spawned']
    assert hashlib.sha256(Path(r['tuning']).read_bytes()).hexdigest() == r['tuning_sha256']
    ulog = ULog(r['ulog_source'], message_name_filter_list=[])
    assert ulog.msg_info_dict['ver_sw'] == '53266349b9c65001b72e8f2d437b7340c8c4756f'
    metadata = dict(source_path=r['ulog_source'],
                    sha256=hashlib.sha256(Path(r['ulog_source']).read_bytes()).hexdigest(),
                    software_version=ulog.msg_info_dict['ver_sw'],
                    software_branch=ulog.msg_info_dict.get('ver_sw_branch'),
                    initial_parameters=ulog.initial_parameters)
    target = (OUT/r['archived_csv']).parent/'ulog_metadata.json'
    target.write_text(json.dumps(metadata, indent=2, sort_keys=True)+'\n')
for relative, digest in formal[0]['controller_source_sha256'].items():
    assert hashlib.sha256((REPO/relative).read_bytes()).hexdigest() == digest, relative
firmware_head = subprocess.check_output(['git', '-C', str(FIRMWARE), 'rev-parse', 'HEAD'], text=True).strip()
assert firmware_head == '53266349b9c65001b72e8f2d437b7340c8c4756f'
firmware_status = subprocess.check_output(['git', '-C', str(FIRMWARE), 'status', '--porcelain'], text=True)
assert not firmware_status.strip(), firmware_status
audit = dict(total_result_records=len(records), formal_records=len(formal),
             expected_counts=expected, tuning_sha256=formal[0]['tuning_sha256'],
             controller_sources_identical=True, controller_sources_match_final=True,
             archived_csv_hashes_verified=True, firmware_commit=firmware_head,
             firmware_ulog_versions_verified=True,
             firmware_worktree_clean=True, pure_delay_used=False)
(OUT/'verification.json').write_text(json.dumps(audit, indent=2)+'\n')
for p in (BASE/'runs').rglob('manifest.json'):
    target = OUT/'run_manifests'/p.relative_to(BASE/'runs')
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(p, target)
incomplete = []
for p in (BASE/'runs').rglob('effective_tuning.json'):
    if not (p.parent/'result.json').exists():
        relative = p.parent.relative_to(BASE/'runs')
        shutil.copytree(p.parent, OUT/'incomplete'/relative, dirs_exist_ok=True)
        incomplete.append(str(relative))
(OUT/'incomplete_runs.json').write_text(json.dumps(incomplete, indent=2)+'\n')
shutil.copy2(FIRMWARE/'Tools/simulation/gz/models/hnuter/model.sdf', OUT/'model.sdf')
for name in ['locked_plan.json', 'run_locked_suite.py', 'analyze_nda_validation.py',
             'run_long_hover.py', 'long_hover_command.json',
             'finalize_report.py', 'REPORT_TEMPLATE.md', 'CONCLUSIONS.md',
             'test_results.xml', 'source_before.tar.gz']:
    shutil.copy2(BASE/name, OUT/name)
with tarfile.open(OUT/'source_final.tar.gz', 'w:gz') as archive:
    for folder in ['controllers', 'config', 'tools', 'tests']:
        for path in sorted((REPO/folder).rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts:
                archive.add(path, arcname=path.relative_to(REPO))
changed = ['controllers/experiments/fixed_attitude_switching/audit_allocator.py',
           'controllers/experiments/fixed_attitude_switching/controller.py',
           'tools/experiments/run_fixed_attitude_switching.py',
           'tests/experiments/test_nda_sampled.py']
diff = []
with tarfile.open(BASE/'source_before.tar.gz') as archive:
    for name in changed:
        try:
            before = archive.extractfile(name).read().decode().splitlines(keepends=True)
        except KeyError:
            before = []
        after = (REPO/name).read_text().splitlines(keepends=True)
        diff.extend(difflib.unified_diff(before, after, fromfile='before/'+name, tofile='after/'+name))
(OUT/'this_turn.patch').write_text(''.join(diff))
text = (BASE/'REPORT_TEMPLATE.md').read_text().replace('<!--RESULTS-->', (OUT/'RESULTS.md').read_text())
text += '\n'+(BASE/'CONCLUSIONS.md').read_text()
(OUT/'README.md').write_text(text)
hashes = []
for p in sorted(OUT.rglob('*')):
    if p.is_file() and p.name != 'SHA256SUMS.txt':
        hashes.append(f'{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.relative_to(OUT)}')
(OUT/'SHA256SUMS.txt').write_text('\n'.join(hashes)+'\n')
print(json.dumps(audit, indent=2))
print(f'Packaged {len(hashes)} hashed files into {OUT}')
