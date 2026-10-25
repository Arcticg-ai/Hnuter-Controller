"""Verify, archive and assemble the completed matched maneuver report."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
from pyulog import ULog

BASE = Path(__file__).resolve().parent
OUT = BASE/'maneuver_report'
ROOT = Path('/home/hnuter/px4_ws_ros2_consolidated')
OLD = ROOT/'reports/nda_baseline_validation_20261011'
FW = Path('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl')
records = json.loads((OUT/'results.json').read_text())
assert len(records) == 40
assert len({(r['scenario'], r['method'], r['repeat']) for r in records}) == 40
assert {(r['scenario'],r['method'],r['repeat']) for r in records} == {
    (s,m,n) for s in ['lissajous_7','lissajous_5','attitude_60_60','attitude_80_120']
    for m in ['direct','paper_nda','full','no_preview','no_horizon'] for n in [1,2]}
assert len({r['tuning_sha256'] for r in records}) == 1
assert len({json.dumps(r['source_sha256'],sort_keys=True) for r in records}) == 1
for name,digest in records[0]['source_sha256'].items():
    assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == digest, name
outer_hashes, model_hashes, firmware_versions = set(), set(), set()
land_parameter_sets = set()
physical_keys = ['servo_gain_positive','servo_gain_negative','servo_tau_positive_s','servo_tau_negative_s',
                 'servo_delay_positive_s','servo_delay_negative_s','servo_state_limit_rad','servo_command_limit_rad',
                 'servo_rate_positive_rad_s','servo_rate_negative_rad_s','servo_command_rate_rad_s',
                 'thrust_min_n','thrust_max_n','thrust_command_rate_n_s','motor_tau_up_s','motor_tau_down_s',
                 'motor_force_rate_floor_n_s','motor_force_rate_cap_n_s']
for r in records:
    parent = (OUT/r['archived_csv']).parent
    assert hashlib.sha256((parent/'diagnostic.csv').read_bytes()).hexdigest() == r['csv_sha256']
    assert hashlib.sha256((parent/'effective_tuning.json').read_bytes()).hexdigest() == r['tuning_sha256']
    runtime = json.loads((parent/'runtime_config.json').read_text())
    outer_hashes.add(json.dumps(runtime['outer'],sort_keys=True))
    assert runtime['shared_force_command_rate_n_s'] == [2500.]*4+[100.]
    if runtime['allocator']:
        cfg = runtime['allocator']
        assert cfg['thrust_command_rate_n_s'] == [2500.]*4+[100.]
        assert cfg['servo_delay_positive_s'] == cfg['servo_delay_negative_s'] == [0.]*4
        assert cfg['antiwindup_gain'] == 0.
        model_hashes.add(json.dumps({k:cfg[k] for k in physical_keys},sort_keys=True))
    assert not r['pure_delay_model_used']
    ulog = ULog(r['ulog_source'], message_name_filter_list=[])
    firmware_versions.add(ulog.msg_info_dict['ver_sw'])
    land_parameter_sets.add(json.dumps({k:ulog.initial_parameters[k] for k in
        ['CA_AIRFRAME','HNTR_MOT_HOV','HNTR_LND_GC_R','HNTR_LND_MIN_R','COM_DISARM_LAND']},sort_keys=True))
    metadata = dict(source_path=r['ulog_source'], sha256=hashlib.sha256(Path(r['ulog_source']).read_bytes()).hexdigest(),
                    software_version=ulog.msg_info_dict['ver_sw'], initial_parameters=ulog.initial_parameters)
    (parent/'ulog_metadata.json').write_text(json.dumps(metadata,indent=2,sort_keys=True)+'\n')
assert len(outer_hashes) == len(model_hashes) == len(firmware_versions) == 1
assert len(land_parameter_sets) == 1
head = subprocess.check_output(['git','-C',str(FW),'rev-parse','HEAD'],text=True).strip()
assert firmware_versions == {head}
assert not subprocess.check_output(['git','-C',str(FW),'status','--porcelain'],text=True).strip()
verification = dict(formal_runs=len(records), identical_outer_runtime=True, identical_allocator_physical_models=True,
                    identical_command_rate_caps=True, identical_controller_source=True, final_source_matches_runs=True,
                    csv_hashes_verified=True, actual_ulog_firmware_versions_verified=True, firmware_head=head,
                    identical_landing_detection_parameters=True,
                    firmware_clean=True, pure_delay_model_used=False,
                    limitations=['Direct and differential allocators have different update intervals',
                                 'Identified actuator model is approximate; motor thrust is not measured',
                                 'Only two repeats per condition'])
(OUT/'verification.json').write_text(json.dumps(verification,indent=2)+'\n')
for name in ['analyze_maneuver_extension.py','audit_maneuver_ulog.py','package_maneuver_extension.py','MANEUVER_REPORT_TEMPLATE.md',
             'MANEUVER_CONCLUSIONS.md','maneuver_source_frozen.tar.gz']:
    shutil.copy2(BASE/name,OUT/name)
shutil.copy2(BASE/'maneuver_test_results.xml',OUT/'test_results.xml')
shutil.copy2(BASE/'maneuver_test_results_rechecked.xml',OUT/'test_results_rechecked.xml')
shutil.copy2(BASE/'maneuver_matched/manifest.json',OUT/'manifest.json')
shutil.copy2(OLD/'source_final.tar.gz',OUT/'source_before_extension.tar.gz')
for source, name in [('README.md','previous_report_README.md'),
                     ('SHA256SUMS.txt','previous_report_SHA256SUMS.txt')]:
    if not (OUT/name).exists():
        shutil.copy2(OLD/source,OUT/name)
shutil.copy2(FW/'Tools/simulation/gz/models/hnuter/model.sdf',OUT/'model.sdf')
for cohort in ['maneuver_pilot','maneuver_pilot_ready']:
    shutil.copytree(BASE/cohort,OUT/'pilots'/cohort,dirs_exist_ok=True)
text = (BASE/'MANEUVER_REPORT_TEMPLATE.md').read_text()
text = text.replace('<!--RESULTS-->',(OUT/'RESULTS.md').read_text())
text = text.replace('<!--CONCLUSIONS-->',(BASE/'MANEUVER_CONCLUSIONS.md').read_text())
(OUT/'README.md').write_text(text)
hashes=[]
for path in sorted(OUT.rglob('*')):
    if path.is_file() and path != OUT/'SHA256SUMS.txt':
        hashes.append(f'{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(OUT)}')
(OUT/'SHA256SUMS.txt').write_text('\n'.join(hashes)+'\n')
print(json.dumps(verification,indent=2))
print(f'Packaged {len(hashes)} files')
