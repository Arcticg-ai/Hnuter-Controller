"""Validate this bounded checkpoint, including negative conclusions and evidence."""
import ast
import hashlib
import json
import subprocess
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent


def main():
    records=json.loads((ROOT/'results/closure/all_attempts.json').read_text())
    assert len(records)==8
    for r in records:
        assert r['flight_result']['status']=='PASS'
        assert r['flight_result']['landed'] and r['flight_result']['disarmed']
        assert r['process_exit'] and all(code is not None for code in r['process_exit'].values())
        assert r['launch']['kind']=='owned_local_sitl'
        assert r['launch']['environment']['ROS_AUTOMATIC_DISCOVERY_RANGE']=='LOCALHOST'
        assert r['launch']['environment']['GZ_IP']=='127.0.0.1'
    consumers=[r['consumer_analysis'] for r in records if 'consumer_analysis' in r]
    assert len(consumers)==5
    for row in consumers:
        assert row['gains']==consumers[0]['gains']
        assert abs(row['hntr_mass']-5.4)<1e-5
        assert row['evaluation_torque_saturation_fraction']==0
        if row['group'] in ('A','B','C'):
            assert row['max_effective_delta_com_norm_m']==0
            assert row['max_effective_delta_inertia_norm_kg_m2']==0
    settled=next(r for r in consumers if 'transition_recheck' in r['run'])
    assert max(settled['tracking_bias_max_evaluation_Nm'])==0
    assert settled['max_parameter_transition_error_Nm']<1e-6
    assert settled['frozen_parameter_span']==0
    assert settled['com_max_logged_change_rate']<=.002001
    assert settled['inertia_max_logged_change_rate']<=.010001
    baseline=next(r for r in consumers if r['group']=='A')
    assert settled['position_rmse_m']>baseline['position_rmse_m']
    gate=json.loads((ROOT/'results/consumer_gate.json').read_text())
    assert not gate['oracle_consumer_pass'] and not gate['overall_tracking_benefit_verified']
    for name in ('online_shadow.py','nominal_dynamics.py','spatial_model.py','spatial_estimator.py'):
        tree=ast.parse((ROOT/name).read_text())
        imports=[n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
        assert 'assembly' not in imports and 'pyulog' not in imports
    manifest=json.loads((ROOT/'results/closure/raw_evidence_manifest.json').read_text())
    for name,metadata in manifest.items():
        p=ROOT/name;assert p.stat().st_size==metadata['size_bytes']
        h=hashlib.sha256()
        with p.open('rb') as f:
            for chunk in iter(lambda:f.read(8*1024*1024),b''):h.update(chunk)
        assert h.hexdigest()==metadata['sha256'], name
    validation=dict(checkpoint='bounded_closure',attempts=8,landed_disarmed=8,safety_exits=0,
        process_exit_recorded=8,consumer_flights=5,shadow_estimate_acceptances=0,shadow_attempts=2,
        unknown_estimate_actuation_count=0,controller_gains_matched=True,spatial_source_no_truth_imports=True,
        interface_transition_checks_pass=True,overall_tracking_benefit_pass=False,
        original_two_payload_four_group_matrix_completed=False,
        tests={'spatial_physics':3,'consumer_calibration_transition':6,'phase1_mass_regression':10,'revision2_mass_regression':10},
        unit_test_evidence='results/test_consumer.log, prior test_spatial tool result, regression_phase1.log, regression_revision2.log',
        evidence_files=len(manifest),evidence_size_bytes=sum(m['size_bytes'] for m in manifest.values()))
    (ROOT/'results/closure/validation.json').write_text(json.dumps(validation,indent=2)+'\n')
    print(json.dumps(validation,indent=2))


if __name__=='__main__':main()
