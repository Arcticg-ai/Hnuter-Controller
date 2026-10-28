import numpy as np
import pytest
import json
from types import SimpleNamespace

from tools.experiments.hardware_transfer import phase_clock, task_plan, OLD_GAINS, INTEGRAL


@pytest.mark.parametrize('t', [.2, .75, 1., 6.9, 7., 7.4])
def test_phase_clock_derivatives(t):
    h = 1e-5
    f, v, a = phase_clock(t)
    before, after = phase_clock(t-h), phase_clock(t+h)
    assert v == pytest.approx((after[0]-before[0])/(2*h), abs=1e-5)
    assert a == pytest.approx((after[1]-before[1])/(2*h), abs=1e-4)


def test_reference_and_matched_constraints():
    assert phase_clock(0.) == (0., 0., 0.)
    assert phase_clock(9.) == (7., 0., 0.)
    assert sum(p['duration_s'] for p in task_plan('attitude_80_180')['phases']) == 173.
    assert sum(p['duration_s'] for p in task_plan('roll_80')['phases']) == 53.
    assert all(p['mode'] == 'offboard' for p in task_plan('lissajous_7')['phases'])
    assert not any('limit' in key for key in OLD_GAINS)
    assert all('integral' in key or 'Ki' in key for key in INTEGRAL)


def test_prepared_candidates_only_change_declared_fields(tmp_path):
    from tools.experiments.hardware_transfer import prepare, FIRMWARE
    m = prepare(SimpleNamespace(output=tmp_path/'campaign', firmware=FIRMWARE, task='roll_80'))
    def config(group):
        return json.loads(open(m['frozen_groups'][group]['tuning']).read())
    flight = config('flight')
    assert flight['HNTR_CG_X'] == flight['HNTR_CG_Z'] == 0.
    assert {k for k, v in config('yaw_unscheduled').items() if flight.get(k) != v} == {
        'direct_large_tilt_yaw_scheduling_enabled'}
    assert {k for k, v in config('full_attitude').items() if flight.get(k) != v} == {
        'direct_large_tilt_yaw_scheduling_enabled', 'direct_reduced_tilt_error_enabled'}
    for group in m['frozen_groups']:
        other = config(group)
        for key in ('direct_tau_limit', 'HNTR_S1_RATE', 'HNTR_S2_RATE', 'alpha_limit_deg',
                    'theta_limit_deg', 'HNTR_MAX_ARM_T', 'HNTR_TAIL_T_POS', 'HNTR_TAIL_T_NEG'):
            assert other[key] == flight[key], (group, key)


def test_manifest_write_failure_keeps_previous_evidence(tmp_path, monkeypatch):
    from tools.experiments import hardware_repair_core as core
    path = tmp_path/'manifest.json'
    core.write_json(path, {'status': 'previous'})
    def fail(*args):
        raise OSError('simulated full disk')
    monkeypatch.setattr(core.os, 'replace', fail)
    with pytest.raises(OSError):
        core.write_json(path, {'status': 'next'})
    assert json.loads(path.read_text()) == {'status': 'previous'}
    assert list(tmp_path.iterdir()) == [path]


def test_reuses_verified_snapshot_not_live_binary(tmp_path):
    from tools.experiments.hardware_repair_core import sha256, write_json
    from tools.experiments.hardware_repair_runner import freeze_firmware
    firmware = tmp_path/'firmware'
    live = firmware/'build/px4_sitl_default/bin/px4'
    live.parent.mkdir(parents=True)
    live.write_bytes(b'fake-immutable-test-binary')
    previous = tmp_path/'previous/firmware_snapshot'
    cached = previous/'bin/px4'
    cached.parent.mkdir(parents=True)
    cached.write_bytes(live.read_bytes())
    sim = dict(sdf_sha256='sdf', airframe_sha256='airframe')
    write_json(previous.parent/'manifest.json', dict(required_firmware_sha256=sha256(live),
        simulation=sim, firmware_snapshot_sha256={'bin/px4': sha256(cached)}))
    target = tmp_path/'next'
    target.mkdir()
    manifest = dict(simulation=sim.copy())
    frozen = freeze_firmware(target, firmware, sha256(live), manifest)/'bin/px4'
    assert frozen.stat().st_ino == cached.stat().st_ino
    assert frozen.stat().st_ino != live.stat().st_ino


def test_report_rms_is_time_weighted_and_reads_archived_data(tmp_path):
    import gzip
    from tools.experiments.analyze_hardware_transfer import weighted_rms, open_trace
    assert weighted_rms([0., 1., 4.], [0., 2., 0.]) == pytest.approx(np.sqrt(2.))
    with gzip.open(tmp_path/'sample.jsonl.gz', 'wt') as handle:
        handle.write('{"sample": 1}\n')
    with open_trace(tmp_path/'sample.jsonl') as handle:
        assert json.load(handle) == {'sample': 1}
