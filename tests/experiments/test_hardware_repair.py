"""Offline harness tests. ROS publishers are mocked; no simulator is launched."""
import ast
import json
import math
import os
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from tools.experiments import hardware_repair_core as core
from tools.experiments.hardware_repair_runner import isolated_environment


@pytest.fixture
def plan():
    return json.loads(core.PLAN.read_text())


@pytest.fixture
def sim():
    if not core.FIRMWARE.exists():
        pytest.skip('Local no-delay Gazebo firmware checkout is not installed')
    return core.inspect_simulation(core.FIRMWARE)


def samples(plan):
    rows = []
    for t in np.arange(0., sum(p['duration_s'] for p in plan['phases']), .05):
        phase = core.phase_at(plan, float(t))
        active = phase['mode'] == 'offboard'
        rows.append(dict(wall_s=float(t + 1), px4_s=float(t + 10), scenario_s=float(t),
                         phase=phase['name'], nav_state=14 if active else 2,
                         hardware_active=active, armed=True, fault='', altitude_m=3.,
                         position_error_m=.03, so3_deg=1., roll_deg=0., pitch_deg=0.,
                         speed_m_s=.1, target_speed_m_s=.1, joint_age_s=.01,
                         source_age_s=.01, rate_deg_s=5., motor_abs_max=.6, servo_abs_max=.1))
    return rows


def test_complete_scenario_and_same_input(plan):
    core.validate_plan(plan)
    assert [core.phase_at(plan, t)['name'] for t in (0, 8, 20, 37, 42)] == [
        'hold', 'yaw_out', 'lateral_out', 'position_exit', 'offboard_reentry']
    assert core.phase_at(plan, 65) is None
    assert core.analyze_samples(samples(plan), plan, True)['status'] == 'PASS'


@pytest.mark.parametrize('fault', ['nan', 'missing_phase', 'mode', 'disarm', 'stale', 'gap', 'spike', 'no_finish'])
def test_fail_closed(plan, fault):
    rows = samples(plan)
    completed = True
    if fault == 'nan': rows[40]['so3_deg'] = None
    if fault == 'missing_phase': rows = [r for r in rows if r['phase'] != 'yaw_back']
    if fault == 'mode':
        for r in rows: r['nav_state'] = 14
    if fault == 'disarm': rows[40]['armed'] = False
    if fault == 'stale': rows[40]['source_age_s'] = 2.
    if fault == 'gap': del rows[40:80]
    if fault == 'spike': rows[40]['target_speed_m_s'] = 9.
    if fault == 'no_finish': completed = False
    assert core.analyze_samples(rows, plan, completed)['status'] != 'PASS'


def test_flight_snapshot_not_repo_defaults(sim):
    flight = json.loads(core.FLIGHT.read_text())
    data, adaptation = core.adapted_tuning(core.SOURCE, flight, sim)
    assert data['HNTR_PITCH_BIAS'] == pytest.approx(.09)
    assert data['direct_Domega'] == [3., 1.5, 2.3]
    assert data['HNTR_S2_GEAR'] == 1
    assert data['HNTR_MASS'] == pytest.approx(4.5)
    assert adaptation['flight_values']['HNTR_S2_GEAR'] == 2
    assert 'HNTR_PITCH_BIAS' not in adaptation['simulation_overrides']


def test_candidate_changes_only_approved_gains(plan, sim):
    flight = json.loads(core.FLIGHT.read_text())
    base, _ = core.adapted_tuning(core.SOURCE, flight, sim)
    tuned, _ = core.adapted_tuning(core.SOURCE, flight, sim, plan['candidate'])
    assert {k for k in base if base[k] != tuned[k]} == set(plan['candidate']) - {'direct_attitude_integral_limit'}
    with pytest.raises(ValueError, match='Candidate'):
        core.adapted_tuning(core.SOURCE, flight, sim, {'HNTR_MASS': 9})


@pytest.mark.parametrize('force', [0., .1, 1., 6., 12., 24., 50., 85.])
def test_front_motor_exact_gazebo_inverse(sim, force):
    k = sim['motor_constants'][0]
    u = core.front_control(force, k)
    assert core.front_force(u, k) == pytest.approx(max(force, k * 100.))


@pytest.mark.parametrize('force', [-6.02, -3., -.01, 0., .01, 3., 12.777084])
def test_tail_motor_signed_curve(sim, force):
    k = sim['motor_constants'][4]
    assert core.tail_force(core.tail_control(force, k), k) == pytest.approx(force)
    assert core.tail_control(-6.02, k) == pytest.approx(-math.sqrt(6.02 / 12.777084))


def test_no_delay_and_no_measured_feedback(plan, monkeypatch):
    monkeypatch.setenv('HNUTER_HARDWARE_JOINT_TOPIC', '/real/joints')
    monkeypatch.setenv('HNUTER_ALLOW_REMOTE_DDS', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    env = isolated_environment(plan, 'unit-test')
    assert env['ROS_DOMAIN_ID'] == '174'
    assert env['PX4_UXRCE_DDS_PORT'] == '8894'
    assert env['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    assert 'HNUTER_HARDWARE_JOINT_TOPIC' not in env
    assert 'HNUTER_ALLOW_REMOTE_DDS' not in env


def test_adapter_never_overrides_control_under_test():
    path = core.ROOT / 'tools/experiments/hardware_repair_node.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == 'HardwareV2SITL')
    names = {n.name for n in cls.body if isinstance(n, ast.FunctionDef)}
    assert not names.intersection(core.PROTECTED_METHODS)


@pytest.fixture
def adapter(monkeypatch, tmp_path, sim):
    from tools.experiments.hardware_repair_node import load_hardware, adapter_type
    for key in list(os.environ):
        if key.startswith('HNUTER_'): monkeypatch.delenv(key)
    monkeypatch.setenv('HNUTER_LOG_DIR', str(tmp_path / 'logs'))
    data, _ = core.adapted_tuning(core.SOURCE, json.loads(core.FLIGHT.read_text()), sim)
    tuning = tmp_path / 'tuning.json'
    core.write_json(tuning, data)
    monkeypatch.setenv('HNUTER_TUNING_FILE', str(tuning))
    hw = load_hardware(core.SOURCE)
    monkeypatch.setattr(hw.Node, '__init__', lambda *a, **k: None)
    monkeypatch.setattr(hw.Node, 'get_logger', lambda *a: Mock())
    monkeypatch.setattr(hw.Node, 'get_clock', lambda *a: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)))
    for name in ('create_publisher', 'create_subscription', 'create_timer'):
        monkeypatch.setattr(hw.Node, name, lambda *a, **k: Mock())
    monkeypatch.setattr(hw, 'KeyboardCommandReader', Mock())
    node = adapter_type(hw, sim)()
    yield node
    assert node._event_writer.close()
    node._diagnostic_file.close()


def test_original_hardware_logic_and_command_model_only(adapter):
    assert not adapter._joint_feedback_enabled
    assert adapter._diagnostic_extra_values()[-1] == 'command_model_only'
    assert adapter.direct_Domega[1] == 1.5
    assert adapter.pitch_torque_bias == pytest.approx(.09)
    assert adapter.vehicle_command_pub is None
    for field in ('servo_delay_positive_s', 'servo_delay_negative_s'):
        np.testing.assert_array_equal(getattr(adapter.drcda.config, field), 0)
    assert adapter.drcda.model.__class__.__name__ == 'SDFWrenchModel'


def test_flu_sdf_wrench_matches_allocator_frame(adapter):
    state = np.array([.1, -.2, .3, .1, 8., 9., 10., 11., -2.])
    flu = adapter.drcda.model.wrench(state)
    flip = np.diag([1., -1., -1.])
    actual = adapter._allocator_wrench_from_body_force_torque(flip @ flu[:3], flip @ flu[3:])
    np.testing.assert_allclose(actual, flu, atol=1e-12)


def test_servo_mapping_and_tail_reverse_limit(adapter, sim):
    primary = adapter._primary_joint_angle_to_normalized(.8)
    secondary = adapter._secondary_joint_angle_to_normalized(.4)
    assert primary * np.radians(185.) == pytest.approx(.8)
    assert secondary * np.pi == pytest.approx(.4)
    assert adapter._tail_control_to_thrust(adapter._tail_thrust_to_control(-20.)) == pytest.approx(-6.02)
    assert adapter._tail_control_to_thrust(adapter._tail_thrust_to_control(20.)) == pytest.approx(12.777084)
    np.testing.assert_allclose(adapter.drcda.config.servo_command_rate_rad_s, [4.7, 2.35, 4.7, 2.35])


def test_synthetic_up_force_pitch_sign():
    arm_flu = np.array([.2, 0., 0.])
    force_world = np.array([0., 0., 3.])
    np.testing.assert_allclose(np.cross(arm_flu, force_world), [0., -.6, 0.])
    yaw90 = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    np.testing.assert_allclose(np.cross(yaw90 @ arm_flu, force_world), [.6, 0., 0.])


def test_public_baseline_reconstructs_exact_remote_source(tmp_path):
    from tools.experiments.hardware_repair_runner import reconstruct_baseline
    output = tmp_path / 'baseline.py'
    reconstruct_baseline(output)
    provenance = json.loads((core.ROOT / 'reports/hardware_repair_20261027/baseline_provenance.json').read_text())
    assert core.sha256(output) == provenance['baseline_sha256']


def test_summary_does_not_treat_truncated_run_as_complete():
    from tools.experiments.bundle_hardware_repair_report import window
    rows = [dict(scenario_s=t, position_error_m=.1, so3_deg=2.,
                 target_speed_m_s=.2, rate_deg_s=3.) for t in np.arange(0., 43., .02)]
    assert window(rows, 0, 37)['position_m_rmse'] == pytest.approx(.1)
    assert window(rows, 37, 65) is None
