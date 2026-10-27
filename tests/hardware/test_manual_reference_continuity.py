"""Offline reference regression; all aircraft interfaces are mocked."""
import csv
from types import SimpleNamespace as Msg
from unittest.mock import Mock

import numpy as np
import pytest

from .test_drcda_v2_hardware import controller, activate, feed


@pytest.mark.parametrize('axis', [0, 1, 2])
@pytest.mark.parametrize('direction', [-1., 1.])
def test_large_tracking_error_tiny_stick_cannot_project_reference(controller, axis, direction):
    activate(controller)
    before = controller.manual_des_pos.copy()
    controller.position[axis] += direction * .7
    velocity = np.zeros(3)
    velocity[axis] = direction * .0001
    controller._advance_manual_position_reference(velocity, .015)
    assert np.linalg.norm(controller.target_velocity) <= .0001 + 1e-12
    np.testing.assert_allclose(controller.target_position - before, .015 * controller.target_velocity, atol=1e-12)
    assert abs(controller.target_position[axis] - before[axis]) <= .000001501


def test_zero_input_does_not_rebase_a_drifting_vehicle(controller):
    activate(controller)
    before = controller.manual_des_pos.copy()
    controller.position += [2., -3., 1.]
    for _ in range(50):
        controller._advance_manual_position_reference(np.zeros(3), .02)
        np.testing.assert_array_equal(controller.target_position, before)
        np.testing.assert_array_equal(controller.target_velocity, 0.)


def test_outward_input_is_limited_without_position_jump(controller):
    activate(controller)
    before = controller.manual_des_pos.copy()
    controller.position[:2] -= [1., 0.]
    controller._advance_manual_position_reference([.3, 0., 0.], .015)
    np.testing.assert_array_equal(controller.target_position, before)
    controller._advance_manual_position_reference([-.3, 0., 0.], .015)
    assert controller.target_velocity[0] < 0
    assert abs(controller.target_velocity[0]) <= np.linalg.norm(controller.rc_input.max_acc_body_xy_mps2) * .015


def test_direction_changes_keep_position_velocity_acceleration_consistent(controller):
    activate(controller)
    rng = np.random.default_rng(271027)
    for i in range(500):
        dt = float(rng.uniform(.004, .025))
        before = controller.manual_des_pos.copy()
        old_velocity = controller._manual_reference_velocity.copy()
        command = rng.uniform(-.3, .3, 3)
        if i % 50 == 0:
            controller.position[:2] += [.3, -.4]
        controller._advance_manual_position_reference(command, dt)
        np.testing.assert_allclose(controller.target_position-before, dt*controller.target_velocity, atol=1e-12)
        np.testing.assert_allclose(controller.target_velocity-old_velocity, dt*controller.target_acceleration, atol=1e-12)
        assert np.linalg.norm(controller.target_velocity[:2]) <= np.linalg.norm(controller.rc_input.max_vxy_body_mps) + 1e-12
        assert abs(controller.target_velocity[2]) <= controller.rc_input.max_vz + 1e-12
        assert np.linalg.norm(controller.target_acceleration[:2]) <= np.linalg.norm(controller.rc_input.max_acc_body_xy_mps2) + 1e-12
        assert abs(controller.target_acceleration[2]) <= .5 + 1e-12


def test_recorded_scale_of_projection_bug_is_removed(controller):
    activate(controller)
    # Independent synthetic reproduction of the original projection formula.
    previous, actual, dt, stick = 0., .7, .015, .0001
    projected = actual + np.clip(previous+dt*stick-actual, -.6, .6)
    assert (projected-previous)/dt > 6.6
    controller.manual_des_pos[:] = 0.
    controller.position[:] = [.7, 0., controller._z0]
    controller._advance_manual_position_reference([stick, 0., 0.], dt)
    assert controller.target_velocity[0] == pytest.approx(stick)


def test_second_manual_session_logs_immediately_and_resets_reference_velocity(controller):
    activate(controller)
    first_id = controller._control_session_id
    controller._last_diagnostic_log_time = 69.617
    controller._manual_reference_velocity[:] = .2
    controller.control_mode_callback(Msg(timestamp=1_020_000, flag_armed=True, flag_control_offboard_enabled=False))
    feed(controller, stamp=1_030_000)
    controller.control_mode_callback(Msg(timestamp=1_040_000, flag_armed=True, flag_control_offboard_enabled=True))
    assert controller._control_session_id == first_id + 1
    np.testing.assert_array_equal(controller._manual_reference_velocity, 0.)
    feed(controller, stamp=1_050_000, offboard=True)
    assert 0 <= controller._last_diagnostic_log_time < .1
    with controller.diagnostic_path.open() as stream:
        rows = list(csv.DictReader(stream))
    assert any(int(row['v2_control_session_id']) == first_id+1 for row in rows)


@pytest.mark.parametrize('dt', [0., -.01, float('nan')])
def test_invalid_reference_dt_is_rejected(controller, dt):
    activate(controller)
    before = controller.manual_des_pos.copy()
    with pytest.raises(ValueError):
        controller._advance_manual_position_reference([0., 0., 0.], dt)
    np.testing.assert_array_equal(controller.manual_des_pos, before)
