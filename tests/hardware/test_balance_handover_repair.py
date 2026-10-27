"""Offline regression: no ROS initialization, all Node operations mocked."""
import json
import time
from types import SimpleNamespace as Msg
from unittest.mock import Mock

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

if __package__:
    from .test_drcda_v2_hardware import controller, activate, feed, hw
else:
    from test_drcda_v2_hardware import controller, activate, feed, hw


def recorded_entry(node):
    node._output_enabled = True
    feed(node)
    node.pitch_torque_bias = .09
    node.direct_Domega[1] = 1.5
    enu = Rotation.from_euler('xyz', [-3.03179976, -7.05024167, 92.81011275], degrees=True)
    matrix = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
    q = Rotation.from_matrix(matrix @ enu.as_matrix() @ np.diag([1., -1., -1.])).as_quat()
    node.attitude_callback(Msg(timestamp=1_004_000, q=q[[3, 0, 1, 2]]))
    node.px4_actuator_motors_callback(Msg(timestamp=1_005_000, reversible_flags=16,
        control=[.5749505162, .5749505162, .5922372937, .5922372937, .6515749693] + [float('nan')] * 7))
    node.px4_actuator_servos_callback(Msg(timestamp=1_005_000,
        control=[.00763967296, .00316087815, .0254874588, .0241343122] + [float('nan')] * 4))
    node.control_mode_callback(Msg(timestamp=1_010_000, flag_armed=True, flag_control_offboard_enabled=True))
    return node._handover_wrench_initial.copy()


def balanced_request(node, raw):
    result = np.array(raw, dtype=float).copy()
    signs = np.array([1., -1., -1.])
    result[3:] = signs * node._balance_hardware_torque(signs * result[3:])
    return node._apply_hardware_wrench_handover(result)


def test_recorded_balance_is_preserved_after_entire_blend(controller):
    capture = recorded_entry(controller)
    raw = np.array([5.64133594, -2.41186608, 46.87442265, .05853558, 1.32142441, -.03311097])
    first = balanced_request(controller, raw)
    np.testing.assert_allclose(first, capture, atol=1e-9)
    assert controller._handover_torque_bias_body[1] < -2.8
    for simulated_seconds in (.01, .25, .5, 1., 10., 100.):
        controller.px4_timestamp = 1_010_000 + int(simulated_seconds * 1e6)
        np.testing.assert_allclose(balanced_request(controller, raw), capture, atol=1e-9)
    # This is session initialization, not a hidden clamp or reference governor.
    delta = np.array([.1, -.2, .3, .01, .02, -.03])
    np.testing.assert_allclose(balanced_request(controller, raw + delta), capture + delta)


def test_balance_force_is_world_fixed_torque_body_fixed(controller):
    recorded_entry(controller)
    raw = np.array([0., 0., 47., 0., 1.4, 0.])
    balanced_request(controller, raw)
    world = controller._handover_force_bias_world.copy()
    torque = controller._handover_torque_bias_body.copy()
    controller.R_ned_frd = Rotation.from_euler('xyz', [32, 20, -50], degrees=True).as_matrix()
    adjusted = balanced_request(controller, raw)
    force = adjusted[:3] - raw[:3]
    force_body = force * [1/controller.allocator_force_x_sign, 1/controller.allocator_force_y_sign, -1]
    np.testing.assert_allclose(controller.R_ned_frd @ force_body, world)
    np.testing.assert_array_equal(controller._handover_torque_bias_body, torque)


def test_balance_does_not_bypass_torque_limits(controller):
    recorded_entry(controller)
    raw = np.array([0., 0., 47., 0., 1.4, 0.])
    balanced_request(controller, raw)
    requested = raw + [0., 0., 0., 100., -100., 100.]
    adjusted = balanced_request(controller, requested)
    assert np.all(np.abs(adjusted[3:]) <= controller.direct_tau_limit)


def test_torque_balance_is_added_before_saturation_not_after_it(controller):
    recorded_entry(controller)
    balanced_request(controller, [0., 0., 47., 0., 1.4, 0.])
    for direction in (-1., 1.):
        torque = controller._add_firmware_attitude_trim([0., direction * 100., 0.])
        final = np.clip(torque, -controller.direct_tau_limit, controller.direct_tau_limit)
        assert final[1] == direction * controller.direct_tau_limit[1]


def test_manual_exit_resets_balance_and_no_post_exit_actuators(controller):
    recorded_entry(controller)
    balanced_request(controller, np.zeros(6))
    controller.control_mode_callback(Msg(timestamp=1_020_000, flag_armed=True, flag_control_offboard_enabled=False))
    assert controller._handover_wrench_initial is None
    assert controller._handover_force_bias_world is None
    controller.actuator_motors_pub.publish.reset_mock()
    controller._publish_normalized_direct_actuator_setpoint([.5]*5, [0.]*4)
    controller.actuator_motors_pub.publish.assert_not_called()
    assert controller._fault_position_pub is None
    feed(controller, stamp=1_030_000)
    controller.control_mode_callback(Msg(timestamp=1_040_000, flag_armed=True, flag_control_offboard_enabled=True))
    assert controller._hardware_control_active
    assert controller._handover_force_bias_world is None


@pytest.mark.parametrize('rpy', [[70, 0, 0], [0, 80, 20], [120, -40, 170], [180, 0, 40]])
def test_capture_and_zero_stick_hold_are_not_angle_gated(controller, rpy):
    controller._output_enabled = True
    feed(controller)
    q = Rotation.from_euler('xyz', rpy, degrees=True).as_quat()
    controller.attitude_callback(Msg(timestamp=1_005_000, q=q[[3, 0, 1, 2]]))
    expected = controller.R_ned_frd.copy()
    controller.control_mode_callback(Msg(timestamp=1_010_000, flag_armed=True, flag_control_offboard_enabled=True))
    assert controller._hardware_control_active and not controller._hardware_fault
    np.testing.assert_allclose(controller.target_R_des_ned_frd, expected)
    controller._hardware_handover_blend = 1.
    controller.rc_input.get_velocity_commands = Mock(return_value=controller._zero_manual_cmd())
    controller.update_trajectory(.6, .01)
    # This checks reference preservation, not whether an inverted aircraft flies.
    converted = controller._direct_desired_attitude_ned_frd(controller.target_attitude)
    np.testing.assert_allclose(converted, expected, atol=1e-12)
    assert not controller._direct_safety_violation_reason()
    assert controller._fault_position_pub is None


def test_fault_requests_position_immediately_and_ack_does_not_confirm(controller, monkeypatch):
    activate(controller)
    now = time.monotonic() + .01
    monkeypatch.setattr(hw.time, 'monotonic', lambda: now)
    controller._latch_fault('nonfinite solver output')
    assert controller._fault_handover_pending
    request = controller._fault_position_pub.publish.call_args.args[0]
    assert request.command == hw.VehicleCommand.VEHICLE_CMD_DO_SET_MODE
    assert (request.param1, request.param2, request.param3) == (1., 3., 0.)
    assert not controller._heartbeat_block_reason()
    controller.vehicle_command_ack_callback(Msg(command=request.command, result=0))
    assert controller._fault_handover_pending
    controller.actuator_motors_pub.publish.reset_mock()
    controller._publish_normalized_direct_actuator_setpoint([.5]*5, [0.]*4)
    controller.actuator_motors_pub.publish.assert_not_called()


def test_fault_confirms_fresh_mode_and_never_requests_arm_offboard(controller, monkeypatch):
    activate(controller)
    now = [time.monotonic() + .01]
    monkeypatch.setattr(hw.time, 'monotonic', lambda: now[0])
    controller._latch_fault('invalid attitude')
    now[0] += .03
    controller.control_mode_callback(Msg(timestamp=1_040_000, flag_armed=True, flag_control_offboard_enabled=False))
    assert not controller._fault_handover_pending
    assert not controller._hardware_control_active
    controller.offboard_control_mode_pub.publish.reset_mock()
    controller.offboard_startup_tick()
    controller.offboard_control_mode_pub.publish.assert_not_called()
    calls = controller._fault_position_pub.publish.call_args_list
    assert all(c.args[0].command == hw.VehicleCommand.VEHICLE_CMD_DO_SET_MODE and c.args[0].param2 == 3. for c in calls)
    assert controller.vehicle_command_pub is None
    events = [json.loads(s) for s in controller._event_path.read_text().splitlines()]
    assert any(e['event'] == 'position_handover_confirmed' for e in events)


def test_missing_mode_confirmation_has_bounded_heartbeat_not_infinite_hold(controller, monkeypatch):
    activate(controller)
    now = [time.monotonic() + .01]
    monkeypatch.setattr(hw.time, 'monotonic', lambda: now[0])
    controller._latch_fault('stale gyro')
    assert not controller._heartbeat_block_reason()
    now[0] += .2
    controller.offboard_startup_tick()
    assert not controller._heartbeat_block_reason()
    now[0] += .2
    controller.offboard_startup_tick()
    assert controller._heartbeat_block_reason()
    assert controller._fault_position_pub.publish.call_count >= 3
    assert controller._fault_handover_timeout_reported


def test_fresh_position_status_confirms_but_cached_position_does_not(controller, monkeypatch):
    activate(controller)
    now = [time.monotonic() + .01]
    monkeypatch.setattr(hw.time, 'monotonic', lambda: now[0])
    controller._latch_fault('bad solve')
    # Last status still says Position from before the Offboard control-mode edge.
    assert controller.nav_state == 2 and controller._fault_handover_pending
    now[0] += .02
    controller.status_callback(Msg(timestamp=1_040_000, arming_state=2, nav_state=2))
    assert not controller._fault_handover_pending
    events = [json.loads(s) for s in controller._event_path.read_text().splitlines()]
    assert events[-1]['details']['position_state_confirmed']


def test_disarm_feedback_stops_fault_requests(controller, monkeypatch):
    activate(controller)
    now = [time.monotonic() + .01]
    monkeypatch.setattr(hw.time, 'monotonic', lambda: now[0])
    controller._latch_fault('invalid pose')
    before = controller._fault_position_pub.publish.call_count
    now[0] += .03
    controller.status_callback(Msg(timestamp=1_040_000, arming_state=1, nav_state=14))
    assert not controller._fault_handover_pending
    now[0] += 1.
    controller.offboard_startup_tick()
    assert controller._fault_position_pub.publish.call_count == before


def test_normal_running_never_requests_mode_changes(controller):
    activate(controller)
    for stamp in (1_020_000, 1_040_000, 1_060_000):
        feed(controller, stamp=stamp, offboard=True)
        controller.offboard_startup_tick()
    assert not controller._hardware_fault
    assert controller._fault_position_pub is None


def test_event_scalar_encoding_and_csv_row_reliability(controller):
    activate(controller)
    feed(controller, stamp=1_020_000, offboard=True)
    controller._record_event('numeric_test', {'f': np.float32(1.2), 'a': np.array([np.float32(2.)])})
    events = [json.loads(s) for s in controller._event_path.read_text().splitlines()]
    assert events[-1]['details']['a'] == [2.]
    import csv
    rows = list(csv.reader(controller.diagnostic_path.open()))
    assert len(rows) >= 2
    assert all(len(row) == len(rows[0]) for row in rows)
