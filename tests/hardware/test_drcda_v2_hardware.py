"""Offline only: every ROS publisher, timer and subscription is mocked."""
import ast
import copy
import importlib.util
import json
import time
from pathlib import Path
from types import SimpleNamespace as Msg
from unittest.mock import Mock

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from controllers.hardware import hnuter_external_direct_drcda_v2_hardware as hw


@pytest.fixture
def controller(monkeypatch, tmp_path):
    for key in list(hw.os.environ):
        if key.startswith('HNUTER_'):
            monkeypatch.delenv(key)
    monkeypatch.setenv('HNUTER_LOG_DIR', str(tmp_path))
    monkeypatch.setattr(hw.Node, '__init__', lambda *a, **k: None)
    monkeypatch.setattr(hw.Node, 'get_logger', lambda *a: Mock())
    monkeypatch.setattr(hw.Node, 'get_clock', lambda *a: Msg(
        now=lambda: Msg(nanoseconds=time.time_ns())))
    monkeypatch.setattr(hw.Node, 'create_publisher', lambda *a, **k: Mock())
    monkeypatch.setattr(hw.Node, 'create_subscription', lambda *a, **k: Mock())
    monkeypatch.setattr(hw.Node, 'create_timer', lambda *a, **k: Mock())
    monkeypatch.setattr(hw, 'KeyboardCommandReader', Mock())
    node = hw.HnuterHardwareDRCDAV2Controller()
    yield node
    node._diagnostic_file.close()


def feed(node, stamp=1_000_000, armed=True, offboard=False):
    node.manual_control_callback(Msg(valid=True, data_source=hw.ManualControlSetpoint.SOURCE_RC,
                                     roll=0., pitch=0., yaw=0., throttle=0., aux1=0., aux2=0.))
    node.local_position_callback(Msg(timestamp=stamp, xy_valid=True, z_valid=True,
                                    v_xy_valid=True, v_z_valid=True,
                                    x=2., y=3., z=-4., vx=0., vy=0., vz=0.))
    node.angular_velocity_callback(Msg(timestamp=stamp, xyz=[0., 0., 0.]))
    node.attitude_callback(Msg(timestamp=stamp, q=[1., 0., 0., 0.]))
    trim = -node._add_firmware_attitude_trim(np.zeros(3))[1]
    tail = trim/node.l2
    main = node._front_thrust_to_control((node.mass*node.gravity-tail)/4)
    node.px4_actuator_motors_callback(Msg(timestamp=stamp, reversible_flags=16,
        control=[main]*4+[node._tail_thrust_to_control(tail)]+[float('nan')]*7))
    node.px4_actuator_servos_callback(Msg(timestamp=stamp, control=[.01, .02, .03, .04]+[float('nan')]*4))
    node.hnuter_attitude_reference_callback(Msg(timestamp=stamp, name='hntr_att', x=0., y=0., z=0.))
    node.status_callback(Msg(timestamp=stamp, arming_state=2 if armed else 1, nav_state=14 if offboard else 2))
    node.control_mode_callback(Msg(timestamp=stamp, flag_armed=armed, flag_control_offboard_enabled=offboard))


def activate(node):
    # Synthetic test fixture only, not evidence of aircraft parameter readback.
    node._firmware_parameters_verified = True
    node._output_enabled = True
    feed(node)
    node.offboard_startup_tick()
    node.control_mode_callback(Msg(timestamp=1_010_000, flag_armed=True, flag_control_offboard_enabled=True))
    assert node._hardware_control_active


def test_constructor_and_monitor_never_publish(controller):
    feed(controller, offboard=True)
    controller.offboard_startup_tick()
    controller._publish_normalized_direct_actuator_setpoint([.5]*5, [0.]*4)
    assert not controller._hardware_control_active
    for pub in (controller.actuator_motors_pub, controller.actuator_servos_pub,
                controller.offboard_control_mode_pub, controller.vehicle_thrust_setpoint_pub,
                controller.vehicle_torque_setpoint_pub, controller.trajectory_setpoint_pub):
        pub.publish.assert_not_called()
    assert controller.vehicle_command_pub is None


def test_runtime_allocator_has_no_delay_or_gazebo_calibration(controller):
    cfg = controller.drcda.config
    assert cfg.bounded_solver
    assert cfg.horizon_s == .1
    for name in ('servo_delay_positive_s', 'servo_delay_negative_s',
                 'servo_tau_positive_s', 'servo_tau_negative_s', 'motor_tau_up_s', 'motor_tau_down_s'):
        np.testing.assert_array_equal(getattr(cfg, name), 0.)
    np.testing.assert_allclose(cfg.servo_gain_positive, 1.)
    np.testing.assert_allclose(cfg.servo_command_rate_rad_s, [4.7, 2.35, 4.7, 2.35])
    assert not controller.use_px4_position_takeoff
    assert not controller.auto_arm_enabled
    assert not controller.publish_allocator_setpoints_in_direct
    assert controller.direct_safety_shutdown_enabled
    assert not hasattr(controller, 'gz_servo0_pub')


def test_handover_and_direct_callback(controller):
    activate(controller)
    np.testing.assert_allclose(controller.target_position, [3., 2., 0.])
    controller.offboard_control_mode_pub.publish.assert_called()
    mode = controller.offboard_control_mode_pub.publish.call_args.args[0]
    assert mode.direct_actuator and not mode.position and not mode.thrust_and_torque
    feed(controller, stamp=1_020_000, offboard=True)
    assert not controller._hardware_fault
    assert controller.drcda.last_result.status == 'solved'
    np.testing.assert_allclose(controller.drcda.last_result.prediction_horizons_s, [.05, .1])
    controller.actuator_motors_pub.publish.assert_called()
    controller.actuator_servos_pub.publish.assert_called()
    controller.trajectory_setpoint_pub.publish.assert_not_called()
    controller.vehicle_torque_setpoint_pub.publish.assert_not_called()
    # Command echoes initialize an estimate, not physical angle measurements.
    np.testing.assert_allclose(controller.drcda.command[:4],
                              controller.last_servo_cmd[[1, 3, 0, 2]] * [np.pi, np.pi/2, np.pi, np.pi/2])
    assert controller._diagnostic_extra_values()[-1] == 'command_model_only'


@pytest.mark.parametrize('name', ['position', 'attitude', 'rates', 'status', 'mode'])
def test_stale_data_requests_position_before_withdrawing_heartbeat(controller, name):
    activate(controller)
    controller.actuator_motors_pub.publish.reset_mock()
    controller.offboard_control_mode_pub.publish.reset_mock()
    controller._received[name] = time.monotonic() - 5
    controller.offboard_startup_tick()
    assert controller._hardware_fault
    assert not controller._hardware_control_active
    controller.actuator_motors_pub.publish.assert_not_called()
    controller.offboard_control_mode_pub.publish.assert_called()
    request = controller._fault_position_pub.publish.call_args.args[0]
    assert request.command == hw.VehicleCommand.VEHICLE_CMD_DO_SET_MODE
    assert request.param2 == 3.


def test_invalid_position_and_duplicate_stamps_are_not_fresh(controller):
    feed(controller)
    prior = controller._received['rates']
    controller.angular_velocity_callback(Msg(timestamp=1_000_000, xyz=[0., 0., 0.]))
    assert controller._received['rates'] == prior
    controller.local_position_callback(Msg(timestamp=1_010_000, xy_valid=True, z_valid=True,
        v_xy_valid=False, v_z_valid=True, x=0., y=0., z=0., vx=0., vy=0., vz=0.))
    assert 'position' not in controller._received


def test_exit_and_reentry_reset_allocator(controller):
    activate(controller)
    controller.control_mode_callback(Msg(timestamp=1_020_000, flag_armed=True, flag_control_offboard_enabled=False))
    assert not controller._hardware_control_active
    assert controller.drcda.last_result is None
    feed(controller, stamp=1_020_001)
    controller.position[:] = [5, 6, 7]
    controller.control_mode_callback(Msg(timestamp=1_030_000, flag_armed=True, flag_control_offboard_enabled=True))
    assert controller._hardware_control_active
    np.testing.assert_allclose(controller.target_position, [5, 6, 0])
    assert controller._z0 == 7


def test_missing_snapshot_uses_allocator_fallback_without_blocking_entry(controller):
    controller._firmware_parameters_verified = True
    controller._output_enabled = True
    feed(controller)
    controller._received.pop('servos')
    controller.px4_out_motor_timestamp = 0
    controller.px4_out_servo_timestamp = 0
    controller.px4_out_motor_cmd[:] = np.nan
    controller.px4_out_servo_cmd[:] = np.nan
    controller.control_mode_callback(Msg(timestamp=1_010_000, flag_armed=True, flag_control_offboard_enabled=True))
    assert controller._hardware_control_active
    assert not controller._hardware_handover_snapshot_valid
    controller.actuator_motors_pub.publish.assert_not_called()
    feed(controller, stamp=1_020_000, offboard=True)
    assert not controller._hardware_fault
    assert np.all(np.isfinite(controller.actuator_motors_pub.publish.call_args.args[0].control[:5]))
    assert np.max(controller.actuator_motors_pub.publish.call_args.args[0].control[:4]) > 0


def test_manual_mode_flags_authorize_control_and_px4_handles_rc_failsafe(controller):
    activate(controller)
    controller.rc_input._manual_valid = False
    controller.offboard_startup_tick()
    assert not controller._hardware_fault
    assert controller._hardware_control_active
    feed(controller, stamp=1_030_000, offboard=True)
    controller.offboard_startup_tick()
    controller.control_mode_callback(Msg(timestamp=1_040_000, flag_armed=True,
                                         flag_control_offboard_enabled=False))
    assert not controller._hardware_control_active


def test_downward_rc_reference_not_clipped_to_handover_height(controller):
    activate(controller)
    controller._hardware_handover_blend = 1.
    command = controller._zero_manual_cmd()
    command['vz'] = -.3
    controller.rc_input.get_velocity_commands = Mock(return_value=command)
    controller.update_trajectory(.1, .1)
    assert controller.target_position[2] == pytest.approx(-.005)
    assert controller.target_velocity[2] == pytest.approx(-.05)
    controller.position[2] -= .005
    command['vz'] = 0
    controller.update_trajectory(.2, .1)
    assert controller.target_position[2] == pytest.approx(-.005)
    assert controller.target_velocity[2] == 0.


def test_attitude_magnitude_alone_does_not_stop_control(controller):
    activate(controller)
    controller.R_ned_frd = np.diag([1., -1., -1.])
    assert not controller._direct_safety_violation_reason()


def test_real_px4_inverted_attitude_is_not_canonicalized(controller):
    controller.attitude_callback(Msg(timestamp=1_000_000, q=[0., 1., 0., 0.]))
    np.testing.assert_allclose(controller.R_ned_frd, np.diag([1., -1., -1.]))


def test_pwm_only_applies_to_servos_and_gear_is_applied_once(controller):
    for normalized, pwm in [(-1., 500.), (0., 1500.), (1., 2500.)]:
        assert controller._normalized_servo_to_expected_pwm_us(normalized) == pwm
    assert controller._secondary_joint_angle_to_normalized(np.pi / 2) == 1
    hover = controller.mass * controller.gravity / 4
    assert controller._front_thrust_to_control(hover) == pytest.approx(.58)
    for thrust in (-6.04, 0, 12.78):
        assert controller._tail_control_to_thrust(controller._tail_thrust_to_control(thrust)) == pytest.approx(thrust)


def test_published_blend_replaces_pending_optimizer_command(controller):
    activate(controller)
    controller.drcda._pending_servo_commands = [[(0., 1.)] for _ in range(4)]
    controller._publish_normalized_direct_actuator_setpoint([.5]*4+[0.], [.1, .2, .3, .4])
    expected = np.array([.2*np.pi, .2*np.pi, .1*np.pi, .15*np.pi])
    np.testing.assert_allclose(controller.drcda.command[:4], expected)
    np.testing.assert_allclose(controller.drcda._delayed_servo_command, expected)
    assert controller.drcda._pending_servo_commands == [[], [], [], []]


def test_opt_in_tasks_are_relative_and_restart_on_manual_mode_cycle(controller):
    activate(controller)
    controller._tasks_enabled = True
    controller.position[:] = [7., 8., 2.]
    controller._start_auto_trajectory('attitude', 1.)
    np.testing.assert_allclose(controller.auto_traj_start_pos, [7., 8., -2.])
    controller.control_mode_callback(Msg(timestamp=1_020_000, flag_armed=True, flag_control_offboard_enabled=False))
    feed(controller, stamp=1_020_001)
    controller.position[:] = [9., 10., 3.]
    controller.control_mode_callback(Msg(timestamp=1_030_000, flag_armed=True, flag_control_offboard_enabled=True))
    assert controller.pending_auto_traj_mode == 'attitude'
    controller._hardware_handover_blend = 1.
    controller.update_trajectory(.1, .01)
    np.testing.assert_allclose(controller.auto_traj_start_pos, [9., 10., 0.])


def test_optional_joint_feedback_corrects_predictor_and_stale_feedback_blocks(controller, monkeypatch):
    controller._joint_feedback_enabled = True
    controller._joint_names = ('aL', 'bL', 'aR', 'bR')
    monkeypatch.setattr(hw.Node, 'get_clock', lambda self: Msg(now=lambda: Msg(nanoseconds=1_000_000_000)))
    message = Msg(name=['bR', 'aR', 'bL', 'aL'], position=[.4, .3, .2, .1],
                  header=Msg(stamp=Msg(sec=1, nanosec=0)))
    controller.joint_feedback_callback(message)
    np.testing.assert_allclose(controller.drcda.measured_joint_angles, [.1, .2, .3, .4])
    controller.drcda._advance_state(.01)
    np.testing.assert_allclose(controller.drcda.state[:4], [.1, .2, .3, .4])
    feed(controller)
    assert not controller._readiness_reason()
    controller._received['joints'] -= 1
    assert 'joints' in controller._readiness_reason()
    message.header.stamp.sec = 0
    controller.joint_feedback_callback(message)
    assert 'joints' not in controller._received


def test_numerical_failure_latches_without_idle_commands(controller, monkeypatch):
    activate(controller)
    controller.actuator_motors_pub.publish.reset_mock()
    monkeypatch.setattr(controller.drcda, 'allocate', Mock(side_effect=ValueError('bad solve')))
    feed(controller, stamp=1_020_000, offboard=True)
    assert 'bad solve' in controller._hardware_fault
    controller.actuator_motors_pub.publish.assert_not_called()


def test_one_60ms_sample_interval_does_not_latch_fault(controller):
    activate(controller)
    controller.px4_timestamp = 1_060_000
    controller.actuator_motors_pub.publish.reset_mock()
    controller.control_loop()
    assert not controller._hardware_fault
    controller.actuator_motors_pub.publish.assert_called()


def test_lissajous_has_no_absolute_height_clamp(controller):
    activate(controller)
    controller._tasks_enabled = True
    controller.position[2] -= 2.
    controller._start_auto_trajectory('lissajous', 0.)
    position, _, _, _ = controller._lissajous_reference(0.)
    assert position[2] == pytest.approx(-2.)


def test_nonfinite_state_requests_position_without_publishing_invalid_commands(controller):
    activate(controller)
    controller.R_ned_frd[:] = np.nan
    controller.actuator_motors_pub.publish.reset_mock()
    controller.offboard_control_mode_pub.publish.reset_mock()
    controller.control_loop()
    controller.offboard_startup_tick()
    assert 'nonfinite' in controller._hardware_fault
    controller.actuator_motors_pub.publish.assert_not_called()
    controller.offboard_control_mode_pub.publish.assert_called()
    controller._fault_position_pub.publish.assert_called()


def test_position_reset_latches_fault(controller):
    activate(controller)
    controller.local_position_callback(Msg(timestamp=1_020_000, xy_valid=True, z_valid=True,
        v_xy_valid=True, v_z_valid=True, x=10., y=0., z=-3., vx=0., vy=0., vz=0., xy_reset_counter=1))
    assert 'reset' in controller._hardware_fault


def test_bounded_optimization_holds_hover_and_respects_command_rates(controller):
    allocator = controller.drcda
    hover = np.r_[np.zeros(4), np.full(4, controller.mass * controller.gravity / 4), 0.]
    allocator.reset(angle_state=hover[:4], thrust_state=hover[4:])
    desired = allocator.model.wrench(hover)
    allocator.synchronize_wrench_reference(desired)
    result = allocator.allocate(desired, .01, hover)
    np.testing.assert_allclose(result.predicted_wrench, desired, atol=1e-8)
    for _ in range(20):
        previous = allocator.command.copy()
        result = allocator.allocate(desired + [0, 3, 0, 0, 0, .1], .01, hover)
        assert np.all(np.abs(result.command[:4] - previous[:4]) <= allocator.config.servo_command_rate_rad_s * .01 + 1e-9)
        assert np.all(result.command[4:] <= allocator.config.thrust_max_n)
        assert np.all(result.command[4:] >= allocator.config.thrust_min_n)


@pytest.mark.parametrize('key,value', [('HNTR_S2_GEAR', 0), ('servo_pwm_min_us', 1000),
    ('HNTR_MASS', float('nan')), ('drcda_antiwindup_gain', .2),
    ('hardware_servo_tau_positive_s', [-1, 0, 0, 0]), ('servo_delay_s', .1)])
def test_invalid_configuration_rejected(key, value):
    config = copy.deepcopy(hw.HARDWARE_V2_DEFAULT_CONFIG)
    config[key] = value
    with pytest.raises(ValueError):
        hw.HnuterHardwareDRCDAV2Controller.validate_hardware_config(config)


def test_standalone_bundle_matches_sources_and_has_no_local_imports():
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('builder', root / 'tools/build_hardware_drcda_v2.py')
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    assert builder.OUTPUT.read_text() == builder.render()
    for node in ast.walk(ast.parse(builder.OUTPUT.read_text())):
        if isinstance(node, ast.ImportFrom):
            assert not node.level
            assert not (node.module or '').startswith(('controllers', 'tools', 'hnuter_'))


def test_standalone_does_not_inherit_always_on_heartbeat_flight_instructions():
    source = Path(hw.__file__).read_text()
    assert '节点常驻发送心跳' not in source
    assert 'no heartbeat or actuator output in monitor mode' in source


def test_standalone_paths_do_not_require_repository_depth(monkeypatch, tmp_path):
    monkeypatch.setattr(hw, '__file__', '/tmp/controller.py')
    monkeypatch.chdir(tmp_path)
    assert hw.workspace_root() == tmp_path


@pytest.mark.parametrize('kind', ['motors', 'servos'])
@pytest.mark.parametrize('bad', ['older_timestamp', 'nonfinite', 'duplicate'])
def test_command_echo_cannot_abort_active_control(controller, kind, bad):
    activate(controller)
    before = controller.drcda.command.copy()
    count = 5 if kind == 'motors' else 4
    controls = [.5] * count
    stamp = 999_000 if bad == 'older_timestamp' else 1_020_000
    if bad == 'duplicate':
        stamp = 1_000_000
    if bad == 'nonfinite':
        controls[0] = float('nan')
    callback = getattr(controller, f'px4_actuator_{kind}_callback')
    callback(Msg(timestamp=stamp, control=controls, reversible_flags=16))
    assert controller._hardware_control_active
    assert not controller._hardware_fault
    np.testing.assert_allclose(controller.drcda.command, before)
    controller.offboard_control_mode_pub.publish.reset_mock()
    controller.offboard_startup_tick()
    controller.offboard_control_mode_pub.publish.assert_called()
    # Valid flight-state updates still produce bounded control after the echo.
    controller.local_position_callback(Msg(timestamp=1_020_000, xy_valid=True, z_valid=True,
        v_xy_valid=True, v_z_valid=True, x=2., y=3., z=-4., vx=0., vy=0., vz=0.))
    controller.attitude_callback(Msg(timestamp=1_020_000, q=[1., 0., 0., 0.]))
    assert controller.drcda.last_result.status == 'solved'
    assert not controller._hardware_fault
    if bad != 'duplicate':
        events = [json.loads(line) for line in controller._event_path.read_text().splitlines()]
        event = next(e for e in events if e['event'] == 'actuator_echo_dropped')
        assert event['details']['topic'] == kind
        assert event['control_active']
        assert event['details']['reason'] in ('older_command_echo', 'nonfinite_or_missing_channels')


def test_nonfinite_actuator_echo_is_not_a_handover_veto(controller):
    controller._firmware_parameters_verified = True
    controller._output_enabled = True
    feed(controller)
    controller.px4_actuator_motors_callback(Msg(timestamp=1_020_000, control=[float('nan')] * 5))
    controller.control_mode_callback(Msg(timestamp=1_030_000, flag_armed=True, flag_control_offboard_enabled=True))
    assert controller._hardware_control_active
    assert not controller._hardware_fault


def test_old_reversible_flags_do_not_veto_entry_and_output_uses_configured_flags(controller):
    controller._firmware_parameters_verified = True
    controller._output_enabled = True
    feed(controller)
    controller.px4_out_motor_reversible_flags = 0
    controller.control_mode_callback(Msg(timestamp=1_010_000, flag_armed=True, flag_control_offboard_enabled=True))
    assert controller._hardware_control_active
    assert controller.actuator_motors_pub.publish.call_args.args[0].reversible_flags == 16


def test_handover_and_fault_events_preserve_actual_command(controller):
    activate(controller)
    controller._latch_fault('test fault')
    events = [json.loads(line) for line in controller._event_path.read_text().splitlines()]
    assert [e['event'] for e in events] == [
        'handover_begin', 'handover_command_published', 'position_handover_requested',
        'fault', 'control_session_end']
    assert len(events[0]['details']['captured_motor_controls']) == 5
    assert events[0]['details']['captured_motor_controls'][4] > .3
    assert events[3]['details']['reason'] == 'test fault'
    assert events[3]['control_active']
    assert len(events[3]['last_published_motors']) == 5


def test_parameter_readback_is_not_an_arming_offboard_gate(controller):
    controller._output_enabled = True
    feed(controller, offboard=False)
    controller.offboard_startup_tick()
    assert not controller._hardware_control_active
    assert 'Armed + Offboard' in controller._gate_reason
    controller.actuator_motors_pub.publish.assert_not_called()
    controller.offboard_control_mode_pub.publish.assert_called()
    controller.control_mode_callback(Msg(timestamp=1_010_000, flag_armed=True,
                                         flag_control_offboard_enabled=True))
    assert controller._hardware_control_active
    assert not controller._firmware_parameters_verified


@pytest.mark.parametrize('offboard', [False, True])
def test_disarmed_standby_heartbeat_does_not_require_armed_handover_topics(controller, offboard):
    controller._firmware_parameters_verified = True  # Synthetic offline fixture.
    controller._output_enabled = True
    feed(controller, armed=False, offboard=offboard)
    for topic in ('motors', 'servos', 'attitude_reference'):
        controller._received.pop(topic)
    controller.offboard_startup_tick()
    controller.offboard_control_mode_pub.publish.assert_called()
    assert not controller._hardware_control_active
    controller.actuator_motors_pub.publish.assert_not_called()
    controller.actuator_servos_pub.publish.assert_not_called()
    assert controller.vehicle_command_pub is None


def test_armed_standby_heartbeat_is_independent_of_handover_readiness(controller):
    controller._firmware_parameters_verified = True
    controller._output_enabled = True
    feed(controller, armed=True, offboard=False)
    controller._received.pop('attitude_reference')
    controller.offboard_startup_tick()
    controller.offboard_control_mode_pub.publish.assert_called()
    controller.actuator_motors_pub.publish.assert_not_called()
    assert 'Armed + Offboard' in controller._gate_reason
    assert not controller._heartbeat_block_reason()
    controller.offboard_control_mode_pub.publish.reset_mock()
    controller.control_mode_callback(Msg(timestamp=1_010_000, flag_armed=True,
                                         flag_control_offboard_enabled=True))
    controller.offboard_startup_tick()
    assert not controller._hardware_fault
    assert controller._hardware_control_active
    controller.offboard_control_mode_pub.publish.assert_called()


def test_no_flight_controller_connected_still_sends_heartbeat_only(controller):
    controller._output_enabled = True
    for _ in range(5):
        controller.offboard_startup_tick()
        controller.control_loop()
    assert controller.offboard_control_mode_pub.publish.call_count == 5
    msg = controller.offboard_control_mode_pub.publish.call_args.args[0]
    assert msg.timestamp > 0
    assert msg.direct_actuator
    assert not any((msg.position, msg.velocity, msg.acceleration, msg.attitude,
                    msg.body_rate, msg.thrust_and_torque))
    assert not controller._hardware_control_active
    assert not controller._hardware_fault
    assert not controller._firmware_parameters_verified
    for pub in (controller.actuator_motors_pub, controller.actuator_servos_pub,
                controller.vehicle_thrust_setpoint_pub, controller.vehicle_torque_setpoint_pub,
                controller.trajectory_setpoint_pub):
        pub.publish.assert_not_called()
    assert controller.vehicle_command_pub is None


@pytest.mark.parametrize('armed,offboard', [(False, False), (True, False), (False, True)])
def test_standby_never_sends_idle_or_control_commands(controller, armed, offboard):
    controller._output_enabled = True
    controller._firmware_parameters_verified = True
    feed(controller, armed=armed, offboard=offboard)
    controller._received.clear()
    controller.offboard_startup_tick()
    controller._publish_normalized_direct_actuator_setpoint([.5]*5, [0.]*4)
    controller.offboard_control_mode_pub.publish.assert_called()
    controller.actuator_motors_pub.publish.assert_not_called()
    controller.actuator_servos_pub.publish.assert_not_called()
    assert not controller._hardware_fault


def test_enabled_constructor_can_wait_for_readback_without_flight_controller(controller, monkeypatch, tmp_path):
    monkeypatch.setenv('HNUTER_LOG_DIR', str(tmp_path/'standby'))
    monkeypatch.setenv('HNUTER_TUNING_FILE', str(tmp_path/'standby'/'tuning.json'))
    monkeypatch.setenv('HNUTER_HARDWARE_DRCDA_ENABLE', '1')
    node = hw.HnuterHardwareDRCDAV2Controller()
    try:
        node.offboard_startup_tick()
        node.offboard_control_mode_pub.publish.assert_called_once()
        node.actuator_motors_pub.publish.assert_not_called()
        node.actuator_servos_pub.publish.assert_not_called()
        assert not node._firmware_parameters_verified
        assert not node._hardware_fault
    finally:
        node._diagnostic_file.close()


def test_bounded_test_uses_configured_profile_without_claiming_live_readback(controller):
    controller._operator_bounded_test = True
    controller._output_enabled = True
    feed(controller, armed=False, offboard=False)
    controller.offboard_startup_tick()
    controller.offboard_control_mode_pub.publish.assert_called()
    assert not controller._firmware_parameters_verified
    assert not controller._hardware_control_active
    controller.actuator_motors_pub.publish.assert_not_called()
    feed(controller, stamp=1_020_000, armed=True, offboard=False)
    assert not controller._hardware_control_active
    controller.control_mode_callback(Msg(timestamp=1_025_000, flag_armed=True,
                                         flag_control_offboard_enabled=True))
    assert controller._hardware_control_active
    assert not controller._firmware_parameters_verified
    controller.actuator_motors_pub.publish.assert_called()
    controller.control_mode_callback(Msg(timestamp=1_030_000, flag_armed=True,
                                         flag_control_offboard_enabled=False))
    assert not controller._hardware_control_active


def test_bounded_output_constructor_records_operator_policy(controller, monkeypatch, tmp_path):
    monkeypatch.setenv('HNUTER_LOG_DIR', str(tmp_path/'bounded'))
    monkeypatch.setenv('HNUTER_TUNING_FILE', str(tmp_path/'bounded'/'tuning.json'))
    monkeypatch.setenv('HNUTER_HARDWARE_DRCDA_ENABLE', '1')
    monkeypatch.setenv('HNUTER_HARDWARE_BOUNDED_TEST', '1')
    node = hw.HnuterHardwareDRCDAV2Controller()
    try:
        snapshot = json.loads((tmp_path/'bounded'/'config'/'drcda_v2.runtime.json').read_text())
        assert snapshot['output_enabled']
        assert snapshot['operator_bounded_test']
        assert snapshot['profile_accepted_without_live_readback']
        assert not snapshot['firmware_parameters_verified']
        node.actuator_motors_pub.publish.assert_not_called()
        assert node.vehicle_command_pub is None
    finally:
        node._diagnostic_file.close()


def test_mismatched_parameter_readback_is_diagnostic_at_startup(controller, monkeypatch, tmp_path):
    path = tmp_path/'mismatched.json'
    path.write_text(json.dumps({'parameters': {'HNTR_MASS': 100.}}))
    monkeypatch.setenv('HNUTER_HARDWARE_FIRMWARE_PARAMS', str(path))
    monkeypatch.setenv('HNUTER_LOG_DIR', str(tmp_path/'parameter_diagnostic'))
    monkeypatch.setenv('HNUTER_HARDWARE_DRCDA_ENABLE', '1')
    node = hw.HnuterHardwareDRCDAV2Controller()
    try:
        assert not node._firmware_parameters_verified
        feed(node)
        node.control_mode_callback(Msg(timestamp=1_010_000, flag_armed=True,
                                       flag_control_offboard_enabled=True))
        assert node._hardware_control_active
        assert not node._hardware_fault
    finally:
        node._diagnostic_file.close()


def test_wrench_difference_does_not_veto_entry_but_active_input_timeout_withdraws(controller):
    controller._operator_bounded_test = True
    controller._output_enabled = True
    feed(controller)
    controller.px4_out_motor_cmd[:5] = [.4]*4+[.6]
    controller.offboard_startup_tick()
    controller.control_mode_callback(Msg(timestamp=1_010_000, flag_armed=True,
                                         flag_control_offboard_enabled=True))
    assert controller._hardware_control_active
    controller._received['rates'] -= 1
    controller.offboard_control_mode_pub.publish.reset_mock()
    controller.offboard_startup_tick()
    assert 'rates' in controller._gate_reason
    controller.offboard_control_mode_pub.publish.assert_called()
    controller._fault_position_pub.publish.assert_called()
    assert 'rates' in controller._hardware_fault


@pytest.mark.parametrize('bounded', [False, True])
def test_measured_pose_outside_manual_range_is_captured_without_clipping(controller, bounded):
    controller._operator_bounded_test = bounded
    controller._firmware_parameters_verified = True
    controller._output_enabled = True
    feed(controller)
    angle = np.radians(-24.)
    controller.attitude_callback(Msg(timestamp=1_004_000,
        q=[np.cos(angle/2), 0., np.sin(angle/2), 0.]))
    controller.hnuter_attitude_reference_callback(Msg(timestamp=1_005_000,
        name='hntr_att', x=0., y=10., z=0.))
    controller.offboard_startup_tick()
    controller.offboard_control_mode_pub.publish.assert_called()
    assert not controller._hardware_control_active
    controller.control_mode_callback(Msg(timestamp=1_010_000, flag_armed=True,
                                         flag_control_offboard_enabled=True))
    assert controller._hardware_control_active
    assert controller.hardware_handover_duration_s == .5
    assert controller.target_attitude[1] == pytest.approx(np.radians(24.))
    controller._hardware_handover_blend = 1.
    command = controller._zero_manual_cmd()
    controller.rc_input.get_velocity_commands = Mock(return_value=command)
    controller.update_trajectory(.1, .1)
    assert np.degrees(controller.target_attitude[1]) == pytest.approx(24.)
    # The measured-pose reference must not jump to the ordinary manual limit.
    command['pitch_rate'] = np.radians(5.)
    controller.update_trajectory(.2, .1)
    assert np.degrees(controller.target_attitude[1]) == pytest.approx(24.)
    command['pitch_rate'] = np.radians(-5.)
    controller.update_trajectory(.3, .1)
    assert np.degrees(controller.target_attitude[1]) == pytest.approx(23.5)


def test_no_reference_or_model_consistency_checks_at_manual_entry(controller):
    controller._output_enabled = True
    controller._firmware_parameters_verified = True
    feed(controller)
    controller.hnuter_attitude_reference_callback(Msg(timestamp=1_005_000,
        name='hntr_att', x=0., y=-10., z=0.))
    assert not controller._readiness_reason(handover=True)
    assert not controller._operator_bounded_test
    controller.px4_out_motor_cmd[:5] = [.4]*4+[.6]
    assert not controller._readiness_reason(handover=True)
    controller.hnuter_attitude_reference_callback(Msg(timestamp=1_006_000,
        name='hntr_att', x=0., y=-24., z=0.))
    assert not controller._readiness_reason(handover=True)
    assert controller._handover_wrench_diagnostics is None
    controller._received['rates'] -= 1
    assert 'rates' in controller._readiness_reason(handover=True)
    controller.actuator_motors_pub.publish.assert_not_called()


def test_manual_handover_policy_is_default_and_recorded(controller, monkeypatch, tmp_path):
    assert controller._ignore_handover_attitude_error
    monkeypatch.setenv('HNUTER_LOG_DIR', str(tmp_path/'ignore_attitude'))
    node = hw.HnuterHardwareDRCDAV2Controller()
    try:
        assert node._ignore_handover_attitude_error
        snapshot = json.loads((tmp_path/'ignore_attitude'/'config'/'drcda_v2.runtime.json').read_text())
        assert snapshot['ignore_handover_attitude_error']
        assert not snapshot['handover_consistency_enforced']
        assert snapshot['handover_reference'] == 'measured_pose'
        assert snapshot['handover_duration_s'] == .5
        assert not snapshot['operator_bounded_test']
        node.actuator_motors_pub.publish.assert_not_called()
    finally:
        node._diagnostic_file.close()


def test_unused_px4_reference_does_not_change_measured_pose_target(controller):
    controller._operator_bounded_test = True
    controller._output_enabled = True
    feed(controller)
    controller.hnuter_attitude_reference_callback(Msg(timestamp=1_005_000,
        name='hntr_att', x=0., y=-45., z=0.))
    controller.offboard_startup_tick()
    controller.offboard_control_mode_pub.publish.assert_called()
    controller.control_mode_callback(Msg(timestamp=1_010_000, flag_armed=True,
                                         flag_control_offboard_enabled=True))
    assert controller._hardware_control_active
    assert not controller._hardware_fault
    np.testing.assert_allclose(controller.target_attitude[:2], 0.)


def parameter_readback(tuning):
    result = {k: v for k, v in tuning.items() if k.startswith('HNTR_')}
    for group, prefix in (('direct_KR', 'HNTR_ATT_KR'), ('direct_Domega', 'HNTR_ATT_D'),
                          ('direct_attitude_Ki', 'HNTR_ATT_I'), ('direct_tau_limit', 'HNTR_TAU')):
        result.update({prefix + '_' + axis: value
                       for axis, value in zip(('R', 'P', 'Y'), tuning[group])})
    result.update(THR_MDL_FAC=0., CA_R_REV=16, HNTR_ROLL_SIGN=1.,
                  CA_SV_TL_COUNT=4, PWM_MAIN_REV=1664)
    for index in range(4):
        result[f'CA_SV_TL{index}_MINA'] = -180.
        result[f'CA_SV_TL{index}_MAXA'] = 180.
        channel = 8+index
        result[f'PWM_MAIN_FUNC{channel}'] = 201+index
        result[f'PWM_MAIN_MIN{channel}'] = 500
        result[f'PWM_MAIN_MAX{channel}'] = 2500
    for channel in range(1,6):
        result[f'PWM_MAIN_FUNC{channel}'] = 100+channel
        result[f'PWM_MAIN_MIN{channel}'] = 1000
        result[f'PWM_MAIN_MAX{channel}'] = 2000
    return result


@pytest.mark.parametrize('failure', [None, 'mismatch', 'missing', 'insufficient_trim_authority',
                                    'legacy_hover_anchor', 'motor_pwm', 'servo_travel'])
def test_parameter_snapshot_validation(controller, monkeypatch, tmp_path, failure):
    tuning = json.loads(Path(controller.tuning_path).read_text())
    tuning['direct_tau_limit'][1] = 10.
    if failure == 'insufficient_trim_authority':
        tuning['direct_tau_limit'][1] = 1.
    controller._apply_tuning(tuning)
    Path(controller.tuning_path).write_text(json.dumps(tuning))
    parameters = parameter_readback(tuning)
    if failure == 'mismatch':
        parameters['HNTR_CG_X'] = 0.
    elif failure == 'missing':
        del parameters['HNTR_MOT_HOV']
    elif failure == 'legacy_hover_anchor':
        parameters['HNTR_MOT_HOV'] = .5
    elif failure == 'motor_pwm':
        parameters['PWM_MAIN_MIN1'] = 500
    elif failure == 'servo_travel':
        parameters['CA_SV_TL2_MAXA'] = 90.
    path = tmp_path / 'synthetic_readback.json'
    path.write_text(json.dumps({'parameters': parameters}))
    monkeypatch.setenv('HNUTER_HARDWARE_FIRMWARE_PARAMS', str(path))
    if failure:
        with pytest.raises(ValueError):
            controller._verify_firmware_parameter_snapshot()
        assert not controller._firmware_parameters_verified
    else:
        controller._verify_firmware_parameter_snapshot()
        assert controller._firmware_parameters_verified


def test_gravity_and_bias_enter_frd_torque_before_limit(controller):
    controller.cg_x, controller.cg_z = .055, .01
    controller.R_ned_frd = np.eye(3)
    trim = -controller.cg_x * controller.mass * controller.gravity
    trim -= controller.pitch_torque_bias * 12.78 * .72
    np.testing.assert_allclose(controller._add_firmware_attitude_trim([.1, .2, .3]),
                              [.1, .2 + trim, .3])
    limited = np.clip(controller._add_firmware_attitude_trim([0, -10, 0]), -3, 3)
    mapped = controller._allocator_wrench_from_body_force_torque(np.zeros(3), limited)
    assert controller._apply_firmware_tail_pitch_bias(mapped[4], 12.78, .72, .07, 1) == 3
    # At vertical pitch, CG-X gravity trim vanishes; CG-Z contribution remains.
    controller.R_ned_frd = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
    expected = -.01 * controller.mass * controller.gravity - .07 * 12.78 * .72
    assert controller._add_firmware_attitude_trim(np.zeros(3))[1] == pytest.approx(expected)


def test_between_solves_holds_optimum_without_double_blending(controller):
    activate(controller)
    controller.px4_timestamp = 1_300_000
    controller._drcda_active_call = True
    controller._drcda_current_dt_s = .004
    captured = controller._captured_drcda_state(np.zeros(9))
    controller.last_W = controller.drcda.model.wrench(captured) + [0, 0, 0, 0, .2, 0]
    controller.publish_direct_actuator_setpoint([.5]*4+[.2], .1, .1, 0., 0.)
    held = controller.drcda.held_solution.copy()
    assert np.linalg.norm(held - controller.drcda.command) > 1e-5
    result = controller.drcda.last_result
    controller.px4_timestamp += 4000
    controller.publish_direct_actuator_setpoint([.5]*4+[.2], .1, .1, 0., 0.)
    assert controller.drcda.last_result is result
    np.testing.assert_array_equal(controller.drcda.held_solution, held)
    target = np.array([controller._front_thrust_to_control(held[6]),
                       controller._front_thrust_to_control(held[7]),
                       controller._front_thrust_to_control(held[4]),
                       controller._front_thrust_to_control(held[5]),
                       controller._tail_thrust_to_control(held[8])])
    blend = controller._hardware_handover_blend
    expected = (1-blend)*controller._hardware_handover_motor_start + blend*target
    np.testing.assert_allclose(controller.last_motor_cmd[:5], expected, atol=5e-8)


def test_handover_captures_full_measured_attitude_instead_of_px4_target(controller):
    controller._firmware_parameters_verified = True
    controller._output_enabled = True
    feed(controller)
    q = [.99, .06, -.1, .02]
    q = np.asarray(q)/np.linalg.norm(q)
    controller.attitude_callback(Msg(timestamp=1_004_000, q=q))
    expected = controller._euler_from_rotation_matrix(controller.R)
    expected_rotation = controller.R_ned_frd.copy()
    controller.hnuter_attitude_reference_callback(Msg(timestamp=1_005_000,
        name=list(b'hntr_att\x00\x00'), x=1., y=2., z=0.))
    controller.control_mode_callback(Msg(timestamp=1_010_000, flag_armed=True,
                                         flag_control_offboard_enabled=True))
    assert controller._hardware_control_active
    np.testing.assert_allclose(controller.target_attitude, expected)
    np.testing.assert_allclose(controller.target_R_des_ned_frd, expected_rotation)
    controller.update_trajectory(.01, .01)
    np.testing.assert_allclose(controller.target_attitude, expected)
    # The captured command begins at blend zero and reaches one in 0.5 s.
    start = controller._hardware_handover_start_timestamp_us
    controller.px4_timestamp = start + 250_000
    assert controller._current_hardware_handover_blend() == pytest.approx(.5)
    controller.px4_timestamp = start + 500_000
    assert controller._current_hardware_handover_blend() == pytest.approx(1.)


@pytest.mark.parametrize('reason', ['missing', 'stale', 'wrong_name', 'nonfinite'])
def test_missing_or_invalid_unused_px4_reference_does_not_block_handover(controller, reason):
    controller._firmware_parameters_verified = True
    controller._output_enabled = True
    feed(controller)
    if reason == 'stale':
        controller._received['attitude_reference'] -= 1
    else:
        controller._received.pop('attitude_reference')
        if reason != 'missing':
            controller.hnuter_attitude_reference_callback(Msg(timestamp=1_005_000,
                name='other' if reason == 'wrong_name' else 'hntr_att',
                x=float('nan') if reason == 'nonfinite' else 0., y=0., z=0.))
    controller.offboard_startup_tick()
    controller.offboard_control_mode_pub.publish.assert_called()
    controller.offboard_control_mode_pub.publish.reset_mock()
    controller.control_mode_callback(Msg(timestamp=1_010_000, flag_armed=True,
                                         flag_control_offboard_enabled=True))
    controller.offboard_startup_tick()
    assert not controller._hardware_fault
    assert controller._hardware_control_active
    controller.offboard_control_mode_pub.publish.assert_called()


def test_historical_log7_attitude_gap_no_longer_rejects_manual_handover(controller):
    controller._firmware_parameters_verified = True
    controller._output_enabled = True
    feed(controller)
    controller.attitude_callback(Msg(timestamp=1_005_000,
        q=[-.9990421533584595, .027977408841252327, -.03354637697339058, -.002600375097244978]))
    controller.hnuter_attitude_reference_callback(Msg(timestamp=1_005_001,
        name='hntr_att', x=0., y=-23.972381090404607, z=0.))
    expected = controller._euler_from_rotation_matrix(controller.R)
    controller.control_mode_callback(Msg(timestamp=1_010_000, flag_armed=True,
                                         flag_control_offboard_enabled=True))
    assert controller._hardware_control_active
    assert not controller._hardware_fault
    # Offline callback regression only, not evidence that this state flies well.
    np.testing.assert_allclose(controller.target_attitude, expected)
    np.testing.assert_allclose(controller.actuator_motors_pub.publish.call_args.args[0].control[:5],
                               controller._hardware_handover_motor_start, atol=1e-7)


def test_handover_wrench_difference_does_not_veto_and_first_output_is_continuous(controller):
    controller._firmware_parameters_verified = True
    controller._output_enabled = True
    feed(controller)
    controller.px4_out_motor_cmd[:5] = [.4]*4+[.6]
    controller.offboard_startup_tick()
    controller.control_mode_callback(Msg(timestamp=1_010_000, flag_armed=True,
                                         flag_control_offboard_enabled=True))
    assert controller._hardware_control_active
    np.testing.assert_allclose(controller.actuator_motors_pub.publish.call_args.args[0].control[:5],
                               [.4]*4+[.6], atol=1e-7)
