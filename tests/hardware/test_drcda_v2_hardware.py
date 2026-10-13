"""Offline only: every ROS publisher, timer and subscription is mocked."""
import ast
import copy
import importlib.util
import time
from pathlib import Path
from types import SimpleNamespace as Msg
from unittest.mock import Mock

import numpy as np
import pytest

from controllers.hardware import hnuter_external_direct_drcda_v2_hardware as hw


@pytest.fixture
def controller(monkeypatch, tmp_path):
    for key in list(hw.os.environ):
        if key.startswith('HNUTER_'):
            monkeypatch.delenv(key)
    monkeypatch.setenv('HNUTER_LOG_DIR', str(tmp_path))
    monkeypatch.setattr(hw.Node, '__init__', lambda *a, **k: None)
    monkeypatch.setattr(hw.Node, 'get_logger', lambda *a: Mock())
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
    node.px4_actuator_motors_callback(Msg(timestamp=stamp, control=[.5]*4+[.07]+[float('nan')]*7))
    node.px4_actuator_servos_callback(Msg(timestamp=stamp, control=[.01, .02, .03, .04]+[float('nan')]*4))
    node.status_callback(Msg(timestamp=stamp, arming_state=2 if armed else 1, nav_state=14 if offboard else 2))
    node.control_mode_callback(Msg(timestamp=stamp, flag_armed=armed, flag_control_offboard_enabled=offboard))


def activate(node):
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
def test_stale_data_withdraws_heartbeat_without_zero_motor_command(controller, name):
    activate(controller)
    controller.actuator_motors_pub.publish.reset_mock()
    controller.offboard_control_mode_pub.publish.reset_mock()
    controller._received[name] = time.monotonic() - 5
    controller.offboard_startup_tick()
    assert controller._hardware_fault
    assert not controller._hardware_control_active
    controller.actuator_motors_pub.publish.assert_not_called()
    controller.offboard_control_mode_pub.publish.assert_not_called()


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
    controller.position[:] = [5, 6, 7]
    controller.control_mode_callback(Msg(timestamp=1_030_000, flag_armed=True, flag_control_offboard_enabled=True))
    assert controller._hardware_control_active
    np.testing.assert_allclose(controller.target_position, [5, 6, 0])
    assert controller._z0 == 7


def test_missing_snapshot_blocks_entry(controller):
    controller._output_enabled = True
    feed(controller)
    controller._received.pop('servos')
    controller.control_mode_callback(Msg(timestamp=1_010_000, flag_armed=True, flag_control_offboard_enabled=True))
    assert not controller._hardware_control_active
    controller.actuator_motors_pub.publish.assert_not_called()


def test_rc_loss_fault_is_latched(controller):
    activate(controller)
    controller.rc_input._manual_valid = False
    controller.offboard_startup_tick()
    assert 'RC' in controller._hardware_fault
    feed(controller, stamp=1_030_000, offboard=True)
    controller.offboard_startup_tick()
    assert not controller._hardware_control_active


def test_downward_rc_reference_not_clipped_to_handover_height(controller):
    activate(controller)
    controller._hardware_handover_blend = 1.
    command = controller._zero_manual_cmd()
    command['vz'] = -.3
    controller.rc_input.get_velocity_commands = Mock(return_value=command)
    controller.update_trajectory(.1, .1)
    assert controller.target_position[2] == pytest.approx(-.03)
    controller.position[2] -= .03
    command['vz'] = 0
    controller.update_trajectory(.2, .1)
    assert controller.target_position[2] == pytest.approx(-.03)
    assert controller.target_velocity[2] == 0.


def test_tilt_protection_does_not_depend_on_relative_altitude(controller):
    activate(controller)
    controller.R_ned_frd = np.diag([1., -1., -1.])
    assert controller._direct_safety_violation_reason().startswith('tilt')
    controller.actuator_motors_pub.publish.reset_mock()
    controller._publish_direct_safety_cutoff(0.)
    controller.actuator_motors_pub.publish.assert_not_called()


def test_real_px4_inverted_attitude_is_not_canonicalized(controller):
    controller.attitude_callback(Msg(timestamp=1_000_000, q=[0., 1., 0., 0.]))
    np.testing.assert_allclose(controller.R_ned_frd, np.diag([1., -1., -1.]))


def test_pwm_only_applies_to_servos_and_gear_is_applied_once(controller):
    for normalized, pwm in [(-1., 500.), (0., 1500.), (1., 2500.)]:
        assert controller._normalized_servo_to_expected_pwm_us(normalized) == pwm
    assert controller._secondary_joint_angle_to_normalized(np.pi / 2) == 1
    hover = controller.mass * controller.gravity / 4
    assert controller._front_thrust_to_control(hover) == pytest.approx(.5)
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


def test_control_sample_gap_latches_fault(controller):
    activate(controller)
    controller.px4_timestamp = 1_060_000
    controller.actuator_motors_pub.publish.reset_mock()
    controller.control_loop()
    assert 'sample gap' in controller._hardware_fault
    controller.actuator_motors_pub.publish.assert_not_called()


def test_lissajous_has_no_absolute_height_clamp(controller):
    activate(controller)
    controller._tasks_enabled = True
    controller.position[2] -= 2.
    controller._start_auto_trajectory('lissajous', 0.)
    position, _, _, _ = controller._lissajous_reference(0.)
    assert position[2] == pytest.approx(-2.)


def test_safety_gate_withdraws_output_before_publishing(controller):
    activate(controller)
    controller.R_ned_frd = np.diag([1., -1., -1.])
    controller.actuator_motors_pub.publish.reset_mock()
    controller.offboard_control_mode_pub.publish.reset_mock()
    controller.control_loop()
    controller.offboard_startup_tick()
    assert 'envelope' in controller._hardware_fault
    controller.actuator_motors_pub.publish.assert_not_called()
    controller.offboard_control_mode_pub.publish.assert_not_called()


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


def test_standalone_paths_do_not_require_repository_depth(monkeypatch, tmp_path):
    monkeypatch.setattr(hw, '__file__', '/tmp/controller.py')
    monkeypatch.chdir(tmp_path)
    assert hw.workspace_root() == tmp_path
