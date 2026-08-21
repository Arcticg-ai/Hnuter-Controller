import inspect
import math
import types

import numpy as np
import pytest

import hnuter_external_controller_px4_position_iebc_hardware as module
from hnuter_external_controller_px4_position_hardware import HnuterController
from hnuter_external_controller_px4_position_iebc_hardware import (
    HnuterActuatorForceEstimator,
    HnuterIebcOffboardController,
    InteractionEnergyBarrierFilter,
    NominalReference,
    Px4TrajectoryCodec,
)


def _trajectory_message(**overrides):
    values = {
        'position': [1.0, 2.0, -3.0],
        'velocity': [0.1, 0.2, -0.3],
        'acceleration': [0.4, 0.5, -0.6],
        'yaw': 0.25,
    }
    values.update(overrides)
    return types.SimpleNamespace(**values)


def test_offboard_gateway_reuses_validated_hardware_controller():
    assert issubclass(HnuterIebcOffboardController, HnuterController)


def test_hardware_gateway_has_no_vehicle_command_or_direct_actuator_path():
    source = inspect.getsource(module)
    assert 'from px4_msgs.msg import VehicleCommand' not in source
    assert "'/fmu/in/vehicle_command'" not in source
    assert "'/fmu/in/actuator_motors'" not in source
    assert "'/fmu/in/actuator_servos'" not in source


def test_simulation_and_hardware_embed_the_same_iebc_core():
    from hnuter_external_controller_px4_position_iebc_simulation import (
        InteractionEnergyBarrierFilter as SimulationEnergyBarrierFilter,
    )

    assert inspect.getsource(InteractionEnergyBarrierFilter) == inspect.getsource(
        SimulationEnergyBarrierFilter)


def test_topic_contract_separates_nominal_reference_and_actual_wrench():
    assert HnuterIebcOffboardController.DEFAULT_NOMINAL_TOPIC.endswith(
        '/trajectory_setpoint')
    assert HnuterIebcOffboardController.DEFAULT_MOTORS_TOPIC == (
        '/fmu/out/actuator_motors')
    assert HnuterIebcOffboardController.DEFAULT_SERVOS_TOPIC == (
        '/fmu/out/actuator_servos')
    assert HnuterIebcOffboardController.DEFAULT_WRENCH_TOPIC.endswith(
        '/actuator_wrench')
    assert HnuterIebcOffboardController.DEFAULT_RECOVERY_TOPIC.endswith(
        '/recovery')
    init_source = inspect.getsource(HnuterIebcOffboardController.__init__)
    assert 'ActuatorMotors, self.motors_topic' in init_source
    assert 'ActuatorServos, self.servos_topic' in init_source


def test_task_switch_default_does_not_conflict_with_firmware_aux3():
    assert (
        HnuterIebcOffboardController.DEFAULT_TASK_RC_FUNCTION
        == module.RcChannels.FUNCTION_AUX_4
    )


def test_px4_trajectory_codec_converts_absolute_ned_to_enu():
    reference = Px4TrajectoryCodec.decode(
        _trajectory_message(), received_monotonic_s=10.0)

    np.testing.assert_allclose(reference.position_enu, [2.0, 1.0, 3.0])
    np.testing.assert_allclose(reference.velocity_enu, [0.2, 0.1, 0.3])
    np.testing.assert_allclose(reference.acceleration_enu, [0.5, 0.4, 0.6])
    assert reference.yaw_enu == pytest.approx(math.pi / 2.0 - 0.25)
    assert reference.received_monotonic_s == 10.0


def test_px4_trajectory_codec_maps_all_nan_acceleration_to_zero_feedforward():
    reference = Px4TrajectoryCodec.decode(
        _trajectory_message(acceleration=[math.nan] * 3))
    np.testing.assert_allclose(reference.acceleration_enu, np.zeros(3))


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('position', [0.0, math.nan, 0.0]),
        ('velocity', [0.0, 0.0, math.inf]),
        ('acceleration', [math.nan, 0.0, math.nan]),
        ('yaw', math.nan),
    ],
)
def test_px4_trajectory_codec_rejects_partial_or_nonfinite_commands(field, value):
    with pytest.raises(ValueError):
        Px4TrajectoryCodec.decode(_trajectory_message(**{field: value}))


@pytest.mark.parametrize('frame_id', ['enu', 'map', 'world', '/world_enu'])
def test_actuator_wrench_contract_accepts_explicit_enu_world_frames(frame_id):
    assert HnuterIebcOffboardController._wrench_frame_is_enu(frame_id)


@pytest.mark.parametrize('frame_id', ['', 'base_link', 'ned', 'frd'])
def test_actuator_wrench_contract_rejects_ambiguous_or_body_frames(frame_id):
    assert not HnuterIebcOffboardController._wrench_frame_is_enu(frame_id)


def test_actuator_command_model_reconstructs_hover_force_in_body_flu():
    estimator = HnuterActuatorForceEstimator()

    force = estimator.estimate_body_force_flu(
        [0.5, 0.5, 0.5, 0.5, 0.0],
        [0.0, 0.0, 0.0, 0.0],
    )

    np.testing.assert_allclose(force, [0.0, 0.0, 4.5 * 9.81], atol=1e-9)


def test_actuator_command_model_applies_primary_tilt_and_secondary_gear():
    estimator = HnuterActuatorForceEstimator()

    primary_force = estimator.estimate_body_force_flu(
        [0.5, 0.5, 0.5, 0.5, 0.0],
        [0.5, 0.5, 0.0, 0.0],
    )
    secondary_force = estimator.estimate_body_force_flu(
        [0.5, 0.5, 0.5, 0.5, 0.0],
        [0.0, 0.0, 1.0, 1.0],
    )

    np.testing.assert_allclose(primary_force, [4.5 * 9.81, 0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(secondary_force, [0.0, -4.5 * 9.81, 0.0], atol=1e-9)


def test_actuator_command_model_reconstructs_signed_tail_force():
    estimator = HnuterActuatorForceEstimator()

    force = estimator.estimate_body_force_flu(
        [0.0, 0.0, 0.0, 0.0, -0.5],
        [0.0, 0.0, 0.0, 0.0],
    )

    np.testing.assert_allclose(force, [0.0, 0.0, -85.48 * 0.25], atol=1e-9)


def test_actuator_command_model_rejects_missing_or_nonfinite_required_channels():
    estimator = HnuterActuatorForceEstimator()
    with pytest.raises(ValueError):
        estimator.estimate_body_force_flu(
            [0.4, 0.4, 0.4, 0.4], [0.0, 0.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        estimator.estimate_body_force_flu(
            [0.4, 0.4, 0.4, 0.4, math.nan], [0.0, 0.0, 0.0, 0.0])


def test_selected_px4_output_force_is_rotated_from_body_flu_to_world_enu():
    yaw = math.pi / 2.0
    node = types.SimpleNamespace(
        actuator_source='px4_outputs',
        iebc=types.SimpleNamespace(wrench_timeout_s=0.2),
        _wrench_age_s=lambda: 0.01,
        actuator_force_estimator=HnuterActuatorForceEstimator(),
        _motor_controls=np.array([0.5, 0.5, 0.5, 0.5, 0.0]),
        _servo_controls=np.array([0.5, 0.5, 0.0, 0.0]),
        R=np.array([
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]),
    )

    force = HnuterIebcOffboardController._fresh_external_force(node)

    np.testing.assert_allclose(force, [0.0, 4.5 * 9.81, 0.0], atol=1e-9)


def test_selected_external_wrench_is_already_world_enu():
    expected = np.array([3.0, -2.0, 1.0])
    node = types.SimpleNamespace(
        actuator_source='external_wrench',
        iebc=types.SimpleNamespace(wrench_timeout_s=0.2),
        _wrench_age_s=lambda: 0.01,
        _external_force_enu=expected,
    )

    force = HnuterIebcOffboardController._fresh_external_force(node)

    np.testing.assert_allclose(force, expected)
    assert force is not expected


def test_initial_topic_reference_must_start_near_measured_position():
    reasons = []
    node = types.SimpleNamespace(
        _topic_reference_active=False,
        initial_command_radius_m=0.75,
        position=np.zeros(3),
        _latch_current_hold=reasons.append,
        iebc=types.SimpleNamespace(reset=lambda: None),
        _failsafe_hold_latched=True,
        _failsafe_reason='old',
        get_logger=lambda: types.SimpleNamespace(info=lambda _text: None),
    )
    reference = NominalReference(
        position_enu=np.array([1.0, 0.0, 0.0]),
        velocity_enu=np.zeros(3),
        acceleration_enu=np.zeros(3),
        yaw_enu=0.0,
        received_monotonic_s=0.0,
    )

    accepted = HnuterIebcOffboardController._activate_topic_reference(
        node, reference)

    assert not accepted
    assert reasons and reasons[0].startswith('initial_command_jump_1.000m')


def test_enabled_iebc_holds_instead_of_failing_open_on_stale_wrench():
    reasons = []
    node = types.SimpleNamespace(
        iebc=types.SimpleNamespace(enabled=True),
        _fresh_external_force=lambda: None,
        _latch_current_hold=reasons.append,
    )

    result = HnuterIebcOffboardController._filter_current_reference(
        node, dt=0.02)

    assert not result
    assert reasons == ['actuator_force_stale']


def test_latched_hold_is_reapplied_after_baseline_overwrites_target():
    resets = []
    warnings = []
    node = types.SimpleNamespace(
        _failsafe_hold_latched=False,
        _failsafe_hold_position=np.zeros(3),
        _failsafe_hold_yaw_enu=0.0,
        _failsafe_hold_attitude_enu=np.zeros(3),
        position=np.array([2.0, 3.0, 4.0]),
        _z0=1.0,
        initial_yaw=0.4,
        iebc=types.SimpleNamespace(reset=lambda: resets.append(True)),
        get_logger=lambda: types.SimpleNamespace(warn=warnings.append),
        target_position=np.zeros(3),
        target_velocity=np.ones(3),
        target_acceleration=np.ones(3),
        target_attitude=np.array([0.1, -0.2, 0.7]),
        target_attitude_rate=np.ones(3),
        _failsafe_reason='',
        _topic_reference_active=True,
        _current_yaw_enu=lambda: 0.7,
    )

    HnuterIebcOffboardController._latch_current_hold(node, 'wrench_stale')
    node.target_position = np.array([99.0, 99.0, 99.0])
    node.target_velocity = np.ones(3)
    HnuterIebcOffboardController._latch_current_hold(node, 'wrench_stale')

    np.testing.assert_allclose(node.target_position, [2.0, 3.0, 3.0])
    np.testing.assert_allclose(node.target_velocity, np.zeros(3))
    assert len(resets) == 1
    assert len(warnings) == 1
    np.testing.assert_allclose(node.target_attitude, [0.1, -0.2, 0.7])


def test_disabled_iebc_is_explicit_pass_through():
    node = types.SimpleNamespace(iebc=types.SimpleNamespace(enabled=False))
    assert HnuterIebcOffboardController._filter_current_reference(
        node, dt=0.02)


def test_task_switch_uses_hysteresis():
    node = types.SimpleNamespace(
        _task_switch_value=math.nan,
        _task_switch_received_s=-math.inf,
        _task_switch_high=False,
        task_switch_high_threshold=0.5,
        task_switch_low_threshold=0.0,
    )

    HnuterIebcOffboardController._update_task_switch_sample(node, 0.8, 1.0)
    assert node._task_switch_high
    HnuterIebcOffboardController._update_task_switch_sample(node, 0.2, 2.0)
    assert node._task_switch_high
    HnuterIebcOffboardController._update_task_switch_sample(node, -0.5, 3.0)
    assert not node._task_switch_high
    assert node._task_switch_received_s == 3.0


def test_push_start_latches_position_and_current_heading():
    resets = []
    messages = []
    yaw = math.pi / 2.0
    node = types.SimpleNamespace(
        iebc=types.SimpleNamespace(
            enabled=True,
            valid_configuration=True,
            axis=np.array([1.0, 0.0, 0.0]),
            reset=lambda: resets.append(True),
        ),
        _fresh_external_force=lambda: np.zeros(3),
        position=np.array([1.0, 2.0, 3.0]),
        R=np.array([
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]),
        target_attitude=np.array([0.1, -0.2, yaw]),
        _current_yaw_enu=None,
        TASK_PUSH=HnuterIebcOffboardController.TASK_PUSH,
        _task_switch_armed=True,
        _failsafe_hold_latched=True,
        _failsafe_reason='old',
        get_logger=lambda: types.SimpleNamespace(warn=messages.append),
    )
    node._current_yaw_enu = types.MethodType(
        HnuterIebcOffboardController._current_yaw_enu, node)

    assert HnuterIebcOffboardController._start_rc_push_task(node)

    assert node.task_state == HnuterIebcOffboardController.TASK_PUSH
    np.testing.assert_allclose(node._task_start_position_abs_enu, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(node._task_axis_enu, [0.0, 1.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(node.iebc.axis, node._task_axis_enu)
    assert node._task_start_roll_enu == pytest.approx(0.1)
    assert node._task_start_pitch_enu == pytest.approx(-0.2)
    assert not node._task_switch_armed
    assert resets == [True]


def test_rejected_task_start_requires_switch_low_before_retry():
    warnings = []
    node = types.SimpleNamespace(
        iebc=types.SimpleNamespace(enabled=False, valid_configuration=False),
        _task_switch_armed=True,
        _failsafe_reason='',
        get_logger=lambda: types.SimpleNamespace(warn=warnings.append),
    )

    assert not HnuterIebcOffboardController._start_rc_push_task(node)
    assert not node._task_switch_armed
    assert node._failsafe_reason.endswith('disabled_or_invalid')


def test_push_reference_is_acceleration_limited_from_latched_start():
    node = types.SimpleNamespace(
        task_state=HnuterIebcOffboardController.TASK_PUSH,
        TASK_PUSH=HnuterIebcOffboardController.TASK_PUSH,
        _task_reference_speed_mps=0.0,
        _task_reference_distance_m=0.0,
        task_max_push_distance_m=3.0,
        _task_start_position_abs_enu=np.array([1.0, 2.0, 3.0]),
        _task_axis_enu=np.array([1.0, 0.0, 0.0]),
        _task_start_yaw_enu=0.3,
        _task_start_roll_enu=0.1,
        _task_start_pitch_enu=-0.2,
        _z0=1.0,
        manual_des_pos=np.zeros(3),
        manual_des_yaw=0.0,
        _slew_scalar=HnuterIebcOffboardController._slew_scalar,
    )

    HnuterIebcOffboardController._set_rc_task_reference(
        node, dt=1.0, target_speed_mps=0.2, accel_limit_mps2=0.05)

    assert node._task_reference_speed_mps == pytest.approx(0.05)
    assert node._task_reference_distance_m == pytest.approx(0.025)
    np.testing.assert_allclose(node.target_position, [1.025, 2.0, 2.0])
    np.testing.assert_allclose(node.target_velocity, [0.05, 0.0, 0.0])
    np.testing.assert_allclose(node.target_acceleration, [0.05, 0.0, 0.0])
    np.testing.assert_allclose(node.target_attitude, [0.1, -0.2, 0.3])


def test_switch_cancel_changes_push_to_return_without_resetting_speed():
    warnings = []
    node = types.SimpleNamespace(
        task_state=HnuterIebcOffboardController.TASK_PUSH,
        TASK_RETURN=HnuterIebcOffboardController.TASK_RETURN,
        _task_reference_speed_mps=0.05,
        _task_return_settle_s=1.0,
        _task_switch_armed=True,
        _task_transition_reason='',
        get_logger=lambda: types.SimpleNamespace(warn=warnings.append),
    )

    HnuterIebcOffboardController._begin_task_return(
        node, 'task_switch_low')

    assert node.task_state == HnuterIebcOffboardController.TASK_RETURN
    assert node._task_reference_speed_mps == pytest.approx(0.05)
    assert node._task_return_settle_s == 0.0
    assert not node._task_switch_armed
    assert node._task_transition_reason == 'task_switch_low'


def test_return_keeps_retreating_until_virtual_reference_reaches_start():
    node = types.SimpleNamespace(
        _task_axis_enu=np.array([1.0, 0.0, 0.0]),
        position=np.zeros(3),
        _task_start_position_abs_enu=np.zeros(3),
        _task_reference_distance_m=0.4,
        task_return_kp=0.8,
        task_return_speed_mps=0.25,
    )

    target_speed = HnuterIebcOffboardController._task_return_target_speed(node)

    assert target_speed == pytest.approx(-0.25)


def test_return_speed_accounts_for_physical_excursion_beyond_reference():
    node = types.SimpleNamespace(
        _task_axis_enu=np.array([1.0, 0.0, 0.0]),
        position=np.array([0.3, 0.0, 0.0]),
        _task_start_position_abs_enu=np.zeros(3),
        _task_reference_distance_m=0.1,
        task_return_kp=0.5,
        task_return_speed_mps=0.25,
    )

    target_speed = HnuterIebcOffboardController._task_return_target_speed(node)

    assert target_speed == pytest.approx(-0.15)


def test_return_completion_restores_manual_reference_without_attitude_step():
    messages = []
    node = types.SimpleNamespace(
        _task_start_position_abs_enu=np.array([1.0, 2.0, 3.0]),
        _task_start_yaw_enu=0.3,
        _task_start_roll_enu=0.1,
        _task_start_pitch_enu=-0.2,
        _z0=1.0,
        iebc=types.SimpleNamespace(reset=lambda: None),
        TASK_MANUAL=HnuterIebcOffboardController.TASK_MANUAL,
        _task_switch_armed=True,
        _task_reference_distance_m=0.2,
        _task_reference_speed_mps=-0.1,
        _task_return_settle_s=0.5,
        _task_transition_reason='',
        rc_input=types.SimpleNamespace(
            filtered_cmds={'vx_b': 1.0},
            _zero_commands=lambda: {'vx_b': 0.0},
        ),
        get_logger=lambda: types.SimpleNamespace(info=messages.append),
    )

    HnuterIebcOffboardController._finish_task_return(node)

    assert node.task_state == HnuterIebcOffboardController.TASK_MANUAL
    np.testing.assert_allclose(node.manual_des_pos, [1.0, 2.0, 2.0])
    np.testing.assert_allclose(node.target_attitude, [0.1, -0.2, 0.3])
    assert node.rc_input.filtered_cmds == {'vx_b': 0.0}
    assert not node._task_switch_armed
