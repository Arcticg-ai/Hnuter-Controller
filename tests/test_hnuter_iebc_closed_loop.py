import math
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from hnuter_external_controller_px4_position_hardware_iebc_closed_loop import (
    InteractionEnergyBarrierFilter,
)


def _configured_filter(monkeypatch, **overrides):
    values = {
        'HNUTER_IEBC_ENABLE': '1',
        'HNUTER_IEBC_MASS_KG': '4.5',
        'HNUTER_IEBC_LAMBDA_BAR_KG': '4.5',
        'HNUTER_IEBC_E_MAX_J': '1.2',
        'HNUTER_IEBC_AXIS_X': '1.0',
        'HNUTER_IEBC_AXIS_Y': '0.0',
        'HNUTER_IEBC_AXIS_Z': '0.0',
        'HNUTER_IEBC_KC_NPM': '11.25',
        'HNUTER_IEBC_DC_NSPM': '11.25',
        'HNUTER_IEBC_CBF_GAMMA': '4.0',
        'HNUTER_IEBC_MAX_REF_SPEED_MPS': '0.12',
        'HNUTER_IEBC_MAX_REF_ACCEL_MPS2': '3.0',
        'HNUTER_IEBC_POWER_MARGIN_W': '0.0',
        'HNUTER_IEBC_FORCE_ERROR_BOUND_N': '0.0',
        'HNUTER_IEBC_RESIDUAL_POWER_BOUND_W': '0.0',
        'HNUTER_IEBC_STORAGE_INITIAL_J': '0.0',
        'HNUTER_IEBC_ENERGY_RESERVE_J': '0.0',
        'HNUTER_IEBC_WRENCH_SOURCE': 'proxy',
    }
    values.update({key: str(value) for key, value in overrides.items()})
    for key, value in values.items():
        monkeypatch.setenv(key, value)
    return InteractionEnergyBarrierFilter()


def test_closed_loop_configuration_requires_equivalent_stiffness(monkeypatch):
    barrier = _configured_filter(monkeypatch, HNUTER_IEBC_KC_NPM='0.0')

    assert barrier.enabled
    assert not barrier.valid_configuration
    assert not barrier.debug['enabled']


def test_reference_rate_qp_projects_onto_barrier_half_space(monkeypatch):
    barrier = _configured_filter(monkeypatch)

    safe, infeasible, slack = barrier._project_reference_velocity(
        v_task=0.10, v_prev=0.0, dt=1.0, g_e=2.0, p_allow=0.04)

    assert safe == pytest.approx(0.02)
    assert not infeasible
    assert slack == 0.0


def test_reference_rate_qp_reports_rate_limited_infeasibility(monkeypatch):
    barrier = _configured_filter(
        monkeypatch, HNUTER_IEBC_MAX_REF_ACCEL_MPS2='0.05')

    safe, infeasible, slack = barrier._project_reference_velocity(
        v_task=0.10, v_prev=0.10, dt=0.02, g_e=2.0, p_allow=-0.20)

    assert safe == pytest.approx(0.099)
    assert infeasible
    assert slack == pytest.approx(0.398)


def test_blocked_contact_caps_virtual_spring_energy(monkeypatch):
    barrier = _configured_filter(monkeypatch)
    dt = 0.02
    push_speed = 0.035
    measured_position = np.zeros(3)
    measured_velocity = np.zeros(3)
    gravity_compensating_force = np.array([0.0, 0.0, barrier.mass * barrier.g])

    for step in range(1500):
        nominal_position = np.array([push_speed * dt * step, 0.0, 0.0])
        barrier.filter_reference(
            dt=dt,
            measured_position_enu=measured_position,
            measured_velocity_enu=measured_velocity,
            nominal_position_enu=nominal_position,
            nominal_velocity_enu=np.array([push_speed, 0.0, 0.0]),
            nominal_acceleration_enu=np.zeros(3),
            actuator_force_enu=gravity_compensating_force,
        )

    certified_displacement = math.sqrt(2.0 * barrier.e_max / barrier.k_c)
    certified_force = math.sqrt(2.0 * barrier.k_c * barrier.e_max)

    assert barrier.barrier_active
    assert not barrier.infeasible
    assert barrier.safe_s <= certified_displacement + 1e-3
    assert barrier.k_c * barrier.safe_s <= certified_force + 0.02
    assert barrier.debug['h_i'] >= -1e-3


def test_proxy_power_balance_ignores_unmodelled_orthogonal_motion(monkeypatch):
    barrier = _configured_filter(monkeypatch)
    actuator_force = np.array([0.0, 0.0, barrier.mass * barrier.g])

    for vy in (0.0, 0.4, -0.3, 0.2, 0.0):
        barrier._update_environment_storage(
            dt=0.02,
            position_enu=np.array([0.0, 1.0, 2.0]),
            velocity_enu=np.array([0.0, vy, 0.1]),
            actuator_force_enu=actuator_force,
        )

    assert barrier.power_hat == pytest.approx(0.0)
    assert barrier.storage_bound == pytest.approx(0.0)


def test_freeze_environment_storage_keeps_barrier_filter_active(monkeypatch):
    barrier = _configured_filter(monkeypatch)
    barrier.storage_bound = 0.75
    barrier.storage_rate = 2.0
    barrier.freeze_environment_storage()

    barrier._update_environment_storage(
        dt=0.1,
        position_enu=np.array([1.0, 0.0, 0.0]),
        velocity_enu=np.array([0.8, 0.0, 0.0]),
        actuator_force_enu=np.array([20.0, 0.0, 0.0]),
    )

    assert barrier.enabled
    assert not barrier.storage_update_enabled
    assert barrier.storage_bound == pytest.approx(0.75)
    assert barrier.storage_rate == 0.0
    assert barrier.delta_e == pytest.approx(barrier.residual_power_bound_w)

    barrier.filter_reference(
        dt=0.02,
        measured_position_enu=np.zeros(3),
        measured_velocity_enu=np.array([0.2, 0.0, 0.0]),
        nominal_position_enu=np.array([0.5, 0.0, 0.0]),
        nominal_velocity_enu=np.zeros(3),
        nominal_acceleration_enu=np.zeros(3),
        actuator_force_enu=np.array([5.0, 0.0, 0.0]),
    )

    assert barrier.debug['enabled']
    assert not barrier.debug['storage_update_enabled']
    assert barrier.debug['k_i'] > 0.0
    assert barrier.debug['v_c'] > 0.0
    assert barrier.debug['e_i'] == pytest.approx(
        barrier.debug['k_i'] + barrier.debug['v_c'] + 0.75)


def test_resume_environment_storage_reinitializes_power_derivative(monkeypatch):
    barrier = _configured_filter(monkeypatch)
    barrier.h_prev = 42.0
    barrier.freeze_environment_storage()
    barrier.resume_environment_storage()

    assert barrier.storage_update_enabled
    assert barrier.h_prev is None


def test_energy_reserve_tightens_constraint_without_changing_reported_barrier(monkeypatch):
    barrier = _configured_filter(
        monkeypatch,
        HNUTER_IEBC_E_MAX_J='80.0',
        HNUTER_IEBC_ENERGY_RESERVE_J='0.1',
    )
    barrier.filter_reference(
        dt=0.02,
        measured_position_enu=np.zeros(3),
        measured_velocity_enu=np.zeros(3),
        nominal_position_enu=np.zeros(3),
        nominal_velocity_enu=np.zeros(3),
        nominal_acceleration_enu=np.zeros(3),
        actuator_force_enu=np.zeros(3),
    )

    assert barrier.debug['h_i'] == pytest.approx(80.0)
    assert barrier.debug['h_constraint'] == pytest.approx(79.9)
    assert barrier.debug['energy_reserve_j'] == pytest.approx(0.1)


def test_enter_recovery_rebases_to_contact_and_transfers_virtual_energy(monkeypatch):
    barrier = _configured_filter(
        monkeypatch,
        HNUTER_IEBC_E_MAX_J='80.0',
        HNUTER_IEBC_STOP_DISTANCE_M='5.0',
    )
    barrier.safe_s = 3.0
    barrier.safe_v = 0.02
    barrier.safe_v_prev2 = 0.019
    barrier.storage_bound = 1.25
    barrier.g_e_last = 20.0

    barrier.enter_recovery(measured_s=0.4)

    assert barrier.mode == barrier.MODE_RECOVERY
    transferred = 0.5 * barrier.k_c * (3.0 - 0.4) ** 2
    assert barrier.safe_s == pytest.approx(0.4)
    assert barrier.safe_v == pytest.approx(0.02)
    assert barrier.storage_bound == pytest.approx(1.25 + transferred)
    assert barrier.recovery_rebase_energy_j == pytest.approx(transferred)
    assert barrier.storage_frozen
    assert not barrier.storage_update_enabled
    assert barrier.release_s == pytest.approx(0.4)
    assert barrier.release_direction == 1.0


def test_recovery_never_commands_reverse_after_contact_rebase(monkeypatch):
    barrier = _configured_filter(
        monkeypatch,
        HNUTER_IEBC_E_MAX_J='80.0',
        HNUTER_IEBC_STOP_DISTANCE_M='5.0',
        HNUTER_IEBC_BRAKE_FORCE_CERT_N='20.0',
        HNUTER_IEBC_RECOVERY_RHO_MIN='0.0',
        HNUTER_IEBC_MAX_REF_JERK_MPS3='5.0',
    )
    barrier.safe_s = 3.0
    barrier.safe_v = 0.0
    barrier.safe_v_prev2 = 0.0
    barrier.g_e_last = 1.0
    barrier.enter_recovery(measured_s=0.0)

    _, safe_velocity, _ = barrier.filter_reference(
        dt=0.02,
        measured_position_enu=np.zeros(3),
        measured_velocity_enu=np.zeros(3),
        nominal_position_enu=np.zeros(3),
        nominal_velocity_enu=np.zeros(3),
        nominal_acceleration_enu=np.zeros(3),
        actuator_force_enu=np.zeros(3),
    )

    assert barrier.mode == barrier.MODE_RECOVERY
    assert safe_velocity[0] >= 0.0
    assert barrier.safe_s == pytest.approx(0.0)
    assert barrier.debug['recoverable_energy'] == pytest.approx(0.0)
    assert barrier.storage_bound == pytest.approx(0.5 * barrier.k_c * 3.0 ** 2)
    assert barrier.debug['stop_distance_barrier'] > 0.0
    assert barrier.debug['qp_slack_w'] == pytest.approx(0.0)


def test_recovery_reports_initially_impossible_one_metre_certificate(monkeypatch):
    barrier = _configured_filter(
        monkeypatch,
        HNUTER_IEBC_E_MAX_J='80.0',
        HNUTER_IEBC_STOP_DISTANCE_M='1.0',
        HNUTER_IEBC_BRAKE_FORCE_CERT_N='20.0',
        HNUTER_IEBC_RECOVERY_RHO_MIN='0.0',
    )
    barrier.safe_s = math.sqrt(2.0 * 79.0 / barrier.k_c)
    barrier.safe_v = 0.0
    barrier.safe_v_prev2 = 0.0
    barrier.g_e_last = 1.0
    barrier.enter_recovery(measured_s=0.0)

    barrier.filter_reference(
        dt=0.02,
        measured_position_enu=np.zeros(3),
        measured_velocity_enu=np.zeros(3),
        nominal_position_enu=np.zeros(3),
        nominal_velocity_enu=np.zeros(3),
        nominal_acceleration_enu=np.zeros(3),
        actuator_force_enu=np.zeros(3),
    )

    assert barrier.debug['reserved_stop_distance'] == pytest.approx(79.0 / 20.0)
    assert barrier.debug['stop_distance_barrier'] < 0.0
    assert barrier.infeasible
    assert barrier.debug['qp_slack_w'] > 0.0


def test_recovery_enters_hold_only_after_all_stop_conditions_persist(monkeypatch):
    barrier = _configured_filter(
        monkeypatch,
        HNUTER_IEBC_E_MAX_J='80.0',
        HNUTER_IEBC_STOP_DISTANCE_M='1.0',
        HNUTER_IEBC_STOP_HOLD_S='0.10',
        HNUTER_IEBC_RECOVERY_STOP_MIN_TIME_S='0.0',
        HNUTER_IEBC_RECOVERY_RHO_MIN='0.0',
    )
    barrier.safe_s = 0.0
    barrier.safe_v = 0.0
    barrier.safe_v_prev2 = 0.0
    barrier.enter_recovery(measured_s=0.0)

    for _ in range(6):
        barrier.filter_reference(
            dt=0.02,
            measured_position_enu=np.zeros(3),
            measured_velocity_enu=np.zeros(3),
            nominal_position_enu=np.zeros(3),
            nominal_velocity_enu=np.zeros(3),
            nominal_acceleration_enu=np.zeros(3),
            actuator_force_enu=np.zeros(3),
        )

    assert barrier.mode == barrier.MODE_HOLD
    assert barrier.safe_s == pytest.approx(0.0)
    assert barrier.safe_v == pytest.approx(0.0)
    assert barrier.debug['recoverable_energy'] == pytest.approx(0.0)


def test_recovery_holds_at_actual_stop_without_chasing_old_virtual_position(monkeypatch):
    barrier = _configured_filter(
        monkeypatch,
        HNUTER_IEBC_E_MAX_J='80.0',
        HNUTER_IEBC_STOP_DISTANCE_M='5.0',
        HNUTER_IEBC_STOP_HOLD_S='0.0',
        HNUTER_IEBC_MAX_REF_SPEED_MPS='2.0',
        HNUTER_IEBC_MAX_REF_ACCEL_MPS2='12.0',
        HNUTER_IEBC_MAX_REF_JERK_MPS3='50.0',
    )
    barrier.safe_s = -0.4
    barrier.safe_v = 0.0
    barrier.safe_v_prev2 = 0.0
    barrier.g_e_last = 1.0
    barrier.enter_recovery(measured_s=0.0)
    barrier.recovery_motion_seen = True

    energy_before_rebase = 0.5 * barrier.k_c * 0.4 ** 2
    safe_position, safe_velocity, _ = barrier.filter_reference(
        dt=0.02,
        measured_position_enu=np.zeros(3),
        measured_velocity_enu=np.zeros(3),
        nominal_position_enu=np.zeros(3),
        nominal_velocity_enu=np.zeros(3),
        nominal_acceleration_enu=np.zeros(3),
        actuator_force_enu=np.zeros(3),
    )

    assert barrier.mode == barrier.MODE_HOLD
    assert barrier.debug['recovery_phase'] == 'hold'
    assert barrier.debug['recovery_terminal_s'] == pytest.approx(0.0)
    assert safe_position[0] == pytest.approx(0.0)
    assert safe_velocity[0] == pytest.approx(0.0)
    assert barrier.safe_s == pytest.approx(0.0)
    assert barrier.safe_v == pytest.approx(0.0)
    assert barrier.recovery_rebase_energy_j == pytest.approx(energy_before_rebase)
    assert barrier.storage_bound == pytest.approx(energy_before_rebase)
    assert barrier.debug['e_i'] == pytest.approx(energy_before_rebase)
    assert not barrier.infeasible

    # HOLD locks the measured stop point once. It must neither follow later
    # measurement drift nor chase the old virtual/nominal target.
    safe_position, safe_velocity, _ = barrier.filter_reference(
        dt=0.02,
        measured_position_enu=np.array([0.10, 0.0, 0.0]),
        measured_velocity_enu=np.zeros(3),
        nominal_position_enu=np.array([3.0, 0.0, 0.0]),
        nominal_velocity_enu=np.array([1.0, 0.0, 0.0]),
        nominal_acceleration_enu=np.zeros(3),
        actuator_force_enu=np.zeros(3),
    )
    assert barrier.mode == barrier.MODE_HOLD
    assert safe_position[0] == pytest.approx(0.0)
    assert safe_velocity[0] == pytest.approx(0.0)


def test_stationary_release_stays_at_contact_point_without_old_target_chase(monkeypatch):
    barrier = _configured_filter(
        monkeypatch,
        HNUTER_IEBC_E_MAX_J='80.0',
        HNUTER_IEBC_STOP_DISTANCE_M='5.0',
        HNUTER_IEBC_MAX_REF_SPEED_MPS='2.0',
        HNUTER_IEBC_MAX_REF_ACCEL_MPS2='12.0',
        HNUTER_IEBC_MAX_REF_JERK_MPS3='50.0',
    )
    barrier.safe_s = 3.0
    barrier.safe_v = 0.0
    barrier.safe_v_prev2 = 0.0
    barrier.g_e_last = 1.0
    barrier.enter_recovery(measured_s=0.0)

    barrier.filter_reference(
        dt=0.02,
        measured_position_enu=np.zeros(3),
        measured_velocity_enu=np.zeros(3),
        nominal_position_enu=np.zeros(3),
        nominal_velocity_enu=np.zeros(3),
        nominal_acceleration_enu=np.zeros(3),
        actuator_force_enu=np.zeros(3),
    )

    assert barrier.debug['recovery_phase'] in ('brake', 'settle')
    assert not barrier.recovery_stop_latched
    assert barrier.safe_s == pytest.approx(0.0)
    assert barrier.debug['recovery_reference_velocity'] == pytest.approx(0.0)


def test_recovery_stop_requires_persistent_low_speed(monkeypatch):
    barrier = _configured_filter(
        monkeypatch,
        HNUTER_IEBC_E_MAX_J='80.0',
        HNUTER_IEBC_STOP_DISTANCE_M='5.0',
        HNUTER_IEBC_STOP_HOLD_S='0.10',
        HNUTER_IEBC_MAX_REF_SPEED_MPS='2.0',
        HNUTER_IEBC_MAX_REF_ACCEL_MPS2='12.0',
        HNUTER_IEBC_MAX_REF_JERK_MPS3='50.0',
    )
    barrier.safe_s = -0.4
    barrier.safe_v = 0.0
    barrier.safe_v_prev2 = 0.0
    barrier.g_e_last = 1.0
    barrier.enter_recovery(measured_s=0.0)
    barrier.recovery_motion_seen = True

    for _ in range(4):
        barrier.filter_reference(
            dt=0.02,
            measured_position_enu=np.zeros(3),
            measured_velocity_enu=np.zeros(3),
            nominal_position_enu=np.zeros(3),
            nominal_velocity_enu=np.zeros(3),
            nominal_acceleration_enu=np.zeros(3),
            actuator_force_enu=np.zeros(3),
        )
    assert not barrier.recovery_stop_latched

    for _ in range(2):
        barrier.filter_reference(
            dt=0.02,
            measured_position_enu=np.zeros(3),
            measured_velocity_enu=np.zeros(3),
            nominal_position_enu=np.zeros(3),
            nominal_velocity_enu=np.zeros(3),
            nominal_acceleration_enu=np.zeros(3),
            actuator_force_enu=np.zeros(3),
        )
    assert barrier.recovery_stop_latched
    assert barrier.mode == barrier.MODE_HOLD
    assert barrier.safe_s == pytest.approx(0.0)
    assert barrier.safe_v == pytest.approx(0.0)


def test_recovery_empty_jerk_intersection_never_breaks_speed_limit(monkeypatch):
    barrier = _configured_filter(
        monkeypatch,
        HNUTER_IEBC_E_MAX_J='80.0',
        HNUTER_IEBC_STOP_DISTANCE_M='5.0',
        HNUTER_IEBC_MAX_REF_SPEED_MPS='2.0',
        HNUTER_IEBC_MAX_REF_ACCEL_MPS2='12.0',
        HNUTER_IEBC_MAX_REF_JERK_MPS3='50.0',
    )
    barrier.safe_s = 1.0
    barrier.safe_v = 2.0
    barrier.safe_v_prev2 = 1.9
    barrier.release_s = 0.0

    safe, _, _, _, _ = barrier._recovery_reference_velocity(
        v_prev=2.0, dt=0.004, g_e=1.0,
        rhs_energy=1000.0, rhs_stop=1000.0,
        v_i=1.0, e_ref=1.0, recoverable_energy=1.0)

    assert barrier.recovery_rate_infeasible
    assert abs(safe) <= barrier.max_ref_speed


def test_jerk_limited_rate_target_converges_without_large_limit_cycle(monkeypatch):
    barrier = _configured_filter(
        monkeypatch,
        HNUTER_IEBC_MAX_REF_SPEED_MPS='2.0',
        HNUTER_IEBC_MAX_REF_ACCEL_MPS2='12.0',
        HNUTER_IEBC_MAX_REF_JERK_MPS3='50.0',
    )
    dt = 0.004
    barrier.safe_v = -1.5
    barrier.safe_v_prev2 = -1.5
    history = []

    for _ in range(1500):
        v_prev = barrier.safe_v
        v_next = barrier._jerk_limited_rate_target(0.2, v_prev, dt)
        barrier.safe_v_prev2 = v_prev
        barrier.safe_v = v_next
        history.append(v_next)

    assert max(history) < 0.25
    assert history[-1] == pytest.approx(0.2, abs=2e-3)


def test_closed_loop_cube_experiment_imports_closed_loop_controller():
    import hnuter_iebc_cube_contact_experiment_closed_loop as experiment

    assert experiment.HnuterController.__module__.endswith(
        'hnuter_external_controller_px4_position_hardware_iebc_closed_loop')
    assert float(experiment.os.environ['HNUTER_IEBC_KC_NPM']) > 0.0
    assert float(experiment.os.environ['HNUTER_IEBC_DC_NSPM']) >= 0.0


def test_pose_callback_uses_vehicle_model_not_scoped_probe_link():
    """The +90 deg probe link must not masquerade as vehicle yaw."""
    import hnuter_iebc_cube_contact_experiment_closed_loop as experiment

    node = object.__new__(experiment.HnuterIebcClosedLoopCubeContactExperiment)
    node._transport_lock = threading.Lock()
    node.cube_x_m = math.nan
    node.cube_y_m = math.nan
    node.vehicle_gz_position = np.full(3, math.nan)
    node.vehicle_gz_yaw = math.nan

    def pose(name, x, yaw):
        q = SimpleNamespace(
            w=math.cos(yaw / 2.0), x=0.0, y=0.0, z=math.sin(yaw / 2.0))
        return SimpleNamespace(
            name=name,
            position=SimpleNamespace(x=x, y=0.0, z=1.0),
            orientation=q)

    message = SimpleNamespace(pose=[
        pose('interaction_cube', 3.0, 0.0),
        pose('hnuter_contact_0', 1.0, math.radians(2.0)),
        pose('hnuter_contact_0::contact_probe', 1.75, math.radians(18.0)),
    ])

    node._pose_callback(message)

    assert node.vehicle_gz_position[0] == pytest.approx(1.0)
    assert math.degrees(node.vehicle_gz_yaw) == pytest.approx(2.0)


def test_alignment_calibrates_controller_frame_then_can_be_latched():
    import hnuter_iebc_cube_contact_experiment_closed_loop as experiment

    node = object.__new__(experiment.HnuterIebcClosedLoopCubeContactExperiment)
    node.desired_controller_yaw = math.radians(20.0)
    node.yaw_align_gain = 2.0
    node.yaw_align_max_rate_rad_s = math.radians(15.0)

    node._update_alignment_yaw_command(math.radians(-10.0), 0.5)

    # Requested -20 deg/s is rate-limited to -15 deg/s for 0.5 s.
    assert math.degrees(node.desired_controller_yaw) == pytest.approx(12.5)


def test_virtual_force_marker_points_toward_aircraft_and_scales_with_force():
    import hnuter_iebc_cube_contact_experiment_closed_loop as experiment

    shaft, head = experiment.HnuterIebcClosedLoopCubeContactExperiment._virtual_force_markers(
        force_n=17.0, cube_x=3.0, cube_y=0.0)

    assert shaft.type == experiment.Marker.LINE_LIST
    assert shaft.point[1].x < shaft.point[0].x
    assert shaft.point[0].x - shaft.point[1].x == pytest.approx(0.85)
    assert head.type == experiment.Marker.CONE
    assert head.pose.position.x < shaft.point[1].x
    assert head.pose.orientation.y < 0.0
    assert shaft.material.diffuse.r > shaft.material.diffuse.g
