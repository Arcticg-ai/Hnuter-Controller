import numpy as np
import pytest

from controllers.experiments.maneuver_validation import ManeuverMixin, rotation_increment
from controllers.experiments.fixed_attitude_switching.controller import FixedAttitudeDirectController
from tools.experiments.run_maneuver_validation import analyze


def test_rotation_increment_zero_and_quarter_turn():
    np.testing.assert_array_equal(rotation_increment(np.zeros(3)), np.eye(3))
    r = rotation_increment(np.array([0., 0., np.pi/2]))
    np.testing.assert_allclose(r, [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]], atol=1e-14)
    np.testing.assert_allclose(r.T@r, np.eye(3), atol=1e-14)


def test_direct_tail_matches_100_n_per_s_limit(monkeypatch):
    monkeypatch.setattr(FixedAttitudeDirectController, 'publish_direct_actuator_setpoint',
                        lambda self, controls, *angles: controls)
    c = controller()
    c._switch_method = 'direct'
    c.allow_tail_reverse = True
    c.tail_thrust_max_forward_n = 12.777084
    c._switch_allocator_engaged = True
    c._switch_direct_dt = .004
    c._maneuver_previous_tail_force = 0.
    outputs = c.publish_direct_actuator_setpoint([0.,0.,0.,0.,1.], 0.,0.,0.,0.)
    np.testing.assert_allclose(outputs[4]**2*c.tail_thrust_max_forward_n, .4)


def controller():
    cls = type('Dummy', (ManeuverMixin, FixedAttitudeDirectController), {})
    c = object.__new__(cls)
    c.maneuver_attitude = False
    c.switch_ingress_s, c.switch_settle_s = 2.5, 1.5
    c.lissajous_period_s = 7.
    c.lissajous_amp_x, c.lissajous_amp_y, c.lissajous_amp_z = 1.2, .8, .3
    c.lissajous_a, c.lissajous_b, c.lissajous_c = 2, 3, 1
    c.auto_traj_path_yaw = 0.
    c.auto_traj_origin_xy = np.array([-1.2, -.8])
    c.auto_traj_z = 3.
    c.min_altitude, c.max_altitude = 0., 10.
    c.auto_traj_start_pos = c._switch_a = np.array([0., 0., 3.])
    return c


def test_lissajous_starts_and_finishes_at_relative_anchor():
    c = controller()
    for t in (4., 11.):
        p, v, _, _, done = c._switch_reference(t)
        np.testing.assert_allclose(p, c._switch_a, atol=1e-10)
        np.testing.assert_allclose(v, 0, atol=1e-10)
        assert done == (t == 11.)


def test_lissajous_analytic_derivatives():
    c = controller()
    t, h = 6.37, 1e-5
    p0, v0, *_ = c._switch_reference(t-h)
    p1, v1, *_ = c._switch_reference(t+h)
    _, v, a, *_ = c._switch_reference(t)
    np.testing.assert_allclose((p1-p0)/(2*h), v, atol=1e-7)
    np.testing.assert_allclose((v1-v0)/(2*h), a, atol=1e-7)


@pytest.mark.parametrize('time,phase', [(6., 'roll_pos_hold'), (26., 'pitch_pos_hold'),
                                      (46., 'roll_neg_hold'), (66., 'pitch_neg_hold'),
                                      (80., 'complete')])
def test_large_attitude_phase(time, phase):
    assert controller()._attitude_phase(time) == phase


def test_future_attitude_wrench_uses_rate_and_does_not_update_integrators():
    c = controller()
    c._switch_elapsed_s = 4.
    c._attitude_reference = lambda t: (np.zeros(3), np.array([.2, 0., 0.]), np.eye(3), False)
    c.R_ned_frd = np.eye(3)
    c.angular_velocity_frd = np.zeros(3)
    c.direct_quaternion_error_enabled = True
    c._attitude_error_quaternion = None
    c.direct_reduced_tilt_error_enabled = c.direct_large_tilt_yaw_scheduling_enabled = False
    c.direct_KR, c.direct_Domega = np.ones(3)*5, np.ones(3)*3
    c.direct_attitude_Ki = np.ones(3)*.1
    c.direct_tau_limit = np.ones(3)*3
    c.integral_e_R = np.zeros(3)
    c.direct_attitude_gyro_compensation_enabled = False
    c.direct_yaw_control_enabled = True
    c._z0, c._z0_initialized = 0., True
    c.position = c.target_position = np.array([0., 0., 3.])
    c.velocity = c.target_velocity = np.zeros(3)
    c.direct_pos_Kp_ned = c.direct_pos_Kd_ned = np.ones(3)
    c.direct_pos_Ki_ned = c.integral_pos_error = np.zeros(3)
    c.max_acc_xy = c.max_acc_z = 8.
    c.mass, c.gravity, c.alpha_limit_rad = 4.5, 9.81, np.pi
    c.allocator_force_x_sign = c.allocator_force_y_sign = 1
    w = c._future_attitude_wrench(0.)
    np.testing.assert_allclose(w, [0., 0., 4.5*9.81, .6, 0., 0.], atol=1e-10)
    np.testing.assert_array_equal(c.integral_e_R, np.zeros(3))
    np.testing.assert_array_equal(c.R_ned_frd, np.eye(3))
