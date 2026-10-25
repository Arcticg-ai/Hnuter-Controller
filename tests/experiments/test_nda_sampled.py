import numpy as np
from controllers.common.hnuter_drcda import DRCDAConfig, HnuterWrenchModel
from controllers.experiments.fixed_attitude_switching.audit_allocator import MeasuredNDAAllocator


def allocator():
    a = MeasuredNDAAllocator(HnuterWrenchModel(), DRCDAConfig.gazebo_joint_pid_no_delay())
    a.reset(thrust_state=[10., 10., 10., 10., 6.])
    return a


def test_continuous_inverse_under_realizes_fast_motor_rate():
    a = allocator()
    dt, rate = .01, 100.
    command = a.state.copy()
    command[4] += a.config.motor_tau_up_s[0]*rate
    actual_rate = a._command_rate(command, dt, a.config.servo_state_limit_rad)[4]
    assert 9.9 < actual_rate < 10.1


def test_sampled_inverse_realizes_fast_motor_rate():
    a = allocator()
    dt, rate = .01, 100.
    command = a.state.copy()
    alpha = -np.expm1(-dt/a.config.motor_tau_up_s[0])
    command[4] += dt*rate/alpha
    actual_rate = a._command_rate(command, dt, a.config.servo_state_limit_rad)[4]
    np.testing.assert_allclose(actual_rate, rate, atol=1e-9)


def test_sampled_rate_agrees_with_shared_predictor_one_step():
    a = allocator()
    command = a.command + np.array([.1, -.1, .15, -.15, 1., -1., 2., -2., .1])
    limits = a.config.servo_state_limit_rad
    predicted, _ = a._predict_horizons(command, limits, [.01])[0]
    np.testing.assert_allclose(a._command_rate(command, .01, limits),
                               (predicted-a.state)/.01, atol=1e-9)


def test_normalized_allocation_unsaturated_wrench_rate():
    a = allocator()
    dt = .01
    desired = a.model.wrench(a.state) + np.array([.01, .01, .1, .001, .001, .001])
    before = a.state.copy()
    result = a.allocate(desired, dt)
    actual_rate = a._command_rate(result.command, dt, a.config.servo_state_limit_rad)
    np.testing.assert_allclose(a.model.jacobian(before) @ actual_rate,
                               result.jerk_reference, atol=5e-4)


def test_hover_speed_nullspace_reduces_counter_tilt():
    a = allocator()
    angle = .4
    front = 4.5*9.81*.664/(4*.769)
    a.reset(angle_state=[0., -angle, 0., angle],
            thrust_state=[front/np.cos(angle)]*4+[4.5*9.81-4*front])
    a.measured_angles = a.state[:4].copy()
    desired = a.model.wrench(a.state)
    result = a.allocate(desired, .01)
    rate = a._command_rate(result.command, .01, a.config.servo_state_limit_rad)
    assert np.dot(a.state[:4], rate[:4]) < 0
    assert np.sum(rate[4:8]) < 0
    np.testing.assert_allclose(a.model.jacobian(a.state) @ rate, 0, atol=5e-4)


def test_common_prelude_does_not_return_to_direct_during_final_hold(monkeypatch):
    from controllers.experiments.fixed_attitude_switching.controller import FixedAttitudeSwitchingMixin
    class Parent:
        def publish_direct_actuator_setpoint(self, *args):
            return 'tested_allocator'
    class Dummy(FixedAttitudeSwitchingMixin, Parent):
        pass
    c = object.__new__(Dummy)
    c.auto_traj_mode = 'hover'
    c._switch_elapsed_s = 12.
    c.switch_ingress_s = 2.5
    c.switch_settle_s = 1.5
    c.armed = True
    c._switch_allocator_engaged = True
    c.drcda = object()
    monkeypatch.setenv('HNUTER_SWITCH_COMMON_PRELUDE', '1')
    assert c.publish_direct_actuator_setpoint([], 0, 0, 0, 0) == 'tested_allocator'


def test_posture_preference_is_projected_without_changing_primary_jerk():
    a = allocator()
    a.posture_gain = 2.
    a.reset(angle_state=[0., -.3, 0., .3], thrust_state=[10.]*4+[6.])
    a.measured_angles = a.state[:4].copy()
    result = a.allocate(a.model.wrench(a.state), .01)
    rate = a._command_rate(result.command, .01, a.config.servo_state_limit_rad)
    assert rate[1] > .1 and rate[3] < -.1
    np.testing.assert_allclose(a.model.jacobian(a.state)@rate, result.jerk_reference, atol=5e-4)


def test_direct_experiment_obeys_shared_thrust_command_rate(monkeypatch):
    from controllers.experiments.fixed_attitude_switching.controller import (
        FixedAttitudeDirectController, FixedAttitudeSwitchingMixin)
    from controllers.simulation.hnuter_external_direct_drcda import HnuterDRCDAController
    monkeypatch.setattr(FixedAttitudeSwitchingMixin, 'publish_direct_actuator_setpoint',
                        lambda self, controls, *angles: controls)
    c = object.__new__(FixedAttitudeDirectController)
    c.allow_tail_reverse = True
    c.tail_thrust_max_forward_n = 12.777084
    c._switch_allocator_engaged = True
    c._switch_direct_dt = .004
    c._switch_previous_direct_forces = np.array([3.,3.,3.,3.,0.])
    controls = [c._thrust_to_normalized_motor_control(25.)]*4 + [0.]
    output = c.publish_direct_actuator_setpoint(controls, 0.,0.,0.,0.)
    forces = [HnuterDRCDAController._motor_control_to_thrust(c, u) for u in output[:4]]
    np.testing.assert_allclose(forces, 13., atol=1e-9)
