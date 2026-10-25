"""Matched no-delay Lissajous and large-attitude allocation experiments."""
import copy
from dataclasses import asdict
import json
import math
import os
from pathlib import Path

import numpy as np
import rclpy

from controllers.common.hnuter_attitude_control import (
    quaternion_attitude_error, reduced_tilt_attitude_error, large_tilt_yaw_scale,
)
from controllers.experiments.fixed_attitude_switching.controller import controller_class
from controllers.simulation.hnuter_external_direct_drcda import HnuterDRCDAController


def rotation_increment(vector):
    """Rodrigues exponential with sinc coefficients finite at zero rate."""
    x, y, z = np.asarray(vector, dtype=float)
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    angle = np.linalg.norm(vector)
    return np.eye(3)+np.sinc(angle/np.pi)*skew+.5*np.sinc(angle/(2*np.pi))**2*(skew@skew)


class ManeuverMixin:
    def __init__(self):
        self.maneuver_scenario = os.environ.get('HNUTER_MANEUVER_SCENARIO', 'lissajous_7')
        self.maneuver_attitude = self.maneuver_scenario.startswith('attitude_')
        super().__init__()
        self.lissajous_amp_x, self.lissajous_amp_y, self.lissajous_amp_z = 1.2, .8, .30
        self.lissajous_a, self.lissajous_b, self.lissajous_c = 2, 3, 1
        self.lissajous_period_s = 5. if self.maneuver_scenario == 'lissajous_5' else 7.
        self.attitude_step_axis_rad = np.radians(
            [60., 60., 0.] if self.maneuver_scenario == 'attitude_60_60' else [80., 120., 0.])
        self.attitude_segment_time_s = 6.
        self.attitude_peak_hold_s = 5.
        self.attitude_level_settle_s = 3.
        self.attitude_test_bidirectional = True
        if hasattr(self, 'drcda'):
            self.drcda.config.thrust_command_rate_n_s[:] = [2500.]*4+[100.]
        runtime = dict(scenario=self.maneuver_scenario, method=self._switch_method,
                       outer={name: getattr(self, name) for name in (
                           'direct_KR', 'direct_Domega', 'direct_attitude_Ki', 'direct_tau_limit',
                           'direct_pos_Kp_ned', 'direct_pos_Kd_ned', 'direct_pos_Ki_ned',
                           'max_acc_xy', 'max_acc_z', 'alpha_limit_rad', 'theta_limit_rad',
                           'servo_rate_limit_rad_s', 'tail_thrust_max_forward_n', 'tail_thrust_max_reverse_n')},
                       allocator=asdict(self.drcda.config) if hasattr(self, 'drcda') else None,
                       shared_force_command_rate_n_s=[2500.]*4+[100.])
        log_root = Path(os.environ['HNUTER_LOG_DIR'])
        log_root.mkdir(parents=True, exist_ok=True)
        (log_root/'runtime_config.json').write_text(json.dumps(runtime, indent=2,
            default=lambda value: value.tolist() if isinstance(value, np.ndarray) else value.item())+'\n')

    def _start_auto_trajectory(self, mode, current_time):
        super()._start_auto_trajectory(mode, current_time)
        if mode == 'lissajous':
            self.auto_traj_start_pos = self._switch_a.copy()
            self.auto_traj_z = self._switch_a[2]
            # Parent initializes the start-offset from the configured amplitudes.
            rotation = self._yaw_rotation_2d(self.auto_traj_path_yaw)
            self.auto_traj_origin_xy = self._switch_a[:2]-rotation@np.array([1.2, .8])

    def _attitude_phase(self, t):
        if t >= 80.:
            return 'complete'
        axis = ('roll_pos', 'pitch_pos', 'roll_neg', 'pitch_neg')[int(t//20.)]
        local = t % 20.
        phase = 'out' if local < 6. else 'hold' if local < 11. else 'return' if local < 17. else 'level'
        return f'{axis}_{phase}'

    def _switch_reference(self, elapsed):
        prelude = self.switch_ingress_s+self.switch_settle_s
        if elapsed < prelude:
            return super()._switch_reference(elapsed)
        t = elapsed-prelude
        if self.maneuver_attitude:
            phase = self._attitude_phase(t)
            return self._switch_a.copy(), np.zeros(3), np.zeros(3), phase, t >= 80.
        pos, vel, acc, done = self._lissajous_reference(t)
        return pos, vel, acc, 'complete' if done else 'lissajous', done

    def _update_auto_trajectory(self, current_time):
        result = super()._update_auto_trajectory(current_time)
        t = self._switch_elapsed_s-self.switch_ingress_s-self.switch_settle_s
        if math.isfinite(t) and t >= 0 and not getattr(self, '_maneuver_started', False):
            self._maneuver_started = True
            self._switch_allocator_engaged = True
            self.get_logger().info('MANEUVER_START')
        if self.maneuver_attitude and self.auto_traj_mode == 'fixed_switch' and t >= 0:
            att, rate, matrix, _ = self._attitude_reference(t)
            self.manual_des_roll, self.manual_des_pitch, self.manual_des_yaw = att
            self.target_attitude, self.target_attitude_rate = att, rate
            self.target_R_des_ned_frd = matrix
        return result

    def _finish_switching(self):
        self._switch_c = self._switch_a.copy()
        super()._finish_switching()
        self.get_logger().info('MANEUVER_DONE')

    def publish_px4_equivalent_direct_commands(self, current_time, dt):
        # Activate the original attitude feed-forward and yaw scheduling for
        # every allocator, while retaining the shared prelude/trajectory engine.
        previous = self.auto_traj_mode
        if self.maneuver_attitude and previous == 'fixed_switch' and getattr(self, '_maneuver_started', False):
            self.auto_traj_mode = 'attitude'
        try:
            return super().publish_px4_equivalent_direct_commands(current_time, dt)
        finally:
            self.auto_traj_mode = previous

    def publish_direct_actuator_setpoint(self, motor_controls, alpha1, alpha2, theta1, theta2):
        if self._switch_method == 'direct':
            controls = list(motor_controls)
            force = HnuterDRCDAController._motor_control_to_thrust(self, controls[4], self.allow_tail_reverse)
            previous = getattr(self, '_maneuver_previous_tail_force', force)
            if getattr(self, '_switch_allocator_engaged', False):
                delta = 100.*max(0., getattr(self, '_switch_direct_dt', .004))
                force = float(np.clip(force, previous-delta, previous+delta))
            self._maneuver_previous_tail_force = force
            controls[4] = (self._tail_thrust_to_normalized_bidirectional_motor_control(force)
                           if self.allow_tail_reverse else self._thrust_to_normalized_motor_control(force))
            motor_controls = controls
        return super().publish_direct_actuator_setpoint(motor_controls, alpha1, alpha2, theta1, theta2)

    def _future_attitude_wrench(self, horizon):
        t = self._switch_elapsed_s-self.switch_ingress_s-self.switch_settle_s+horizon
        att, rate, desired_rotation, _ = self._attitude_reference(t)
        if desired_rotation is None:
            desired_rotation = self._direct_desired_attitude_ned_frd(att)
        predicted_rotation = self.R_ned_frd @ rotation_increment(self.angular_velocity_frd*horizon)
        if self.direct_quaternion_error_enabled:
            error, _, _ = quaternion_attitude_error(desired_rotation, predicted_rotation,
                                                    self._attitude_error_quaternion)
        else:
            relative = desired_rotation.T@predicted_rotation
            skew = .5*(relative-relative.T)
            error = np.array([skew[2,1], skew[0,2], skew[1,0]])
        if self.direct_reduced_tilt_error_enabled:
            error, _, _ = reduced_tilt_attitude_error(desired_rotation, predicted_rotation, error)
        kp, kd = self.direct_KR.copy(), self.direct_Domega.copy()
        ki, limit = self.direct_attitude_Ki.copy(), self.direct_tau_limit.copy()
        if self.direct_large_tilt_yaw_scheduling_enabled:
            tilt = max(math.acos(np.clip(desired_rotation[2,2], -1, 1)),
                       math.acos(np.clip(predicted_rotation[2,2], -1, 1)))
            scale = large_tilt_yaw_scale(tilt, self.direct_large_tilt_yaw_start_rad,
                                        self.direct_large_tilt_yaw_full_rad, self.direct_large_tilt_yaw_min_scale)
            kp[2] *= scale; kd[2] *= math.sqrt(scale); ki[2] *= scale; limit[2] *= scale
        desired_rate = np.array([rate[0], -rate[1], -rate[2]])
        omega_error = self.angular_velocity_frd-predicted_rotation.T@desired_rotation@desired_rate
        torque = -kp*error-kd*omega_error-ki*self.integral_e_R
        if self.direct_attitude_gyro_compensation_enabled:
            torque += np.cross(self.angular_velocity_frd, self.J@self.angular_velocity_frd)
        if not self.direct_yaw_control_enabled:
            torque[2] = 0.
        proxy = copy.copy(self)
        proxy.R_ned_frd = predicted_rotation
        proxy.last_tau_c = np.clip(torque, -limit, limit)
        return proxy._predict_reference_wrench(self._switch_a, np.zeros(3), np.zeros(3), horizon)

    def _future_drcda_wrench_references(self):
        if self._switch_method in {'no_preview', 'no_horizon'}:
            return super()._future_drcda_wrench_references()
        if self.maneuver_attitude and self.auto_traj_mode == 'attitude':
            return [(h, self._future_attitude_wrench(h), w)
                    for h, w in zip(self.switch_prediction_horizons_s, self.switch_prediction_weights)]
        return super()._future_drcda_wrench_references()


def main():
    rclpy.init()
    base = controller_class(os.environ.get('HNUTER_SWITCH_METHOD', 'direct'))
    cls = type('MatchedManeuverController', (ManeuverMixin, base), {})
    controller = cls()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
