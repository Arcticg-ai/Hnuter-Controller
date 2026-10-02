#!/usr/bin/env python3
"""Fixed-attitude minimum-jerk lateral switching experiment for SITL."""

from __future__ import annotations

import math
import os

import numpy as np
import rclpy

from controllers.experiments.drcda_closed_loop.controller import (
    HnuterClosedLoopDRCDAController,
)
from controllers.experiments.drcda_closed_loop.feedback import JointAngleFeedback
from controllers.simulation.hnuter_external_direct_controller_debug import (
    HnuterController as DirectController,
    env_float,
)
from controllers.simulation.hnuter_external_direct_drcda import HnuterDRCDAController


def minimum_jerk(start: np.ndarray, finish: np.ndarray, elapsed: float,
                 duration: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return position, velocity, and acceleration for a quintic segment."""
    duration = max(float(duration), 1e-6)
    s = float(np.clip(elapsed / duration, 0.0, 1.0))
    blend = 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5
    blend_rate = (30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4) / duration
    blend_accel = (60.0 * s - 180.0 * s**2 + 120.0 * s**3) / duration**2
    delta = finish - start
    return start + delta * blend, delta * blend_rate, delta * blend_accel


class FixedAttitudeSwitchingMixin:
    """Replace trajectory key 2 with a fixed-attitude A-B-A-C maneuver."""

    def _node_name(self):
        method = getattr(self, '_switch_method', 'direct')
        return f'hnuter_fixed_attitude_switch_{method}'

    def _diagnostic_file_prefix(self):
        method = getattr(self, '_switch_method', 'direct')
        return f'experiments/fixed_attitude_switching/{method}/switching'

    def __init__(self):
        self._switch_method = os.environ.get(
            'HNUTER_SWITCH_METHOD', 'direct'
        ).strip().lower()
        self.switch_half_span_m = env_float('HNUTER_SWITCH_HALF_SPAN_M', 1.2)
        self.switch_altitude_m = env_float('HNUTER_SWITCH_ALTITUDE_M', 1.5)
        self.switch_duration_s = env_float('HNUTER_SWITCH_DURATION_S', 2.0)
        self.switch_ingress_s = env_float('HNUTER_SWITCH_INGRESS_S', 2.5)
        self.switch_settle_s = env_float('HNUTER_SWITCH_SETTLE_S', 1.5)
        self.switch_dwell_s = env_float('HNUTER_SWITCH_DWELL_S', 0.5)
        self.switch_prediction_horizons_s = (
            env_float('HNUTER_SWITCH_PREDICTION_H1_S', 0.05),
            env_float('HNUTER_SWITCH_PREDICTION_H2_S', 0.10),
        )
        self.switch_prediction_weights = (
            env_float('HNUTER_SWITCH_PREDICTION_LAMBDA1', 0.5),
            env_float('HNUTER_SWITCH_PREDICTION_LAMBDA2', 1.0),
        )
        self._switch_elapsed_s = math.nan
        self._switch_phase = 'idle'
        self._switch_start = np.zeros(3)
        self._switch_a = np.zeros(3)
        self._switch_b = np.zeros(3)
        self._switch_c = np.zeros(3)
        self._switch_joint_feedback = None
        super().__init__()
        self.diagnostic_period_s = env_float(
            'HNUTER_SWITCH_DIAGNOSTIC_PERIOD_S', 0.02
        )
        existing = getattr(self, '_feedback', None)
        self._switch_joint_feedback = (
            existing if existing is not None else JointAngleFeedback()
        )

    def _start_auto_trajectory(self, mode: str, current_time: float):
        if mode != 'lissajous':
            return super()._start_auto_trajectory(mode, current_time)
        super()._start_auto_trajectory(mode, current_time)
        self.auto_traj_mode = 'fixed_switch'
        self._switch_start = self.auto_traj_start_pos.copy()
        self._switch_a = self._switch_start.copy()
        self._switch_b = self._switch_start.copy()
        self._switch_c = self._switch_start.copy()
        self._switch_b[1] += self.switch_half_span_m
        self._switch_c[1] -= self.switch_half_span_m
        self._switch_a[2] = self.switch_altitude_m
        self._switch_b[2] = self.switch_altitude_m
        self._switch_c[2] = self.switch_altitude_m
        self.get_logger().info(
            '开始执行定姿快速侧移：'
            f'A={np.round(self._switch_a, 2).tolist()} -> '
            f'B={np.round(self._switch_b, 2).tolist()} -> A -> '
            f'C={np.round(self._switch_c, 2).tolist()}, '
            f'T={self.switch_duration_s:.2f}s'
        )

    def _switch_reference(
        self, elapsed: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, str, bool]:
        ingress_end = self.switch_ingress_s
        settle_end = ingress_end + self.switch_settle_s
        outward_end = settle_end + self.switch_duration_s
        dwell_b_end = outward_end + self.switch_dwell_s
        return_end = dwell_b_end + self.switch_duration_s
        dwell_a_end = return_end + self.switch_dwell_s
        opposite_end = dwell_a_end + self.switch_duration_s
        zero = np.zeros(3)
        if elapsed < ingress_end:
            values = minimum_jerk(
                self._switch_start, self._switch_a, elapsed, self.switch_ingress_s
            )
            return *values, 'ingress', False
        if elapsed < settle_end:
            return self._switch_a.copy(), zero.copy(), zero.copy(), 'settle_a', False
        if elapsed < outward_end:
            values = minimum_jerk(
                self._switch_a, self._switch_b, elapsed - settle_end,
                self.switch_duration_s,
            )
            return *values, 'a_to_b', False
        if elapsed < dwell_b_end:
            return self._switch_b.copy(), zero.copy(), zero.copy(), 'dwell_b', False
        if elapsed < return_end:
            values = minimum_jerk(
                self._switch_b, self._switch_a, elapsed - dwell_b_end,
                self.switch_duration_s,
            )
            return *values, 'b_to_a', False
        if elapsed < dwell_a_end:
            return self._switch_a.copy(), zero.copy(), zero.copy(), 'dwell_a', False
        if elapsed < opposite_end:
            values = minimum_jerk(
                self._switch_a, self._switch_c, elapsed - dwell_a_end,
                self.switch_duration_s,
            )
            return *values, 'a_to_c', False
        return self._switch_c.copy(), zero.copy(), zero.copy(), 'complete', True

    def _finish_switching(self):
        final = self._switch_c.copy()
        self.auto_traj_mode = 'hover'
        self.manual_des_pos = final.copy()
        self.manual_des_yaw = self.auto_traj_yaw
        self.manual_des_roll = 0.0
        self.manual_des_pitch = 0.0
        self.target_position = final.copy()
        self.target_velocity = np.zeros(3)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([0.0, 0.0, self.auto_traj_yaw])
        self.target_attitude_rate = np.zeros(3)
        self.target_R_des_ned_frd = None
        self.integral_pos_error[:] = 0.0
        self.integral_e_R[:] = 0.0
        self._attitude_error_quaternion = None
        self._switch_phase = 'post_hold_c'
        self.get_logger().info(
            f'定姿快速侧移完成：T={self.switch_duration_s:.2f}s'
        )

    def _update_auto_trajectory(self, current_time: float):
        if self.auto_traj_mode != 'fixed_switch':
            return super()._update_auto_trajectory(current_time)
        elapsed = max(0.0, current_time - self.auto_traj_start_time)
        self._switch_elapsed_s = elapsed
        pos, vel, acc, self._switch_phase, done = self._switch_reference(elapsed)
        if done:
            self._finish_switching()
            return True
        self.manual_des_pos = pos.copy()
        self.manual_des_yaw = self.auto_traj_yaw
        self.manual_des_roll = 0.0
        self.manual_des_pitch = 0.0
        self._last_manual_cmd = self._zero_manual_cmd()
        self.target_position = pos
        self.target_velocity = vel
        self.target_acceleration = acc
        self.target_attitude = np.array([0.0, 0.0, self.auto_traj_yaw])
        self.target_attitude_rate = np.zeros(3)
        self.target_R_des_ned_frd = None
        return True

    def _future_drcda_wrench_references(self):
        if (
            getattr(self, '_drcda_variant', '') != 'full'
            or self.auto_traj_mode != 'fixed_switch'
            or not math.isfinite(self._switch_elapsed_s)
        ):
            return super()._future_drcda_wrench_references()
        references = []
        for horizon_s, weight in zip(
            self.switch_prediction_horizons_s,
            self.switch_prediction_weights,
        ):
            position, velocity, acceleration, _, _ = self._switch_reference(
                self._switch_elapsed_s + horizon_s
            )
            wrench = self._predict_reference_wrench(
                position, velocity, acceleration, horizon_s
            )
            references.append((horizon_s, wrench, weight))
        return references

    def _diagnostic_extra_header(self):
        return super()._diagnostic_extra_header() + [
            'switch_elapsed_s', 'switch_phase', 'switch_duration_s',
            'switch_joint_feedback_age_s', 'switch_joint_feedback_available',
            *[f'switch_measured_joint_{name}_rad'
              for name in ('a_l', 'b_l', 'a_r', 'b_r')],
            *[f'switch_commanded_joint_{name}_rad'
              for name in ('a_l', 'b_l', 'a_r', 'b_r')],
        ]

    def _diagnostic_extra_values(self):
        measured = None
        age = math.inf
        if self._switch_joint_feedback is not None:
            measured, age = self._switch_joint_feedback.read()
        commanded = np.array([
            self._alpha1_cmd, self._theta1_cmd,
            self._alpha2_cmd, self._theta2_cmd,
        ])
        return super()._diagnostic_extra_values() + [
            self._switch_elapsed_s, self._switch_phase, self.switch_duration_s,
            age if math.isfinite(age) else float('nan'), int(measured is not None),
            *([*measured] if measured is not None else [float('nan')] * 4),
            *commanded,
        ]


class FixedAttitudeDirectController(FixedAttitudeSwitchingMixin, DirectController):
    pass


class FixedAttitudeDRCDAController(
    FixedAttitudeSwitchingMixin, HnuterDRCDAController
):
    pass


class FixedAttitudeClosedLoopController(
    FixedAttitudeSwitchingMixin, HnuterClosedLoopDRCDAController
):
    pass


def controller_class(method: str):
    method = method.strip().lower()
    if method == 'direct':
        return FixedAttitudeDirectController
    if method == 'basic_da':
        os.environ['HNUTER_DRCDA_VARIANT'] = 'basic_da'
        return FixedAttitudeDRCDAController
    if method in {'full', 'no_horizon'}:
        os.environ['HNUTER_DRCDA_VARIANT'] = method
        return FixedAttitudeClosedLoopController
    raise ValueError(f'unsupported HNUTER_SWITCH_METHOD={method!r}')


def main(args=None):
    rclpy.init(args=args)
    method = os.environ.get('HNUTER_SWITCH_METHOD', 'direct')
    controller = controller_class(method)()
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
