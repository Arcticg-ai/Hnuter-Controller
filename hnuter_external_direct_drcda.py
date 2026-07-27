#!/usr/bin/env python3
"""Hnuter direct external controller using DRCDA control allocation.

The flight-control and safety state machine comes from the established direct
debug controller.  This class replaces only its algebraic actuator inverse with
delay-aware, reachability-constrained differential allocation.
"""

from __future__ import annotations

import math
import os

import numpy as np

from hnuter_drcda import (
    ACTUATOR_COUNT,
    ANGLE_COUNT,
    DRCDAAllocator,
    DRCDAConfig,
    HnuterWrenchModel,
)
from hnuter_external_direct_controller_debug import (
    HnuterController as DirectController,
    env_float,
)

import rclpy


class HnuterDRCDAController(DirectController):
    def _node_name(self):
        return 'hnuter_controller_direct_drcda'

    def __init__(self) -> None:
        self._drcda_ready = False
        self._drcda_active_call = False
        self._drcda_current_time_s = 0.0
        self._drcda_current_dt_s = 0.01
        self._drcda_accumulated_dt_s = 0.0
        super().__init__()

        model_name = os.environ.get(
            'HNUTER_DRCDA_SERVO_MODEL', 'identified'
        ).strip().lower()
        config_kwargs = {
            'prediction_dt_s': env_float('HNUTER_DRCDA_PREDICTION_DT_S', 0.01),
            'horizon_s': env_float('HNUTER_DRCDA_HORIZON_S', 0.18),
            'gauss_newton_iterations': int(env_float('HNUTER_DRCDA_ITERATIONS', 2)),
            'wrench_error_gain': env_float('HNUTER_DRCDA_WRENCH_GAIN', 8.0),
        }
        if model_name in ('ideal', 'instant', 'sitl_ideal'):
            config = DRCDAConfig.ideal_servos(**config_kwargs)
        else:
            config = DRCDAConfig(**config_kwargs)

        front_thrust_max = env_float('HNUTER_DRCDA_FRONT_MOTOR_MAX_N', 25.0)
        tail_thrust_max = env_float('HNUTER_DRCDA_TAIL_MOTOR_MAX_N', 50.0)
        config.thrust_max_n[:4] = front_thrust_max
        config.thrust_max_n[4] = tail_thrust_max
        config.thrust_min_n[4] = -tail_thrust_max if self.allow_tail_reverse else 0.0
        config.command_scale[ANGLE_COUNT:8] = front_thrust_max
        config.command_scale[8] = tail_thrust_max
        config.antiwindup_gain = env_float('HNUTER_DRCDA_ANTIWINDUP_GAIN', 0.35)

        wrench_model = HnuterWrenchModel(
            arm_half_span_m=self.l1,
            tail_x_m=-self.l2,
            reaction_torque_ratio_m=env_float('HNUTER_DRCDA_REACTION_RATIO_M', 0.016),
        )
        self.drcda = DRCDAAllocator(wrench_model, config)
        self._drcda_update_period_s = env_float('HNUTER_DRCDA_UPDATE_PERIOD_S', 0.01)
        self._drcda_update_period_s = float(np.clip(
            self._drcda_update_period_s, 0.002, 0.05
        ))
        self._drcda_model_name = model_name
        self._drcda_ready = True
        self.get_logger().info(
            'DRCDA initialized: '
            f'servo_model={model_name}, horizon={config.horizon_s * 1000.0:.0f}ms, '
            f'prediction_dt={config.prediction_dt_s * 1000.0:.1f}ms, '
            f'update_period={self._drcda_update_period_s * 1000.0:.1f}ms, '
            f'log={self.diagnostic_path}'
        )

    def _diagnostic_file_prefix(self):
        return 'hnuter_drcda_debug'

    def _diagnostic_extra_header(self):
        columns = ['drcda_status', 'drcda_solve_ms', 'drcda_iterations']
        columns += [
            'drcda_q_alpha_left_rad', 'drcda_q_beta_left_rad',
            'drcda_q_alpha_right_rad', 'drcda_q_beta_right_rad',
            'drcda_q_f_left_1_n', 'drcda_q_f_left_2_n',
            'drcda_q_f_right_1_n', 'drcda_q_f_right_2_n',
            'drcda_q_f_tail_n',
        ]
        columns += [f'drcda_predicted_wrench_{name}' for name in (
            'fx_n', 'fy_n', 'fz_n', 'tx_nm', 'ty_nm', 'tz_nm'
        )]
        columns += [f'drcda_wrench_residual_{name}' for name in (
            'fx_n', 'fy_n', 'fz_n', 'tx_nm', 'ty_nm', 'tz_nm'
        )]
        columns += ['drcda_wrench_rate_residual_norm']
        return columns

    def _diagnostic_extra_values(self):
        if not self._drcda_ready or self.drcda.last_result is None:
            return ['', float('nan'), 0] + [float('nan')] * 22
        result = self.drcda.last_result
        return [
            result.status,
            float(result.solve_time_ms),
            int(result.iterations),
            *[float(value) for value in result.predicted_state],
            *[float(value) for value in result.predicted_wrench],
            *[float(value) for value in result.wrench_residual],
            float(np.linalg.norm(result.wrench_rate_residual)),
        ]

    @staticmethod
    def _motor_control_to_thrust(control: float, bidirectional: bool = False) -> float:
        if not np.isfinite(control) or abs(control) <= 1e-9:
            return 0.0
        motor_constant = 8.54858e-05
        if bidirectional:
            velocity = abs(float(control)) * 1000.0
            thrust = motor_constant * velocity * velocity
            return math.copysign(thrust, float(control))
        velocity = 10.0 + float(np.clip(control, 0.0, 1.0)) * 990.0
        return motor_constant * velocity * velocity

    def _preferred_drcda_command(
        self,
        motor_controls,
        alpha1: float,
        alpha2: float,
        theta1: float,
        theta2: float,
    ) -> np.ndarray:
        controls = np.asarray(motor_controls, dtype=float)
        if controls.size < 5:
            controls = np.pad(controls, (0, 5 - controls.size))
        # Logical thrust order in DRCDA is left pair, right pair, tail.
        return np.array([
            alpha1,
            theta1,
            alpha2,
            theta2,
            self._motor_control_to_thrust(controls[2]),
            self._motor_control_to_thrust(controls[3]),
            self._motor_control_to_thrust(controls[0]),
            self._motor_control_to_thrust(controls[1]),
            self._motor_control_to_thrust(controls[4], self.allow_tail_reverse),
        ], dtype=float)

    def _active_drcda_angle_limits(self) -> np.ndarray:
        elapsed_s = (
            self._drcda_current_time_s - self._takeoff_lock_start_time_s
            if self._takeoff_lock_start_time_s is not None else 100.0
        )
        if elapsed_s < self.takeoff_tilt_suppress_time_s:
            alpha_limit = self.takeoff_tilt_limit_rad
            beta_limit = self.takeoff_tilt_limit_rad
        elif elapsed_s < self.takeoff_xy_lock_time_s:
            alpha_limit = self.xy_lock_tilt_limit_rad
            beta_limit = self.xy_lock_tilt_limit_rad
        else:
            alpha_limit = self.alpha_limit_rad
            beta_limit = self.theta_limit_rad
        return np.array(
            [alpha_limit, beta_limit, alpha_limit, beta_limit], dtype=float
        )

    def _apply_drcda_antiwindup(self, wrench_residual: np.ndarray, dt: float) -> None:
        config = self.drcda.config
        normalized_error = wrench_residual / config.wrench_scale
        if np.linalg.norm(normalized_error) < 0.02:
            return

        delta_force_body = np.array([
            wrench_residual[0] / max(abs(self.allocator_force_x_sign), 1e-6),
            wrench_residual[1] / (
                self.allocator_force_y_sign
                if abs(self.allocator_force_y_sign) > 1e-6 else 1.0
            ),
            -wrench_residual[2],
        ])
        delta_acceleration_ned = self.R_ned_frd @ delta_force_body / self.mass
        position_integral_gain = np.array([0.0, 0.0, 3.0])
        active = position_integral_gain > 1e-6
        correction = np.zeros(3)
        correction[active] = (
            -config.antiwindup_gain
            * delta_acceleration_ned[active]
            / position_integral_gain[active]
            * dt
        )
        self.integral_pos_error += correction
        self.integral_pos_error[0:2] = np.clip(self.integral_pos_error[0:2], -1.0, 1.0)
        self.integral_pos_error[2] = float(np.clip(self.integral_pos_error[2], -2.0, 2.0))

    def publish_px4_equivalent_direct_commands(self, current_time: float, dt: float):
        self._drcda_active_call = True
        self._drcda_current_time_s = float(current_time)
        self._drcda_current_dt_s = float(dt)
        try:
            return super().publish_px4_equivalent_direct_commands(current_time, dt)
        finally:
            self._drcda_active_call = False

    def publish_idle_direct_actuator_setpoint(self):
        if self._drcda_ready:
            self.drcda.reset()
            self._drcda_accumulated_dt_s = 0.0
        return super().publish_idle_direct_actuator_setpoint()

    def publish_direct_actuator_setpoint(
        self,
        motor_controls,
        alpha1,
        alpha2,
        theta1,
        theta2,
    ):
        drcda_active = (
            self._drcda_ready
            and self._drcda_active_call
            and self.armed
            and self.takeoff_requested
        )
        if not drcda_active:
            if self._drcda_ready:
                self.drcda.reset(
                    angle_state=[alpha1, theta1, alpha2, theta2],
                    thrust_state=[0.0] * 5,
                )
            return DirectController.publish_direct_actuator_setpoint(
                self, motor_controls, alpha1, alpha2, theta1, theta2
            )

        preferred = self._preferred_drcda_command(
            motor_controls, alpha1, alpha2, theta1, theta2
        )
        self._drcda_accumulated_dt_s += self._drcda_current_dt_s
        solve_due = (
            self.drcda.last_result is None
            or self._drcda_accumulated_dt_s >= self._drcda_update_period_s
        )
        if solve_due:
            allocation_dt = max(
                self._drcda_accumulated_dt_s, self._drcda_current_dt_s
            )
            result = self.drcda.allocate(
                desired_wrench=self.last_W,
                dt=allocation_dt,
                preferred_command=preferred,
                active_angle_limits=self._active_drcda_angle_limits(),
            )
            self._drcda_accumulated_dt_s = 0.0
            self._apply_drcda_antiwindup(result.wrench_residual, allocation_dt)
        else:
            result = self.drcda.last_result
        command = self.drcda.command
        logical_thrust = command[ANGLE_COUNT:]
        output_motor_controls = [
            self._thrust_to_normalized_motor_control(logical_thrust[2]),
            self._thrust_to_normalized_motor_control(logical_thrust[3]),
            self._thrust_to_normalized_motor_control(logical_thrust[0]),
            self._thrust_to_normalized_motor_control(logical_thrust[1]),
            (
                self._thrust_to_normalized_bidirectional_motor_control(logical_thrust[4])
                if self.allow_tail_reverse
                else self._thrust_to_normalized_motor_control(logical_thrust[4])
            ),
        ]

        self._alpha1_cmd = float(command[0])
        self._theta1_cmd = float(command[1])
        self._alpha2_cmd = float(command[2])
        self._theta2_cmd = float(command[3])
        self.last_F1 = float(logical_thrust[0] + logical_thrust[1])
        self.last_F2 = float(logical_thrust[2] + logical_thrust[3])
        self.last_F3 = float(logical_thrust[4])

        return DirectController.publish_direct_actuator_setpoint(
            self,
            output_motor_controls,
            self._alpha1_cmd,
            self._alpha2_cmd,
            self._theta1_cmd,
            self._theta2_cmd,
        )


def main(args=None):
    rclpy.init(args=args)
    controller = HnuterDRCDAController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('DRCDA controller stopped.')
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
