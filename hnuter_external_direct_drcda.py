#!/usr/bin/env python3
"""Hnuter direct external controller using DRCDA control allocation.

The flight-control and safety state machine comes from the established direct
debug controller.  This class replaces only its algebraic actuator inverse with
delay-aware, reachability-constrained differential allocation.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np

from hnuter_drcda import (
    ACTUATOR_COUNT,
    ANGLE_COUNT,
    ALLOCATOR_VARIANTS,
    BasicDifferentialAllocator,
    DRCDAAllocator,
    DRCDAConfig,
    HnuterWrenchModel,
    configure_allocator_variant,
)
from hnuter_external_direct_controller_debug import (
    HnuterController as DirectController,
    env_float,
)

import rclpy


class HnuterDRCDAController(DirectController):
    def _node_name(self):
        variant = getattr(self, '_drcda_variant', 'full')
        return f'hnuter_controller_direct_{variant}'

    def _gamepad_attitude_axis_toggle_enabled(self):
        return True

    def __init__(self) -> None:
        os.environ.setdefault(
            'HNUTER_TUNING_FILE',
            str(Path(__file__).resolve().parent / 'config/no_delay_drcda_tuning.json'),
        )
        self._drcda_variant = os.environ.get(
            'HNUTER_DRCDA_VARIANT', 'full'
        ).strip().lower()
        if self._drcda_variant not in ALLOCATOR_VARIANTS:
            choices = ', '.join(ALLOCATOR_VARIANTS)
            raise ValueError(
                f'unknown HNUTER_DRCDA_VARIANT={self._drcda_variant!r}; '
                f'choose from {choices}'
            )
        self._drcda_ready = False
        self._drcda_active_call = False
        self._drcda_current_time_s = 0.0
        self._drcda_current_dt_s = 0.01
        self._drcda_accumulated_dt_s = 0.0
        self._drcda_estimator_reset_pending = False
        super().__init__()

        # Retain only the identified directional static gains. The old pure
        # delay and first-order lag fit are not used by the active model.
        # Command slew limits remain active independently.
        model_name = 'identified_gain_no_delay'
        config_kwargs = {
            'prediction_dt_s': env_float('HNUTER_DRCDA_PREDICTION_DT_S', 0.01),
            'horizon_s': env_float('HNUTER_DRCDA_HORIZON_S', 0.18),
            'gauss_newton_iterations': int(env_float('HNUTER_DRCDA_ITERATIONS', 2)),
            'wrench_error_gain': env_float('HNUTER_DRCDA_WRENCH_GAIN', 8.0),
        }
        config = DRCDAConfig.identified_gain_no_delay(**config_kwargs)
        configure_allocator_variant(config, self._drcda_variant)

        front_thrust_max = env_float('HNUTER_DRCDA_FRONT_MOTOR_MAX_N', 25.0)
        tail_forward_max = env_float(
            'HNUTER_DRCDA_TAIL_FORWARD_MAX_N', self.tail_thrust_max_forward_n
        )
        tail_reverse_max = env_float(
            'HNUTER_DRCDA_TAIL_REVERSE_MAX_N', self.tail_thrust_max_reverse_n
        )
        config.thrust_max_n[:4] = front_thrust_max
        config.thrust_max_n[4] = tail_forward_max
        config.thrust_min_n[4] = -tail_reverse_max if self.allow_tail_reverse else 0.0
        config.command_scale[ANGLE_COUNT:8] = front_thrust_max
        config.command_scale[8] = max(tail_forward_max, tail_reverse_max)
        config.motor_tau_up_s[4] = env_float('HNUTER_DRCDA_TAIL_TAU_UP_S', 0.05)
        config.motor_tau_down_s[4] = env_float('HNUTER_DRCDA_TAIL_TAU_DOWN_S', 0.05)
        # The optimizer's tail move penalty suppresses brief sign chatter;
        # retain enough rate here to establish takeoff pitch authority quickly.
        config.thrust_command_rate_n_s[4] = env_float(
            'HNUTER_DRCDA_TAIL_COMMAND_RATE_N_S', 100.0
        )
        config.antiwindup_gain = env_float('HNUTER_DRCDA_ANTIWINDUP_GAIN', 0.35)

        wrench_model = HnuterWrenchModel(
            arm_half_span_m=self.l1,
            tail_x_m=-self.l2,
            reaction_torque_ratio_m=env_float('HNUTER_DRCDA_REACTION_RATIO_M', 0.016),
        )
        allocator_type = (
            BasicDifferentialAllocator
            if self._drcda_variant == 'basic_da'
            else DRCDAAllocator
        )
        self.drcda = allocator_type(wrench_model, config)
        self._drcda_update_period_s = env_float('HNUTER_DRCDA_UPDATE_PERIOD_S', 0.01)
        self._drcda_update_period_s = float(np.clip(
            self._drcda_update_period_s, 0.002, 0.05
        ))
        self._drcda_model_name = model_name
        self._drcda_ready = True
        self._load_tuning_file(force=True)
        self.get_logger().info(
            'DRCDA initialized: '
            f'variant={self._drcda_variant}, servo_model={model_name}, '
            f'horizon={config.horizon_s * 1000.0:.0f}ms, '
            f'prediction_dt={config.prediction_dt_s * 1000.0:.1f}ms, '
            f'update_period={self._drcda_update_period_s * 1000.0:.1f}ms, '
            f'log={self.diagnostic_path}'
        )

    def _apply_tuning(self, data: dict):
        super()._apply_tuning(data)
        if not getattr(self, '_drcda_ready', False):
            return

        config = self.drcda.config
        config.horizon_s = max(
            self._tuning_float(data, 'drcda_horizon_s', config.horizon_s),
            config.prediction_dt_s,
        )
        config.wrench_error_gain = max(
            self._tuning_float(
                data, 'drcda_wrench_error_gain', config.wrench_error_gain
            ),
            0.0,
        )
        config.wrench_ff_tau_s = max(
            self._tuning_float(data, 'drcda_wrench_ff_tau_s', config.wrench_ff_tau_s),
            0.0,
        )
        config.wrench_rate_weight = max(
            self._tuning_float(
                data, 'drcda_wrench_rate_weight', config.wrench_rate_weight
            ),
            0.0,
        )
        config.wrench_weight = np.maximum(
            self._tuning_array(data, 'drcda_wrench_weight', config.wrench_weight),
            0.0,
        )
        config.command_move_weight = np.maximum(
            self._tuning_array(
                data, 'drcda_command_move_weight', config.command_move_weight
            ),
            0.0,
        )
        config.command_preference_weight = np.maximum(
            self._tuning_array(
                data,
                'drcda_command_preference_weight',
                config.command_preference_weight,
            ),
            0.0,
        )
        config.antiwindup_gain = max(
            self._tuning_float(
                data, 'drcda_antiwindup_gain', config.antiwindup_gain
            ),
            0.0,
        )
        # Tuning files contain the full-controller horizon.  Reapply the
        # selected ablation last so a reload cannot silently turn a removed
        # reachability term back on.
        configure_allocator_variant(config, self._drcda_variant)
        self.get_logger().info(
            'DRCDA 在线参数已加载: '
            f'horizon={config.horizon_s:.3f}s, '
            f'wrench_gain={config.wrench_error_gain:.2f}, '
            f'ff_tau={config.wrench_ff_tau_s:.3f}s, '
            f'rate_weight={config.wrench_rate_weight:.3f}'
        )

    def _diagnostic_file_prefix(self):
        return (
            f'ablation/{self._drcda_variant}/'
            f'hnuter_{self._drcda_variant}_debug'
        )

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

    def _motor_control_to_thrust(self, control: float, bidirectional: bool = False) -> float:
        if not np.isfinite(control) or abs(control) <= 1e-9:
            return 0.0
        if bidirectional:
            # This reconstructs the force produced by the symmetric Gazebo
            # motor constant. Directional command limits are applied in the
            # allocator, not by changing the motor constant per sign.
            thrust = (
                self.tail_thrust_max_forward_n
                * min(abs(float(control)), 1.0) ** 2
            )
            return math.copysign(thrust, float(control))
        motor_constant = 8.54858e-05
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
        physical_angles = np.array([alpha1, theta1, alpha2, theta2], dtype=float)
        if getattr(self, '_drcda_ready', False):
            config = self.drcda.config
            gains = np.where(
                physical_angles >= 0.0,
                config.servo_gain_positive,
                config.servo_gain_negative,
            )
            servo_inputs = physical_angles / np.maximum(gains, 1e-6)
        else:
            servo_inputs = physical_angles
        # Logical thrust order in DRCDA is left pair, right pair, tail. The
        # first four preferred values are actuator inputs, not physical angles.
        return np.array([
            servo_inputs[0],
            servo_inputs[1],
            servo_inputs[2],
            servo_inputs[3],
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
        position_integral_gain = self.direct_pos_Ki_ned
        active = position_integral_gain > 1e-6
        correction = np.zeros(3)
        correction[active] = (
            -config.antiwindup_gain
            * delta_acceleration_ned[active]
            / position_integral_gain[active]
            * dt
        )
        self.integral_pos_error += correction
        self.integral_pos_error = np.clip(
            self.integral_pos_error,
            -self.direct_pos_integral_limit_ned,
            self.direct_pos_integral_limit_ned,
        )

    def publish_px4_equivalent_direct_commands(self, current_time: float, dt: float):
        self._drcda_active_call = True
        self._drcda_current_time_s = float(current_time)
        self._drcda_current_dt_s = float(dt)
        try:
            return super().publish_px4_equivalent_direct_commands(current_time, dt)
        finally:
            self._drcda_active_call = False

    def _apply_estimator_yaw_reset(
        self,
        delta_yaw_enu: float,
        reset_counter: int,
    ) -> None:
        super()._apply_estimator_yaw_reset(delta_yaw_enu, reset_counter)
        if self._drcda_ready:
            self._drcda_estimator_reset_pending = True
            self._drcda_accumulated_dt_s = self._drcda_update_period_s

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
            if self._drcda_estimator_reset_pending:
                self.drcda.synchronize_wrench_reference(self.last_W)
                self._drcda_estimator_reset_pending = False
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
        # Gazebo's four JointPositionController instances follow their input
        # almost instantaneously and do not implement the identified static
        # gain. Drive them with the allocator's current physical servo state so
        # the simulated plant matches identified_gain_no_delay.
        # The standalone hardware controller still publishes actuator input
        # commands and is intentionally unaffected by this SITL-only emulation.
        servo_state = self.drcda.state[:ANGLE_COUNT]
        logical_thrust = command[ANGLE_COUNT:]
        output_motor_controls = [
            self._thrust_to_normalized_motor_control(logical_thrust[2]),
            self._thrust_to_normalized_motor_control(logical_thrust[3]),
            self._thrust_to_normalized_motor_control(logical_thrust[0]),
            self._thrust_to_normalized_motor_control(logical_thrust[1]),
            (
                self._tail_thrust_to_normalized_bidirectional_motor_control(logical_thrust[4])
                if self.allow_tail_reverse
                else self._thrust_to_normalized_motor_control(logical_thrust[4])
            ),
        ]

        self._alpha1_cmd = float(servo_state[0])
        self._theta1_cmd = float(servo_state[1])
        self._alpha2_cmd = float(servo_state[2])
        self._theta2_cmd = float(servo_state[3])
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
