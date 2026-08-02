#!/usr/bin/env python3
"""Hnuter direct external controller using DRCDA control allocation.

The flight-control and safety state machine comes from the established direct
debug controller.  This class replaces only its algebraic actuator inverse with
dynamic-reachability-constrained differential allocation.
"""

from __future__ import annotations

import copy
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
    PaperDifferentialAllocator,
    configure_allocator_variant,
)
from hnuter_external_direct_controller_debug import (
    HnuterController as DirectController,
    env_float,
)
from hnuter_position_control import (
    body_frame_horizontal_feedback_ned,
    heading_rotation_ned_body_xy,
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
            str(
                Path(__file__).resolve().parent
                / 'config'
                / 'identified_delay_damped_xy.json'
            ),
        )
        self.direct_pos_body_frame_xy_enabled = True
        self.direct_pos_Kp_body_xy = np.array([2.0, 1.0])
        self.direct_pos_Kd_body_xy = np.array([2.8, 1.8])
        self.direct_pos_Ki_body_xy = np.array([0.2, 0.1])
        self.direct_pos_deadband_body_xy_m = np.zeros(2)
        self.direct_vel_deadband_body_xy_mps = np.zeros(2)
        self.direct_manual_max_acc_body_xy = np.array([3.0, 2.0])
        self.direct_body_force_trim_xy_n = np.array([0.0, 1.35])
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
        super().__init__()

        model_name = os.environ.get(
            'HNUTER_DRCDA_SERVO_MODEL', 'identified'
        ).strip().lower()
        config_kwargs = {
            'prediction_dt_s': env_float('HNUTER_DRCDA_PREDICTION_DT_S', 0.01),
            'prediction_far_dt_s': env_float(
                'HNUTER_DRCDA_PREDICTION_FAR_DT_S', 0.02
            ),
            'horizon_s': env_float('HNUTER_DRCDA_HORIZON_S', 0.18),
            'motor_block_switch_s': env_float(
                'HNUTER_DRCDA_MOTOR_BLOCK_SWITCH_S', 0.10
            ),
            'gauss_newton_iterations': int(env_float('HNUTER_DRCDA_ITERATIONS', 2)),
            'wrench_error_gain': env_float('HNUTER_DRCDA_WRENCH_GAIN', 12.0),
            'lm_damping': env_float('HNUTER_DRCDA_LM_DAMPING', 1.0e-3),
            'paper_delay_aware_motor_residual_enabled': (
                env_float(
                    'HNUTER_DRCDA_DELAY_MOTOR_RESIDUAL_ENABLED',
                    0.0,
                ) >= 0.5
            ),
        }
        if model_name in ('ideal', 'instant', 'sitl_ideal'):
            config = DRCDAConfig.ideal_servos(**config_kwargs)
        else:
            config = DRCDAConfig(**config_kwargs)
        configure_allocator_variant(config, self._drcda_variant)

        servo_move_scale = env_float(
            'HNUTER_DRCDA_SERVO_MOVE_WEIGHT_SCALE', 10.0
        )
        motor_move_scale = env_float(
            'HNUTER_DRCDA_MOTOR_MOVE_WEIGHT_SCALE', 1.0
        )
        config.command_move_weight[:ANGLE_COUNT] *= servo_move_scale
        config.command_move_weight[ANGLE_COUNT:] *= motor_move_scale
        config.servo_command_rate_rad_s *= env_float(
            'HNUTER_DRCDA_SERVO_COMMAND_SLEW_SCALE', 0.5
        )
        config.late_transition_weight *= env_float(
            'HNUTER_DRCDA_LATE_TRANSITION_WEIGHT_SCALE', 100.0
        )
        config.late_trim_weight *= env_float(
            'HNUTER_DRCDA_LATE_TRIM_WEIGHT_SCALE', 25.0
        )

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
        if self._drcda_variant == 'basic_da':
            self.drcda = BasicDifferentialAllocator(wrench_model, config)
        elif self._drcda_variant in ('full', 'ada', 'nda', 'pda'):
            paper_method = (
                'pda' if self._drcda_variant == 'full'
                else self._drcda_variant
            )
            self.drcda = PaperDifferentialAllocator(
                wrench_model, config, paper_method
            )
            if self._drcda_variant == 'full':
                self._drcda_paper_allocator = self.drcda
                self._drcda_large_attitude_allocator = DRCDAAllocator(
                    wrench_model,
                    copy.deepcopy(config),
                )
        else:
            self.drcda = DRCDAAllocator(wrench_model, config)
        self._drcda_update_period_s = env_float('HNUTER_DRCDA_UPDATE_PERIOD_S', 0.01)
        self._drcda_update_period_s = float(np.clip(
            self._drcda_update_period_s, 0.002, 0.05
        ))
        self._drcda_model_name = model_name
        self._drcda_ready = True
        self.get_logger().info(
            'DRCDA initialized: '
            f'variant={self._drcda_variant}, servo_model={model_name}, '
            f'horizon={config.horizon_s * 1000.0:.0f}ms, '
            f'prediction_dt={config.prediction_dt_s * 1000.0:.1f}ms, '
            f'update_period={self._drcda_update_period_s * 1000.0:.1f}ms, '
            f'log={self.diagnostic_path}'
        )
        if self._drcda_variant == 'full':
            self.get_logger().info(
                'Full allocator scheduling: paper PDA for nominal/aggressive '
                'flight, finite-time reachable allocation for automatic '
                'large-attitude trajectories.'
            )
        self.get_logger().info(
            'DRCDA body-frame XY position loop: '
            f'enabled={self.direct_pos_body_frame_xy_enabled}, '
            f'Kp={self.direct_pos_Kp_body_xy.tolist()}, '
            f'Kd={self.direct_pos_Kd_body_xy.tolist()}, '
            f'Ki={self.direct_pos_Ki_body_xy.tolist()}, '
            f'force_trim_N={self.direct_body_force_trim_xy_n.tolist()}'
        )

    def _tuning_snapshot(self) -> dict:
        snapshot = super()._tuning_snapshot()
        snapshot.update({
            'direct_pos_body_frame_xy_enabled': bool(
                self.direct_pos_body_frame_xy_enabled
            ),
            'direct_pos_Kp_body_xy': self.direct_pos_Kp_body_xy.tolist(),
            'direct_pos_Kd_body_xy': self.direct_pos_Kd_body_xy.tolist(),
            'direct_pos_Ki_body_xy': self.direct_pos_Ki_body_xy.tolist(),
            'direct_pos_deadband_body_xy_m': (
                self.direct_pos_deadband_body_xy_m.tolist()
            ),
            'direct_vel_deadband_body_xy_mps': (
                self.direct_vel_deadband_body_xy_mps.tolist()
            ),
            'direct_manual_max_acc_body_xy': (
                self.direct_manual_max_acc_body_xy.tolist()
            ),
            'direct_body_force_trim_xy_n': self.direct_body_force_trim_xy_n.tolist(),
        })
        return snapshot

    def _apply_tuning(self, data: dict):
        super()._apply_tuning(data)
        self.direct_pos_body_frame_xy_enabled = self._tuning_bool(
            data,
            'direct_pos_body_frame_xy_enabled',
            self.direct_pos_body_frame_xy_enabled,
        )
        self.direct_pos_Kp_body_xy = np.maximum(
            self._tuning_array(
                data, 'direct_pos_Kp_body_xy', self.direct_pos_Kp_body_xy
            ),
            0.0,
        )
        self.direct_pos_Kd_body_xy = np.maximum(
            self._tuning_array(
                data, 'direct_pos_Kd_body_xy', self.direct_pos_Kd_body_xy
            ),
            0.0,
        )
        self.direct_pos_Ki_body_xy = np.maximum(
            self._tuning_array(
                data, 'direct_pos_Ki_body_xy', self.direct_pos_Ki_body_xy
            ),
            0.0,
        )
        self.direct_pos_deadband_body_xy_m = np.maximum(
            self._tuning_array(
                data,
                'direct_pos_deadband_body_xy_m',
                self.direct_pos_deadband_body_xy_m,
            ),
            0.0,
        )
        self.direct_vel_deadband_body_xy_mps = np.maximum(
            self._tuning_array(
                data,
                'direct_vel_deadband_body_xy_mps',
                self.direct_vel_deadband_body_xy_mps,
            ),
            0.0,
        )
        self.direct_manual_max_acc_body_xy = np.maximum(
            self._tuning_array(
                data,
                'direct_manual_max_acc_body_xy',
                self.direct_manual_max_acc_body_xy,
            ),
            0.05,
        )
        self.direct_body_force_trim_xy_n = self._tuning_array(
            data,
            'direct_body_force_trim_xy_n',
            self.direct_body_force_trim_xy_n,
        )

    def _direct_position_acceleration_ned(
        self,
        acc_ff_ned: np.ndarray,
        pos_error_ned: np.ndarray,
        vel_error_ned: np.ndarray,
        xy_lock_active: bool,
    ) -> np.ndarray:
        if not self.direct_pos_body_frame_xy_enabled:
            return super()._direct_position_acceleration_ned(
                acc_ff_ned,
                pos_error_ned,
                vel_error_ned,
                xy_lock_active,
            )

        kp_body_xy = self.direct_pos_Kp_body_xy.copy()
        if xy_lock_active:
            kp_body_xy *= 0.8
        acceleration_ned = acc_ff_ned.copy()
        acceleration_ned[:2] += body_frame_horizontal_feedback_ned(
            self.R_ned_frd,
            pos_error_ned[:2],
            vel_error_ned[:2],
            self.integral_pos_error[:2],
            kp_body_xy,
            self.direct_pos_Kd_body_xy,
            self.direct_pos_Ki_body_xy,
            getattr(
                self, 'direct_pos_deadband_body_xy_m', np.zeros(2)
            ),
            getattr(
                self, 'direct_vel_deadband_body_xy_mps', np.zeros(2)
            ),
        )
        acceleration_ned[2] += (
            self.direct_pos_Kp_ned[2] * pos_error_ned[2]
            + self.direct_pos_Kd_ned[2] * vel_error_ned[2]
            + self.direct_pos_Ki_ned[2] * self.integral_pos_error[2]
        )
        return acceleration_ned

    def _limit_direct_horizontal_acceleration_ned(
        self,
        acceleration_xy_ned: np.ndarray,
        max_acc_xy: float,
    ) -> np.ndarray:
        if not self.direct_pos_body_frame_xy_enabled:
            return super()._limit_direct_horizontal_acceleration_ned(
                acceleration_xy_ned,
                max_acc_xy,
            )
        heading = heading_rotation_ned_body_xy(self.R_ned_frd)
        body_limit = np.full(2, max_acc_xy)
        if self.auto_traj_mode == 'hover':
            body_limit = np.minimum(
                body_limit,
                self.direct_manual_max_acc_body_xy,
            )
        acceleration_body_xy = np.clip(
            heading.T @ acceleration_xy_ned,
            -body_limit,
            body_limit,
        )
        return heading @ acceleration_body_xy

    def _apply_direct_body_force_trim(self, force_body: np.ndarray) -> np.ndarray:
        if not self.direct_pos_body_frame_xy_enabled:
            return force_body
        trimmed_force = force_body.copy()
        trimmed_force[:2] += self.direct_body_force_trim_xy_n
        return trimmed_force

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
        columns += [f'drcda_servo_authority_{index}' for index in range(4)]
        columns += [f'drcda_servo_gated_{index}' for index in range(4)]
        columns += [f'drcda_servo_rate_active_fraction_{index}' for index in range(4)]
        columns += [f'drcda_servo_angle_active_fraction_{index}' for index in range(4)]
        columns += ['drcda_objective', 'drcda_lm_damping']
        columns += [f'drcda_late_thrust_{index}_n' for index in range(5)]
        return columns

    def _diagnostic_extra_values(self):
        if not self._drcda_ready or self.drcda.last_result is None:
            return ['', float('nan'), 0] + [float('nan')] * 45
        result = self.drcda.last_result
        return [
            result.status,
            float(result.solve_time_ms),
            int(result.iterations),
            *[float(value) for value in result.predicted_state],
            *[float(value) for value in result.predicted_wrench],
            *[float(value) for value in result.wrench_residual],
            float(np.linalg.norm(result.wrench_rate_residual)),
            *[float(value) for value in result.servo_authority],
            *[int(value) for value in result.servo_gated],
            *[float(value) for value in result.servo_rate_active_fraction],
            *[float(value) for value in result.servo_angle_active_fraction],
            float(result.objective),
            float(result.lm_damping),
            *[float(value) for value in result.late_thrust_command],
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
        correction = np.zeros(3)
        if self.direct_pos_body_frame_xy_enabled:
            heading = heading_rotation_ned_body_xy(self.R_ned_frd)
            delta_acceleration_body_xy = heading.T @ delta_acceleration_ned[:2]
            active_xy = self.direct_pos_Ki_body_xy > 1e-6
            correction_body_xy = np.zeros(2)
            correction_body_xy[active_xy] = (
                -config.antiwindup_gain
                * delta_acceleration_body_xy[active_xy]
                / self.direct_pos_Ki_body_xy[active_xy]
                * dt
            )
            correction[:2] = heading @ correction_body_xy
        else:
            active_xy = self.direct_pos_Ki_ned[:2] > 1e-6
            correction_xy = correction[:2]
            correction_xy[active_xy] = (
                -config.antiwindup_gain
                * delta_acceleration_ned[:2][active_xy]
                / self.direct_pos_Ki_ned[:2][active_xy]
                * dt
            )
        if self.direct_pos_Ki_ned[2] > 1e-6:
            correction[2] = (
                -config.antiwindup_gain
                * delta_acceleration_ned[2]
                / self.direct_pos_Ki_ned[2]
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

    def publish_idle_direct_actuator_setpoint(self):
        if self._drcda_ready:
            self.drcda.reset()
            for name in (
                '_drcda_paper_allocator',
                '_drcda_large_attitude_allocator',
            ):
                allocator = getattr(self, name, None)
                if allocator is not None and allocator is not self.drcda:
                    allocator.reset()
            self._drcda_accumulated_dt_s = 0.0
        return super().publish_idle_direct_actuator_setpoint()

    def _select_full_allocator(self) -> None:
        if self._drcda_variant != 'full':
            return
        target = (
            self._drcda_large_attitude_allocator
            if self.auto_traj_mode == 'attitude'
            else self._drcda_paper_allocator
        )
        if target is self.drcda:
            return
        previous = self.drcda
        target.reset(
            angle_state=previous.state[:ANGLE_COUNT],
            thrust_state=previous.state[ANGLE_COUNT:],
        )
        self.drcda = target
        self._drcda_accumulated_dt_s = 0.0
        mode = (
            'finite-time reachable'
            if target is self._drcda_large_attitude_allocator
            else 'paper PDA'
        )
        self.get_logger().info(f'Full allocator switched to {mode}.')

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

        self._select_full_allocator()
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

        # DRCDA optimizes calibrated physical target angles. Convert them back
        # through g^-1 before publishing in the simulator command space.
        self._alpha1_cmd = self.drcda.servo_target_to_command(0, float(command[0]))
        self._theta1_cmd = self.drcda.servo_target_to_command(1, float(command[1]))
        self._alpha2_cmd = self.drcda.servo_target_to_command(2, float(command[2]))
        self._theta2_cmd = self.drcda.servo_target_to_command(3, float(command[3]))
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
