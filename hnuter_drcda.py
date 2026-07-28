#!/usr/bin/env python3
"""Delay-aware reachability-constrained differential allocation for Hnuter.

The allocator keeps the paper's differential-allocation structure, but replaces
fixed actuator-rate bounds with a command-history predictor.  A short horizon
is retained while one command is held over the horizon (move blocking), keeping
the online problem small enough to solve with NumPy only.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from typing import Iterable

import numpy as np


ANGLE_COUNT = 4
THRUST_COUNT = 5
ACTUATOR_COUNT = ANGLE_COUNT + THRUST_COUNT
DECISION_COUNT = ANGLE_COUNT + 2 * THRUST_COUNT
ALLOCATOR_VARIANTS = (
    'full',
    'basic_da',
    'no_delay',
    'no_horizon',
    'no_physical_rate',
    'no_command_slew',
    'no_reachability_gate',
    'no_multirate',
)


def _array(values: Iterable[float], count: int, name: str) -> np.ndarray:
    result = np.asarray(tuple(values), dtype=float)
    if result.shape != (count,):
        raise ValueError(f'{name} must contain {count} values')
    return result


def _clip_scalar(value: float, lower: float, upper: float) -> float:
    return min(max(float(value), float(lower)), float(upper))


def _cross3(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    return np.array([
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ])


@dataclass
class DRCDAConfig:
    prediction_dt_s: float = 0.01
    prediction_far_dt_s: float = 0.02
    prediction_near_duration_s: float = 0.04
    horizon_s: float = 0.0
    horizon_tau_factor: float = 0.8
    motor_block_switch_s: float = 0.10
    gauss_newton_iterations: int = 2
    wrench_error_gain: float = 8.0
    wrench_ff_tau_s: float = 0.04
    wrench_rate_weight: float = 0.15
    wrench_scale: np.ndarray = field(default_factory=lambda: np.array(
        [80.0, 80.0, 100.0, 12.0, 12.0, 12.0], dtype=float
    ))
    wrench_weight: np.ndarray = field(default_factory=lambda: np.array(
        [1.0, 1.0, 2.0, 3.0, 3.0, 2.0], dtype=float
    ))
    command_move_weight: np.ndarray = field(default_factory=lambda: np.array(
        [0.020, 0.030, 0.020, 0.030, 0.004, 0.004, 0.004, 0.004, 0.006],
        dtype=float,
    ))
    command_preference_weight: np.ndarray = field(default_factory=lambda: np.array(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        dtype=float,
    ))
    command_scale: np.ndarray = field(default_factory=lambda: np.array(
        [math.pi, math.pi, math.pi, math.pi, 25.0, 25.0, 25.0, 25.0, 50.0],
        dtype=float,
    ))
    servo_gain_positive: np.ndarray = field(default_factory=lambda: np.array(
        [1.404, 0.705, 1.404, 0.705], dtype=float
    ))
    servo_gain_negative: np.ndarray = field(default_factory=lambda: np.array(
        [1.423, 0.695, 1.423, 0.695], dtype=float
    ))
    servo_tau_positive_s: np.ndarray = field(default_factory=lambda: np.array(
        [0.076, 0.153, 0.076, 0.153], dtype=float
    ))
    servo_tau_negative_s: np.ndarray = field(default_factory=lambda: np.array(
        [0.065, 0.149, 0.065, 0.149], dtype=float
    ))
    servo_delay_positive_s: np.ndarray = field(default_factory=lambda: np.array(
        [0.110, 0.156, 0.110, 0.156], dtype=float
    ))
    servo_delay_negative_s: np.ndarray = field(default_factory=lambda: np.array(
        [0.106, 0.137, 0.106, 0.137], dtype=float
    ))
    servo_rate_positive_rad_s: np.ndarray = field(default_factory=lambda: np.array(
        [6.082, 3.252, 6.082, 3.252], dtype=float
    ))
    servo_rate_negative_rad_s: np.ndarray = field(default_factory=lambda: np.array(
        [5.419, 2.886, 5.419, 2.886], dtype=float
    ))
    servo_state_limit_rad: np.ndarray = field(default_factory=lambda: np.array(
        [math.radians(185.0), math.pi, math.radians(185.0), math.pi],
        dtype=float,
    ))
    servo_command_limit_rad: np.ndarray = field(default_factory=lambda: np.array(
        [math.radians(185.0), math.pi, math.radians(185.0), math.pi],
        dtype=float,
    ))
    servo_command_rate_rad_s: np.ndarray = field(default_factory=lambda: np.array(
        [8.0, 4.0, 8.0, 4.0], dtype=float
    ))
    thrust_min_n: np.ndarray = field(default_factory=lambda: np.array(
        [0.0, 0.0, 0.0, 0.0, -50.0], dtype=float
    ))
    thrust_max_n: np.ndarray = field(default_factory=lambda: np.array(
        [25.0, 25.0, 25.0, 25.0, 50.0], dtype=float
    ))
    thrust_command_rate_n_s: np.ndarray = field(default_factory=lambda: np.array(
        [2500.0, 2500.0, 2500.0, 2500.0, 3500.0], dtype=float
    ))
    motor_tau_up_s: np.ndarray = field(default_factory=lambda: np.full(5, 0.001))
    motor_tau_down_s: np.ndarray = field(default_factory=lambda: np.full(5, 0.002))
    motor_force_rate_floor_n_s: float = 50.0
    motor_force_rate_cap_n_s: float = 6000.0
    antiwindup_gain: float = 0.35
    reachability_gate_threshold: float = 0.05
    checkpoint_times_s: np.ndarray = field(default_factory=lambda: np.array(
        [0.04, 0.12], dtype=float
    ))
    checkpoint_cost_scale: np.ndarray = field(default_factory=lambda: np.array(
        [0.35, 0.65, 1.0], dtype=float
    ))
    checkpoint_wrench_weight_short: np.ndarray = field(default_factory=lambda: np.array(
        [0.5, 0.5, 1.5, 4.0, 4.0, 2.0], dtype=float
    ))
    late_transition_weight: np.ndarray = field(default_factory=lambda: np.array(
        [0.004, 0.004, 0.004, 0.004, 0.006], dtype=float
    ))
    late_trim_weight: np.ndarray = field(default_factory=lambda: np.array(
        [0.0005, 0.0005, 0.0005, 0.0005, 0.0008], dtype=float
    ))
    motor_trim_n: np.ndarray = field(default_factory=lambda: np.array(
        [9.5, 9.5, 9.5, 9.5, 6.0], dtype=float
    ))
    lm_damping: float = 1.0e-3
    multirate_enabled: bool = True

    def __post_init__(self) -> None:
        for name in (
            'wrench_scale', 'wrench_weight',
            'checkpoint_wrench_weight_short',
        ):
            setattr(self, name, _array(getattr(self, name), 6, name))
        for name in (
            'command_move_weight', 'command_preference_weight', 'command_scale',
        ):
            setattr(self, name, _array(getattr(self, name), ACTUATOR_COUNT, name))
        for name in (
            'servo_gain_positive', 'servo_gain_negative',
            'servo_tau_positive_s', 'servo_tau_negative_s',
            'servo_delay_positive_s', 'servo_delay_negative_s',
            'servo_rate_positive_rad_s', 'servo_rate_negative_rad_s',
            'servo_state_limit_rad', 'servo_command_limit_rad',
            'servo_command_rate_rad_s',
        ):
            setattr(self, name, _array(getattr(self, name), ANGLE_COUNT, name))
        for name in (
            'thrust_min_n', 'thrust_max_n', 'thrust_command_rate_n_s',
            'motor_tau_up_s', 'motor_tau_down_s',
            'late_transition_weight', 'late_trim_weight', 'motor_trim_n',
        ):
            setattr(self, name, _array(getattr(self, name), THRUST_COUNT, name))

        self.prediction_dt_s = max(float(self.prediction_dt_s), 0.001)
        self.prediction_far_dt_s = max(
            float(self.prediction_far_dt_s), self.prediction_dt_s
        )
        self.prediction_near_duration_s = max(
            float(self.prediction_near_duration_s), self.prediction_dt_s
        )
        self.checkpoint_times_s = np.asarray(
            self.checkpoint_times_s, dtype=float
        ).reshape(-1)
        self.checkpoint_cost_scale = np.asarray(
            self.checkpoint_cost_scale, dtype=float
        ).reshape(-1)
        if self.horizon_s <= 0.0:
            directional_horizon = np.maximum(
                self.servo_delay_positive_s
                + self.horizon_tau_factor * self.servo_tau_positive_s,
                self.servo_delay_negative_s
                + self.horizon_tau_factor * self.servo_tau_negative_s,
            )
            self.horizon_s = float(np.max(directional_horizon))
        self.horizon_s = max(float(self.horizon_s), self.prediction_dt_s)
        self.motor_block_switch_s = float(np.clip(
            self.motor_block_switch_s,
            self.prediction_dt_s,
            self.horizon_s,
        ))
        self.checkpoint_times_s = np.unique(np.clip(
            self.checkpoint_times_s,
            self.prediction_dt_s,
            self.horizon_s,
        ))
        expected_checkpoint_count = self.checkpoint_times_s.size + 1
        if self.checkpoint_cost_scale.size != expected_checkpoint_count:
            raise ValueError(
                'checkpoint_cost_scale must contain one weight per checkpoint '
                'plus the terminal horizon'
            )
        self.gauss_newton_iterations = max(int(self.gauss_newton_iterations), 1)
        self.reachability_gate_threshold = max(
            float(self.reachability_gate_threshold), 0.0
        )
        self.lm_damping = max(float(self.lm_damping), 1e-9)
        if np.any(self.wrench_scale <= 0.0) or np.any(self.command_scale <= 0.0):
            raise ValueError('normalization scales must be positive')

    @classmethod
    def ideal_servos(cls, **kwargs) -> 'DRCDAConfig':
        config = cls(**kwargs)
        config.servo_gain_positive[:] = 1.0
        config.servo_gain_negative[:] = 1.0
        config.servo_tau_positive_s[:] = config.prediction_dt_s * 0.25
        config.servo_tau_negative_s[:] = config.prediction_dt_s * 0.25
        config.servo_delay_positive_s[:] = 0.0
        config.servo_delay_negative_s[:] = 0.0
        config.servo_rate_positive_rad_s[:] = 50.0
        config.servo_rate_negative_rad_s[:] = 50.0
        return config


@dataclass(order=True)
class ServoCommandEvent:
    activation_time_s: float
    sequence: int
    target_angle_rad: float = field(compare=False)
    command: float = field(compare=False)
    target_sensitivity: np.ndarray = field(compare=False)


class ServoPredictor:
    """Command-history servo predictor with calibrated command/state spaces."""

    def __init__(self, index: int, config: DRCDAConfig) -> None:
        if not 0 <= index < ANGLE_COUNT:
            raise ValueError(f'servo index must be in [0, {ANGLE_COUNT})')
        self.index = int(index)
        self.config = config
        self.time_s = 0.0
        self.theta = 0.0
        self.active_target_angle_rad = 0.0
        self.active_target_sensitivity = np.zeros(0)
        self.last_command = 0.0
        self.pending_queue: list[ServoCommandEvent] = []
        self._next_sequence = 0
        self.sensitivity = np.zeros(0)
        self.rate_active_count = 0
        self.integration_count = 0
        self.angle_active_count = 0

    def reset(self, theta: float = 0.0, target_angle_rad: float | None = None) -> None:
        self.time_s = 0.0
        self.theta = float(theta)
        target = self.theta if target_angle_rad is None else float(target_angle_rad)
        target = self.clip_target(target)
        self.active_target_angle_rad = target
        self.last_command = self.target_to_command(target)
        self.pending_queue = []
        self._next_sequence = 0
        self.active_target_sensitivity = np.zeros(0)
        self.sensitivity = np.zeros(0)
        self.rate_active_count = 0
        self.integration_count = 0
        self.angle_active_count = 0

    def command_to_target(self, command: float) -> float:
        cfg = self.config
        gain = (
            cfg.servo_gain_positive[self.index]
            if command >= 0.0
            else cfg.servo_gain_negative[self.index]
        )
        return self.clip_target(float(gain * command))

    def target_to_command(self, target_angle_rad: float) -> float:
        cfg = self.config
        gain = (
            cfg.servo_gain_positive[self.index]
            if target_angle_rad >= 0.0
            else cfg.servo_gain_negative[self.index]
        )
        if abs(float(gain)) <= 1e-9:
            return 0.0
        limit = cfg.servo_command_limit_rad[self.index]
        return _clip_scalar(target_angle_rad / gain, -limit, limit)

    def target_bounds(self, active_state_limit_rad: float | None = None) -> tuple[float, float]:
        cfg = self.config
        command_limit = cfg.servo_command_limit_rad[self.index]
        state_limit = cfg.servo_state_limit_rad[self.index]
        if active_state_limit_rad is not None:
            state_limit = min(state_limit, max(float(active_state_limit_rad), 0.0))
        lower = -min(
            state_limit,
            cfg.servo_gain_negative[self.index] * command_limit,
        )
        upper = min(
            state_limit,
            cfg.servo_gain_positive[self.index] * command_limit,
        )
        return float(lower), float(upper)

    def clip_target(
        self,
        target_angle_rad: float,
        active_state_limit_rad: float | None = None,
    ) -> float:
        lower, upper = self.target_bounds(active_state_limit_rad)
        return _clip_scalar(target_angle_rad, lower, upper)

    def _delay_for_command_change(self, command: float) -> float:
        cfg = self.config
        command_delta = float(command - self.last_command)
        if command_delta >= 0.0:
            return float(cfg.servo_delay_positive_s[self.index])
        return float(cfg.servo_delay_negative_s[self.index])

    def enqueue(
        self,
        target_angle_rad: float,
        target_sensitivity: np.ndarray | None = None,
        active_state_limit_rad: float | None = None,
    ) -> None:
        target = self.clip_target(target_angle_rad, active_state_limit_rad)
        command = self.target_to_command(target)
        delay_s = self._delay_for_command_change(command)
        if target_sensitivity is None:
            sensitivity = np.zeros_like(self.sensitivity)
        else:
            sensitivity = np.asarray(target_sensitivity, dtype=float).copy()
            if sensitivity.shape != self.sensitivity.shape:
                raise ValueError('target sensitivity shape does not match predictor')
        event = ServoCommandEvent(
            activation_time_s=self.time_s + delay_s,
            sequence=self._next_sequence,
            target_angle_rad=target,
            command=command,
            target_sensitivity=sensitivity,
        )
        self._next_sequence += 1
        self.pending_queue.append(event)
        self.pending_queue.sort()
        self.last_command = command

    def copy_for_prediction(self, decision_count: int) -> 'ServoPredictor':
        predictor = ServoPredictor(self.index, self.config)
        predictor.time_s = self.time_s
        predictor.theta = self.theta
        predictor.active_target_angle_rad = self.active_target_angle_rad
        predictor.last_command = self.last_command
        predictor._next_sequence = self._next_sequence
        predictor.sensitivity = np.zeros(decision_count)
        predictor.active_target_sensitivity = np.zeros(decision_count)
        predictor.pending_queue = [
            ServoCommandEvent(
                activation_time_s=event.activation_time_s,
                sequence=event.sequence,
                target_angle_rad=event.target_angle_rad,
                command=event.command,
                target_sensitivity=np.zeros(decision_count),
            )
            for event in self.pending_queue
        ]
        predictor.rate_active_count = 0
        predictor.integration_count = 0
        predictor.angle_active_count = 0
        return predictor

    def _integrate(self, dt: float, active_state_limit_rad: float | None) -> None:
        if dt <= 0.0:
            return
        cfg = self.config
        error = self.active_target_angle_rad - self.theta
        moving_positive = error >= 0.0
        tau_s = (
            cfg.servo_tau_positive_s[self.index]
            if moving_positive else cfg.servo_tau_negative_s[self.index]
        )
        alpha = 1.0 if tau_s <= 1e-6 else 1.0 - math.exp(-dt / tau_s)
        requested_delta = alpha * error
        max_positive = cfg.servo_rate_positive_rad_s[self.index] * dt
        max_negative = cfg.servo_rate_negative_rad_s[self.index] * dt
        applied_delta = _clip_scalar(
            requested_delta, -max_negative, max_positive
        )
        rate_active = not (-max_negative < requested_delta < max_positive)
        if not rate_active:
            self.sensitivity = (
                (1.0 - alpha) * self.sensitivity
                + alpha * self.active_target_sensitivity
            )

        lower, upper = self.target_bounds(active_state_limit_rad)
        next_theta_unclipped = self.theta + applied_delta
        self.theta = _clip_scalar(next_theta_unclipped, lower, upper)
        angle_active = self.theta != next_theta_unclipped
        if angle_active:
            self.sensitivity[:] = 0.0

        self.rate_active_count += int(rate_active)
        self.angle_active_count += int(angle_active)
        self.integration_count += 1

    def advance(
        self,
        dt: float,
        active_state_limit_rad: float | None = None,
    ) -> None:
        end_time_s = self.time_s + max(float(dt), 0.0)
        epsilon = 1e-12
        while self.pending_queue and self.pending_queue[0].activation_time_s <= end_time_s + epsilon:
            event = self.pending_queue.pop(0)
            segment_dt = max(event.activation_time_s - self.time_s, 0.0)
            self._integrate(segment_dt, active_state_limit_rad)
            self.time_s = max(self.time_s, event.activation_time_s)
            self.active_target_angle_rad = event.target_angle_rad
            self.active_target_sensitivity = event.target_sensitivity.copy()

        self._integrate(max(end_time_s - self.time_s, 0.0), active_state_limit_rad)
        self.time_s = end_time_s


def configure_allocator_variant(config: DRCDAConfig, variant: str) -> DRCDAConfig:
    """Apply one isolated allocator ablation to an existing configuration."""
    variant = variant.strip().lower()
    if variant not in ALLOCATOR_VARIANTS:
        choices = ', '.join(ALLOCATOR_VARIANTS)
        raise ValueError(f'unknown allocator variant {variant!r}; choose from {choices}')

    if variant == 'no_delay':
        config.servo_delay_positive_s[:] = 0.0
        config.servo_delay_negative_s[:] = 0.0
    elif variant == 'no_horizon':
        config.horizon_s = config.prediction_dt_s
        config.motor_block_switch_s = config.horizon_s
        config.checkpoint_times_s = np.zeros(0)
        config.checkpoint_cost_scale = np.ones(1)
    elif variant == 'no_physical_rate':
        config.servo_rate_positive_rad_s[:] = 1.0e6
        config.servo_rate_negative_rad_s[:] = 1.0e6
        config.motor_force_rate_floor_n_s = 1.0e6
        config.motor_force_rate_cap_n_s = 1.0e6
    elif variant == 'no_command_slew':
        config.servo_command_rate_rad_s[:] = 1.0e6
        config.thrust_command_rate_n_s[:] = 1.0e6
    elif variant == 'no_reachability_gate':
        config.reachability_gate_threshold = 0.0
    elif variant == 'no_multirate':
        config.multirate_enabled = False
    return config


class HnuterWrenchModel:
    """Nonlinear 6D wrench model for four coaxial and one tail rotor."""

    def __init__(
        self,
        arm_half_span_m: float = 0.33,
        front_x_m: float = 0.105,
        front_z_m: float = -0.013,
        coaxial_half_separation_m: float = 0.045,
        tail_x_m: float = -0.664,
        reaction_torque_ratio_m: float = 0.016,
    ) -> None:
        upper_z = front_z_m + coaxial_half_separation_m
        lower_z = front_z_m - coaxial_half_separation_m
        # Logical force order is L1, L2, R1, R2, tail.
        self.positions = np.array([
            [front_x_m, arm_half_span_m, upper_z],
            [front_x_m, arm_half_span_m, lower_z],
            [front_x_m, -arm_half_span_m, upper_z],
            [front_x_m, -arm_half_span_m, lower_z],
            [tail_x_m, 0.0, 0.0],
        ], dtype=float)
        self.spin_sign = np.array([-1.0, 1.0, -1.0, 1.0, -1.0])
        self.reaction_torque_ratio_m = float(reaction_torque_ratio_m)

    @staticmethod
    def _direction(alpha: float, beta: float) -> np.ndarray:
        ca, sa = math.cos(alpha), math.sin(alpha)
        cb, sb = math.cos(beta), math.sin(beta)
        return np.array([cb * sa, -sb, cb * ca], dtype=float)

    @staticmethod
    def _direction_derivatives(alpha: float, beta: float) -> tuple[np.ndarray, np.ndarray]:
        ca, sa = math.cos(alpha), math.sin(alpha)
        cb, sb = math.cos(beta), math.sin(beta)
        d_alpha = np.array([cb * ca, 0.0, -cb * sa], dtype=float)
        d_beta = np.array([-sb * sa, -cb, -sb * ca], dtype=float)
        return d_alpha, d_beta

    def _rotor_directions(self, angles: np.ndarray) -> np.ndarray:
        left = self._direction(float(angles[0]), float(angles[1]))
        right = self._direction(float(angles[2]), float(angles[3]))
        return np.vstack((left, left, right, right, np.array([0.0, 0.0, 1.0])))

    def wrench(self, q: np.ndarray) -> np.ndarray:
        q = _array(q, ACTUATOR_COUNT, 'q')
        directions = self._rotor_directions(q[:ANGLE_COUNT])
        force = np.sum(q[ANGLE_COUNT:, None] * directions, axis=0)
        moment = np.zeros(3)
        for index in range(THRUST_COUNT):
            effectiveness = (
                _cross3(self.positions[index], directions[index])
                + self.spin_sign[index] * self.reaction_torque_ratio_m * directions[index]
            )
            moment += q[ANGLE_COUNT + index] * effectiveness
        return np.concatenate((force, moment))

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        q = _array(q, ACTUATOR_COUNT, 'q')
        angles = q[:ANGLE_COUNT]
        thrust = q[ANGLE_COUNT:]
        directions = self._rotor_directions(angles)
        jacobian = np.zeros((6, ACTUATOR_COUNT))

        for index in range(THRUST_COUNT):
            direction = directions[index]
            jacobian[:3, ANGLE_COUNT + index] = direction
            jacobian[3:, ANGLE_COUNT + index] = (
                _cross3(self.positions[index], direction)
                + self.spin_sign[index] * self.reaction_torque_ratio_m * direction
            )

        left_da, left_db = self._direction_derivatives(float(angles[0]), float(angles[1]))
        right_da, right_db = self._direction_derivatives(float(angles[2]), float(angles[3]))
        angle_groups = (
            ((0, 1), left_da),
            ((0, 1), left_db),
            ((2, 3), right_da),
            ((2, 3), right_db),
        )
        for column, (rotor_indices, derivative) in enumerate(angle_groups):
            for rotor_index in rotor_indices:
                rotor_thrust = thrust[rotor_index]
                jacobian[:3, column] += rotor_thrust * derivative
                jacobian[3:, column] += rotor_thrust * (
                    _cross3(self.positions[rotor_index], derivative)
                    + self.spin_sign[rotor_index] * self.reaction_torque_ratio_m * derivative
                )
        return jacobian

    def jacobian_error(self, q: np.ndarray, epsilon: float = 1e-6) -> float:
        analytical = self.jacobian(q)
        numerical = np.empty_like(analytical)
        for index in range(ACTUATOR_COUNT):
            offset = np.zeros(ACTUATOR_COUNT)
            offset[index] = epsilon
            numerical[:, index] = (
                self.wrench(q + offset) - self.wrench(q - offset)
            ) / (2.0 * epsilon)
        return float(np.max(np.abs(analytical - numerical)))


@dataclass
class DRCDAResult:
    command: np.ndarray
    late_thrust_command: np.ndarray
    predicted_state: np.ndarray
    estimated_wrench: np.ndarray
    predicted_wrench: np.ndarray
    desired_wrench: np.ndarray
    jerk_reference: np.ndarray
    wrench_rate_residual: np.ndarray
    wrench_residual: np.ndarray
    servo_authority: np.ndarray
    servo_gated: np.ndarray
    servo_rate_active_fraction: np.ndarray
    servo_angle_active_fraction: np.ndarray
    objective: float
    lm_damping: float
    solve_time_ms: float
    iterations: int
    status: str


@dataclass
class PredictionTrajectory:
    times_s: np.ndarray
    states: list[np.ndarray]
    state_sensitivities: list[np.ndarray]
    servo_rate_active_fraction: np.ndarray
    servo_angle_active_fraction: np.ndarray
    motor_limit_active_fraction: np.ndarray


class DRCDAAllocator:
    """Delay-aware finite-horizon allocator with multirate motor blocking."""

    def __init__(self, model: HnuterWrenchModel, config: DRCDAConfig | None = None) -> None:
        self.model = model
        self.config = config or DRCDAConfig()
        self.state = np.zeros(ACTUATOR_COUNT)
        self.command = np.zeros(ACTUATOR_COUNT)
        self.late_thrust_command = np.zeros(THRUST_COUNT)
        self._servo_predictors = [
            ServoPredictor(index, self.config) for index in range(ANGLE_COUNT)
        ]
        self._previous_desired_wrench = np.zeros(6)
        self._filtered_wrench_ff = np.zeros(6)
        self._prediction_servo_rate_fraction = np.zeros(ANGLE_COUNT)
        self._prediction_servo_angle_fraction = np.zeros(ANGLE_COUNT)
        self.last_result: DRCDAResult | None = None

    def reset(
        self,
        angle_state: Iterable[float] | None = None,
        thrust_state: Iterable[float] | None = None,
    ) -> None:
        self.state[:] = 0.0
        if angle_state is not None:
            self.state[:ANGLE_COUNT] = _array(angle_state, ANGLE_COUNT, 'angle_state')
        if thrust_state is not None:
            self.state[ANGLE_COUNT:] = _array(thrust_state, THRUST_COUNT, 'thrust_state')
        self.command[:] = self.state
        self.late_thrust_command[:] = self.command[ANGLE_COUNT:]
        for index, predictor in enumerate(self._servo_predictors):
            predictor.reset(
                theta=float(self.state[index]),
                target_angle_rad=float(self.command[index]),
            )
        self._previous_desired_wrench[:] = 0.0
        self._filtered_wrench_ff[:] = 0.0
        self._prediction_servo_rate_fraction[:] = 0.0
        self._prediction_servo_angle_fraction[:] = 0.0
        self.last_result = None

    def servo_target_to_command(self, index: int, target_angle_rad: float) -> float:
        return self._servo_predictors[index].target_to_command(target_angle_rad)

    def servo_command_to_target(self, index: int, command: float) -> float:
        return self._servo_predictors[index].command_to_target(command)

    def _motor_rate_bounds(self, index: int, state: float, command: float) -> tuple[float, float]:
        cfg = self.config
        span = max(cfg.thrust_max_n[index] - cfg.thrust_min_n[index], 1.0)
        upper_margin = max(cfg.thrust_max_n[index] - state, 0.0) / span
        lower_margin = max(state - cfg.thrust_min_n[index], 0.0) / span
        rate_up = cfg.motor_force_rate_floor_n_s + (
            cfg.motor_force_rate_cap_n_s - cfg.motor_force_rate_floor_n_s
        ) * math.sqrt(upper_margin)
        rate_down = cfg.motor_force_rate_floor_n_s + (
            cfg.motor_force_rate_cap_n_s - cfg.motor_force_rate_floor_n_s
        ) * math.sqrt(lower_margin)
        if command < state:
            rate_up = min(rate_up, cfg.motor_force_rate_cap_n_s * 0.5)
        return rate_down, rate_up

    def _motor_step(
        self,
        index: int,
        state: float,
        command: float,
        dt: float,
        sensitivity: float = 0.0,
        command_sensitivity: float = 0.0,
    ) -> tuple[float, float | np.ndarray, bool]:
        cfg = self.config
        tau_s = cfg.motor_tau_up_s[index] if command >= state else cfg.motor_tau_down_s[index]
        alpha = 1.0 if tau_s <= 1e-6 else 1.0 - math.exp(-dt / tau_s)
        requested_delta = alpha * (command - state)
        rate_down, rate_up = self._motor_rate_bounds(index, state, command)
        lower_delta = -rate_down * dt
        upper_delta = rate_up * dt
        applied_delta = _clip_scalar(requested_delta, lower_delta, upper_delta)
        limit_active = not (lower_delta < requested_delta < upper_delta)
        if lower_delta < requested_delta < upper_delta:
            sensitivity = sensitivity + alpha * (
                command_sensitivity - sensitivity
            )

        lower = cfg.thrust_min_n[index]
        upper = cfg.thrust_max_n[index]
        next_state_unclipped = state + applied_delta
        next_state = _clip_scalar(next_state_unclipped, lower, upper)
        if next_state != next_state_unclipped:
            sensitivity = (
                np.zeros_like(sensitivity)
                if isinstance(sensitivity, np.ndarray)
                else 0.0
            )
            limit_active = True
        return next_state, sensitivity, limit_active

    def _advance_state(self, dt: float) -> None:
        for index, predictor in enumerate(self._servo_predictors):
            predictor.advance(dt)
            self.state[index] = predictor.theta

        for index in range(THRUST_COUNT):
            state_index = ANGLE_COUNT + index
            self.state[state_index], _, _ = self._motor_step(
                index,
                float(self.state[state_index]),
                float(self.command[state_index]),
                dt,
            )

    def _decision_scale(self) -> np.ndarray:
        cfg = self.config
        return np.concatenate((
            cfg.command_scale[:ANGLE_COUNT],
            cfg.command_scale[ANGLE_COUNT:],
            cfg.command_scale[ANGLE_COUNT:],
        ))

    def _build_prediction_steps(self) -> np.ndarray:
        cfg = self.config
        boundaries = np.unique(np.concatenate((
            cfg.checkpoint_times_s,
            np.array([
                cfg.prediction_near_duration_s,
                cfg.motor_block_switch_s,
                cfg.horizon_s,
            ]),
        )))
        boundaries = boundaries[
            (boundaries > 0.0) & (boundaries <= cfg.horizon_s)
        ]
        steps: list[float] = []
        elapsed = 0.0
        epsilon = 1e-12
        while elapsed < cfg.horizon_s - epsilon:
            nominal_dt = (
                cfg.prediction_dt_s
                if elapsed < cfg.prediction_near_duration_s - epsilon
                else cfg.prediction_far_dt_s
            )
            next_time = min(elapsed + nominal_dt, cfg.horizon_s)
            future_boundaries = boundaries[
                (boundaries > elapsed + epsilon)
                & (boundaries < next_time - epsilon)
            ]
            if future_boundaries.size:
                next_time = float(future_boundaries[0])
            steps.append(next_time - elapsed)
            elapsed = next_time
        return np.asarray(steps, dtype=float)

    def _predict_trajectory(
        self,
        decision: np.ndarray,
        active_angle_limits: np.ndarray,
    ) -> PredictionTrajectory:
        decision = _array(decision, DECISION_COUNT, 'decision')
        cfg = self.config
        state = self.state.copy()
        state_sensitivity = np.zeros((ACTUATOR_COUNT, DECISION_COUNT))
        prediction_servos: list[ServoPredictor] = []
        for index in range(ANGLE_COUNT):
            predictor = self._servo_predictors[index].copy_for_prediction(
                DECISION_COUNT
            )
            target_sensitivity = np.zeros(DECISION_COUNT)
            target_sensitivity[index] = 1.0
            predictor.enqueue(
                float(decision[index]),
                target_sensitivity,
                float(active_angle_limits[index]),
            )
            prediction_servos.append(predictor)

        times = [0.0]
        states = [state.copy()]
        sensitivities = [state_sensitivity.copy()]
        motor_limit_counts = np.zeros(THRUST_COUNT)
        motor_step_count = 0
        elapsed = 0.0
        for prediction_dt in self._build_prediction_steps():
            use_late = (
                cfg.multirate_enabled
                and elapsed >= cfg.motor_block_switch_s - 1e-12
            )
            thrust_offset = (
                ANGLE_COUNT + THRUST_COUNT if use_late else ANGLE_COUNT
            )

            for index, predictor in enumerate(prediction_servos):
                predictor.advance(
                    float(prediction_dt),
                    float(active_angle_limits[index]),
                )
                state[index] = predictor.theta
                state_sensitivity[index] = predictor.sensitivity

            for index in range(THRUST_COUNT):
                state_index = ANGLE_COUNT + index
                decision_index = thrust_offset + index
                command_sensitivity = np.zeros(DECISION_COUNT)
                command_sensitivity[decision_index] = 1.0
                (
                    state[state_index],
                    state_sensitivity[state_index],
                    limit_active,
                ) = self._motor_step(
                    index,
                    float(state[state_index]),
                    float(decision[decision_index]),
                    float(prediction_dt),
                    state_sensitivity[state_index],
                    command_sensitivity,
                )
                motor_limit_counts[index] += int(limit_active)

            motor_step_count += 1
            elapsed += float(prediction_dt)
            times.append(elapsed)
            states.append(state.copy())
            sensitivities.append(state_sensitivity.copy())

        servo_rate_fraction = np.zeros(ANGLE_COUNT)
        servo_angle_fraction = np.zeros(ANGLE_COUNT)
        for index, predictor in enumerate(prediction_servos):
            count = max(predictor.integration_count, 1)
            servo_rate_fraction[index] = predictor.rate_active_count / count
            servo_angle_fraction[index] = predictor.angle_active_count / count
        self._prediction_servo_rate_fraction = servo_rate_fraction.copy()
        self._prediction_servo_angle_fraction = servo_angle_fraction.copy()
        return PredictionTrajectory(
            times_s=np.asarray(times),
            states=states,
            state_sensitivities=sensitivities,
            servo_rate_active_fraction=servo_rate_fraction,
            servo_angle_active_fraction=servo_angle_fraction,
            motor_limit_active_fraction=(
                motor_limit_counts / max(motor_step_count, 1)
            ),
        )

    @staticmethod
    def _trajectory_index(trajectory: PredictionTrajectory, time_s: float) -> int:
        return int(np.argmin(np.abs(trajectory.times_s - float(time_s))))

    def _predict_terminal(
        self,
        candidate_command: np.ndarray,
        active_angle_limits: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        decision = np.concatenate((
            candidate_command[:ANGLE_COUNT],
            candidate_command[ANGLE_COUNT:],
            candidate_command[ANGLE_COUNT:],
        ))
        trajectory = self._predict_trajectory(decision, active_angle_limits)
        terminal_state = trajectory.states[-1]
        terminal_decision_sensitivity = trajectory.state_sensitivities[-1]
        command_sensitivity = np.zeros((ACTUATOR_COUNT, ACTUATOR_COUNT))
        command_sensitivity[:, :ANGLE_COUNT] = (
            terminal_decision_sensitivity[:, :ANGLE_COUNT]
        )
        for index in range(THRUST_COUNT):
            command_sensitivity[:, ANGLE_COUNT + index] = (
                terminal_decision_sensitivity[:, ANGLE_COUNT + index]
                + terminal_decision_sensitivity[
                    :, ANGLE_COUNT + THRUST_COUNT + index
                ]
            )
        return terminal_state, command_sensitivity

    def predict_terminal(
        self,
        candidate_command: Iterable[float],
        active_angle_limits: Iterable[float] | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        command = _array(candidate_command, ACTUATOR_COUNT, 'candidate_command')
        limits = (
            self.config.servo_state_limit_rad
            if active_angle_limits is None
            else _array(active_angle_limits, ANGLE_COUNT, 'active_angle_limits')
        )
        return self._predict_terminal(command, limits)

    def _servo_authority(
        self,
        predicted_state: np.ndarray,
        state_sensitivity: np.ndarray,
    ) -> np.ndarray:
        cfg = self.config
        weighting = np.sqrt(cfg.wrench_weight) / cfg.wrench_scale
        command_jacobian = (
            self.model.jacobian(predicted_state)
            @ state_sensitivity
            @ np.diag(self._decision_scale())
        )
        current = np.linalg.norm(
            weighting[:, None] * command_jacobian[:, :ANGLE_COUNT],
            axis=0,
        )

        nominal_state = np.zeros(ACTUATOR_COUNT)
        nominal_state[ANGLE_COUNT:] = np.array(
            [9.5, 9.5, 9.5, 9.5, 6.0], dtype=float
        )
        nominal_jacobian = self.model.jacobian(nominal_state)
        nominal = np.linalg.norm(
            weighting[:, None]
            * nominal_jacobian[:, :ANGLE_COUNT]
            * cfg.command_scale[None, :ANGLE_COUNT],
            axis=0,
        )
        return current / np.maximum(nominal, 1e-9)

    def _project_command(
        self,
        command: np.ndarray,
        previous: np.ndarray,
        dt: float,
        active_angle_limits: np.ndarray,
    ) -> np.ndarray:
        cfg = self.config
        projected = command.copy()
        for index, predictor in enumerate(self._servo_predictors):
            lower, upper = predictor.target_bounds(float(active_angle_limits[index]))
            projected[index] = float(np.clip(projected[index], lower, upper))
        servo_delta = cfg.servo_command_rate_rad_s * dt
        projected[:ANGLE_COUNT] = np.clip(
            projected[:ANGLE_COUNT],
            previous[:ANGLE_COUNT] - servo_delta,
            previous[:ANGLE_COUNT] + servo_delta,
        )
        for index, predictor in enumerate(self._servo_predictors):
            lower, upper = predictor.target_bounds(float(active_angle_limits[index]))
            projected[index] = float(np.clip(projected[index], lower, upper))
        projected[ANGLE_COUNT:] = np.clip(
            projected[ANGLE_COUNT:], cfg.thrust_min_n, cfg.thrust_max_n
        )
        thrust_delta = cfg.thrust_command_rate_n_s * dt
        projected[ANGLE_COUNT:] = np.clip(
            projected[ANGLE_COUNT:],
            previous[ANGLE_COUNT:] - thrust_delta,
            previous[ANGLE_COUNT:] + thrust_delta,
        )
        return projected

    def _project_decision(
        self,
        decision: np.ndarray,
        previous: np.ndarray,
        dt: float,
        active_angle_limits: np.ndarray,
    ) -> np.ndarray:
        cfg = self.config
        projected = _array(decision, DECISION_COUNT, 'decision').copy()
        previous = _array(previous, DECISION_COUNT, 'previous_decision')
        servo_delta = cfg.servo_command_rate_rad_s * dt
        for index, predictor in enumerate(self._servo_predictors):
            lower, upper = predictor.target_bounds(float(active_angle_limits[index]))
            projected[index] = float(np.clip(
                projected[index],
                max(lower, previous[index] - servo_delta[index]),
                min(upper, previous[index] + servo_delta[index]),
            ))

        fast_slice = slice(ANGLE_COUNT, ANGLE_COUNT + THRUST_COUNT)
        late_slice = slice(ANGLE_COUNT + THRUST_COUNT, DECISION_COUNT)
        thrust_delta = cfg.thrust_command_rate_n_s * dt
        projected[fast_slice] = np.clip(
            projected[fast_slice],
            np.maximum(
                cfg.thrust_min_n,
                previous[fast_slice] - thrust_delta,
            ),
            np.minimum(
                cfg.thrust_max_n,
                previous[fast_slice] + thrust_delta,
            ),
        )
        if cfg.multirate_enabled:
            late_dt = max(cfg.horizon_s - cfg.motor_block_switch_s, dt)
            late_delta = cfg.thrust_command_rate_n_s * late_dt
            projected[late_slice] = np.clip(
                projected[late_slice],
                np.maximum(
                    cfg.thrust_min_n,
                    projected[fast_slice] - late_delta,
                ),
                np.minimum(
                    cfg.thrust_max_n,
                    projected[fast_slice] + late_delta,
                ),
            )
        else:
            projected[late_slice] = projected[fast_slice]
        return projected

    def _build_residual_and_jacobian(
        self,
        decision: np.ndarray,
        previous: np.ndarray,
        desired_wrench: np.ndarray,
        active_angle_limits: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, PredictionTrajectory]:
        cfg = self.config
        trajectory = self._predict_trajectory(decision, active_angle_limits)
        decision_scale = self._decision_scale()
        residual_blocks: list[np.ndarray] = []
        jacobian_blocks: list[np.ndarray] = []
        evaluation_times = np.unique(np.concatenate((
            cfg.checkpoint_times_s[cfg.checkpoint_times_s < cfg.horizon_s - 1e-12],
            np.array([cfg.horizon_s]),
        )))

        for checkpoint_index, checkpoint_time in enumerate(evaluation_times):
            trajectory_index = self._trajectory_index(
                trajectory, float(checkpoint_time)
            )
            state = trajectory.states[trajectory_index]
            sensitivity = trajectory.state_sensitivities[trajectory_index]
            wrench = self.model.wrench(state)
            wrench_weight = (
                cfg.checkpoint_wrench_weight_short
                if checkpoint_index == 0
                else cfg.wrench_weight
            )
            cost_scale = math.sqrt(
                cfg.checkpoint_cost_scale[min(
                    checkpoint_index,
                    cfg.checkpoint_cost_scale.size - 1,
                )]
            )
            weighting = (
                cost_scale * np.sqrt(wrench_weight) / cfg.wrench_scale
            )
            wrench_jacobian = (
                self.model.jacobian(state)
                @ sensitivity
                @ np.diag(decision_scale)
            )
            residual_blocks.append(weighting * (wrench - desired_wrench))
            jacobian_blocks.append(weighting[:, None] * wrench_jacobian)

        normalized = decision / decision_scale
        previous_normalized = previous / decision_scale
        servo_weights = np.sqrt(cfg.command_move_weight[:ANGLE_COUNT])
        servo_jacobian = np.zeros((ANGLE_COUNT, DECISION_COUNT))
        servo_jacobian[
            np.arange(ANGLE_COUNT), np.arange(ANGLE_COUNT)
        ] = servo_weights
        residual_blocks.append(
            servo_weights
            * (normalized[:ANGLE_COUNT] - previous_normalized[:ANGLE_COUNT])
        )
        jacobian_blocks.append(servo_jacobian)

        fast_slice = slice(ANGLE_COUNT, ANGLE_COUNT + THRUST_COUNT)
        late_slice = slice(ANGLE_COUNT + THRUST_COUNT, DECISION_COUNT)
        fast_weights = np.sqrt(cfg.command_move_weight[ANGLE_COUNT:])
        fast_jacobian = np.zeros((THRUST_COUNT, DECISION_COUNT))
        fast_jacobian[
            np.arange(THRUST_COUNT), ANGLE_COUNT + np.arange(THRUST_COUNT)
        ] = fast_weights
        residual_blocks.append(
            fast_weights
            * (normalized[fast_slice] - previous_normalized[fast_slice])
        )
        jacobian_blocks.append(fast_jacobian)

        if cfg.multirate_enabled:
            transition_weights = np.sqrt(cfg.late_transition_weight)
            transition_jacobian = np.zeros((THRUST_COUNT, DECISION_COUNT))
            row = np.arange(THRUST_COUNT)
            transition_jacobian[row, ANGLE_COUNT + row] = -transition_weights
            transition_jacobian[
                row, ANGLE_COUNT + THRUST_COUNT + row
            ] = transition_weights
            residual_blocks.append(
                transition_weights
                * (normalized[late_slice] - normalized[fast_slice])
            )
            jacobian_blocks.append(transition_jacobian)

            trim_weights = np.sqrt(cfg.late_trim_weight)
            trim_jacobian = np.zeros((THRUST_COUNT, DECISION_COUNT))
            trim_jacobian[
                row, ANGLE_COUNT + THRUST_COUNT + row
            ] = trim_weights
            trim_normalized = cfg.motor_trim_n / decision_scale[late_slice]
            residual_blocks.append(
                trim_weights * (normalized[late_slice] - trim_normalized)
            )
            jacobian_blocks.append(trim_jacobian)

        residual = np.concatenate(residual_blocks)
        jacobian = np.vstack(jacobian_blocks)
        return residual, jacobian, trajectory

    def _motor_only_fallback(
        self,
        desired_wrench: np.ndarray,
        previous: np.ndarray,
        dt: float,
        active_angle_limits: np.ndarray,
    ) -> np.ndarray:
        cfg = self.config
        command = previous.copy()
        fixed_state = self.state.copy()
        jacobian = self.model.jacobian(fixed_state)[:, ANGLE_COUNT:]
        fixed_state[ANGLE_COUNT:] = 0.0
        fixed_wrench = self.model.wrench(fixed_state)
        weighting = np.diag(np.sqrt(cfg.wrench_weight) / cfg.wrench_scale)
        matrix = weighting @ jacobian
        target = weighting @ (desired_wrench - fixed_wrench)
        regularization = 1e-4 * np.eye(THRUST_COUNT)
        try:
            thrust = np.linalg.solve(
                matrix.T @ matrix + regularization,
                matrix.T @ target,
            )
        except np.linalg.LinAlgError:
            thrust = previous[ANGLE_COUNT:]
        command[ANGLE_COUNT:] = thrust
        return self._project_command(command, previous, dt, active_angle_limits)

    def allocate(
        self,
        desired_wrench: Iterable[float],
        dt: float,
        preferred_command: Iterable[float] | None = None,
        active_angle_limits: Iterable[float] | None = None,
    ) -> DRCDAResult:
        start_time = time.perf_counter()
        cfg = self.config
        desired = _array(desired_wrench, 6, 'desired_wrench')
        dt = float(np.clip(dt, 0.0005, 0.05))
        limits = (
            cfg.servo_state_limit_rad.copy()
            if active_angle_limits is None
            else _array(active_angle_limits, ANGLE_COUNT, 'active_angle_limits')
        )
        limits = np.minimum(limits, cfg.servo_state_limit_rad)
        preferred = (
            self.command.copy()
            if preferred_command is None
            else _array(preferred_command, ACTUATOR_COUNT, 'preferred_command')
        )

        self._advance_state(dt)
        estimated_wrench = self.model.wrench(self.state)
        raw_ff = (desired - self._previous_desired_wrench) / dt
        ff_alpha = dt / (max(cfg.wrench_ff_tau_s, 0.0) + dt)
        self._filtered_wrench_ff += ff_alpha * (raw_ff - self._filtered_wrench_ff)
        jerk_reference = (
            self._filtered_wrench_ff
            + cfg.wrench_error_gain * (desired - estimated_wrench)
        )
        self._previous_desired_wrench = desired.copy()

        previous = np.concatenate((
            self.command[:ANGLE_COUNT],
            self.command[ANGLE_COUNT:],
            self.late_thrust_command,
        ))
        preferred_decision = np.concatenate((
            preferred[:ANGLE_COUNT],
            preferred[ANGLE_COUNT:],
            preferred[ANGLE_COUNT:],
        ))
        decision = self._project_decision(
            0.75 * previous + 0.25 * preferred_decision,
            previous,
            dt,
            limits,
        )
        normalizer = self._decision_scale()
        status = 'solved'
        completed_iterations = 0
        servo_authority = np.zeros(ANGLE_COUNT)
        servo_gated = np.zeros(ANGLE_COUNT, dtype=bool)
        damping = cfg.lm_damping
        objective = float('inf')
        trajectory: PredictionTrajectory | None = None

        try:
            residual, jacobian, trajectory = self._build_residual_and_jacobian(
                decision, previous, desired, limits
            )
            for iteration in range(cfg.gauss_newton_iterations):
                predicted_state = trajectory.states[-1]
                state_sensitivity = trajectory.state_sensitivities[-1]
                servo_authority = self._servo_authority(
                    predicted_state, state_sensitivity
                )
                servo_gated = servo_authority < cfg.reachability_gate_threshold
                if np.any(servo_gated):
                    gated_indices = np.flatnonzero(servo_gated)
                    gated_decision = decision.copy()
                    gated_decision[gated_indices] = previous[gated_indices]
                    if not np.array_equal(gated_decision, decision):
                        decision = self._project_decision(
                            gated_decision, previous, dt, limits
                        )
                        (
                            residual,
                            jacobian,
                            trajectory,
                        ) = self._build_residual_and_jacobian(
                            decision, previous, desired, limits
                        )
                objective = 0.5 * float(residual @ residual)
                free = np.ones(DECISION_COUNT, dtype=bool)
                free[np.flatnonzero(servo_gated)] = False
                if not cfg.multirate_enabled:
                    free[ANGLE_COUNT + THRUST_COUNT:] = False

                accepted = False
                accepted_decision = decision
                accepted_objective = objective
                accepted_residual = residual
                accepted_jacobian = jacobian
                accepted_trajectory = trajectory
                for damping_attempt in range(2):
                    reduced_jacobian = jacobian[:, free]
                    normal_matrix = reduced_jacobian.T @ reduced_jacobian
                    diagonal = np.maximum(np.diag(normal_matrix), 1e-6)
                    reduced_step = np.linalg.solve(
                        normal_matrix + damping * np.diag(diagonal),
                        -(reduced_jacobian.T @ residual),
                    )
                    if not np.all(np.isfinite(reduced_step)):
                        raise np.linalg.LinAlgError('non-finite DRCDA LM step')
                    step = np.zeros(DECISION_COUNT)
                    step[free] = reduced_step

                    for step_scale in (1.0, 0.5, 0.25):
                        trial = self._project_decision(
                            decision + step_scale * normalizer * step,
                            previous,
                            dt,
                            limits,
                        )
                        gated_indices = np.flatnonzero(servo_gated)
                        trial[gated_indices] = previous[gated_indices]
                        (
                            trial_residual,
                            trial_jacobian,
                            trial_trajectory,
                        ) = self._build_residual_and_jacobian(
                            trial, previous, desired, limits
                        )
                        trial_objective = 0.5 * float(
                            trial_residual @ trial_residual
                        )
                        if trial_objective < objective - 1e-12:
                            accepted = True
                            accepted_decision = trial
                            accepted_objective = trial_objective
                            accepted_residual = trial_residual
                            accepted_jacobian = trial_jacobian
                            accepted_trajectory = trial_trajectory
                            damping = max(0.5 * damping, 1e-9)
                            break
                    if accepted:
                        break
                    damping *= 10.0

                completed_iterations = iteration + 1
                if not accepted:
                    status = 'solved_lm_stalled'
                    break
                normalized_change = np.linalg.norm(
                    (accepted_decision - decision) / normalizer
                )
                decision = accepted_decision
                objective = accepted_objective
                residual = accepted_residual
                jacobian = accepted_jacobian
                trajectory = accepted_trajectory
                if normalized_change < 1e-5:
                    break
        except (np.linalg.LinAlgError, FloatingPointError, ValueError):
            status = 'motor_only_fallback'
            previous_command = np.concatenate((
                previous[:ANGLE_COUNT],
                previous[ANGLE_COUNT:ANGLE_COUNT + THRUST_COUNT],
            ))
            fallback = self._motor_only_fallback(
                desired, previous_command, dt, limits
            )
            decision = np.concatenate((
                fallback[:ANGLE_COUNT],
                fallback[ANGLE_COUNT:],
                fallback[ANGLE_COUNT:],
            ))
            residual, _, trajectory = self._build_residual_and_jacobian(
                decision, previous, desired, limits
            )
        objective = 0.5 * float(residual @ residual)
        predicted_state = trajectory.states[-1]
        state_sensitivity = trajectory.state_sensitivities[-1]
        servo_authority = self._servo_authority(predicted_state, state_sensitivity)
        servo_gated = servo_authority < cfg.reachability_gate_threshold
        predicted_wrench = self.model.wrench(predicted_state)
        predicted_rate = (
            predicted_wrench - estimated_wrench
        ) / max(cfg.horizon_s, cfg.prediction_dt_s)
        wrench_rate_residual = jerk_reference - predicted_rate
        wrench_residual = predicted_wrench - desired

        command = np.concatenate((
            decision[:ANGLE_COUNT],
            decision[ANGLE_COUNT:ANGLE_COUNT + THRUST_COUNT],
        ))
        self.command = command
        self.late_thrust_command = decision[
            ANGLE_COUNT + THRUST_COUNT:
        ].copy()
        for index, predictor in enumerate(self._servo_predictors):
            predictor.enqueue(
                float(command[index]),
                active_state_limit_rad=float(limits[index]),
            )

        solve_time_ms = (time.perf_counter() - start_time) * 1000.0
        result = DRCDAResult(
            command=command.copy(),
            late_thrust_command=self.late_thrust_command.copy(),
            predicted_state=predicted_state,
            estimated_wrench=estimated_wrench,
            predicted_wrench=predicted_wrench,
            desired_wrench=desired,
            jerk_reference=jerk_reference,
            wrench_rate_residual=wrench_rate_residual,
            wrench_residual=wrench_residual,
            servo_authority=servo_authority,
            servo_gated=servo_gated,
            servo_rate_active_fraction=trajectory.servo_rate_active_fraction.copy(),
            servo_angle_active_fraction=trajectory.servo_angle_active_fraction.copy(),
            objective=objective,
            lm_damping=damping,
            solve_time_ms=solve_time_ms,
            iterations=completed_iterations,
            status=status,
        )
        self.last_result = result
        return result


class BasicDifferentialAllocator(DRCDAAllocator):
    """State-aware one-step differential allocator with actuator-rate bounds.

    This stronger baseline reconstructs q(k) from command history, but does not
    optimize against delayed future reachability or a finite-horizon trajectory.
    """

    def allocate(
        self,
        desired_wrench: Iterable[float],
        dt: float,
        preferred_command: Iterable[float] | None = None,
        active_angle_limits: Iterable[float] | None = None,
    ) -> DRCDAResult:
        start_time = time.perf_counter()
        cfg = self.config
        desired = _array(desired_wrench, 6, 'desired_wrench')
        dt = float(np.clip(dt, 0.0005, 0.05))
        limits = (
            cfg.servo_state_limit_rad.copy()
            if active_angle_limits is None
            else _array(active_angle_limits, ANGLE_COUNT, 'active_angle_limits')
        )
        limits = np.minimum(limits, cfg.servo_state_limit_rad)
        preferred = (
            self.command.copy()
            if preferred_command is None
            else _array(preferred_command, ACTUATOR_COUNT, 'preferred_command')
        )

        # Reconstruct the current actuator state from command history. The
        # standard DA baseline receives q(k), but does not predict when a new
        # delayed command will become reachable.
        self._advance_state(dt)
        estimated_wrench = self.model.wrench(self.state)
        raw_ff = (desired - self._previous_desired_wrench) / dt
        ff_alpha = dt / (max(cfg.wrench_ff_tau_s, 0.0) + dt)
        self._filtered_wrench_ff += ff_alpha * (raw_ff - self._filtered_wrench_ff)
        jerk_reference = (
            self._filtered_wrench_ff
            + cfg.wrench_error_gain * (desired - estimated_wrench)
        )
        self._previous_desired_wrench = desired.copy()

        rate_scale = np.concatenate((
            np.maximum(
                cfg.servo_rate_positive_rad_s,
                cfg.servo_rate_negative_rad_s,
            ),
            cfg.thrust_command_rate_n_s,
        ))
        sqrt_weight = np.sqrt(cfg.wrench_weight) / cfg.wrench_scale
        weighted_jacobian = (
            sqrt_weight[:, None]
            * self.model.jacobian(self.state)
            * rate_scale[None, :]
        )
        weighted_target = sqrt_weight * jerk_reference
        preferred_rate = np.clip(
            (preferred - self.state) / (dt * rate_scale),
            -1.0,
            1.0,
        )
        regularization = (
            cfg.command_move_weight + cfg.command_preference_weight + 1e-4
        )
        hessian = weighted_jacobian.T @ weighted_jacobian + np.diag(regularization)
        gradient = (
            weighted_jacobian.T @ weighted_target
            + cfg.command_preference_weight * preferred_rate
        )
        status = 'basic_da'
        try:
            normalized_rate = np.linalg.solve(hessian, gradient)
            if not np.all(np.isfinite(normalized_rate)):
                raise np.linalg.LinAlgError('non-finite basic DA step')
            normalized_rate = np.clip(normalized_rate, -1.0, 1.0)
            command = self._project_command(
                self.state + dt * rate_scale * normalized_rate,
                self.command,
                dt,
                limits,
            )
        except (np.linalg.LinAlgError, FloatingPointError, ValueError):
            status = 'basic_da_motor_only_fallback'
            command = self._motor_only_fallback(
                desired, self.command, dt, limits
            )

        self.command = command
        for index, predictor in enumerate(self._servo_predictors):
            predictor.enqueue(
                float(command[index]),
                active_state_limit_rad=float(limits[index]),
            )
        predicted_state = command.copy()
        predicted_wrench = self.model.wrench(predicted_state)
        predicted_rate = (predicted_wrench - estimated_wrench) / dt
        result = DRCDAResult(
            command=command.copy(),
            late_thrust_command=command[ANGLE_COUNT:].copy(),
            predicted_state=predicted_state,
            estimated_wrench=estimated_wrench,
            predicted_wrench=predicted_wrench,
            desired_wrench=desired,
            jerk_reference=jerk_reference,
            wrench_rate_residual=jerk_reference - predicted_rate,
            wrench_residual=predicted_wrench - desired,
            servo_authority=np.zeros(ANGLE_COUNT),
            servo_gated=np.zeros(ANGLE_COUNT, dtype=bool),
            servo_rate_active_fraction=np.zeros(ANGLE_COUNT),
            servo_angle_active_fraction=np.zeros(ANGLE_COUNT),
            objective=float(np.linalg.norm(
                (predicted_wrench - desired) / cfg.wrench_scale
            ) ** 2),
            lm_damping=0.0,
            solve_time_ms=(time.perf_counter() - start_time) * 1000.0,
            iterations=1,
            status=status,
        )
        self.last_result = result
        return result
