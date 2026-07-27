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


def _array(values: Iterable[float], count: int, name: str) -> np.ndarray:
    result = np.asarray(tuple(values), dtype=float)
    if result.shape != (count,):
        raise ValueError(f'{name} must contain {count} values')
    return result


@dataclass
class DRCDAConfig:
    prediction_dt_s: float = 0.01
    horizon_s: float = 0.18
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
        [0.002, 0.003, 0.002, 0.003, 0.001, 0.001, 0.001, 0.001, 0.002],
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

    def __post_init__(self) -> None:
        for name in (
            'wrench_scale', 'wrench_weight',
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
        ):
            setattr(self, name, _array(getattr(self, name), THRUST_COUNT, name))

        self.prediction_dt_s = max(float(self.prediction_dt_s), 0.001)
        self.horizon_s = max(float(self.horizon_s), self.prediction_dt_s)
        self.gauss_newton_iterations = max(int(self.gauss_newton_iterations), 1)
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
                np.cross(self.positions[index], directions[index])
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
                np.cross(self.positions[index], direction)
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
                    np.cross(self.positions[rotor_index], derivative)
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
    predicted_state: np.ndarray
    estimated_wrench: np.ndarray
    predicted_wrench: np.ndarray
    desired_wrench: np.ndarray
    jerk_reference: np.ndarray
    wrench_rate_residual: np.ndarray
    wrench_residual: np.ndarray
    solve_time_ms: float
    iterations: int
    status: str


class DRCDAAllocator:
    """Move-blocked short-horizon DRCDA allocator."""

    def __init__(self, model: HnuterWrenchModel, config: DRCDAConfig | None = None) -> None:
        self.model = model
        self.config = config or DRCDAConfig()
        self.state = np.zeros(ACTUATOR_COUNT)
        self.command = np.zeros(ACTUATOR_COUNT)
        self._delayed_servo_command = np.zeros(ANGLE_COUNT)
        self._pending_servo_commands: list[list[tuple[float, float]]] = [
            [] for _ in range(ANGLE_COUNT)
        ]
        self._previous_desired_wrench = np.zeros(6)
        self._filtered_wrench_ff = np.zeros(6)
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
        self._delayed_servo_command[:] = self.command[:ANGLE_COUNT]
        self._pending_servo_commands = [[] for _ in range(ANGLE_COUNT)]
        self._previous_desired_wrench[:] = 0.0
        self._filtered_wrench_ff[:] = 0.0
        self.last_result = None

    def _servo_parameters(self, index: int, command: float) -> tuple[float, float, float, float]:
        cfg = self.config
        if command >= 0.0:
            return (
                cfg.servo_gain_positive[index],
                cfg.servo_tau_positive_s[index],
                cfg.servo_delay_positive_s[index],
                cfg.servo_rate_positive_rad_s[index],
            )
        return (
            cfg.servo_gain_negative[index],
            cfg.servo_tau_negative_s[index],
            cfg.servo_delay_negative_s[index],
            cfg.servo_rate_negative_rad_s[index],
        )

    def _servo_step(
        self,
        index: int,
        state: float,
        delayed_command: float,
        dt: float,
        sensitivity: float = 0.0,
        delayed_command_sensitivity: float = 0.0,
        state_limit: float | None = None,
    ) -> tuple[float, float]:
        cfg = self.config
        gain, tau_s, _, _ = self._servo_parameters(index, delayed_command)
        alpha = 1.0 if tau_s <= 1e-6 else 1.0 - math.exp(-dt / tau_s)
        requested_delta = alpha * (gain * delayed_command - state)
        max_positive = cfg.servo_rate_positive_rad_s[index] * dt
        max_negative = cfg.servo_rate_negative_rad_s[index] * dt
        applied_delta = float(np.clip(requested_delta, -max_negative, max_positive))

        if -max_negative < requested_delta < max_positive:
            sensitivity += alpha * (
                gain * delayed_command_sensitivity - sensitivity
            )

        limit = (
            cfg.servo_state_limit_rad[index]
            if state_limit is None else min(cfg.servo_state_limit_rad[index], state_limit)
        )
        next_state_unclipped = state + applied_delta
        next_state = float(np.clip(next_state_unclipped, -limit, limit))
        if next_state != next_state_unclipped:
            sensitivity = 0.0
        return next_state, sensitivity

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
    ) -> tuple[float, float]:
        cfg = self.config
        tau_s = cfg.motor_tau_up_s[index] if command >= state else cfg.motor_tau_down_s[index]
        alpha = 1.0 if tau_s <= 1e-6 else 1.0 - math.exp(-dt / tau_s)
        requested_delta = alpha * (command - state)
        rate_down, rate_up = self._motor_rate_bounds(index, state, command)
        lower_delta = -rate_down * dt
        upper_delta = rate_up * dt
        applied_delta = float(np.clip(requested_delta, lower_delta, upper_delta))
        if lower_delta < requested_delta < upper_delta:
            sensitivity += alpha * (command_sensitivity - sensitivity)

        lower = cfg.thrust_min_n[index]
        upper = cfg.thrust_max_n[index]
        next_state_unclipped = state + applied_delta
        next_state = float(np.clip(next_state_unclipped, lower, upper))
        if next_state != next_state_unclipped:
            sensitivity = 0.0
        return next_state, sensitivity

    def _advance_state(self, dt: float) -> None:
        for index in range(ANGLE_COUNT):
            pending = []
            for remaining_s, command in self._pending_servo_commands[index]:
                remaining_s -= dt
                if remaining_s <= 0.0:
                    self._delayed_servo_command[index] = command
                else:
                    pending.append((remaining_s, command))
            self._pending_servo_commands[index] = pending
            self.state[index], _ = self._servo_step(
                index,
                float(self.state[index]),
                float(self._delayed_servo_command[index]),
                dt,
            )

        for index in range(THRUST_COUNT):
            state_index = ANGLE_COUNT + index
            self.state[state_index], _ = self._motor_step(
                index,
                float(self.state[state_index]),
                float(self.command[state_index]),
                dt,
            )

    def _predict_terminal(
        self,
        candidate_command: np.ndarray,
        active_angle_limits: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        cfg = self.config
        prediction_dt = cfg.prediction_dt_s
        steps = max(1, int(math.ceil(cfg.horizon_s / prediction_dt)))
        state = self.state.copy()
        sensitivity = np.zeros(ACTUATOR_COUNT)
        delayed = self._delayed_servo_command.copy()
        prediction_events: list[list[tuple[float, float, float]]] = []
        for index in range(ANGLE_COUNT):
            _, _, delay_s, _ = self._servo_parameters(index, candidate_command[index])
            events = [
                (remaining_s, command, 0.0)
                for remaining_s, command in self._pending_servo_commands[index]
            ]
            events.append((delay_s, float(candidate_command[index]), 1.0))
            prediction_events.append(events)

        elapsed_s = 0.0
        delayed_sensitivity = np.zeros(ANGLE_COUNT)
        for _ in range(steps):
            elapsed_s += prediction_dt
            for index in range(ANGLE_COUNT):
                while (
                    prediction_events[index]
                    and prediction_events[index][0][0] <= elapsed_s
                ):
                    _, delayed[index], delayed_sensitivity[index] = (
                        prediction_events[index].pop(0)
                    )
                state[index], sensitivity[index] = self._servo_step(
                    index,
                    float(state[index]),
                    float(delayed[index]),
                    prediction_dt,
                    float(sensitivity[index]),
                    float(delayed_sensitivity[index]),
                    float(active_angle_limits[index]),
                )

            for index in range(THRUST_COUNT):
                state_index = ANGLE_COUNT + index
                state[state_index], sensitivity[state_index] = self._motor_step(
                    index,
                    float(state[state_index]),
                    float(candidate_command[state_index]),
                    prediction_dt,
                    float(sensitivity[state_index]),
                    1.0,
                )
        return state, np.diag(sensitivity)

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

    def _project_command(
        self,
        command: np.ndarray,
        previous: np.ndarray,
        dt: float,
        active_angle_limits: np.ndarray,
    ) -> np.ndarray:
        cfg = self.config
        projected = command.copy()
        servo_limit = np.minimum(cfg.servo_command_limit_rad, active_angle_limits)
        projected[:ANGLE_COUNT] = np.clip(
            projected[:ANGLE_COUNT], -servo_limit, servo_limit
        )
        servo_delta = cfg.servo_command_rate_rad_s * dt
        projected[:ANGLE_COUNT] = np.clip(
            projected[:ANGLE_COUNT],
            previous[:ANGLE_COUNT] - servo_delta,
            previous[:ANGLE_COUNT] + servo_delta,
        )
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

        previous = self.command.copy()
        command = self._project_command(
            0.75 * previous + 0.25 * preferred,
            previous,
            dt,
            limits,
        )
        normalizer = cfg.command_scale
        sqrt_weight = np.sqrt(cfg.wrench_weight) / cfg.wrench_scale
        status = 'solved'
        completed_iterations = 0

        try:
            for iteration in range(cfg.gauss_newton_iterations):
                predicted_state, state_sensitivity = self._predict_terminal(command, limits)
                predicted_wrench = self.model.wrench(predicted_state)
                command_jacobian = (
                    self.model.jacobian(predicted_state)
                    @ state_sensitivity
                    @ np.diag(normalizer)
                )
                weighted_jacobian_base = sqrt_weight[:, None] * command_jacobian
                weighted_wrench_error = sqrt_weight * (predicted_wrench - desired)
                weighted_rate_error = (
                    math.sqrt(cfg.wrench_rate_weight)
                    * sqrt_weight
                    * (
                        predicted_wrench
                        - estimated_wrench
                        - cfg.horizon_s * jerk_reference
                    )
                )
                weighted_jacobian = np.vstack((
                    weighted_jacobian_base,
                    math.sqrt(cfg.wrench_rate_weight) * weighted_jacobian_base,
                ))
                weighted_error = np.concatenate((
                    weighted_wrench_error,
                    weighted_rate_error,
                ))
                command_normalized = command / normalizer
                previous_normalized = previous / normalizer
                preferred_normalized = preferred / normalizer
                hessian = (
                    weighted_jacobian.T @ weighted_jacobian
                    + np.diag(cfg.command_move_weight + cfg.command_preference_weight)
                    + 1e-8 * np.eye(ACTUATOR_COUNT)
                )
                gradient = (
                    weighted_jacobian.T @ weighted_error
                    + cfg.command_move_weight * (command_normalized - previous_normalized)
                    + cfg.command_preference_weight * (
                        command_normalized - preferred_normalized
                    )
                )
                step = np.linalg.solve(hessian, -gradient)
                if not np.all(np.isfinite(step)):
                    raise np.linalg.LinAlgError('non-finite DRCDA step')
                next_command = self._project_command(
                    command + normalizer * step,
                    previous,
                    dt,
                    limits,
                )
                completed_iterations = iteration + 1
                if np.linalg.norm((next_command - command) / normalizer) < 1e-5:
                    command = next_command
                    break
                command = next_command
        except (np.linalg.LinAlgError, FloatingPointError, ValueError):
            status = 'motor_only_fallback'
            command = self._motor_only_fallback(desired, previous, dt, limits)

        predicted_state, state_sensitivity = self._predict_terminal(command, limits)
        predicted_wrench = self.model.wrench(predicted_state)
        predicted_rate = (
            predicted_wrench - estimated_wrench
        ) / max(cfg.horizon_s, cfg.prediction_dt_s)
        wrench_rate_residual = jerk_reference - predicted_rate
        wrench_residual = predicted_wrench - desired

        self.command = command
        for index in range(ANGLE_COUNT):
            _, _, delay_s, _ = self._servo_parameters(index, command[index])
            self._pending_servo_commands[index].append((delay_s, float(command[index])))

        solve_time_ms = (time.perf_counter() - start_time) * 1000.0
        result = DRCDAResult(
            command=command.copy(),
            predicted_state=predicted_state,
            estimated_wrench=estimated_wrench,
            predicted_wrench=predicted_wrench,
            desired_wrench=desired,
            jerk_reference=jerk_reference,
            wrench_rate_residual=wrench_rate_residual,
            wrench_residual=wrench_residual,
            solve_time_ms=solve_time_ms,
            iterations=completed_iterations,
            status=status,
        )
        self.last_result = result
        return result
