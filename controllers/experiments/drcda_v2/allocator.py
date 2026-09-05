#!/usr/bin/env python3
"""Independent DRCDA v2 and paper-normalized differential allocators."""

from __future__ import annotations

import math
import time
from typing import Iterable

import numpy as np

from controllers.common.hnuter_drcda import (
    ACTUATOR_COUNT,
    ANGLE_COUNT,
    THRUST_COUNT,
    DRCDAAllocator,
    DRCDAConfig,
    DRCDAResult,
    HnuterWrenchModel,
)


def _vector(values: Iterable[float], count: int, name: str) -> np.ndarray:
    result = np.asarray(tuple(values), dtype=float)
    if result.shape != (count,):
        raise ValueError(f'{name} must contain {count} values')
    return result


def uniform_saturate(vector: Iterable[float], limit: float = 1.0) -> np.ndarray:
    """Scale a vector uniformly so its largest magnitude reaches ``limit``."""
    result = np.asarray(tuple(vector), dtype=float)
    peak = float(np.max(np.abs(result))) if result.size else 0.0
    if peak > limit > 0.0:
        result = result * (limit / peak)
    return result


class _JerkReferenceMixin:
    def _jerk_reference(self, desired: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        cfg = self.config
        estimated = self.model.wrench(self.state)
        raw_ff = (desired - self._previous_desired_wrench) / dt
        ff_alpha = dt / (max(cfg.wrench_ff_tau_s, 0.0) + dt)
        self._filtered_wrench_ff += ff_alpha * (raw_ff - self._filtered_wrench_ff)
        jerk = self._filtered_wrench_ff + cfg.wrench_error_gain * (desired - estimated)
        self._previous_desired_wrench = desired.copy()
        return estimated, jerk


class PaperNormalizedDifferentialAllocator(_JerkReferenceMixin, DRCDAAllocator):
    """Normalized differential allocation following Cuniato et al. Eq. 7-13.

    The actuator-rate bounds define the normalization, the augmented wrench
    error supplies jerk without acceleration feedback, and saturation scales
    the complete normalized rate vector rather than clipping components.
    """

    def __init__(
        self,
        model: HnuterWrenchModel,
        config: DRCDAConfig | None = None,
        nullspace_gain: float = 0.08,
        rcond: float = 1e-4,
    ) -> None:
        super().__init__(model, config)
        self.nullspace_gain = max(float(nullspace_gain), 0.0)
        self.rcond = max(float(rcond), 1e-9)

    def _rate_bounds(self, dt: float, limits: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        cfg = self.config
        lower = np.empty(ACTUATOR_COUNT)
        upper = np.empty(ACTUATOR_COUNT)

        servo_rate = cfg.servo_command_rate_rad_s
        physical_limit = np.minimum(cfg.servo_state_limit_rad, limits)
        lower[:ANGLE_COUNT] = np.maximum(
            -servo_rate,
            (-physical_limit - self.command[:ANGLE_COUNT]) / dt,
        )
        upper[:ANGLE_COUNT] = np.minimum(
            servo_rate,
            (physical_limit - self.command[:ANGLE_COUNT]) / dt,
        )

        for index in range(THRUST_COUNT):
            state_index = ANGLE_COUNT + index
            rate_down, rate_up = self._motor_rate_bounds(
                index,
                float(self.command[state_index]),
                float(self.command[state_index]),
            )
            command_rate = cfg.thrust_command_rate_n_s[index]
            lower[state_index] = max(
                -rate_down,
                -command_rate,
                (cfg.thrust_min_n[index] - self.command[state_index]) / dt,
            )
            upper[state_index] = min(
                rate_up,
                command_rate,
                (cfg.thrust_max_n[index] - self.command[state_index]) / dt,
            )
        return lower, upper

    def allocate(
        self,
        desired_wrench: Iterable[float],
        dt: float,
        preferred_command: Iterable[float] | None = None,
        active_angle_limits: Iterable[float] | None = None,
    ) -> DRCDAResult:
        started = time.perf_counter()
        cfg = self.config
        desired = _vector(desired_wrench, 6, 'desired_wrench')
        dt = float(np.clip(dt, 0.0005, 0.05))
        limits = (
            cfg.servo_state_limit_rad.copy()
            if active_angle_limits is None
            else _vector(active_angle_limits, ANGLE_COUNT, 'active_angle_limits')
        )
        limits = np.minimum(limits, cfg.servo_state_limit_rad)
        preferred = (
            self.command.copy()
            if preferred_command is None
            else _vector(preferred_command, ACTUATOR_COUNT, 'preferred_command')
        )

        # The no-delay comparison uses measured/current actuator state. In SITL
        # this state is published directly to the plant after each allocation.
        self.state = self.command.copy()
        estimated, jerk = self._jerk_reference(desired, dt)
        lower, upper = self._rate_bounds(dt, limits)
        midpoint = 0.5 * (upper + lower)
        half_span = np.maximum(0.5 * (upper - lower), 1e-9)
        jacobian = self.model.jacobian(self.state)
        normalized_jacobian = jacobian @ np.diag(half_span)
        normalized_target = jerk - jacobian @ midpoint
        row_scale = np.sqrt(cfg.wrench_weight) / cfg.wrench_scale
        matrix = row_scale[:, None] * normalized_jacobian
        target = row_scale * normalized_target

        status = 'paper_nda'
        try:
            pseudo = np.linalg.pinv(matrix, rcond=self.rcond)
            normalized_rate = pseudo @ target
            preferred_rate = np.clip((preferred - self.command) / dt, lower, upper)
            preferred_normalized = (preferred_rate - midpoint) / half_span
            nullspace = np.eye(ACTUATOR_COUNT) - pseudo @ matrix
            normalized_rate += (
                self.nullspace_gain * nullspace @ preferred_normalized
            )
            normalized_rate = uniform_saturate(normalized_rate)
            actuator_rate = midpoint + half_span * normalized_rate
            command = self._project_command(
                self.command + dt * actuator_rate,
                self.command,
                dt,
                limits,
            )
        except (np.linalg.LinAlgError, FloatingPointError, ValueError):
            status = 'paper_nda_motor_only_fallback'
            command = self._motor_only_fallback(desired, self.command, dt, limits)

        self.command = command
        self.state = command.copy()
        predicted = self.model.wrench(self.state)
        predicted_rate = (predicted - estimated) / dt
        result = DRCDAResult(
            command=command.copy(),
            predicted_state=self.state.copy(),
            estimated_wrench=estimated,
            predicted_wrench=predicted,
            desired_wrench=desired,
            jerk_reference=jerk,
            wrench_rate_residual=jerk - predicted_rate,
            wrench_residual=predicted - desired,
            solve_time_ms=(time.perf_counter() - started) * 1000.0,
            iterations=1,
            status=status,
        )
        self.last_result = result
        return result


class ReachabilityNormalizedDRCDAAllocator(_JerkReferenceMixin, DRCDAAllocator):
    """DRCDA v2 with reachability prediction and monotone bounded updates."""

    def __init__(
        self,
        model: HnuterWrenchModel,
        config: DRCDAConfig | None = None,
        lm_damping: float = 2e-4,
        max_normalized_step: float = 0.65,
        line_search_steps: int = 5,
    ) -> None:
        super().__init__(model, config)
        self.lm_damping = max(float(lm_damping), 1e-10)
        self.max_normalized_step = float(np.clip(max_normalized_step, 0.05, 2.0))
        self.line_search_steps = max(int(line_search_steps), 1)

    def _cost(
        self,
        command: np.ndarray,
        previous: np.ndarray,
        preferred: np.ndarray,
        desired: np.ndarray,
        estimated: np.ndarray,
        jerk: np.ndarray,
        limits: np.ndarray,
    ) -> tuple[float, np.ndarray, np.ndarray]:
        cfg = self.config
        predicted_state, sensitivity = self._predict_terminal(command, limits)
        predicted = self.model.wrench(predicted_state)
        row_scale = np.sqrt(cfg.wrench_weight) / cfg.wrench_scale
        terminal_error = row_scale * (predicted - desired)
        rate_target = estimated + cfg.horizon_s * jerk
        rate_error = row_scale * (predicted - rate_target)
        normalizer = cfg.command_scale
        move = (command - previous) / normalizer
        preference = (command - preferred) / normalizer
        cost = (
            float(terminal_error @ terminal_error)
            + cfg.wrench_rate_weight * float(rate_error @ rate_error)
            + float(cfg.command_move_weight @ (move * move))
            + float(cfg.command_preference_weight @ (preference * preference))
        )
        return cost, predicted_state, sensitivity

    def _uniform_feasible_update(
        self,
        command: np.ndarray,
        delta: np.ndarray,
        previous: np.ndarray,
        dt: float,
        limits: np.ndarray,
    ) -> np.ndarray:
        raw_target = command + delta
        clipped = self._project_command(raw_target, previous, dt, limits)
        clipped_delta = clipped - command
        ratios = [1.0]
        for raw, feasible in zip(delta, clipped_delta):
            if abs(raw) <= 1e-12:
                continue
            ratio = feasible / raw
            if ratio <= 0.0:
                ratios.append(0.0)
            elif ratio < 1.0:
                ratios.append(float(ratio))
        scale = float(np.clip(min(ratios), 0.0, 1.0))
        return self._project_command(command + scale * delta, previous, dt, limits)

    def allocate(
        self,
        desired_wrench: Iterable[float],
        dt: float,
        preferred_command: Iterable[float] | None = None,
        active_angle_limits: Iterable[float] | None = None,
    ) -> DRCDAResult:
        started = time.perf_counter()
        cfg = self.config
        desired = _vector(desired_wrench, 6, 'desired_wrench')
        dt = float(np.clip(dt, 0.0005, 0.05))
        limits = (
            cfg.servo_state_limit_rad.copy()
            if active_angle_limits is None
            else _vector(active_angle_limits, ANGLE_COUNT, 'active_angle_limits')
        )
        limits = np.minimum(limits, cfg.servo_state_limit_rad)
        preferred = (
            self.command.copy()
            if preferred_command is None
            else _vector(preferred_command, ACTUATOR_COUNT, 'preferred_command')
        )

        self._advance_state(dt)
        estimated, jerk = self._jerk_reference(desired, dt)
        previous = self.command.copy()
        command = self._project_command(previous.copy(), previous, dt, limits)
        normalizer = cfg.command_scale
        row_scale = np.sqrt(cfg.wrench_weight) / cfg.wrench_scale
        status = 'drcda_v2'
        completed = 0

        try:
            cost, predicted_state, sensitivity = self._cost(
                command, previous, preferred, desired, estimated, jerk, limits
            )
            for iteration in range(cfg.gauss_newton_iterations):
                predicted = self.model.wrench(predicted_state)
                command_jacobian = (
                    self.model.jacobian(predicted_state)
                    @ sensitivity
                    @ np.diag(normalizer)
                )
                weighted_jacobian = row_scale[:, None] * command_jacobian
                terminal_error = row_scale * (predicted - desired)
                rate_target = estimated + cfg.horizon_s * jerk
                rate_error = row_scale * (predicted - rate_target)
                stacked_jacobian = np.vstack((
                    weighted_jacobian,
                    math.sqrt(cfg.wrench_rate_weight) * weighted_jacobian,
                ))
                stacked_error = np.concatenate((
                    terminal_error,
                    math.sqrt(cfg.wrench_rate_weight) * rate_error,
                ))
                command_n = command / normalizer
                previous_n = previous / normalizer
                preferred_n = preferred / normalizer
                regularization = cfg.command_move_weight + cfg.command_preference_weight
                hessian = (
                    stacked_jacobian.T @ stacked_jacobian
                    + np.diag(regularization + self.lm_damping)
                )
                gradient = (
                    stacked_jacobian.T @ stacked_error
                    + cfg.command_move_weight * (command_n - previous_n)
                    + cfg.command_preference_weight * (command_n - preferred_n)
                )
                normalized_step = np.linalg.solve(hessian, -gradient)
                if not np.all(np.isfinite(normalized_step)):
                    raise np.linalg.LinAlgError('non-finite DRCDA v2 step')
                normalized_step = uniform_saturate(
                    normalized_step, self.max_normalized_step
                )

                accepted = False
                for search_index in range(self.line_search_steps):
                    trial_delta = normalizer * normalized_step * (0.5 ** search_index)
                    trial = self._uniform_feasible_update(
                        command, trial_delta, previous, dt, limits
                    )
                    trial_cost, trial_state, trial_sensitivity = self._cost(
                        trial, previous, preferred, desired, estimated, jerk, limits
                    )
                    if trial_cost <= cost + 1e-12:
                        command = trial
                        cost = trial_cost
                        predicted_state = trial_state
                        sensitivity = trial_sensitivity
                        accepted = True
                        completed = iteration + 1
                        break
                if not accepted:
                    status = 'drcda_v2_stalled'
                    break
                if np.linalg.norm(normalized_step) < 1e-5:
                    break
        except (np.linalg.LinAlgError, FloatingPointError, ValueError):
            status = 'drcda_v2_motor_only_fallback'
            command = self._motor_only_fallback(desired, previous, dt, limits)

        predicted_state, _ = self._predict_terminal(command, limits)
        predicted = self.model.wrench(predicted_state)
        predicted_rate = (predicted - estimated) / max(
            cfg.horizon_s, cfg.prediction_dt_s
        )
        self.command = command
        for index in range(ANGLE_COUNT):
            _, _, delay_s, _ = self._servo_parameters(index, command[index])
            self._pending_servo_commands[index].append((delay_s, float(command[index])))

        result = DRCDAResult(
            command=command.copy(),
            predicted_state=predicted_state,
            estimated_wrench=estimated,
            predicted_wrench=predicted,
            desired_wrench=desired,
            jerk_reference=jerk,
            wrench_rate_residual=jerk - predicted_rate,
            wrench_residual=predicted - desired,
            solve_time_ms=(time.perf_counter() - started) * 1000.0,
            iterations=completed,
            status=status,
        )
        self.last_result = result
        return result
