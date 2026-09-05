#!/usr/bin/env python3
"""Independent DRCDA v2 and paper-normalized differential allocators."""

from __future__ import annotations

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

    def _jerk_reference(
        self, desired: np.ndarray, dt: float
    ) -> tuple[np.ndarray, np.ndarray]:
        del dt
        estimated = self.model.wrench(self.state)
        jerk = self.config.wrench_error_gain * (desired - estimated)
        self._previous_desired_wrench = desired.copy()
        self._filtered_wrench_ff[:] = 0.0
        return estimated, jerk

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
        matrix = normalized_jacobian
        target = normalized_target

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

class ReachabilityDRCDAAllocatorV2(DRCDAAllocator):
    """Independent v2 entry point using the validated reachability solver."""

    def allocate(
        self,
        desired_wrench: Iterable[float],
        dt: float,
        preferred_command: Iterable[float] | None = None,
        active_angle_limits: Iterable[float] | None = None,
    ) -> DRCDAResult:
        result = super().allocate(
            desired_wrench,
            dt,
            preferred_command=preferred_command,
            active_angle_limits=active_angle_limits,
        )
        if result.status == 'solved':
            result.status = 'drcda_v2'
        elif result.status == 'motor_only_fallback':
            result.status = 'drcda_v2_motor_only_fallback'
        return result
