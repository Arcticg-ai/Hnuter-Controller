"""Measured actuator predictor and a checked reachable-wrench contract."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from controllers.common.hnuter_drcda import ACTUATOR_COUNT, DRCDAAllocator


@dataclass(frozen=True)
class ReachableWrenchContract:
    """A witness in the actuator-rate box and its nonlinear predicted image."""

    requested_wrench: np.ndarray
    reachable_wrench: np.ndarray
    reachable_rate: np.ndarray
    command: np.ndarray
    command_lower: np.ndarray
    command_upper: np.ndarray
    predicted_yaw_rate_frd: float

    @classmethod
    def from_allocation(cls, allocator: DRCDAAllocator, result,
                        previous_command: np.ndarray, dt: float,
                        active_angle_limits: np.ndarray,
                        measured_yaw_rate_frd: float, inertia_z: float):
        lower = allocator._project_command(
            np.full(ACTUATOR_COUNT, -1e6), previous_command, dt,
            active_angle_limits,
        )
        upper = allocator._project_command(
            np.full(ACTUATOR_COUNT, 1e6), previous_command, dt,
            active_angle_limits,
        )
        if (np.any(result.command < lower - 1e-7)
                or np.any(result.command > upper + 1e-7)):
            raise ValueError('allocator command violates reachable actuator-rate box')
        if not np.all(np.isfinite(result.predicted_wrench)):
            raise ValueError('non-finite reachable wrench')
        if not np.allclose(
            allocator.model.wrench(result.predicted_state),
            result.predicted_wrench, atol=1e-7, rtol=1e-7,
        ):
            raise ValueError('reachable wrench lacks a model-state witness')
        if not np.isfinite(measured_yaw_rate_frd):
            raise ValueError('measured yaw rate is non-finite')
        horizon = max(allocator.config.horizon_s,
                      allocator.config.prediction_dt_s)
        rate = (result.predicted_wrench - result.estimated_wrench) / horizon
        yaw_rate = (measured_yaw_rate_frd
                    - horizon * result.predicted_wrench[5] / inertia_z)
        return cls(
            requested_wrench=result.desired_wrench.copy(),
            reachable_wrench=result.predicted_wrench.copy(),
            reachable_rate=rate,
            command=result.command.copy(),
            command_lower=lower,
            command_upper=upper,
            predicted_yaw_rate_frd=float(yaw_rate),
        )


class MeasuredDRCDAAllocator(DRCDAAllocator):
    """Correct the predicted servo state after every model time advance."""

    def __init__(self, model, config=None):
        super().__init__(model, config)
        self.measured_angles: np.ndarray | None = None

    def _advance_state(self, dt: float) -> None:
        super()._advance_state(dt)
        if self.measured_angles is not None:
            self.state[:4] = self.measured_angles
