"""Measured-state NDA adaptation used only in no-delay Gazebo comparisons.

Implements Cuniato Eq. 7-14 in Hnuter force coordinates. Motor force limits
are empirical, not the paper's identified electrical power curves (PDA).
"""
import time

import numpy as np

from controllers.common.hnuter_drcda import DRCDAResult
from controllers.experiments.drcda_closed_loop.governor import MeasuredDRCDAAllocator
from controllers.experiments.drcda_v2.allocator import uniform_saturate


class MeasuredNDAAllocator(MeasuredDRCDAAllocator):
    def _command_rate(self, command, dt, limits):
        """Average rate over one zero-order-held control interval.

        Eq. 14 is continuous. Its sampled realization must use the exact
        held-input map, especially when dt is longer than the motor tau.
        """
        cfg = self.config
        if getattr(self, 'inversion_mode', 'sampled') == 'continuous':
            gains = np.where(command[:4] >= 0, cfg.servo_gain_positive, cfg.servo_gain_negative)
            taus = np.where(command[:4] >= 0, cfg.servo_tau_positive_s, cfg.servo_tau_negative_s)
            rate = np.empty(9)
            rate[:4] = np.clip((gains*command[:4]-self.state[:4])/taus,
                              -cfg.servo_rate_negative_rad_s, cfg.servo_rate_positive_rad_s)
            taus_motor = np.where(command[4:] >= self.state[4:], cfg.motor_tau_up_s, cfg.motor_tau_down_s)
            rate[4:] = (command[4:]-self.state[4:])/taus_motor
            for i in range(5):
                down, up = self._motor_rate_bounds(i, self.state[4+i], command[4+i])
                rate[4+i] = np.clip(rate[4+i], -down, up)
            return rate
        next_state = self.state.copy()
        for i in range(4):
            next_state[i], _ = self._servo_step(i, self.state[i], command[i], dt, state_limit=limits[i])
        for i in range(5):
            next_state[4+i], _ = self._motor_step(i, self.state[4+i], command[4+i], dt)
        return (next_state-self.state)/dt

    def allocate(self, desired_wrench, dt, preferred_command=None,
                 active_angle_limits=None, future_wrench_references=None):
        start = time.perf_counter()
        cfg = self.config
        dt = float(np.clip(dt, 0.0005, 0.05))
        desired = np.asarray(desired_wrench, dtype=float)
        limits = cfg.servo_state_limit_rad if active_angle_limits is None else np.asarray(active_angle_limits)
        self._advance_state(dt)
        estimated = self.model.wrench(self.state)
        jerk = getattr(self, 'nda_gain', cfg.wrench_error_gain) * (desired - estimated)
        previous = self.command.copy()
        low_command = self._project_command(np.full(9, -1e6), previous, dt, limits)
        high_command = self._project_command(np.full(9, 1e6), previous, dt, limits)

        def dynamics(command):
            return self._command_rate(command, dt, limits)

        lower, upper = dynamics(low_command), dynamics(high_command)
        midpoint = (upper + lower) / 2
        span = np.maximum((upper - lower) / 2, 1e-9)
        jacobian = self.model.jacobian(self.state)
        matrix = jacobian * span[None, :]
        pseudo = np.linalg.pinv(matrix, rcond=1e-5)
        preferred_rate = np.zeros(9)
        # Eq. 4/12 permits posture and mechanical-limit secondary objectives.
        # The two-axis Hnuter arms have an opposing-tilt self-motion absent
        # from the paper's single-axis arm layout. Project this preference,
        # never add it directly to the primary wrench command.
        preferred_rate[:4] = -getattr(self, 'posture_gain', 0.0)*self.state[:4]
        # Eq. 6 targets a fixed hovering speed, not the current mean speed.
        # With F=k*omega^2, Fdot*=2*k_omega*sqrt(F)*(sqrt(Fhover)-sqrt(F)).
        front_x = self.model.positions[0, 0]
        tail_x = self.model.positions[4, 0]
        hover_front = getattr(self, 'hover_front_thrust', 4.5*9.81*(-tail_x)/(4*(front_x-tail_x)))
        force_root = np.sqrt(np.maximum(self.state[4:8], 0.0))
        preferred_rate[4:8] = (2*getattr(self, 'nullspace_gain', 2.0)*force_root
                               * (np.sqrt(hover_front)-force_root))
        preferred_normalized = (np.clip(preferred_rate, lower, upper)-midpoint)/span
        # Eq. 12's nullspace term is essential with asymmetric bounds: the
        # minimum normalized norm alone need not mean zero physical motion.
        normalized = (pseudo @ (jerk - jacobian @ midpoint)
                      + (np.eye(9)-pseudo@matrix) @ preferred_normalized)
        self.saturation_scale = 1.0/max(1.0, float(np.max(np.abs(normalized))))
        self.normalized_condition = float(np.linalg.cond(matrix))
        rate = midpoint + span * uniform_saturate(normalized)
        # Invert the same monotone, piecewise first-order dynamics, including
        # asymmetric gains and the feasible command-rate box.
        lo, hi = low_command.copy(), high_command.copy()
        for _ in range(24):
            mid = (lo + hi) / 2
            below = dynamics(mid) < rate
            lo = np.where(below, mid, lo)
            hi = np.where(below, hi, mid)
        command = (lo + hi) / 2
        predicted_state, _ = self._predict_horizons(command, limits, [cfg.horizon_s])[0]
        predicted = self.model.wrench(predicted_state)
        self.command = command
        for i in range(4):
            self._pending_servo_commands[i].append((0.0, float(command[i])))
        self._previous_desired_wrench = desired.copy()
        result = DRCDAResult(
            command=command.copy(), predicted_state=predicted_state,
            estimated_wrench=estimated, predicted_wrench=predicted,
            desired_wrench=desired, jerk_reference=jerk,
            wrench_rate_residual=jerk - (predicted-estimated)/cfg.horizon_s,
            wrench_residual=predicted-desired,
            solve_time_ms=(time.perf_counter()-start)*1000,
            iterations=1, status='measured_nda',
        )
        self.last_result = result
        return result
