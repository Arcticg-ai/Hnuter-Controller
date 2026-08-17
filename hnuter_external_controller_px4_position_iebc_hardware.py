#!/usr/bin/env python3
"""Reusable hardware IEBC gateway for the validated Hnuter Offboard stack.

The node deliberately inherits the hardware controller instead of copying it.
PX4/RC retains Arm and Offboard authority and this file never publishes a
``VehicleCommand``.  The default hardware mode keeps the inherited manual RC
flight path active in Offboard, then runs a latched-heading push/return task
from a configurable logical RC channel.  A separate composition mode accepts
an upstream nominal PX4 ``TrajectorySetpoint``.  IEBC filters the resulting
position, velocity and acceleration references before the validated hardware
base class publishes them on ``/fmu/in/trajectory_setpoint``.

Topic contract
--------------

Inputs:

* ``/hnuter/iebc/in/trajectory_setpoint`` (px4_msgs/TrajectorySetpoint):
  absolute NED position, velocity and acceleration nominal reference.
* ``/fmu/out/actuator_motors`` (px4_msgs/ActuatorMotors) and
  ``/fmu/out/actuator_servos`` (px4_msgs/ActuatorServos): PX4 actuator
  commands used by the default hardware model to estimate propulsive force.
* ``/hnuter/iebc/in/actuator_wrench`` (geometry_msgs/WrenchStamped): optional
  external actuator-force estimate in the ENU world frame.  This is not
  contact force and is selected explicitly instead of the command model.
* ``/hnuter/iebc/in/recovery`` (std_msgs/Bool): a rising ``True`` edge marks
  physical load release and enters the certified stopping controller.
* ``/hnuter/iebc/in/reset`` (std_msgs/Empty): reset IEBC storage and reference
  state at the current flight-session origin.

Outputs are the inherited PX4 Offboard heartbeat and trajectory-setpoint
topics plus ``/hnuter/iebc/out/status`` (std_msgs/String, JSON).

The default hardware source reconstructs force from PX4's post-allocation
motor and servo commands.  It is a command/model estimate, not measured motor
RPM, thrust or servo position.  A true external estimator can be selected when
available.  Stale nominal commands or stale force estimates latch a
zero-velocity hold instead of failing open.
"""

import json
import math
import os
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

import rclpy
from geometry_msgs.msg import WrenchStamped
from px4_msgs.msg import ActuatorMotors, ActuatorServos, RcChannels, TrajectorySetpoint
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String

from hnuter_external_controller_px4_position_hardware import (
    HnuterController as ValidatedHardwareController,
)


def env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name, '1' if default else '0')
    return str(raw).strip().lower() in ('1', 'true', 'yes', 'on')


def env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


def env_vec3(prefix: str, default) -> np.ndarray:
    default = np.asarray(default, dtype=float).reshape(3)
    return np.array([
        env_float(f'{prefix}_X', default[0]),
        env_float(f'{prefix}_Y', default[1]),
        env_float(f'{prefix}_Z', default[2]),
    ], dtype=float)


class InteractionEnergyBarrierFilter:
    """Closed-loop Interaction-Energy Barrier Control (IEBC), 1-D specialization.

    Revised certificate and reference-power barrier:

        E_I     = K_I + V_c + S_e_bar
        K_I     = 0.5 * lambda_bar * v_I^2
        V_c     = 0.5 * K_c * e_I^2,        e_I = s_d - s
        g_E     = K_c * e_I + D_c * v_I
        pi_E    = v_I * u_ff + P_r_bar + Delta_e
        P_allow = D_c*v_I^2 - pi_E + gamma*(E_max - E_I)
        g_E * v_d <= P_allow

    The decision variable is desired interaction velocity v_d.  This is the
    key revision relative to the previous acceleration-level implementation:
    at blocked contact v_I=0 but e_I!=0, so g_E=K_c*e_I remains nonzero and
    the filter can stop further controller-side virtual-energy accumulation.

    K_c and D_c must be certified equivalent interaction-axis gains of the
    lower-level controller; unmodelled mismatch belongs in P_r_bar.  Actual
    actuator wrench reconstruction is required for quantitative experiments.
    """

    MODE_INTERACTION = 'interaction'
    MODE_RECOVERY = 'recovery'
    MODE_HOLD = 'hold'

    def __init__(self, logger=None):
        self.logger = logger
        self.enabled = env_bool('HNUTER_IEBC_ENABLE', False)
        self.mass = env_float('HNUTER_IEBC_MASS_KG', 0.0)
        self.e_max = env_float('HNUTER_IEBC_E_MAX_J', 0.0)
        self.energy_reserve_j = max(
            env_float('HNUTER_IEBC_ENERGY_RESERVE_J', 0.0), 0.0)
        self.g = env_float('HNUTER_IEBC_GRAVITY_MPS2', 9.80665)

        axis = env_vec3('HNUTER_IEBC_AXIS', [1.0, 0.0, 0.0])
        axis_norm = float(np.linalg.norm(axis))
        self.axis = axis / axis_norm if axis_norm > 1e-9 else np.array([1.0, 0.0, 0.0])

        self.lambda_bar = env_float(
            'HNUTER_IEBC_LAMBDA_BAR_KG', self.mass if self.mass > 0.0 else 0.0)
        self.k_c = max(env_float('HNUTER_IEBC_KC_NPM', 0.0), 0.0)
        self.d_c = max(env_float('HNUTER_IEBC_DC_NSPM', 0.0), 0.0)
        self.gamma = max(env_float('HNUTER_IEBC_CBF_GAMMA', 4.0), 0.0)

        # Admissible reference-rate set V_I and optional nominal-path resync.
        self.reference_sync_gain = max(env_float('HNUTER_IEBC_REF_SYNC_GAIN', 0.0), 0.0)
        self.max_ref_speed = max(env_float('HNUTER_IEBC_MAX_REF_SPEED_MPS', 0.8), 0.02)
        self.max_ref_accel = max(env_float('HNUTER_IEBC_MAX_REF_ACCEL_MPS2', 3.0), 0.05)
        self.max_ref_jerk = max(env_float(
            'HNUTER_IEBC_MAX_REF_JERK_MPS3', 5.0), 0.05)
        self.g_epsilon_n = max(env_float('HNUTER_IEBC_GE_EPS_N', 1e-4), 1e-8)

        # Release-recovery certificate.  The stopping-distance barrier reserves
        # enough forward travel to dissipate the complete certified energy at
        # no less than brake_force_cert.  These are certificate values, not
        # claims about unmeasured real-hardware force capability.
        self.stop_distance = max(env_float(
            'HNUTER_IEBC_STOP_DISTANCE_M', 1.0), 0.01)
        self.brake_force_cert = max(env_float(
            'HNUTER_IEBC_BRAKE_FORCE_CERT_N', 20.0), 1e-3)
        self.rho_min = max(env_float(
            'HNUTER_IEBC_RECOVERY_RHO_MIN', 0.2), 0.0)
        self.rho_max = max(env_float(
            'HNUTER_IEBC_RECOVERY_RHO_MAX', 4.0), self.rho_min)
        self.stop_gamma = max(env_float(
            'HNUTER_IEBC_STOP_CBF_GAMMA', 4.0), 0.0)
        self.stop_vel_tol = max(env_float(
            'HNUTER_IEBC_STOP_VEL_TOL_MPS', 0.05), 0.0)
        self.stop_error_tol = max(env_float(
            'HNUTER_IEBC_STOP_ERROR_TOL_M', 0.05), 0.0)
        self.stop_energy_tol = max(env_float(
            'HNUTER_IEBC_STOP_ENERGY_TOL_J', 0.1), 0.0)
        self.stop_hold_s = max(env_float(
            'HNUTER_IEBC_STOP_HOLD_S', 1.50), 0.0)
        self.recovery_power_bound = max(env_float(
            'HNUTER_IEBC_RECOVERY_POWER_BOUND_W', 0.0), 0.0)
        self.recovery_stop_arm_vel = max(env_float(
            'HNUTER_IEBC_RECOVERY_STOP_ARM_VEL_MPS', 0.30),
            self.stop_vel_tol)
        self.recovery_stop_min_time = max(env_float(
            'HNUTER_IEBC_RECOVERY_STOP_MIN_TIME_S', 0.50), 0.0)

        # Power-estimation and residual-power bounds.
        self.power_lpf_tau = max(env_float('HNUTER_IEBC_POWER_LPF_TAU_S', 0.03), 0.0)
        self.power_margin_w = max(env_float('HNUTER_IEBC_POWER_MARGIN_W', 0.0), 0.0)
        self.force_error_bound_n = max(env_float('HNUTER_IEBC_FORCE_ERROR_BOUND_N', 0.0), 0.0)
        self.residual_power_bound_w = max(
            env_float('HNUTER_IEBC_RESIDUAL_POWER_BOUND_W', 0.0), 0.0)
        self.storage_initial_j = max(env_float('HNUTER_IEBC_STORAGE_INITIAL_J', 0.0), 0.0)

        # Known feedforward term u_ff.  In 'zero' mode, interaction-axis
        # acceleration feedforward is removed from the PX4 setpoint.
        self.accel_ff_mode = os.environ.get(
            'HNUTER_IEBC_ACCEL_FF_MODE', 'nominal').strip().lower()
        if self.accel_ff_mode not in ('nominal', 'zero'):
            self.accel_ff_mode = 'nominal'
        self.ff_mass = max(env_float(
            'HNUTER_IEBC_FF_MASS_KG',
            self.lambda_bar if self.lambda_bar > 0.0 else self.mass), 0.0)

        self.wrench_source = os.environ.get('HNUTER_IEBC_WRENCH_SOURCE', 'proxy').strip().lower()
        if self.wrench_source not in ('proxy', 'external'):
            self.wrench_source = 'proxy'
        self.wrench_topic = os.environ.get(
            'HNUTER_IEBC_WRENCH_TOPIC', '/hnuter/actuator_wrench_estimate')
        self.wrench_timeout_s = max(env_float('HNUTER_IEBC_WRENCH_TIMEOUT_S', 0.20), 0.02)

        self.valid_configuration = bool(
            self.mass > 0.0 and self.lambda_bar > 0.0 and self.e_max > 0.0
            and self.k_c > 0.0 and self.d_c >= 0.0)
        if self.e_max > 0.0:
            self.energy_reserve_j = min(self.energy_reserve_j, 0.5 * self.e_max)
        if self.enabled and not self.valid_configuration:
            self._warn('IEBC 配置无效：MASS、LAMBDA_BAR、E_MAX、KC 必须为正值。IEBC 将旁路。')
        if self.enabled and self.valid_configuration and self.wrench_source == 'proxy':
            self._warn(
                'IEBC 使用 actuator-wrench proxy，仅适合 Gazebo/软件联调；论文定量实验请使用 external wrench。')
        self.reset()

    def _warn(self, text: str) -> None:
        if self.logger is not None:
            self.logger.warn(text)

    def reset(self) -> None:
        # No power-triggered contact gate: the revised certificate must also
        # cover blocked contact where physical interaction power is nearly zero.
        self.interaction_active = bool(self.enabled and self.valid_configuration)
        self.h_prev = None
        self.power_hat_raw = 0.0
        self.power_hat = 0.0
        self.power_error_bound = 0.0
        self.storage_rate = 0.0
        self.storage_bound = float(self.storage_initial_j)
        self.storage_update_enabled = True
        self.storage_frozen = False
        self.delta_e = 0.0
        self.safe_s = None
        self.safe_v = None
        self.safe_v_prev2 = 0.0
        self.mode = self.MODE_INTERACTION
        self.release_s = None
        self.release_direction = 1.0
        self.g_e_last = 0.0
        self.rho = 0.0
        self.recoverable_energy = 0.0
        self.release_excursion = 0.0
        self.stop_distance_barrier = math.inf
        self.reserved_stop_distance = 0.0
        self.recovery_done_duration_s = 0.0
        self.recovery_dissipation_slack_w = 0.0
        self.recovery_stop_latched = False
        self.recovery_motion_seen = False
        self.recovery_elapsed_s = 0.0
        self.recovery_terminal_s = None
        self.recovery_stop_candidate_s = 0.0
        self.recovery_rebase_energy_j = 0.0
        self.recovery_phase = 'inactive'
        self.recovery_reference_velocity = 0.0
        self.recovery_rate_infeasible = False
        self.barrier_active = False
        self.infeasible = False
        self._last_infeasible_warn_s = -math.inf
        self.debug = {
            'enabled': bool(self.enabled and self.valid_configuration),
            'active': self.interaction_active,
            'barrier_active': False,
            'infeasible': False,
            'p_hat': 0.0,
            'p_bar_e': 0.0,
            's_dot_bar': 0.0,
            's_bar': self.storage_bound,
            'storage_update_enabled': True,
            'k_i': 0.0,
            'v_c': 0.0,
            'e_ref': 0.0,
            'e_i': self.storage_bound,
            'h_i': self.e_max - self.storage_bound,
            'h_constraint': self.e_max - self.storage_bound - self.energy_reserve_j,
            'energy_reserve_j': self.energy_reserve_j,
            'v_i': 0.0,
            'v_nom_i': 0.0,
            'v_task_i': 0.0,
            'v_safe_i': 0.0,
            's_nom_i': 0.0,
            's_safe_i': 0.0,
            'g_e': 0.0,
            'pi_e': 0.0,
            'p_allow': 0.0,
            'p_ref_nominal': 0.0,
            'p_ref_safe': 0.0,
            'equivalent_stiffness_force_n': 0.0,
            'qp_slack_w': 0.0,
            'a_safe_i': 0.0,
            'mode': self.mode,
            'recoverable_energy': 0.0,
            'release_excursion': 0.0,
            'stop_distance_barrier': math.inf,
            'reserved_stop_distance': 0.0,
            'rho': 0.0,
            'release_s': math.nan,
            'recovery_done_duration_s': 0.0,
            'recovery_dissipation_slack_w': 0.0,
            'recovery_phase': self.recovery_phase,
            'recovery_reference_velocity': 0.0,
            'recovery_motion_seen': False,
            'recovery_elapsed_s': 0.0,
            'recovery_rate_infeasible': False,
            'recovery_terminal_s': math.nan,
            'recovery_stop_candidate_s': 0.0,
            'recovery_stop_latched': False,
            'recovery_rebase_energy_j': 0.0,
        }

    def freeze_environment_storage(self) -> None:
        """Freeze S_e_bar while keeping the closed-loop certificate active."""
        self.storage_update_enabled = False
        self.storage_frozen = True
        self.storage_rate = 0.0
        self.delta_e = self.residual_power_bound_w
        self.debug['storage_update_enabled'] = False
        self.debug['s_dot_bar'] = 0.0

    def resume_environment_storage(self) -> None:
        """Resume S_e_bar updates without differentiating across the pause."""
        self.storage_update_enabled = True
        self.storage_frozen = False
        self.storage_rate = 0.0
        self.h_prev = None
        self.debug['storage_update_enabled'] = True
        self.debug['s_dot_bar'] = 0.0

    def enter_recovery(self, measured_s: float) -> None:
        """Enter continuous, energy-dissipating release recovery.

        The accumulated push reference must not remain as a forward target
        after the load disappears. Rebase it to the physical contact/release
        point, but conservatively transfer the removed virtual-spring energy
        into frozen storage so the total IEBC energy cannot decrease by a
        coordinate reset. safe_v remains continuous and is jerk-limited to
        zero by RECOVERY.
        """
        if not self.enabled or not self.valid_configuration:
            return
        self.mode = self.MODE_RECOVERY
        self.release_s = float(measured_s)
        self.release_direction = 1.0 if self.g_e_last >= 0.0 else -1.0
        if self.wrench_source == 'proxy':
            self.freeze_environment_storage()
        release_error = float(self.safe_s - measured_s)
        release_rebase_energy = 0.5 * self.k_c * release_error * release_error
        self.storage_bound += release_rebase_energy
        self.safe_s = float(measured_s)
        self.rho = 0.0
        self.recoverable_energy = 0.0
        self.release_excursion = 0.0
        self.stop_distance_barrier = math.inf
        self.reserved_stop_distance = 0.0
        self.recovery_done_duration_s = 0.0
        self.recovery_dissipation_slack_w = 0.0
        self.recovery_stop_latched = False
        self.recovery_motion_seen = False
        self.recovery_elapsed_s = 0.0
        self.recovery_terminal_s = None
        self.recovery_stop_candidate_s = 0.0
        self.recovery_rebase_energy_j = release_rebase_energy
        self.recovery_phase = 'brake'
        self.recovery_reference_velocity = float(self.safe_v)
        self.recovery_rate_infeasible = False
        self.debug.update({
            'mode': self.mode,
            'release_s': self.release_s,
            'storage_update_enabled': self.storage_update_enabled,
        })

    def _mechanical_energy(self, position_enu: np.ndarray, velocity_enu: np.ndarray) -> float:
        # The software proxy reconstructs only the interaction-axis actuator
        # force. Its power balance must therefore use the same 1-D kinetic
        # energy; mixing that force with full XYZ kinetic/potential energy turns
        # ordinary lateral/altitude regulation into fictitious environment
        # storage. External wrench mode retains the full translational balance.
        if self.wrench_source == 'proxy':
            v_i = float(np.dot(self.axis, velocity_enu))
            return 0.5 * self.mass * v_i * v_i

        # Translational H_r = T + U_g. Planned IEBC interaction experiments
        # keep attitude nearly fixed; add rotational energy/torque for aggressive
        # 6-D interaction experiments.
        return float(
            0.5 * self.mass * np.dot(velocity_enu, velocity_enu)
            + self.mass * self.g * position_enu[2])

    def _update_environment_storage(
            self, dt: float, position_enu: np.ndarray, velocity_enu: np.ndarray,
            actuator_force_enu: np.ndarray) -> None:
        """Dynamics-residual power estimate and projected storage upper bound."""
        if not self.storage_update_enabled:
            self.storage_rate = 0.0
            self.delta_e = self.residual_power_bound_w
            return

        H = self._mechanical_energy(position_enu, velocity_enu)
        if self.wrench_source == 'proxy':
            v_i = float(np.dot(self.axis, velocity_enu))
            f_i = float(np.dot(self.axis, actuator_force_enu))
            p_act = f_i * v_i
            uncertainty_speed = abs(v_i)
        else:
            p_act = float(np.dot(actuator_force_enu, velocity_enu))
            uncertainty_speed = float(np.linalg.norm(velocity_enu))

        if self.h_prev is None:
            self.h_prev = H
            self.power_error_bound = (
                self.power_margin_w
                + self.force_error_bound_n * uncertainty_speed)
            self.delta_e = 2.0 * self.power_error_bound
            return

        dH_dt = (H - self.h_prev) / max(dt, 1e-6)
        self.h_prev = H
        self.power_hat_raw = p_act - dH_dt

        if self.power_lpf_tau > 1e-6:
            alpha = float(np.clip(dt / (self.power_lpf_tau + dt), 0.0, 1.0))
            self.power_hat += alpha * (self.power_hat_raw - self.power_hat)
        else:
            self.power_hat = self.power_hat_raw

        self.power_error_bound = (
            self.power_margin_w
            + self.force_error_bound_n * uncertainty_speed)

        # dot(Sbar) = P_+(Sbar, P_hat + Pbar).  Unlike the previous cumulative
        # work implementation, Sbar can decrease when stored energy is returned.
        drive = self.power_hat + self.power_error_bound
        self.storage_rate = drive if (self.storage_bound > 1e-12 or drive >= 0.0) else 0.0
        self.storage_bound = max(0.0, self.storage_bound + self.storage_rate * max(dt, 0.0))

        # dot(Sbar) - P_e <= Delta_e.
        self.delta_e = 2.0 * self.power_error_bound + max(-drive, 0.0)

    def _project_reference_velocity(
            self, v_task: float, v_prev: float, dt: float,
            g_e: float, p_allow: float):
        """Analytic 1-D QP projection with speed and reference-rate bounds."""
        dt = max(float(dt), 1e-6)
        v_low = -self.max_ref_speed
        v_high = self.max_ref_speed
        dv = self.max_ref_accel * dt
        v_low = max(v_low, v_prev - dv)
        v_high = min(v_high, v_prev + dv)
        reachable_low, reachable_high = v_low, v_high

        # The release maneuver is braking, not reversal. The safe reference is
        # allowed to slow to zero only in the original push direction. This
        # prevents a negative reference from commanding a retreat after the
        # obstacle/load has disappeared.
        if self.release_direction >= 0.0:
            v_low = max(v_low, 0.0)
        else:
            v_high = min(v_high, 0.0)
        if v_low > v_high:
            self.recovery_rate_infeasible = True
            v_low = v_high = float(np.clip(0.0, reachable_low, reachable_high))
        reachable_low, reachable_high = v_low, v_high

        if abs(g_e) <= self.g_epsilon_n:
            v_safe = float(np.clip(v_task, v_low, v_high))
            slack = max(0.0, -p_allow)
            return v_safe, bool(slack > 0.0), slack

        boundary = p_allow / g_e
        if g_e > 0.0:
            v_high = min(v_high, boundary)
        else:
            v_low = max(v_low, boundary)

        if v_low <= v_high:
            return float(np.clip(v_task, v_low, v_high)), False, 0.0

        # Infeasible because the barrier requests a faster reference-rate change
        # than V_I allows.  Return the reachable point with minimum violation.
        v_safe = float(reachable_low if g_e > 0.0 else reachable_high)
        slack = max(0.0, g_e * v_safe - p_allow)
        return v_safe, True, slack

    def _apply_power_upper_bound(
            self, v_low: float, v_high: float, g_e: float, rhs: float):
        """Intersect a velocity interval with g_e * v_d <= rhs."""
        if abs(g_e) < self.g_epsilon_n:
            return v_low, v_high, bool(rhs >= 0.0)
        boundary = rhs / g_e
        if g_e > 0.0:
            v_high = min(v_high, boundary)
        else:
            v_low = max(v_low, boundary)
        return v_low, v_high, bool(v_low <= v_high)

    def _jerk_limited_rate_target(
            self, v_target: float, v_prev: float, dt: float) -> float:
        """Return a stopping-aware velocity step under accel/jerk limits.

        Clipping a velocity target to the instantaneous jerk interval is not
        sufficient: with nonzero acceleration it drives through the target,
        then reverses jerk, creating a limit cycle.  The square-root braking
        curve starts reducing acceleration when its jerk stopping distance in
        velocity, a^2/(2J), reaches the remaining velocity error.
        """
        dt = max(float(dt), 1e-6)
        a_prev = float(np.clip(
            (v_prev - self.safe_v_prev2) / dt,
            -self.max_ref_accel, self.max_ref_accel))
        velocity_error = float(v_target - v_prev)
        if abs(velocity_error) <= 1e-9 and abs(a_prev) <= self.max_ref_jerk * dt:
            return float(np.clip(v_target, -self.max_ref_speed, self.max_ref_speed))

        direction = 1.0 if velocity_error >= 0.0 else -1.0
        stopping_accel = math.sqrt(
            max(0.0, 2.0 * self.max_ref_jerk * abs(velocity_error)))
        a_target = direction * min(self.max_ref_accel, stopping_accel)
        jerk_step = self.max_ref_jerk * dt
        a_next = float(np.clip(a_target, a_prev - jerk_step, a_prev + jerk_step))
        v_next = v_prev + a_next * dt
        return float(np.clip(v_next, -self.max_ref_speed, self.max_ref_speed))

    def _recovery_reference_velocity(
            self, v_prev: float, dt: float, g_e: float, rhs_energy: float,
            rhs_stop: float, v_i: float, e_ref: float,
            recoverable_energy: float):
        """Fast monotone recovery within speed/accel/jerk bounds.

        Energy and stopping-distance inequalities are hard priorities.  If the
        requested exponential recovery rate is temporarily unreachable, the
        returned point preserves the hard interval and reports the remaining
        dissipation slack instead of violating a safety barrier.

        Selecting a raw interval endpoint from sign(g_e) is bang-bang and made
        the delayed PX4 position loop re-accelerate after g_e changed sign.  A
        time-optimal reference braking curve instead retracts toward e_ref=0
        and reduces its own speed with the certified reference acceleration.
        """
        dt = max(float(dt), 1e-6)
        # Speed and acceleration are the primary reachable set. Intersect the
        # jerk set only when it is non-empty. This ordering is important at a
        # speed boundary: an inherited acceleration can make the jerk set
        # demand a velocity outside |v|<=v_max. The old empty-interval fallback
        # then returned that out-of-range endpoint and amplified oscillations.
        dv = self.max_ref_accel * dt
        base_low = max(-self.max_ref_speed, v_prev - dv)
        base_high = min(self.max_ref_speed, v_prev + dv)

        a_prev = float(np.clip(
            (v_prev - self.safe_v_prev2) / dt,
            -self.max_ref_accel, self.max_ref_accel))
        a_low = max(-self.max_ref_accel, a_prev - self.max_ref_jerk * dt)
        a_high = min(self.max_ref_accel, a_prev + self.max_ref_jerk * dt)
        jerk_low = v_prev + a_low * dt
        jerk_high = v_prev + a_high * dt
        v_low = max(base_low, jerk_low)
        v_high = min(base_high, jerk_high)
        self.recovery_rate_infeasible = bool(v_low > v_high)
        if self.recovery_rate_infeasible:
            # Never violate the speed/acceleration envelope to satisfy an
            # already-unreachable jerk continuation. Expose this event in the
            # diagnostics; a certified run must not contain it.
            v_low, v_high = base_low, base_high
        reachable_low, reachable_high = v_low, v_high

        hard_rhs = (rhs_energy, rhs_stop)
        hard_ok = True
        for rhs in hard_rhs:
            v_low, v_high, ok = self._apply_power_upper_bound(
                v_low, v_high, g_e, rhs)
            hard_ok = hard_ok and ok

        if not hard_ok or v_low > v_high:
            # Both hard constraints share g_e, so the dissipating reachable
            # endpoint minimizes every positive violation simultaneously.
            if g_e > self.g_epsilon_n:
                v_safe = reachable_low
            elif g_e < -self.g_epsilon_n:
                v_safe = reachable_high
            else:
                v_safe = float(np.clip(0.0, reachable_low, reachable_high))
            slack = max(0.0, *(g_e * v_safe - rhs for rhs in hard_rhs))
            return float(v_safe), True, float(slack), 0.0, 0.0

        if recoverable_energy <= self.stop_energy_tol:
            self.recovery_phase = 'settle'
            self.recovery_reference_velocity = 0.0
            v_rate_task = self._jerk_limited_rate_target(0.0, v_prev, dt)
            return float(np.clip(v_rate_task, v_low, v_high)), False, 0.0, 0.0, 0.0

        # A negative rate is permitted only while braking and only when the
        # safety/energy inequalities require it. Once actual motion reaches
        # the stop condition, filter_reference() latches HOLD and this
        # recovery rate can no longer pull the aircraft backward.
        self.recovery_phase = 'brake'
        v_task_recovery = 0.0
        self.recovery_reference_velocity = v_task_recovery
        v_rate_task = self._jerk_limited_rate_target(
            v_task_recovery, v_prev, dt)
        v_candidate = float(np.clip(v_rate_task, v_low, v_high))

        rho_feasible = (
            self.d_c * v_i * v_i
            - self.recovery_power_bound
            - g_e * v_candidate
        ) / max(recoverable_energy, 1e-6)
        # rho_min is the requested convergence rate, but it cannot be a hard
        # lower bound during the jerk-limited mode transition: a positive
        # incoming safe_v can make even rho=0 temporarily unreachable. Use the
        # maximum actually feasible non-negative rho and report any residual
        # CLF shortfall separately; the energy and distance CBFs remain hard.
        rho = float(np.clip(rho_feasible, 0.0, self.rho_max))
        rhs_dissipation = (
            self.d_c * v_i * v_i
            - self.recovery_power_bound
            - rho * recoverable_energy)

        diss_low, diss_high, diss_ok = self._apply_power_upper_bound(
            v_low, v_high, g_e, rhs_dissipation)
        if diss_ok and diss_low <= diss_high:
            v_safe = float(np.clip(v_rate_task, diss_low, diss_high))
            return float(v_safe), False, 0.0, rho, 0.0

        # A positive rho_min may be unreachable while jerk/acceleration are
        # ramping. Preserve the two hard barriers and expose the shortfall.
        v_safe = v_candidate
        slack = max(0.0, g_e * v_safe - rhs_dissipation)
        return float(v_safe), False, 0.0, rho, float(slack)

    def filter_reference(
            self, dt: float, measured_position_enu: np.ndarray,
            measured_velocity_enu: np.ndarray, nominal_position_enu: np.ndarray,
            nominal_velocity_enu: np.ndarray, nominal_acceleration_enu: np.ndarray,
            actuator_force_enu):
        nominal_position_enu = np.asarray(nominal_position_enu, dtype=float).reshape(3)
        nominal_velocity_enu = np.asarray(nominal_velocity_enu, dtype=float).reshape(3)
        nominal_acceleration_enu = np.asarray(nominal_acceleration_enu, dtype=float).reshape(3)

        if (not self.enabled or not self.valid_configuration or actuator_force_enu is None):
            s_nom = float(np.dot(self.axis, nominal_position_enu))
            v_nom = float(np.dot(self.axis, nominal_velocity_enu))
            self.safe_s = s_nom
            self.safe_v = v_nom
            self.debug.update({
                'enabled': False,
                'active': False,
                'barrier_active': False,
                'infeasible': False,
                'v_nom_i': v_nom,
                'v_task_i': v_nom,
                'v_safe_i': v_nom,
                's_nom_i': s_nom,
                's_safe_i': s_nom,
                'p_ref_nominal': 0.0,
                'p_ref_safe': 0.0,
                'equivalent_stiffness_force_n': 0.0,
                'qp_slack_w': 0.0,
            })
            return (nominal_position_enu.copy(), nominal_velocity_enu.copy(),
                    nominal_acceleration_enu.copy())

        measured_position_enu = np.asarray(measured_position_enu, dtype=float).reshape(3)
        measured_velocity_enu = np.asarray(measured_velocity_enu, dtype=float).reshape(3)
        actuator_force_enu = np.asarray(actuator_force_enu, dtype=float).reshape(3)

        s_nom = float(np.dot(self.axis, nominal_position_enu))
        v_nom = float(np.dot(self.axis, nominal_velocity_enu))
        a_nom = float(np.dot(self.axis, nominal_acceleration_enu))
        s_meas = float(np.dot(self.axis, measured_position_enu))
        v_i = float(np.dot(self.axis, measured_velocity_enu))

        if self.safe_s is None or self.safe_v is None:
            self.safe_s = s_nom
            self.safe_v = v_nom

        self._update_environment_storage(
            dt, measured_position_enu, measured_velocity_enu, actuator_force_enu)

        # Revised certificate: robot kinetic + controller virtual + environment.
        e_ref = float(self.safe_s - s_meas)
        kinetic_i = 0.5 * self.lambda_bar * v_i * v_i
        controller_storage = 0.5 * self.k_c * e_ref * e_ref
        energy_i = kinetic_i + controller_storage + self.storage_bound
        h_i = self.e_max - energy_i
        h_constraint = h_i - self.energy_reserve_j

        # Known feedforward term in the nominal stiffness-damping controller.
        u_ff_i = self.ff_mass * a_nom if self.accel_ff_mode == 'nominal' else 0.0

        # dot(E_I) <= g_E*v_d - D_c*v_I^2 + pi_E.
        g_e = self.k_c * e_ref + self.d_c * v_i
        pi_e = v_i * u_ff_i + self.residual_power_bound_w + self.delta_e
        # Tighten the certified set by an explicit numerical reserve while
        # retaining E_max and the reported physical barrier h_i unchanged.
        p_allow = self.d_c * v_i * v_i - pi_e + self.gamma * h_constraint
        self.g_e_last = g_e

        recoverable_energy = kinetic_i + controller_storage
        release_excursion = 0.0
        if self.release_s is not None:
            release_excursion = max(
                self.release_direction * (s_meas - self.release_s), 0.0)
        reserved_stop_distance = energy_i / self.brake_force_cert
        stop_distance_barrier = (
            self.stop_distance - release_excursion - reserved_stop_distance)

        if self.mode == self.MODE_RECOVERY:
            self.recovery_elapsed_s += max(dt, 0.0)
            if self.release_direction * v_i >= self.recovery_stop_arm_vel:
                self.recovery_motion_seen = True

        v_prev = float(self.safe_v)
        rho = 0.0
        recovery_dissipation_slack_w = 0.0
        if self.mode == self.MODE_INTERACTION:
            # REF_SYNC_GAIN=0 reproduces the paper definition
            # nu_I,t=nominal trajectory velocity exactly.
            v_task = v_nom + self.reference_sync_gain * (s_nom - self.safe_s)
            v_task = float(np.clip(v_task, -self.max_ref_speed, self.max_ref_speed))
            v_safe, infeasible, qp_slack = self._project_reference_velocity(
                v_task, v_prev, dt, g_e, p_allow)
        elif self.mode == self.MODE_RECOVERY:
            # The task is zero terminal speed.  The optimizer selects the
            # reachable endpoint that dissipates E_R fastest while satisfying
            # both accumulation and stopping-distance barriers.
            v_task = 0.0
            rhs_stop = (
                self.d_c * v_i * v_i
                - pi_e
                + self.brake_force_cert * (
                    self.stop_gamma * stop_distance_barrier
                    - self.release_direction * v_i))
            (v_safe, infeasible, qp_slack, rho,
             recovery_dissipation_slack_w) = self._recovery_reference_velocity(
                 v_prev, dt, g_e, p_allow, rhs_stop, v_i, e_ref,
                 recoverable_energy)
        else:
            v_task = 0.0
            v_safe = 0.0
            infeasible = False
            qp_slack = 0.0
            self.recovery_phase = 'hold'
            self.recovery_reference_velocity = 0.0

        self.barrier_active = bool(abs(v_safe - v_task) > 1e-6 or qp_slack > 1e-9)
        self.infeasible = bool(infeasible)
        if self.mode == self.MODE_RECOVERY and self.recovery_rate_infeasible:
            self.infeasible = True

        # Safe desired position is the integral of the filtered desired rate.
        if self.mode != self.MODE_HOLD:
            self.safe_s += 0.5 * (v_prev + v_safe) * dt
        self.safe_v_prev2 = v_prev
        self.safe_v = v_safe
        a_safe_i = (v_safe - v_prev) / max(dt, 1e-6)

        if self.mode == self.MODE_RECOVERY:
            stop_confirmation_armed = (
                self.recovery_motion_seen
                or self.recovery_elapsed_s >= self.recovery_stop_min_time)
            # A short zero crossing is not a physical stop. Keep the recovery
            # target at the contact/release point until low speed persists for
            # stop_hold_s; only then rebase once to the measured stop point.
            stop_conditions_met = (
                stop_confirmation_armed
                and abs(v_i) <= self.stop_vel_tol)
            if stop_conditions_met:
                self.recovery_stop_candidate_s += max(dt, 0.0)
            else:
                self.recovery_stop_candidate_s = 0.0
            self.recovery_done_duration_s = self.recovery_stop_candidate_s
            if self.recovery_stop_candidate_s >= self.stop_hold_s:
                # Physical stop is the terminal condition. Do not command a
                # return to release_s or to any accumulated virtual reference.
                # Transfer the remaining virtual-spring energy into the frozen
                # conservative storage account before rebasing safe_s, so the
                # certificate does not gain energy by a coordinate reset.
                residual_e_ref = float(self.safe_s - s_meas)
                terminal_rebase_energy = (
                    0.5 * self.k_c * residual_e_ref * residual_e_ref)
                self.storage_bound += terminal_rebase_energy
                self.recovery_rebase_energy_j += terminal_rebase_energy
                self.mode = self.MODE_HOLD
                self.recovery_stop_latched = True
                self.recovery_terminal_s = s_meas
                self.safe_s = s_meas
                self.safe_v = 0.0
                self.safe_v_prev2 = 0.0
                v_safe = 0.0
                a_safe_i = 0.0
                self.recovery_phase = 'hold'
                self.recovery_reference_velocity = 0.0
                e_ref = 0.0
                controller_storage = 0.0
                recoverable_energy = kinetic_i
                energy_i = kinetic_i + self.storage_bound
                h_i = self.e_max - energy_i
                h_constraint = h_i - self.energy_reserve_j
                g_e = self.d_c * v_i
                p_allow = self.d_c * v_i * v_i - pi_e + self.gamma * h_constraint
                reserved_stop_distance = energy_i / self.brake_force_cert
                stop_distance_barrier = (
                    self.stop_distance - release_excursion
                    - reserved_stop_distance)

        self.rho = rho
        self.recovery_dissipation_slack_w = recovery_dissipation_slack_w
        self.recoverable_energy = recoverable_energy
        self.release_excursion = release_excursion
        self.stop_distance_barrier = stop_distance_barrier
        self.reserved_stop_distance = reserved_stop_distance
        reference_power_nominal = g_e * v_task
        reference_power_safe = g_e * v_safe
        equivalent_stiffness_force_n = self.k_c * e_ref

        safe_position = nominal_position_enu + self.axis * (self.safe_s - s_nom)
        safe_velocity = nominal_velocity_enu + self.axis * (v_safe - v_nom)
        if self.accel_ff_mode == 'zero':
            safe_acceleration = nominal_acceleration_enu - self.axis * a_nom
        else:
            safe_acceleration = nominal_acceleration_enu.copy()

        self.debug.update({
            'enabled': True, 'active': True,
            'barrier_active': self.barrier_active, 'infeasible': self.infeasible,
            'p_hat': self.power_hat, 'p_bar_e': self.power_error_bound,
            's_dot_bar': self.storage_rate, 's_bar': self.storage_bound,
            'storage_update_enabled': self.storage_update_enabled,
            'k_i': kinetic_i, 'v_c': controller_storage, 'e_ref': e_ref,
            'e_i': energy_i, 'h_i': h_i, 'h_constraint': h_constraint,
            'energy_reserve_j': self.energy_reserve_j, 'v_i': v_i,
            'v_nom_i': v_nom, 'v_task_i': v_task, 'v_safe_i': v_safe,
            's_nom_i': s_nom, 's_safe_i': self.safe_s,
            'g_e': g_e, 'pi_e': pi_e, 'p_allow': p_allow,
            'p_ref_nominal': reference_power_nominal,
            'p_ref_safe': reference_power_safe,
            'equivalent_stiffness_force_n': equivalent_stiffness_force_n,
            'qp_slack_w': qp_slack, 'a_safe_i': a_safe_i,
            'mode': self.mode,
            'recoverable_energy': recoverable_energy,
            'release_excursion': release_excursion,
            'stop_distance_barrier': stop_distance_barrier,
            'reserved_stop_distance': reserved_stop_distance,
            'rho': rho,
            'release_s': self.release_s if self.release_s is not None else math.nan,
            'recovery_done_duration_s': self.recovery_done_duration_s,
            'recovery_dissipation_slack_w': recovery_dissipation_slack_w,
            'recovery_phase': self.recovery_phase,
            'recovery_reference_velocity': self.recovery_reference_velocity,
            'recovery_motion_seen': self.recovery_motion_seen,
            'recovery_elapsed_s': self.recovery_elapsed_s,
            'recovery_rate_infeasible': self.recovery_rate_infeasible,
            'recovery_terminal_s': (
                self.recovery_terminal_s
                if self.recovery_terminal_s is not None else math.nan),
            'recovery_stop_candidate_s': self.recovery_stop_candidate_s,
            'recovery_stop_latched': self.recovery_stop_latched,
            'recovery_rebase_energy_j': self.recovery_rebase_energy_j,
        })

        # Rate-limit warnings to avoid flooding ROS logs during an infeasible test.
        now_s = time.monotonic()
        if infeasible and self.logger is not None and now_s - self._last_infeasible_warn_s > 1.0:
            self._last_infeasible_warn_s = now_s
            self.logger.warn(
                f'IEBC reference-rate QP infeasible: slack={qp_slack:.3f} W, '
                f'E={energy_i:.3f}/{self.e_max:.3f} J. 严格安全保证暂时失效。')

        return safe_position, safe_velocity, safe_acceleration


class HnuterActuatorForceEstimator:
    """Reconstruct propulsive force from PX4 post-allocation commands.

    This mirrors the hardware branch of ``ActuatorEffectivenessHnuter``.  The
    result is deliberately called an estimate: PX4 actuator outputs are
    commands and the aircraft currently has no rotor-thrust/RPM or servo-angle
    feedback.  The estimate is expressed in the controller's body-FLU frame;
    the node rotates it into world ENU using the measured attitude.
    """

    MOTOR_COUNT = 5
    SERVO_COUNT = 4

    def __init__(
            self,
            mass_kg: float = 4.5,
            gravity_mps2: float = 9.81,
            hover_control: float = 0.40,
            thrust_exponent: float = 0.50,
            max_arm_thrust_n: float = 170.96,
            max_tail_thrust_n: float = 85.48,
            primary_servo_max_rad: float = math.pi,
            secondary_servo_max_rad: float = math.pi,
            secondary_gear_ratio: float = 2.0,
    ):
        self.mass_kg = max(float(mass_kg), 0.1)
        self.gravity_mps2 = max(float(gravity_mps2), 1.0)
        self.hover_control = float(np.clip(hover_control, 0.05, 0.95))
        self.thrust_exponent = float(np.clip(thrust_exponent, 0.2, 1.5))
        self.max_arm_thrust_n = max(float(max_arm_thrust_n), 1.0)
        self.max_tail_thrust_n = max(float(max_tail_thrust_n), 1.0)
        self.primary_servo_max_rad = max(abs(float(primary_servo_max_rad)), 1e-3)
        self.secondary_servo_max_rad = max(abs(float(secondary_servo_max_rad)), 1e-3)
        self.secondary_gear_ratio = max(abs(float(secondary_gear_ratio)), 1e-3)

    @classmethod
    def from_environment(cls):
        return cls(
            mass_kg=env_float('HNUTER_IEBC_ACT_MASS_KG', 4.5),
            gravity_mps2=env_float('HNUTER_IEBC_ACT_GRAVITY_MPS2', 9.81),
            hover_control=env_float('HNUTER_IEBC_ACT_MOT_HOV', 0.40),
            thrust_exponent=env_float('HNUTER_IEBC_ACT_MOT_EXPO', 0.50),
            max_arm_thrust_n=env_float('HNUTER_IEBC_ACT_MAX_ARM_T_N', 170.96),
            max_tail_thrust_n=env_float('HNUTER_IEBC_ACT_MAX_TAIL_T_N', 85.48),
            primary_servo_max_rad=math.radians(
                env_float('HNUTER_IEBC_ACT_S1_MAX_DEG', 180.0)),
            secondary_servo_max_rad=math.radians(
                env_float('HNUTER_IEBC_ACT_S2_SERVO_MAX_DEG', 180.0)),
            secondary_gear_ratio=env_float('HNUTER_IEBC_ACT_S2_GEAR', 2.0),
        )

    @staticmethod
    def _required_controls(values, count: int, field: str) -> np.ndarray:
        controls = np.asarray(values, dtype=float).reshape(-1)
        if controls.size < count:
            raise ValueError(f'{field} must contain at least {count} controls')
        controls = controls[:count]
        if not np.all(np.isfinite(controls)):
            raise ValueError(f'{field} required controls must be finite')
        return controls

    def _front_motor_force_n(self, control: float) -> float:
        control = float(np.clip(control, 0.0, 1.0))
        hover_force_per_motor = self.mass_kg * self.gravity_mps2 * 0.25
        force = hover_force_per_motor * (
            control / self.hover_control) ** (1.0 / self.thrust_exponent)
        return float(np.clip(force, 0.0, 0.5 * self.max_arm_thrust_n))

    def _tail_force_n(self, control: float) -> float:
        control = float(np.clip(control, -1.0, 1.0))
        return float(
            math.copysign(
                self.max_tail_thrust_n
                * abs(control) ** (1.0 / self.thrust_exponent),
                control,
            ) if control != 0.0 else 0.0)

    @staticmethod
    def _arm_direction(alpha_rad: float, theta_rad: float) -> np.ndarray:
        return np.array([
            math.cos(theta_rad) * math.sin(alpha_rad),
            -math.sin(theta_rad),
            math.cos(theta_rad) * math.cos(alpha_rad),
        ], dtype=float)

    def estimate_body_force_flu(self, motor_controls, servo_controls) -> np.ndarray:
        motors = self._required_controls(
            motor_controls, self.MOTOR_COUNT, 'actuator_motors.control')
        servos = self._required_controls(
            servo_controls, self.SERVO_COUNT, 'actuator_servos.control')

        # Firmware channel order:
        # M0/M1 right arm, M2/M3 left arm, M4 signed tail motor;
        # S0 alpha2 right, S1 alpha1 left, S2 theta2 right shaft,
        # S3 theta1 left shaft.  Secondary physical angle is after gear ratio.
        right_thrust_n = (
            self._front_motor_force_n(motors[0])
            + self._front_motor_force_n(motors[1]))
        left_thrust_n = (
            self._front_motor_force_n(motors[2])
            + self._front_motor_force_n(motors[3]))
        tail_thrust_n = self._tail_force_n(motors[4])

        alpha2 = float(np.clip(servos[0], -1.0, 1.0)) * self.primary_servo_max_rad
        alpha1 = float(np.clip(servos[1], -1.0, 1.0)) * self.primary_servo_max_rad
        theta2 = (
            float(np.clip(servos[2], -1.0, 1.0))
            * self.secondary_servo_max_rad / self.secondary_gear_ratio)
        theta1 = (
            float(np.clip(servos[3], -1.0, 1.0))
            * self.secondary_servo_max_rad / self.secondary_gear_ratio)

        return (
            left_thrust_n * self._arm_direction(alpha1, theta1)
            + right_thrust_n * self._arm_direction(alpha2, theta2)
            + np.array([0.0, 0.0, tail_thrust_n], dtype=float))


@dataclass
class NominalReference:
    """Coherent absolute-ENU reference decoded from one PX4 message."""

    position_enu: np.ndarray
    velocity_enu: np.ndarray
    acceleration_enu: np.ndarray
    yaw_enu: float
    received_monotonic_s: float


class Px4TrajectoryCodec:
    """Pure PX4 NED <-> controller ENU conversions used by node and tests."""

    @staticmethod
    def _vec3(value, field: str, allow_all_nan: bool = False) -> np.ndarray:
        vector = np.asarray(value, dtype=float).reshape(-1)
        if vector.size < 3:
            raise ValueError(f'{field} must contain three elements')
        vector = vector[:3]
        if allow_all_nan and np.all(np.isnan(vector)):
            return np.zeros(3, dtype=float)
        if not np.all(np.isfinite(vector)):
            raise ValueError(f'{field} must be finite (or all NaN for optional acceleration)')
        return vector

    @staticmethod
    def ned_vector_to_enu(vector_ned: np.ndarray) -> np.ndarray:
        vector_ned = np.asarray(vector_ned, dtype=float).reshape(3)
        return np.array(
            [vector_ned[1], vector_ned[0], -vector_ned[2]], dtype=float)

    @staticmethod
    def yaw_ned_to_enu(yaw_ned: float) -> float:
        yaw_enu = 0.5 * math.pi - float(yaw_ned)
        return float(math.atan2(math.sin(yaw_enu), math.cos(yaw_enu)))

    @classmethod
    def decode(cls, message, received_monotonic_s: Optional[float] = None) -> NominalReference:
        position_ned = cls._vec3(message.position, 'position')
        velocity_ned = cls._vec3(message.velocity, 'velocity')
        acceleration_ned = cls._vec3(
            message.acceleration, 'acceleration', allow_all_nan=True)
        yaw_ned = float(message.yaw)
        if not math.isfinite(yaw_ned):
            raise ValueError('yaw must be finite')
        return NominalReference(
            position_enu=cls.ned_vector_to_enu(position_ned),
            velocity_enu=cls.ned_vector_to_enu(velocity_ned),
            acceleration_enu=cls.ned_vector_to_enu(acceleration_ned),
            yaw_enu=cls.yaw_ned_to_enu(yaw_ned),
            received_monotonic_s=(
                time.monotonic()
                if received_monotonic_s is None else float(received_monotonic_s)),
        )


class HnuterIebcOffboardController(ValidatedHardwareController):
    """IEBC reference filter integrated into the validated hardware controller."""

    DEFAULT_NOMINAL_TOPIC = '/hnuter/iebc/in/trajectory_setpoint'
    DEFAULT_MOTORS_TOPIC = '/fmu/out/actuator_motors'
    DEFAULT_SERVOS_TOPIC = '/fmu/out/actuator_servos'
    DEFAULT_WRENCH_TOPIC = '/hnuter/iebc/in/actuator_wrench'
    DEFAULT_RECOVERY_TOPIC = '/hnuter/iebc/in/recovery'
    DEFAULT_RESET_TOPIC = '/hnuter/iebc/in/reset'
    DEFAULT_STATUS_TOPIC = '/hnuter/iebc/out/status'

    TASK_MANUAL = 'manual'
    TASK_PUSH = 'push'
    TASK_RETURN = 'return'

    def __init__(self):
        # Real hardware must never silently fall back to the Gazebo proxy.
        os.environ.setdefault('HNUTER_IEBC_WRENCH_SOURCE', 'external')
        super().__init__()

        self.nominal_source = os.environ.get(
            'HNUTER_IEBC_NOMINAL_SOURCE', 'rc_task').strip().lower()
        if self.nominal_source not in ('topic', 'baseline', 'rc_task'):
            raise ValueError(
                'HNUTER_IEBC_NOMINAL_SOURCE must be topic, baseline or rc_task')

        self.actuator_source = os.environ.get(
            'HNUTER_IEBC_ACTUATOR_SOURCE', 'px4_outputs').strip().lower()
        if self.actuator_source not in ('px4_outputs', 'external_wrench'):
            raise ValueError(
                'HNUTER_IEBC_ACTUATOR_SOURCE must be px4_outputs or external_wrench')

        self.command_timeout_s = max(
            env_float('HNUTER_IEBC_COMMAND_TIMEOUT_S', 0.30), 0.05)
        self.initial_command_radius_m = max(
            env_float('HNUTER_IEBC_INITIAL_COMMAND_RADIUS_M', 0.75), 0.0)
        self.require_explicit_enu_frame = env_bool(
            'HNUTER_IEBC_REQUIRE_WRENCH_FRAME', True)
        self.nominal_topic = os.environ.get(
            'HNUTER_IEBC_NOMINAL_TOPIC', self.DEFAULT_NOMINAL_TOPIC)
        self.motors_topic = os.environ.get(
            'HNUTER_IEBC_MOTORS_TOPIC', self.DEFAULT_MOTORS_TOPIC)
        self.servos_topic = os.environ.get(
            'HNUTER_IEBC_SERVOS_TOPIC', self.DEFAULT_SERVOS_TOPIC)
        self.wrench_topic = os.environ.get(
            'HNUTER_IEBC_WRENCH_TOPIC', self.DEFAULT_WRENCH_TOPIC)
        self.recovery_topic = os.environ.get(
            'HNUTER_IEBC_RECOVERY_TOPIC', self.DEFAULT_RECOVERY_TOPIC)
        self.reset_topic = os.environ.get(
            'HNUTER_IEBC_RESET_TOPIC', self.DEFAULT_RESET_TOPIC)
        self.status_topic = os.environ.get(
            'HNUTER_IEBC_STATUS_TOPIC', self.DEFAULT_STATUS_TOPIC)

        self.iebc = InteractionEnergyBarrierFilter(logger=self.get_logger())
        if self.iebc.enabled and not self.iebc.valid_configuration:
            raise ValueError(
                'IEBC enabled with invalid MASS/LAMBDA_BAR/E_MAX/KC configuration')
        if self.iebc.enabled and self.iebc.wrench_source != 'external':
            raise ValueError(
                'Hardware IEBC requires HNUTER_IEBC_WRENCH_SOURCE=external')

        self._nominal_reference: Optional[NominalReference] = None
        self.actuator_force_estimator = HnuterActuatorForceEstimator.from_environment()
        self._motor_controls = np.full(
            HnuterActuatorForceEstimator.MOTOR_COUNT, np.nan, dtype=float)
        self._servo_controls = np.full(
            HnuterActuatorForceEstimator.SERVO_COUNT, np.nan, dtype=float)
        self._motor_received_s = -math.inf
        self._servo_received_s = -math.inf
        self._external_force_enu = np.zeros(3, dtype=float)
        self._external_force_received_s = -math.inf
        self._topic_reference_active = False
        self._failsafe_reason = 'waiting_for_hardware_gate'
        self._failsafe_hold_latched = False
        self._failsafe_hold_position = np.zeros(3, dtype=float)
        self._failsafe_hold_yaw_enu = 0.0
        self._failsafe_hold_attitude_enu = np.zeros(3, dtype=float)
        self._recovery_input_high = False
        self._rejected_commands = 0
        self._rejected_actuator_outputs = 0
        self._last_actuator_output_warn_s = -math.inf
        self._rejected_wrenches = 0

        # RC-triggered hardware task.  AUX3 is deliberately separate from the
        # AUX1/AUX2 attitude channels used by the validated manual controller.
        self.task_rc_function = int(env_float(
            'HNUTER_IEBC_TASK_RC_FUNCTION', RcChannels.FUNCTION_AUX_3))
        self.task_switch_high_threshold = env_float(
            'HNUTER_IEBC_TASK_SWITCH_HIGH', 0.50)
        self.task_switch_low_threshold = env_float(
            'HNUTER_IEBC_TASK_SWITCH_LOW', 0.00)
        if self.task_switch_low_threshold >= self.task_switch_high_threshold:
            raise ValueError('task switch LOW threshold must be below HIGH threshold')
        self.task_switch_timeout_s = max(env_float(
            'HNUTER_IEBC_TASK_SWITCH_TIMEOUT_S', 0.50), 0.05)
        self.task_push_speed_mps = max(env_float(
            'HNUTER_IEBC_TASK_PUSH_SPEED_MPS', 0.05), 0.005)
        self.task_push_accel_mps2 = max(env_float(
            'HNUTER_IEBC_TASK_PUSH_ACCEL_MPS2', 0.15), 0.01)
        self.task_max_push_distance_m = max(env_float(
            'HNUTER_IEBC_TASK_MAX_PUSH_M', 3.0), 0.05)
        self.task_return_speed_mps = max(env_float(
            'HNUTER_IEBC_TASK_RETURN_SPEED_MPS', 0.25), 0.02)
        self.task_return_accel_mps2 = max(env_float(
            'HNUTER_IEBC_TASK_RETURN_ACCEL_MPS2', 0.35), 0.02)
        self.task_return_kp = max(env_float(
            'HNUTER_IEBC_TASK_RETURN_KP', 0.8), 0.05)
        self.task_return_position_tolerance_m = max(env_float(
            'HNUTER_IEBC_TASK_RETURN_POS_TOL_M', 0.12), 0.02)
        self.task_return_velocity_tolerance_mps = max(env_float(
            'HNUTER_IEBC_TASK_RETURN_VEL_TOL_MPS', 0.08), 0.01)
        self.task_return_hold_s = max(env_float(
            'HNUTER_IEBC_TASK_RETURN_HOLD_S', 0.75), 0.0)

        self.task_state = self.TASK_MANUAL
        self._task_switch_value = math.nan
        self._task_switch_received_s = -math.inf
        self._task_switch_high = False
        self._task_switch_armed = False
        self._task_start_position_abs_enu: Optional[np.ndarray] = None
        self._task_start_yaw_enu = 0.0
        self._task_start_roll_enu = 0.0
        self._task_start_pitch_enu = 0.0
        self._task_axis_enu = np.array([1.0, 0.0, 0.0], dtype=float)
        self._task_reference_distance_m = 0.0
        self._task_reference_speed_mps = 0.0
        self._task_return_settle_s = 0.0
        self._task_transition_reason = 'startup'

        live_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.nominal_reference_sub = self.create_subscription(
            TrajectorySetpoint, self.nominal_topic,
            self.nominal_reference_callback, live_qos)
        self.actuator_motors_sub = self.create_subscription(
            ActuatorMotors, self.motors_topic,
            self.actuator_motors_callback, sensor_qos)
        self.actuator_servos_sub = self.create_subscription(
            ActuatorServos, self.servos_topic,
            self.actuator_servos_callback, sensor_qos)
        self.wrench_sub = self.create_subscription(
            WrenchStamped, self.wrench_topic,
            self.actuator_wrench_callback, sensor_qos)
        self.recovery_sub = self.create_subscription(
            Bool, self.recovery_topic, self.recovery_callback, live_qos)
        self.reset_sub = self.create_subscription(
            Empty, self.reset_topic, self.reset_callback, live_qos)
        self.iebc_status_pub = self.create_publisher(
            String, self.status_topic, live_qos)
        self.iebc_status_timer = self.create_timer(0.2, self.publish_iebc_status)

        # Topic mode is controlled only by the upstream nominal-reference node.
        # Baseline mode retains the proven RC/keyboard reference generator and
        # inserts IEBC immediately before the inherited PX4 publisher.
        if self.nominal_source == 'topic':
            self.manual_enabled = False

        self.get_logger().info(
            'Reusable hardware IEBC Offboard gateway initialized: '
            f'nominal_source={self.nominal_source}, nominal={self.nominal_topic}, '
            f'actuator_source={self.actuator_source}, motors={self.motors_topic}, '
            f'servos={self.servos_topic}, external_wrench={self.wrench_topic}, '
            f'recovery={self.recovery_topic}, '
            f'task_rc_function={self.task_rc_function}, '
            f'IEBC enabled={self.iebc.enabled}. Arm/Offboard remain transmitter-owned.')

    @staticmethod
    def _wrench_frame_is_enu(frame_id: str) -> bool:
        normalized = str(frame_id).strip().lower().lstrip('/')
        return normalized in ('enu', 'map', 'world', 'world_enu')

    def nominal_reference_callback(self, message: TrajectorySetpoint) -> None:
        try:
            self._nominal_reference = Px4TrajectoryCodec.decode(message)
        except (TypeError, ValueError) as exc:
            self._rejected_commands += 1
            self.get_logger().warn(f'Rejected nominal TrajectorySetpoint: {exc}')

    def actuator_wrench_callback(self, message: WrenchStamped) -> None:
        frame_id = getattr(getattr(message, 'header', None), 'frame_id', '')
        if (self.require_explicit_enu_frame
                and not self._wrench_frame_is_enu(frame_id)):
            self._rejected_wrenches += 1
            self.get_logger().warn(
                f'Rejected actuator wrench frame={frame_id!r}; expected ENU map/world frame')
            return
        force = message.wrench.force
        value = np.array([force.x, force.y, force.z], dtype=float)
        if not np.all(np.isfinite(value)):
            self._rejected_wrenches += 1
            self.get_logger().warn('Rejected non-finite actuator wrench')
            return
        self._external_force_enu = value
        self._external_force_received_s = time.monotonic()

    def actuator_motors_callback(self, message: ActuatorMotors) -> None:
        try:
            controls = HnuterActuatorForceEstimator._required_controls(
                message.control, HnuterActuatorForceEstimator.MOTOR_COUNT,
                'actuator_motors.control')
        except (TypeError, ValueError) as exc:
            self._reject_actuator_output(f'motor output: {exc}')
            return
        self._motor_controls = controls
        self._motor_received_s = time.monotonic()

    def actuator_servos_callback(self, message: ActuatorServos) -> None:
        try:
            controls = HnuterActuatorForceEstimator._required_controls(
                message.control, HnuterActuatorForceEstimator.SERVO_COUNT,
                'actuator_servos.control')
        except (TypeError, ValueError) as exc:
            self._reject_actuator_output(f'servo output: {exc}')
            return
        self._servo_controls = controls
        self._servo_received_s = time.monotonic()

    def _reject_actuator_output(self, detail: str) -> None:
        self._rejected_actuator_outputs += 1
        now_s = time.monotonic()
        if now_s - self._last_actuator_output_warn_s >= 1.0:
            self._last_actuator_output_warn_s = now_s
            self.get_logger().warn(f'Rejected PX4 actuator {detail}')

    def rc_channels_callback(self, message: RcChannels) -> None:
        """Keep validated manual RC handling and sample the task switch."""
        super().rc_channels_callback(message)
        value = self.rc_input._mapped_channel(message, self.task_rc_function)
        valid = bool(not getattr(message, 'signal_lost', True) and value is not None)
        if not valid:
            return
        self._update_task_switch_sample(float(value), time.monotonic())

    def _update_task_switch_sample(self, value: float, received_s: float) -> None:
        self._task_switch_value = float(value)
        self._task_switch_received_s = float(received_s)
        if self._task_switch_value >= self.task_switch_high_threshold:
            self._task_switch_high = True
        elif self._task_switch_value <= self.task_switch_low_threshold:
            self._task_switch_high = False

    def recovery_callback(self, message: Bool) -> None:
        high = bool(message.data)
        if high and not self._recovery_input_high:
            if (self.nominal_source == 'rc_task'
                    and self.task_state != self.TASK_MANUAL):
                self._begin_task_return('external_recovery_topic')
                self._recovery_input_high = high
                return
            if (not self._hardware_control_active
                    or not self.iebc.enabled
                    or self.iebc.safe_s is None):
                self.get_logger().warn(
                    'Ignored IEBC recovery trigger before an active safe reference exists')
                self._recovery_input_high = high
                return
            measured_s = float(np.dot(self.iebc.axis, self.position))
            self.iebc.enter_recovery(measured_s)
            self.get_logger().warn(
                f'IEBC recovery triggered at interaction coordinate {measured_s:.3f} m')
        self._recovery_input_high = high

    def reset_callback(self, _message: Empty) -> None:
        if (self.nominal_source == 'rc_task'
                and self.task_state != self.TASK_MANUAL):
            self.get_logger().warn(
                'Ignored IEBC reset while RC push/return task is active')
            return
        self.iebc.reset()
        self._topic_reference_active = False
        self._failsafe_hold_latched = False
        self._failsafe_reason = 'operator_reset'
        self.get_logger().info('IEBC state reset by topic request')

    def _begin_hardware_control(self):
        super()._begin_hardware_control()
        self.iebc.reset()
        self._topic_reference_active = False
        self._failsafe_hold_latched = False
        self._failsafe_reason = (
            'manual_rc_ready_task_switch_low_required'
            if self.nominal_source == 'rc_task'
            else 'waiting_for_nominal_reference')
        self._reset_rc_task('offboard_entered')

    def _end_hardware_control(self):
        super()._end_hardware_control()
        self.iebc.reset()
        self._topic_reference_active = False
        self._failsafe_hold_latched = False
        self._failsafe_reason = 'hardware_gate_inactive'
        self._reset_rc_task('hardware_gate_inactive')

    def _reference_age_s(self, now_s: Optional[float] = None) -> float:
        if self._nominal_reference is None:
            return math.inf
        now_s = time.monotonic() if now_s is None else float(now_s)
        return now_s - self._nominal_reference.received_monotonic_s

    def _external_wrench_age_s(self, now_s: Optional[float] = None) -> float:
        now_s = time.monotonic() if now_s is None else float(now_s)
        return now_s - self._external_force_received_s

    def _motor_age_s(self, now_s: Optional[float] = None) -> float:
        now_s = time.monotonic() if now_s is None else float(now_s)
        return now_s - self._motor_received_s

    def _servo_age_s(self, now_s: Optional[float] = None) -> float:
        now_s = time.monotonic() if now_s is None else float(now_s)
        return now_s - self._servo_received_s

    def _wrench_age_s(self, now_s: Optional[float] = None) -> float:
        """Age of the selected actuator-force source (legacy status name)."""
        now_s = time.monotonic() if now_s is None else float(now_s)
        if self.actuator_source == 'external_wrench':
            return self._external_wrench_age_s(now_s)
        return max(self._motor_age_s(now_s), self._servo_age_s(now_s))

    def _task_switch_age_s(self, now_s: Optional[float] = None) -> float:
        now_s = time.monotonic() if now_s is None else float(now_s)
        return now_s - self._task_switch_received_s

    def _task_switch_is_fresh(self) -> bool:
        return self._task_switch_age_s() <= self.task_switch_timeout_s

    def _reset_rc_task(self, reason: str) -> None:
        self.task_state = self.TASK_MANUAL
        self._task_switch_armed = False
        self._task_start_position_abs_enu = None
        self._task_reference_distance_m = 0.0
        self._task_reference_speed_mps = 0.0
        self._task_return_settle_s = 0.0
        self._task_transition_reason = reason

    def _current_yaw_enu(self) -> float:
        return float(math.atan2(self.R[1, 0], self.R[0, 0]))

    def _start_rc_push_task(self) -> bool:
        if not self.iebc.enabled or not self.iebc.valid_configuration:
            self._task_switch_armed = False
            self._failsafe_reason = 'task_start_rejected_iebc_disabled_or_invalid'
            self.get_logger().warn(
                'Task switch ignored: valid hardware IEBC configuration is required')
            return False
        if self._fresh_external_force() is None:
            self._task_switch_armed = False
            self._failsafe_reason = 'task_start_rejected_actuator_force_stale'
            self.get_logger().warn(
                'Task switch ignored: selected actuator-force input is missing or stale')
            return False

        self._task_start_position_abs_enu = self.position.copy()
        self._task_start_yaw_enu = self._current_yaw_enu()
        self._task_start_roll_enu = float(self.target_attitude[0])
        self._task_start_pitch_enu = float(self.target_attitude[1])
        self._task_axis_enu = np.array([
            math.cos(self._task_start_yaw_enu),
            math.sin(self._task_start_yaw_enu),
            0.0,
        ], dtype=float)
        self.iebc.axis = self._task_axis_enu.copy()
        self.iebc.reset()
        self.task_state = self.TASK_PUSH
        self._task_reference_distance_m = 0.0
        self._task_reference_speed_mps = 0.0
        self._task_return_settle_s = 0.0
        self._task_switch_armed = False
        self._task_transition_reason = 'task_switch_rising_edge'
        self._failsafe_hold_latched = False
        self._failsafe_reason = ''
        self.get_logger().warn(
            'IEBC push task started: current position and heading latched; '
            f'axis=[{self._task_axis_enu[0]:+.3f}, '
            f'{self._task_axis_enu[1]:+.3f}, 0.000]')
        return True

    def _begin_task_return(self, reason: str) -> None:
        if self.task_state == self.TASK_RETURN:
            return
        self.task_state = self.TASK_RETURN
        self._task_return_settle_s = 0.0
        self._task_switch_armed = False
        self._task_transition_reason = reason
        self.get_logger().warn(
            f'IEBC push cancelled; decelerating and returning to task start: {reason}')

    @staticmethod
    def _slew_scalar(current: float, target: float, max_delta: float) -> float:
        return float(current + np.clip(target - current, -max_delta, max_delta))

    def _set_rc_task_reference(self, dt: float, target_speed_mps: float,
                               accel_limit_mps2: float) -> None:
        previous_speed = self._task_reference_speed_mps
        self._task_reference_speed_mps = self._slew_scalar(
            previous_speed, target_speed_mps, accel_limit_mps2 * dt)
        acceleration = (
            (self._task_reference_speed_mps - previous_speed) / max(dt, 1e-6))
        next_distance = self._task_reference_distance_m + 0.5 * (
            previous_speed + self._task_reference_speed_mps) * dt

        if self.task_state == self.TASK_PUSH:
            self._task_reference_distance_m = float(np.clip(
                next_distance, 0.0, self.task_max_push_distance_m))
        else:
            # The virtual reference never retreats behind the latched start.
            self._task_reference_distance_m = max(float(next_distance), 0.0)

        start = self._task_start_position_abs_enu
        nominal_abs = start + self._task_axis_enu * self._task_reference_distance_m
        self.target_position = nominal_abs.copy()
        self.target_position[2] -= self._z0
        self.target_velocity = self._task_axis_enu * self._task_reference_speed_mps
        self.target_acceleration = self._task_axis_enu * acceleration
        self.target_attitude = np.array(
            [self._task_start_roll_enu,
             self._task_start_pitch_enu,
             self._task_start_yaw_enu], dtype=float)
        self.target_attitude_rate = np.zeros(3, dtype=float)
        self.manual_des_pos = self.target_position.copy()
        self.manual_des_yaw = self._task_start_yaw_enu

    def _finish_task_return(self) -> None:
        start = self._task_start_position_abs_enu.copy()
        self.iebc.reset()
        self.task_state = self.TASK_MANUAL
        self._task_switch_armed = False
        self._task_reference_distance_m = 0.0
        self._task_reference_speed_mps = 0.0
        self._task_return_settle_s = 0.0
        self._task_transition_reason = 'returned_to_task_start'
        self.manual_des_pos = start.copy()
        self.manual_des_pos[2] -= self._z0
        self.manual_des_yaw = self._task_start_yaw_enu
        self.manual_des_roll = self._task_start_roll_enu
        self.manual_des_pitch = self._task_start_pitch_enu
        self.rc_input.filtered_cmds = self.rc_input._zero_commands()
        self.target_position = self.manual_des_pos.copy()
        self.target_velocity = np.zeros(3, dtype=float)
        self.target_acceleration = np.zeros(3, dtype=float)
        self.target_attitude = np.array(
            [self.manual_des_roll,
             self.manual_des_pitch,
             self.manual_des_yaw], dtype=float)
        self.target_attitude_rate = np.zeros(3, dtype=float)
        self.get_logger().info(
            'IEBC task return complete; manual RC control restored. '
            'Task switch must be low before the next start.')

    def _task_return_target_speed(self) -> float:
        measured_forward_excursion = float(np.dot(
            self._task_axis_enu,
            self.position - self._task_start_position_abs_enu))
        remaining_forward_distance = max(
            self._task_reference_distance_m,
            measured_forward_excursion,
            0.0)
        if remaining_forward_distance <= 1e-6:
            return 0.0
        return -min(
            self.task_return_kp * remaining_forward_distance,
            self.task_return_speed_mps)

    def _update_rc_task(self, current_time: float, dt: float) -> None:
        switch_fresh = self._task_switch_is_fresh()

        if self.task_state == self.TASK_MANUAL:
            super().update_trajectory(current_time, dt)
            if switch_fresh and not self._task_switch_high:
                self._task_switch_armed = True
            if (switch_fresh and self._task_switch_high
                    and self._task_switch_armed):
                if self._start_rc_push_task():
                    self._set_rc_task_reference(
                        dt, self.task_push_speed_mps,
                        self.task_push_accel_mps2)
                    self._filter_current_reference(dt)
            return

        # Once a task is active, switch loss is treated the same as a cancel.
        if not switch_fresh:
            self._begin_task_return('task_switch_stale')
        elif not self._task_switch_high:
            self._begin_task_return('task_switch_low')

        if self.iebc.enabled and self._fresh_external_force() is None:
            self._latch_current_hold('actuator_force_stale_during_task')
            return

        self._failsafe_hold_latched = False
        self._failsafe_reason = ''
        if self.task_state == self.TASK_PUSH:
            self._set_rc_task_reference(
                dt, self.task_push_speed_mps, self.task_push_accel_mps2)
            if self._task_reference_distance_m >= self.task_max_push_distance_m:
                self._begin_task_return('maximum_push_distance_reached')
        else:
            return_speed = self._task_return_target_speed()
            self._set_rc_task_reference(
                dt, return_speed, self.task_return_accel_mps2)

            position_error = float(np.linalg.norm(
                self.position - self._task_start_position_abs_enu))
            speed = float(np.linalg.norm(self.velocity))
            if (position_error <= self.task_return_position_tolerance_m
                    and speed <= self.task_return_velocity_tolerance_mps
                    and abs(self._task_reference_speed_mps)
                    <= self.task_return_velocity_tolerance_mps):
                self._task_return_settle_s += max(dt, 0.0)
            else:
                self._task_return_settle_s = 0.0
            if self._task_return_settle_s >= self.task_return_hold_s:
                self._finish_task_return()
                return

        self._filter_current_reference(dt)

    def _latch_current_hold(self, reason: str) -> None:
        if not self._failsafe_hold_latched:
            self._failsafe_hold_position = np.array([
                self.position[0], self.position[1],
                self.position[2] - self._z0,
            ], dtype=float)
            self._failsafe_hold_yaw_enu = self._current_yaw_enu()
            self._failsafe_hold_attitude_enu = self.target_attitude.copy()
            if not np.all(np.isfinite(self._failsafe_hold_attitude_enu)):
                self._failsafe_hold_attitude_enu = np.array(
                    [0.0, 0.0, self._failsafe_hold_yaw_enu], dtype=float)
            self.iebc.reset()
            self._failsafe_hold_latched = True
            self.get_logger().warn(f'IEBC Offboard holding current position: {reason}')
        # Apply the latched reference every cycle.  In baseline mode the
        # inherited RC generator runs before this method and must not be able
        # to overwrite a hold caused by stale certified inputs.
        self.target_position = self._failsafe_hold_position.copy()
        self.target_velocity = np.zeros(3, dtype=float)
        self.target_acceleration = np.zeros(3, dtype=float)
        self.target_attitude = self._failsafe_hold_attitude_enu.copy()
        self.target_attitude_rate = np.zeros(3, dtype=float)
        self._failsafe_reason = reason
        self._topic_reference_active = False

    def _activate_topic_reference(self, reference: NominalReference) -> bool:
        if self._topic_reference_active:
            return True
        distance = float(np.linalg.norm(reference.position_enu - self.position))
        if distance > self.initial_command_radius_m:
            self._latch_current_hold(
                f'initial_command_jump_{distance:.3f}m_exceeds_'
                f'{self.initial_command_radius_m:.3f}m')
            return False
        self.iebc.reset()
        self._topic_reference_active = True
        self._failsafe_hold_latched = False
        self._failsafe_reason = ''
        self.get_logger().info(
            f'Accepted initial nominal reference at distance {distance:.3f} m')
        return True

    def _set_topic_nominal_reference(self, reference: NominalReference) -> None:
        self.target_position = reference.position_enu.copy()
        self.target_position[2] -= self._z0
        self.target_velocity = reference.velocity_enu.copy()
        self.target_acceleration = reference.acceleration_enu.copy()
        self.target_attitude = np.array([0.0, 0.0, reference.yaw_enu], dtype=float)
        self.target_attitude_rate = np.zeros(3, dtype=float)
        self.manual_des_pos = self.target_position.copy()
        self.manual_des_yaw = reference.yaw_enu

    def _fresh_external_force(self) -> Optional[np.ndarray]:
        """Return selected actuator-force estimate in world ENU.

        The method name is retained for compatibility with the tested IEBC
        integration seam.  ``px4_outputs`` is a command/model estimate;
        ``external_wrench`` is supplied already in ENU by an external estimator.
        """
        if self._wrench_age_s() > self.iebc.wrench_timeout_s:
            return None
        if self.actuator_source == 'external_wrench':
            return self._external_force_enu.copy()
        try:
            force_body_flu = self.actuator_force_estimator.estimate_body_force_flu(
                self._motor_controls, self._servo_controls)
            rotation_body_to_enu = np.asarray(self.R, dtype=float).reshape(3, 3)
        except (TypeError, ValueError):
            return None
        if not np.all(np.isfinite(rotation_body_to_enu)):
            return None
        return rotation_body_to_enu @ force_body_flu

    def _filter_current_reference(self, dt: float) -> bool:
        if not self.iebc.enabled:
            return True
        actuator_force = self._fresh_external_force()
        if actuator_force is None:
            self._latch_current_hold('actuator_force_stale')
            return False

        nominal_position_abs = self.target_position.copy()
        nominal_position_abs[2] += self._z0
        safe_position, safe_velocity, safe_acceleration = self.iebc.filter_reference(
            dt=dt,
            measured_position_enu=self.position,
            measured_velocity_enu=self.velocity,
            nominal_position_enu=nominal_position_abs,
            nominal_velocity_enu=self.target_velocity,
            nominal_acceleration_enu=self.target_acceleration,
            actuator_force_enu=actuator_force,
        )
        self.target_position = safe_position
        self.target_position[2] -= self._z0
        self.target_velocity = safe_velocity
        self.target_acceleration = safe_acceleration
        return True

    def update_trajectory(self, current_time: float, dt: float):
        if not self._hardware_control_active:
            super().update_trajectory(current_time, dt)
            self._failsafe_reason = 'hardware_gate_inactive'
            return

        if self.nominal_source == 'rc_task':
            self._update_rc_task(current_time, dt)
            return

        if self.nominal_source == 'baseline':
            super().update_trajectory(current_time, dt)
            self._filter_current_reference(dt)
            return

        if self._nominal_reference is None:
            self._latch_current_hold('nominal_reference_missing')
            return
        if self._reference_age_s() > self.command_timeout_s:
            self._latch_current_hold('nominal_reference_stale')
            return
        if self.iebc.enabled and self._fresh_external_force() is None:
            self._latch_current_hold('actuator_force_stale')
            return
        if not self._activate_topic_reference(self._nominal_reference):
            return

        self._failsafe_hold_latched = False
        self._failsafe_reason = ''
        self._set_topic_nominal_reference(self._nominal_reference)
        self._filter_current_reference(dt)

    def publish_iebc_status(self) -> None:
        debug = self.iebc.debug
        actuator_force_enu = self._fresh_external_force()
        payload = {
            'hardware_gate_active': bool(self._hardware_control_active),
            'nominal_source': self.nominal_source,
            'nominal_age_s': self._reference_age_s(),
            'actuator_source': self.actuator_source,
            'actuator_force_quality': (
                'command_model' if self.actuator_source == 'px4_outputs'
                else 'external_estimate'),
            'actuator_model': {
                'mass_kg': self.actuator_force_estimator.mass_kg,
                'hover_control': self.actuator_force_estimator.hover_control,
                'thrust_exponent': self.actuator_force_estimator.thrust_exponent,
                'max_arm_thrust_n': self.actuator_force_estimator.max_arm_thrust_n,
                'max_tail_thrust_n': self.actuator_force_estimator.max_tail_thrust_n,
                'primary_servo_max_deg': math.degrees(
                    self.actuator_force_estimator.primary_servo_max_rad),
                'secondary_servo_max_deg': math.degrees(
                    self.actuator_force_estimator.secondary_servo_max_rad),
                'secondary_gear_ratio': (
                    self.actuator_force_estimator.secondary_gear_ratio),
            },
            'wrench_age_s': self._wrench_age_s(),
            'actuator_motors_age_s': self._motor_age_s(),
            'actuator_servos_age_s': self._servo_age_s(),
            'external_wrench_age_s': self._external_wrench_age_s(),
            'actuator_force_enu_n': (
                None if actuator_force_enu is None else actuator_force_enu.tolist()),
            'failsafe_reason': self._failsafe_reason,
            'topic_reference_active': self._topic_reference_active,
            'task_state': self.task_state,
            'task_switch_function': self.task_rc_function,
            'task_switch_value': self._task_switch_value,
            'task_switch_age_s': self._task_switch_age_s(),
            'task_switch_high': self._task_switch_high,
            'task_switch_armed': self._task_switch_armed,
            'task_transition_reason': self._task_transition_reason,
            'task_reference_distance_m': self._task_reference_distance_m,
            'task_reference_speed_mps': self._task_reference_speed_mps,
            'task_start_position_enu': (
                None if self._task_start_position_abs_enu is None
                else self._task_start_position_abs_enu.tolist()),
            'iebc_enabled': bool(self.iebc.enabled),
            'iebc_mode': self.iebc.mode,
            'barrier_active': bool(debug.get('barrier_active', False)),
            'infeasible': bool(debug.get('infeasible', False)),
            'energy_j': float(debug.get('e_i', 0.0)),
            'barrier_j': float(debug.get('h_i', 0.0)),
            'qp_slack_w': float(debug.get('qp_slack_w', 0.0)),
            'rejected_commands': self._rejected_commands,
            'rejected_actuator_outputs': self._rejected_actuator_outputs,
            'rejected_wrenches': self._rejected_wrenches,
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, allow_nan=True)
        self.iebc_status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    controller = HnuterIebcOffboardController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # SIGINT may already have invalidated the rclpy context.  Do not emit
        # through rosout after shutdown; cleanup still runs below.
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
