#!/usr/bin/env python3
"""Canonical self-contained Gazebo IEBC contact simulation for HNUTER.

The interaction-energy certificate ``K_I + V_c + S_e_bar`` is regulated
through a reference-rate power barrier. Gazebo contact, resistance-release,
recovery and recording logic is included in this file. This entrypoint may
Arm and enter Offboard automatically and must never be used on real hardware;
startup is guarded by ``HNUTER_IEBC_CUBE_SIM=1`` and the expected world name.
"""

import sys
import os
import time
import math
import queue
import select
import termios
import threading
import tty
from dataclasses import dataclass

from hnuter_log_paths import configure_ros_log_dir

# PX4 uses fixed DDS topic names. Keep SITL telemetry local unless remote DDS
# access is explicitly requested, otherwise another PX4 on the LAN can mix in.
if os.environ.get('HNUTER_ALLOW_REMOTE_DDS', '0') != '1':
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_STATIC_PEERS', None)
configure_ros_log_dir()

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import ManualControlSetpoint
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import RcChannels
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleStatus

try:
    from geometry_msgs.msg import WrenchStamped
except Exception:
    WrenchStamped = None


def env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


def env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name, '1' if default else '0')
    return str(raw).strip().lower() in ('1', 'true', 'yes', 'on')


def env_vec3(prefix: str, default) -> np.ndarray:
    default = np.asarray(default, dtype=float).reshape(3)
    return np.array([
        env_float(f'{prefix}_X', default[0]),
        env_float(f'{prefix}_Y', default[1]),
        env_float(f'{prefix}_Z', default[2]),
    ], dtype=float)


@dataclass
class _StickSample:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    throttle: float = 0.0


class RCCommandManager:
    """Convert PX4 RC telemetry into body-frame velocity references."""

    def __init__(self, logger=None):
        self.logger = logger
        self.max_vxy = env_float('HNUTER_RC_MAX_VXY_MPS', 0.6)
        self.max_vz = env_float('HNUTER_RC_MAX_VZ_MPS', 0.3)
        self.max_yaw_rate = env_float('HNUTER_RC_MAX_YAW_RATE_RPS', 0.4)
        self.deadzone = env_float('HNUTER_RC_DEADZONE', 0.10)
        self.expo = env_float('HNUTER_RC_EXPO', 0.40)
        self.filter_tau = env_float('HNUTER_RC_FILTER_TAU_S', 0.20)
        self.timeout_s = max(env_float('HNUTER_RC_TIMEOUT_S', 0.50), 0.05)
        self.pitch_sign = env_float('HNUTER_RC_PITCH_SIGN', 1.0)
        self.roll_sign = env_float('HNUTER_RC_ROLL_SIGN', -1.0)
        self.throttle_sign = env_float('HNUTER_RC_THROTTLE_SIGN', 1.0)
        self.yaw_sign = env_float('HNUTER_RC_YAW_SIGN', -1.0)

        self._manual_sample = _StickSample()
        self._manual_valid = False
        self._manual_received_s = -math.inf
        self._channels_sample = _StickSample()
        self._channels_valid = False
        self._channels_received_s = -math.inf
        self._source = 'none'
        self.filtered_cmds = self._zero_commands()

    @staticmethod
    def _zero_commands() -> dict:
        return {
            'vx_b': 0.0,
            'vy_b': 0.0,
            'vz': 0.0,
            'yaw_rate': 0.0,
            'roll_rate': 0.0,
            'lt': 0.0,
            'rt': 0.0,
        }

    @staticmethod
    def _finite_sticks(sample: _StickSample) -> bool:
        return bool(np.all(np.isfinite([
            sample.roll, sample.pitch, sample.yaw, sample.throttle
        ])))

    def feed_manual_control(self, message) -> None:
        sample = _StickSample(
            roll=float(getattr(message, 'roll', math.nan)),
            pitch=float(getattr(message, 'pitch', math.nan)),
            yaw=float(getattr(message, 'yaw', math.nan)),
            throttle=float(getattr(message, 'throttle', math.nan)),
        )
        source = int(getattr(
            message, 'data_source', ManualControlSetpoint.SOURCE_RC
        ))
        self._manual_valid = bool(
            getattr(message, 'valid', False)
            and source == ManualControlSetpoint.SOURCE_RC
            and self._finite_sticks(sample)
        )
        if self._manual_valid:
            self._manual_sample = sample
        self._manual_received_s = time.monotonic()

    @staticmethod
    def _mapped_channel(message, function_id: int):
        mapping = tuple(getattr(message, 'function', ()))
        channels = tuple(getattr(message, 'channels', ()))
        channel_count = min(int(getattr(message, 'channel_count', 0)), len(channels))
        if not 0 <= function_id < len(mapping):
            return None
        channel_index = int(mapping[function_id])
        if not 0 <= channel_index < channel_count:
            return None
        value = float(channels[channel_index])
        return value if math.isfinite(value) else None

    def feed_rc_channels(self, message) -> None:
        roll = self._mapped_channel(message, RcChannels.FUNCTION_ROLL)
        pitch = self._mapped_channel(message, RcChannels.FUNCTION_PITCH)
        yaw = self._mapped_channel(message, RcChannels.FUNCTION_YAW)
        throttle = self._mapped_channel(message, RcChannels.FUNCTION_THROTTLE)
        values = (roll, pitch, yaw, throttle)
        self._channels_valid = bool(
            not getattr(message, 'signal_lost', True)
            and all(value is not None for value in values)
        )
        if self._channels_valid:
            self._channels_sample = _StickSample(
                roll=float(roll),
                pitch=float(pitch),
                yaw=float(yaw),
                throttle=2.0 * float(throttle) - 1.0,
            )
        self._channels_received_s = time.monotonic()

    def _active_sample(self):
        now = time.monotonic()
        if self._manual_valid and now - self._manual_received_s <= self.timeout_s:
            return self._manual_sample, 'manual_control_setpoint'
        if self._channels_valid and now - self._channels_received_s <= self.timeout_s:
            return self._channels_sample, 'rc_channels'
        return _StickSample(), 'stale'

    def _shape(self, value: float) -> float:
        value = float(np.clip(value, -1.0, 1.0))
        if abs(value) <= self.deadzone:
            return 0.0
        magnitude = (abs(value) - self.deadzone) / max(1.0 - self.deadzone, 1e-6)
        magnitude = self.expo * magnitude ** 3 + (1.0 - self.expo) * magnitude
        return math.copysign(magnitude, value)

    def get_velocity_commands(self, dt: float) -> dict:
        previous_source = self._source
        sample, self._source = self._active_sample()
        if self.logger is not None and self._source != previous_source:
            if self._source == 'stale':
                self.logger.warn('RC 输入超时，速度期望正在回零。')
            else:
                self.logger.info(f'RC 输入源: {self._source}')

        targets = {
            'vx_b': self.pitch_sign * self._shape(sample.pitch) * self.max_vxy,
            'vy_b': self.roll_sign * self._shape(sample.roll) * self.max_vxy,
            'vz': self.throttle_sign * self._shape(sample.throttle) * self.max_vz,
            'yaw_rate': self.yaw_sign * self._shape(sample.yaw) * self.max_yaw_rate,
        }
        alpha = dt / (self.filter_tau + dt) if self.filter_tau > 1e-3 else 1.0
        alpha = float(np.clip(alpha, 0.0, 1.0))
        for key, target in targets.items():
            self.filtered_cmds[key] += alpha * (target - self.filtered_cmds[key])
        return self.filtered_cmds.copy()

    @property
    def source(self) -> str:
        return self._active_sample()[1]

    @property
    def age_s(self) -> float:
        latest = max(self._manual_received_s, self._channels_received_s)
        return float(time.monotonic() - latest) if math.isfinite(latest) else math.inf

    @property
    def valid(self) -> bool:
        return self._active_sample()[1] != 'stale'

    def close(self) -> None:
        pass


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


class KeyboardCommandReader:
    """后台读取单字符键盘命令，避免阻塞 ROS2 spin。"""

    def __init__(self, logger=None):
        self.logger = logger
        self.commands = queue.Queue()
        self._stop_event = threading.Event()
        self._thread = None
        self._old_termios = None
        self._stdin_fd = None

        try:
            if not sys.stdin or not sys.stdin.isatty():
                self._log_warn('标准输入不是 TTY，键盘轨迹输入不可用；悬停/手柄功能不受影响。')
                return

            self._stdin_fd = sys.stdin.fileno()
            self._old_termios = termios.tcgetattr(self._stdin_fd)
            tty.setcbreak(self._stdin_fd)
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            self._log_info(
                '键盘已启用：按 1/2/3 分别执行相对当前位置的矩形/李萨如/姿态角轨迹；'
                '实机版本忽略 o。'
            )
        except Exception as exc:
            self._log_warn(f'键盘输入初始化失败: {exc}；悬停/手柄功能不受影响。')
            self._restore_terminal()

    def _log_info(self, text: str):
        if self.logger:
            self.logger.info(text)
        else:
            print(text)

    def _log_warn(self, text: str):
        if self.logger:
            self.logger.warn(text)
        else:
            print(text)

    def _read_loop(self):
        while not self._stop_event.is_set():
            try:
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not readable:
                    continue

                key = sys.stdin.read(1)
                if key in ('1', '2', '3', 'o', 'O'):
                    self.commands.put(key)
            except Exception as exc:
                if not self._stop_event.is_set():
                    self._log_warn(f'读取键盘失败: {exc}')
                break

    def get_commands(self):
        result = []
        while True:
            try:
                result.append(self.commands.get_nowait())
            except queue.Empty:
                break
        return result

    def _restore_terminal(self):
        if self._old_termios is None or self._stdin_fd is None:
            return
        try:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_termios)
        except Exception:
            pass
        self._old_termios = None

    def close(self):
        self._stop_event.set()
        self._restore_terminal()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.2)


class HnuterIebcSimulationController(Node):
    HARDWARE_FIRMWARE_PROFILE = '3131ddd4_500_2500_gear2'

    def __init__(self):
        super().__init__('hnuter_px4_position_hardware')

        qos_profile_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_profile_command = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile_command)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile_command)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_position_callback, qos_profile_out)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile_out)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_callback, qos_profile_out)
        self.vehicle_control_mode_sub = self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self.control_mode_callback, qos_profile_out)
        qos_profile_rc = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.manual_control_sub = self.create_subscription(
            ManualControlSetpoint,
            '/fmu/out/manual_control_setpoint',
            self.manual_control_callback,
            qos_profile_rc,
        )
        self.rc_channels_sub = self.create_subscription(
            RcChannels,
            '/fmu/out/rc_channels',
            self.rc_channels_callback,
            qos_profile_rc,
        )

        # PX4 常量，兼容不同 px4_msgs 版本
        self.ARMING_STATE_ARMED = getattr(VehicleStatus, 'ARMING_STATE_ARMED', 2)

        # State variables
        self.position = np.zeros(3)       # ENU: x East, y North, z Up
        self.velocity = np.zeros(3)       # ENU
        self.R = np.eye(3)                # ENU <- FLU
        self.nav_state = None
        self.control_offboard_enabled = False
        self.armed = False
        self.data_received = False
        self.local_position_received = False
        self.attitude_received = False
        self.px4_timestamp = 0

        self._hardware_control_active = False
        self._interrupted_task = None

        # IEBC: closed-loop interaction-energy reference-rate filter.  The existing
        # PX4 position controller remains the low-level nominal interaction controller.
        self.iebc = InteractionEnergyBarrierFilter(logger=self.get_logger())
        self._iebc_external_force_enu = np.zeros(3)
        self._iebc_external_force_received_s = -math.inf
        self._iebc_last_safe_acceleration_enu = np.zeros(3)
        self.iebc_wrench_sub = None
        if self.iebc.enabled and self.iebc.wrench_source == 'external':
            if WrenchStamped is None:
                self.get_logger().warn(
                    'geometry_msgs/WrenchStamped 不可用，IEBC external wrench 输入无法建立；IEBC 将旁路。')
            else:
                self.iebc_wrench_sub = self.create_subscription(
                    WrenchStamped, self.iebc.wrench_topic,
                    self.iebc_wrench_callback, qos_profile_out)
                self.get_logger().info(
                    f'IEBC external actuator-wrench topic: {self.iebc.wrench_topic} (force must be ENU world frame)')

        # Runtime status
        self.control_loop_count = 0
        self._last_manual_cmd = {
            'vx_b': 0.0,
            'vy_b': 0.0,
            'vz': 0.0,
            'yaw_rate': 0.0,
            'roll_rate': 0.0,
            'lt': 0.0,
            'rt': 0.0,
        }

        # Yaw variables
        self._yaw_initialized = False
        self.initial_yaw = 0.0

        self.target_position = np.zeros(3)
        self.target_velocity = np.zeros(3)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([0.0, 0.0, 0.0])
        self.target_attitude_rate = np.zeros(3)

        self.max_altitude = 5.0
        self.min_altitude = -5.0
        self.manual_enabled = True
        self.manual_pos_initialized = False
        self.manual_des_pos = np.zeros(3)   # [x_enu, y_enu, z_relative]
        self.manual_des_yaw = 0.0
        # LT/RT 积分得到横滚姿态期望。
        self.manual_des_roll = 0.0
        self.manual_roll_limit_rad = np.radians(90.0)
        self._z0_initialized = False
        self._z0 = 0.0

        # Keyboard-triggered auto trajectories. 轨迹在当前 yaw 坐标系下生成，位置仍发布为 ENU。
        self.auto_traj_mode = 'hover'
        self.pending_auto_traj_mode = None
        self.auto_traj_start_time = 0.0
        self.auto_traj_start_pos = np.zeros(3)
        self.auto_traj_origin_xy = np.zeros(2)
        self.auto_traj_z = 0.0
        self.auto_traj_yaw = 0.0
        self.auto_traj_start_attitude = np.zeros(3)
        self.rectangle_size_x = 2.0
        self.rectangle_size_y = 1.5
        self.rectangle_segment_time_s = 5.0
        self.lissajous_amp_x = 1.0
        self.lissajous_amp_y = 0.75
        self.lissajous_a = 2
        self.lissajous_b = 3
        self.lissajous_period_s = 24.0
        self.attitude_step_angle_rad = math.radians(50.0)
        self.attitude_segment_time_s = 4.0

        # Time
        self.sim_start_time_s = 0.0
        self._last_timestamp_s = 0.0

        # Timers: Offboard heartbeat should be comfortably > 2 Hz
        self.offboard_timer = self.create_timer(0.05, self.offboard_startup_tick)
        self.status_timer = self.create_timer(1.0, self.print_status)
        self.debug_print_period_s = 1.0
        self._last_debug_print_time = 0.0

        self.rc_input = RCCommandManager(logger=self.get_logger())
        self.keyboard = KeyboardCommandReader(logger=self.get_logger())
        self.keyboard_timer = self.create_timer(0.1, self.poll_keyboard_commands)

        self.get_logger().info(
            'Hnuter PX4 position hardware controller initialized. Arm and '
            'Offboard remain under transmitter/PX4 authority. '
            f'Firmware profile={self.HARDWARE_FIRMWARE_PROFILE}; servo-only PWM '
            'mapping remains inside PX4 and does not apply to motors.'
        )

    # ============================================================
    # PX4 callbacks
    # ============================================================
    def local_position_callback(self, msg):
        if not (bool(msg.xy_valid) and bool(msg.z_valid)):
            return

        self.px4_timestamp = int(msg.timestamp)
        self.position = np.array([msg.y, msg.x, -msg.z], dtype=float)
        if bool(msg.v_xy_valid) and bool(msg.v_z_valid):
            self.velocity = np.array([msg.vy, msg.vx, -msg.vz], dtype=float)

        self.local_position_received = True
        self.data_received = self.local_position_received and self.attitude_received

    def attitude_callback(self, msg):
        self.px4_timestamp = int(msg.timestamp)
        w, x, y, z = msg.q
        R_ned_frd = np.array([
            [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x ** 2 + y ** 2)]
        ])
        R_enu_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        R_frd_flu = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        self.R = R_enu_ned @ R_ned_frd @ R_frd_flu

        if not self._yaw_initialized:
            self.initial_yaw = float(np.arctan2(self.R[1, 0], self.R[0, 0]))
            self.target_attitude[2] = self.initial_yaw
            self.manual_des_yaw = self.initial_yaw
            self._yaw_initialized = True

        self.attitude_received = True
        self.data_received = self.local_position_received and self.attitude_received
        if self.data_received:
            self.control_loop()

    def status_callback(self, msg):
        self.armed = int(getattr(msg, 'arming_state', -1)) == self.ARMING_STATE_ARMED
        self.nav_state = int(getattr(msg, 'nav_state', -1))
        self._update_hardware_control_gate()

    def control_mode_callback(self, msg):
        self.control_offboard_enabled = bool(getattr(msg, 'flag_control_offboard_enabled', False))
        if hasattr(msg, 'flag_armed'):
            self.armed = bool(msg.flag_armed)
        self._update_hardware_control_gate()

    def manual_control_callback(self, msg):
        self.rc_input.feed_manual_control(msg)

    def rc_channels_callback(self, msg):
        self.rc_input.feed_rc_channels(msg)

    def iebc_wrench_callback(self, msg):
        """Receive ACTUAL actuator force estimate in ENU world frame.

        The topic contract is intentionally generic: another ROS2 node may
        reconstruct actuator wrench from motor thrusts + measured tilt angles.
        Only force is used by this translational IEBC specialization.
        """
        try:
            force = msg.wrench.force
            value = np.array([force.x, force.y, force.z], dtype=float)
            if np.all(np.isfinite(value)):
                self._iebc_external_force_enu = value
                self._iebc_external_force_received_s = time.monotonic()
        except Exception:
            pass

    def _iebc_actuator_force_estimate(self):
        if not self.iebc.enabled or not self.iebc.valid_configuration:
            return None

        if self.iebc.wrench_source == 'external':
            age = time.monotonic() - self._iebc_external_force_received_s
            if age <= self.iebc.wrench_timeout_s:
                return self._iebc_external_force_enu.copy()
            return None

        # Software/Gazebo bring-up proxy only.  Reconstruct the interaction-axis
        # control force from the certified equivalent stiffness/damping model.
        # This is NOT an actual-wrench estimate; use external wrench for paper data.
        s_meas = float(np.dot(self.iebc.axis, self.position))
        v_meas = float(np.dot(self.iebc.axis, self.velocity))
        if self.iebc.safe_s is None or self.iebc.safe_v is None:
            u_i = 0.0
        else:
            e_i = float(self.iebc.safe_s - s_meas)
            u_i = self.iebc.k_c * e_i + self.iebc.d_c * (self.iebc.safe_v - v_meas)
        return self.iebc.axis * u_i + np.array(
            [0.0, 0.0, self.iebc.mass * self.iebc.g], dtype=float)

    def _apply_iebc_to_reference(self, dt: float):
        if not self.iebc.enabled:
            # Keep nominal/safe logging coherent for nominal comparison runs.
            self.iebc.filter_reference(
                dt=dt,
                measured_position_enu=self.position,
                measured_velocity_enu=self.velocity,
                nominal_position_enu=(self.target_position + np.array(
                    [0.0, 0.0, self._z0 if self._z0_initialized else 0.0])),
                nominal_velocity_enu=self.target_velocity,
                nominal_acceleration_enu=self.target_acceleration,
                actuator_force_enu=None,
            )
            return

        nominal_pos_abs = self.target_position.copy()
        if self._z0_initialized:
            nominal_pos_abs[2] += self._z0
        nominal_vel = self.target_velocity.copy()
        nominal_acc = self.target_acceleration.copy()

        actuator_force = self._iebc_actuator_force_estimate()
        safe_pos_abs, safe_vel, safe_acc = self.iebc.filter_reference(
            dt=dt,
            measured_position_enu=self.position,
            measured_velocity_enu=self.velocity,
            nominal_position_enu=nominal_pos_abs,
            nominal_velocity_enu=nominal_vel,
            nominal_acceleration_enu=nominal_acc,
            actuator_force_enu=actuator_force,
        )

        self.target_position = safe_pos_abs.copy()
        if self._z0_initialized:
            self.target_position[2] -= self._z0
        self.target_velocity = safe_vel
        self.target_acceleration = safe_acc
        self._iebc_last_safe_acceleration_enu = safe_acc.copy()

    def _after_iebc_reference_update(self, current_time: float, dt: float) -> None:
        """Optional experiment hook, called after nominal reference filtering."""
        del current_time, dt

    # ============================================================
    # Transmitter-owned Arm/Offboard gate
    # ============================================================
    def is_offboard(self) -> bool:
        return bool(self.control_offboard_enabled)

    def timestamp_now_us(self) -> int:
        return int(self.px4_timestamp) if self.px4_timestamp > 0 else int(self.get_clock().now().nanoseconds / 1000)

    def offboard_startup_tick(self):
        # Required proof-of-life only. This hardware node has no VehicleCommand
        # publisher and cannot request Arm, Disarm, or Offboard.
        self.publish_offboard_control_mode()
        self._update_hardware_control_gate()

        if not self.data_received or self.px4_timestamp <= 0:
            return
        if not self._hardware_control_active:
            self._hold_current_position()
            self.publish_px4_trajectory_setpoint()

    def _hold_current_position(self):
        self._z0 = float(self.position[2])
        self._z0_initialized = True
        self.target_position = np.array([self.position[0], self.position[1], 0.0])
        self.target_velocity = np.zeros(3)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([0.0, 0.0, self.initial_yaw])
        self.target_attitude_rate = np.zeros(3)

    def _begin_hardware_control(self):
        self._hardware_control_active = True
        self._z0 = float(self.position[2])
        self._z0_initialized = True
        self.manual_des_pos = np.array([self.position[0], self.position[1], 0.0])
        self.manual_des_yaw = self.initial_yaw
        self.manual_des_roll = 0.0
        self.manual_pos_initialized = True
        self.target_position = self.manual_des_pos.copy()
        self.target_velocity = np.zeros(3)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([0.0, 0.0, self.manual_des_yaw])
        self.target_attitude_rate = np.zeros(3)
        self.auto_traj_mode = 'hover'
        self.pending_auto_traj_mode = (
            self._interrupted_task or self.pending_auto_traj_mode
        )
        self._interrupted_task = None
        self._last_timestamp_s = self.px4_timestamp / 1_000_000.0
        self.iebc.reset()
        self._iebc_last_safe_acceleration_enu = np.zeros(3)
        restart = (
            f'，任务 {self.pending_auto_traj_mode} 将从当前位置重新开始'
            if self.pending_auto_traj_mode else ''
        )
        self.get_logger().info(f'检测到 Armed + Offboard，当前位置接管{restart}。')

    def _end_hardware_control(self):
        if self.auto_traj_mode != 'hover':
            self._interrupted_task = self.auto_traj_mode
        elif self.pending_auto_traj_mode is not None:
            self._interrupted_task = self.pending_auto_traj_mode
        if self._hardware_control_active:
            self.get_logger().warn('Armed 或 Offboard 已关闭，停止推进控制任务。')
        self._hardware_control_active = False
        self.iebc.reset()
        self._iebc_last_safe_acceleration_enu = np.zeros(3)
        self.manual_pos_initialized = False
        self.auto_traj_mode = 'hover'
        self.pending_auto_traj_mode = None
        self.rc_input.filtered_cmds = self.rc_input._zero_commands()
        if self.data_received:
            self._hold_current_position()

    def _update_hardware_control_gate(self):
        should_control = bool(self.data_received and self.armed and self.is_offboard())
        if should_control and not self._hardware_control_active:
            self._begin_hardware_control()
        elif not should_control and self._hardware_control_active:
            self._end_hardware_control()

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        # 兼容不同 px4_msgs 版本
        if hasattr(msg, 'thrust_and_torque'):
            msg.thrust_and_torque = False
        if hasattr(msg, 'direct_actuator'):
            msg.direct_actuator = False
        msg.timestamp = self.timestamp_now_us()
        self.offboard_control_mode_pub.publish(msg)

    def _yaw_enu_to_ned(self, yaw_enu: float) -> float:
        yaw_ned = 0.5 * math.pi - float(yaw_enu)
        return float(math.atan2(math.sin(yaw_ned), math.cos(yaw_ned)))

    @staticmethod
    def _euler_from_rotation_matrix(R: np.ndarray) -> tuple:
        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
        pitch = math.asin(float(np.clip(-R[2, 0], -1.0, 1.0)))
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
        return roll, pitch, yaw

    def _attitude_enu_flu_to_ned_frd(self, attitude_enu_flu: np.ndarray) -> tuple:
        R_enu_flu = self.euler_to_rotation_matrix(attitude_enu_flu)
        R_enu_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
        R_frd_flu = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)
        R_ned_frd = R_enu_ned.T @ R_enu_flu @ R_frd_flu.T
        return self._euler_from_rotation_matrix(R_ned_frd)

    def publish_px4_trajectory_setpoint(self):
        timestamp = self.timestamp_now_us()
        target_abs_z_enu = float(self._z0 + self.target_position[2]) if self._z0_initialized else float(self.position[2])
        msg = TrajectorySetpoint()
        msg.timestamp = timestamp
        msg.position = [
            float(self.target_position[1]),       # NED North
            float(self.target_position[0]),       # NED East
            float(-target_abs_z_enu),             # NED Down
        ]
        msg.velocity = [
            float(self.target_velocity[1]),
            float(self.target_velocity[0]),
            float(-self.target_velocity[2]),
        ]
        msg.acceleration = [
            float(self.target_acceleration[1]),
            float(self.target_acceleration[0]),
            float(-self.target_acceleration[2]),
        ]
        roll_ned, pitch_ned, yaw_ned = self._attitude_enu_flu_to_ned_frd(self.target_attitude)
        # Hnuter PX4 extension: jerk[0]/jerk[1] carry roll/pitch attitude setpoints.
        msg.jerk = [float(roll_ned), float(pitch_ned), float('nan')]
        msg.yaw = float(yaw_ned)
        msg.yawspeed = float(-self.target_attitude_rate[2])
        self.trajectory_setpoint_pub.publish(msg)

    # ============================================================
    # Keyboard trajectory commands
    # ============================================================
    def _zero_manual_cmd(self) -> dict:
        return {
            'vx_b': 0.0,
            'vy_b': 0.0,
            'vz': 0.0,
            'yaw_rate': 0.0,
            'roll_rate': 0.0,
            'lt': 0.0,
            'rt': 0.0,
        }

    def poll_keyboard_commands(self):
        for key in self.keyboard.get_commands():
            if key in ('o', 'O'):
                self.get_logger().warn(
                    '实机版本不接受键盘起飞命令；请用遥控器控制 Arm、Offboard 和升降。'
                )
            elif key == '1':
                self.pending_auto_traj_mode = 'rectangle'
                self.get_logger().info('收到键盘 1：矩形轨迹将从当前实测位置开始。')
            elif key == '2':
                self.pending_auto_traj_mode = 'lissajous'
                self.get_logger().info('收到键盘 2：李萨如轨迹将从当前实测位置开始。')
            elif key == '3':
                self.pending_auto_traj_mode = 'attitude'
                self.get_logger().info('收到键盘 3：姿态角轨迹将从当前实测位置开始。')

    def _trajectory_ready(self, current_time: float) -> bool:
        del current_time
        return bool(
            self._hardware_control_active
            and self.is_offboard()
            and self.armed
            and self.manual_pos_initialized
        )

    def _yaw_rotation_2d(self, yaw: float) -> np.ndarray:
        c = math.cos(yaw)
        s = math.sin(yaw)
        return np.array([[c, -s], [s, c]], dtype=float)

    def _wrap_angle_rad(self, angle: float) -> float:
        return float(math.atan2(math.sin(angle), math.cos(angle)))

    def _start_auto_trajectory(self, mode: str, current_time: float):
        # PX4 needs an absolute local setpoint, so resolve every relative task
        # against the measured position at the instant the task starts.
        current_relative_z = float(self.position[2] - self._z0)
        self.manual_des_pos = np.array([
            self.position[0], self.position[1], current_relative_z
        ])
        self.auto_traj_mode = mode
        self.auto_traj_start_time = current_time
        self.auto_traj_yaw = float(self.manual_des_yaw)
        self.auto_traj_start_attitude = np.array([0.0, 0.0, self.auto_traj_yaw], dtype=float)
        self.auto_traj_start_pos = self.manual_des_pos.copy()
        self.auto_traj_start_pos[2] = float(np.clip(
            self.auto_traj_start_pos[2],
            self.min_altitude,
            self.max_altitude
        ))
        self.auto_traj_z = float(self.auto_traj_start_pos[2])

        R_yaw = self._yaw_rotation_2d(self.auto_traj_yaw)
        if mode == 'lissajous':
            first_rel_xy = np.array([self.lissajous_amp_x, self.lissajous_amp_y], dtype=float)
            self.auto_traj_origin_xy = self.auto_traj_start_pos[:2] - R_yaw @ first_rel_xy
            mode_text = '李萨如'
        elif mode == 'attitude':
            self.auto_traj_origin_xy = self.auto_traj_start_pos[:2].copy()
            mode_text = '姿态角'
        else:
            self.auto_traj_origin_xy = self.auto_traj_start_pos[:2].copy()
            mode_text = '矩形'

        self.manual_des_pos = self.auto_traj_start_pos.copy()
        self.manual_des_roll = 0.0
        self.get_logger().info(
            f'开始执行{mode_text}轨迹：起点 [{self.auto_traj_start_pos[0]:.2f}, '
            f'{self.auto_traj_start_pos[1]:.2f}, {self.auto_traj_start_pos[2]:.2f}]，'
            '完成后回到该点悬停。'
        )

    def _finish_auto_trajectory(self):
        finished_mode = self.auto_traj_mode
        if finished_mode == 'lissajous':
            mode_text = '李萨如'
        elif finished_mode == 'attitude':
            mode_text = '姿态角'
        else:
            mode_text = '矩形'
        self.auto_traj_mode = 'hover'
        self.manual_des_pos = self.auto_traj_start_pos.copy()
        self.manual_des_yaw = self.auto_traj_yaw
        self.manual_des_roll = 0.0
        self.target_position = self.manual_des_pos.copy()
        self.target_velocity = np.zeros(3)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([0.0, 0.0, self.manual_des_yaw], dtype=float)
        self.target_attitude_rate = np.zeros(3)
        self.get_logger().info(f'{mode_text}轨迹完成，已回到悬停目标点。')

    def _rectangle_reference(self, elapsed: float):
        segment_time = float(self.rectangle_segment_time_s)
        total_time = 4.0 * segment_time
        if elapsed >= total_time:
            return self.auto_traj_start_pos.copy(), np.zeros(3), np.zeros(3), True

        waypoints = np.array([
            [0.0, 0.0],
            [self.rectangle_size_x, 0.0],
            [self.rectangle_size_x, self.rectangle_size_y],
            [0.0, self.rectangle_size_y],
            [0.0, 0.0],
        ], dtype=float)
        segment_idx = min(int(elapsed / segment_time), 3)
        segment_elapsed = elapsed - segment_idx * segment_time
        u = float(np.clip(segment_elapsed / segment_time, 0.0, 1.0))
        smooth_u = 3.0 * u ** 2 - 2.0 * u ** 3
        smooth_du = (6.0 * u * (1.0 - u)) / segment_time
        smooth_ddu = (6.0 * (1.0 - 2.0 * u)) / (segment_time ** 2)

        p0 = waypoints[segment_idx]
        delta = waypoints[segment_idx + 1] - p0
        local_xy = p0 + smooth_u * delta
        local_vel_xy = smooth_du * delta
        local_acc_xy = smooth_ddu * delta

        R_yaw = self._yaw_rotation_2d(self.auto_traj_yaw)
        pos = np.array([
            *(self.auto_traj_origin_xy + R_yaw @ local_xy),
            self.auto_traj_z
        ], dtype=float)
        vel = np.array([*(R_yaw @ local_vel_xy), 0.0], dtype=float)
        acc = np.array([*(R_yaw @ local_acc_xy), 0.0], dtype=float)
        return pos, vel, acc, False

    def _lissajous_reference(self, elapsed: float):
        period = float(self.lissajous_period_s)
        if elapsed >= period:
            return self.auto_traj_start_pos.copy(), np.zeros(3), np.zeros(3), True

        theta = 2.0 * math.pi * elapsed / period
        theta_dot = 2.0 * math.pi / period
        ax = float(self.lissajous_a)
        by = float(self.lissajous_b)

        local_xy = np.array([
            self.lissajous_amp_x * math.cos(ax * theta),
            self.lissajous_amp_y * math.cos(by * theta),
        ], dtype=float)
        local_vel_xy = np.array([
            -self.lissajous_amp_x * ax * theta_dot * math.sin(ax * theta),
            -self.lissajous_amp_y * by * theta_dot * math.sin(by * theta),
        ], dtype=float)
        local_acc_xy = np.array([
            -self.lissajous_amp_x * (ax * theta_dot) ** 2 * math.cos(ax * theta),
            -self.lissajous_amp_y * (by * theta_dot) ** 2 * math.cos(by * theta),
        ], dtype=float)

        R_yaw = self._yaw_rotation_2d(self.auto_traj_yaw)
        pos = np.array([
            *(self.auto_traj_origin_xy + R_yaw @ local_xy),
            self.auto_traj_z
        ], dtype=float)
        vel = np.array([*(R_yaw @ local_vel_xy), 0.0], dtype=float)
        acc = np.array([*(R_yaw @ local_acc_xy), 0.0], dtype=float)
        return pos, vel, acc, False

    def _attitude_reference(self, elapsed: float):
        segment_time = float(self.attitude_segment_time_s)
        cycle_time = 2.0 * segment_time
        total_time = 3.0 * cycle_time
        if elapsed >= total_time:
            return self.auto_traj_start_attitude.copy(), np.zeros(3), True

        axis_idx = min(int(elapsed / cycle_time), 2)
        cycle_elapsed = elapsed - axis_idx * cycle_time
        rising = cycle_elapsed < segment_time
        segment_elapsed = cycle_elapsed if rising else cycle_elapsed - segment_time
        u = float(np.clip(segment_elapsed / segment_time, 0.0, 1.0))
        smooth_u = 3.0 * u ** 2 - 2.0 * u ** 3
        smooth_du = (6.0 * u * (1.0 - u)) / segment_time

        if rising:
            offset = self.attitude_step_angle_rad * smooth_u
            offset_rate = self.attitude_step_angle_rad * smooth_du
        else:
            offset = self.attitude_step_angle_rad * (1.0 - smooth_u)
            offset_rate = -self.attitude_step_angle_rad * smooth_du

        attitude = self.auto_traj_start_attitude.copy()
        attitude_rate = np.zeros(3)
        attitude[axis_idx] += offset
        attitude_rate[axis_idx] = offset_rate
        attitude[2] = self._wrap_angle_rad(attitude[2])
        return attitude, attitude_rate, False

    def _update_auto_trajectory(self, current_time: float):
        elapsed = max(0.0, current_time - self.auto_traj_start_time)
        if self.auto_traj_mode == 'attitude':
            attitude, attitude_rate, done = self._attitude_reference(elapsed)
            if done:
                self._finish_auto_trajectory()
                return True

            self.manual_des_pos = self.auto_traj_start_pos.copy()
            self.manual_des_yaw = attitude[2]
            self.manual_des_pitch = attitude[1]
            self._last_manual_cmd = self._zero_manual_cmd()
            self.target_position = self.auto_traj_start_pos.copy()
            self.target_velocity = np.zeros(3)
            self.target_acceleration = np.zeros(3)
            self.target_attitude = attitude
            self.target_attitude_rate = attitude_rate
            return True

        if self.auto_traj_mode == 'rectangle':
            pos, vel, acc, done = self._rectangle_reference(elapsed)
        elif self.auto_traj_mode == 'lissajous':
            pos, vel, acc, done = self._lissajous_reference(elapsed)
        else:
            return False

        if done:
            self._finish_auto_trajectory()
            return True

        self.manual_des_pos = pos.copy()
        self.manual_des_yaw = self.auto_traj_yaw
        self.manual_des_roll = 0.0
        self._last_manual_cmd = self._zero_manual_cmd()
        self.target_position = pos
        self.target_velocity = vel
        self.target_acceleration = acc
        self.target_attitude = np.array([0.0, 0.0, self.auto_traj_yaw], dtype=float)
        self.target_attitude_rate = np.zeros(3)
        return True

    # ============================================================
    # Manual trajectory: RC velocity -> desired position/yaw
    # ============================================================
    def update_trajectory(self, current_time: float, dt: float):
        if not self._z0_initialized:
            self._z0 = float(self.position[2])
            self._z0_initialized = True

        # Arm 或 Offboard 无效时只发布当前位置，绝不推进遥控/轨迹状态。
        if not self._hardware_control_active:
            self.manual_pos_initialized = False
            self.auto_traj_mode = 'hover'
            self.manual_des_roll = 0.0
            self._last_manual_cmd = self._zero_manual_cmd()
            self._hold_current_position()
            return

        if not self.manual_pos_initialized:
            self.manual_des_pos = np.array([
                self.position[0], self.position[1], self.position[2] - self._z0
            ])
            self.manual_des_yaw = self.initial_yaw if self._yaw_initialized else 0.0
            self.manual_des_roll = 0.0
            self.manual_pos_initialized = True

        if self.pending_auto_traj_mode is not None and self._trajectory_ready(current_time):
            self._start_auto_trajectory(self.pending_auto_traj_mode, current_time)
            self.pending_auto_traj_mode = None

        if self.auto_traj_mode != 'hover':
            if self._update_auto_trajectory(current_time):
                return

        cmds = self.rc_input.get_velocity_commands(dt) if self.manual_enabled else self._zero_manual_cmd()
        self._last_manual_cmd = cmds.copy()

        yaw_ref = self.manual_des_yaw
        vx_w = cmds['vx_b'] * math.cos(yaw_ref) - cmds['vy_b'] * math.sin(yaw_ref)
        vy_w = cmds['vx_b'] * math.sin(yaw_ref) + cmds['vy_b'] * math.cos(yaw_ref)
        vz_w = cmds['vz']
        yaw_rate = cmds['yaw_rate']
        roll_rate = cmds.get('roll_rate', 0.0)

        self.manual_des_pos[0] += vx_w * dt
        self.manual_des_pos[1] += vy_w * dt
        self.manual_des_pos[2] += vz_w * dt
        self.manual_des_pos[2] = float(np.clip(self.manual_des_pos[2], self.min_altitude, self.max_altitude))
        self.manual_des_yaw = float(np.arctan2(math.sin(self.manual_des_yaw + yaw_rate * dt), math.cos(self.manual_des_yaw + yaw_rate * dt)))
        self.manual_des_roll = float(np.clip(
            self.manual_des_roll + roll_rate * dt,
            -self.manual_roll_limit_rad,
            self.manual_roll_limit_rad
        ))

        self.target_position = self.manual_des_pos.copy()
        self.target_velocity = np.array([vx_w, vy_w, vz_w], dtype=float)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([self.manual_des_roll, 0.0, self.manual_des_yaw], dtype=float)
        self.target_attitude_rate = np.array([roll_rate, 0.0, yaw_rate], dtype=float)

    # Hnuter firmware attitude-extension coordinate conversion
    def euler_to_rotation_matrix(self, euler):
        roll, pitch, yaw = euler
        R_x = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
        R_y = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        return R_z @ R_y @ R_x

    def control_loop(self):
        if not self.data_received or self.px4_timestamp <= 0:
            return

        now_s = self.px4_timestamp / 1_000_000.0
        if self.sim_start_time_s == 0.0:
            self.sim_start_time_s = now_s
            self._last_timestamp_s = now_s
            return

        dt = now_s - self._last_timestamp_s
        if dt <= 0.0001 or dt > 0.2:
            self._last_timestamp_s = now_s
            return

        self._last_timestamp_s = now_s
        current_time = now_s - self.sim_start_time_s

        self.update_trajectory(current_time, dt)
        self._apply_iebc_to_reference(dt)
        self._after_iebc_reference_update(current_time, dt)
        self.control_loop_count += 1
        self.publish_px4_trajectory_setpoint()

        now = time.time()
        if now - self._last_debug_print_time >= self.debug_print_period_s:
            state = 'RC/轨迹控制' if self._hardware_control_active else '等待 Armed + Offboard'
            self.get_logger().info(
                f'PX4 position Offboard {state} dt={dt * 1000:.1f}ms | '
                f'Offboard={self.is_offboard()} | Armed={self.armed} | '
                f'z={self.position[2] - self._z0:+.2f}m -> {self.target_position[2]:.2f}m'
            )
            self._last_debug_print_time = now

    # Status/shutdown
    def print_status(self):
        if not self.data_received:
            self.get_logger().info('等待 PX4 odometry/attitude/status 数据...')
            return

        control_hz = self.control_loop_count
        self.control_loop_count = 0
        pos_curr_rel_z = self.position[2] - self._z0 if self._z0_initialized else self.position[2]
        current_pitch_deg = float(np.degrees(np.arcsin(np.clip(-self.R[2, 0], -1.0, 1.0))))
        self.get_logger().info(
            f"\n{'=' * 72}\n"
            f"Mode: Offboard={self.is_offboard()} | Armed={self.armed} | nav_state={self.nav_state} | ctrl≈{control_hz}Hz\n"
            f"Hardware gate: active={self._hardware_control_active} | RC={self.rc_input.source} | age={self.rc_input.age_s:.3f}s\n"
            f"Target ENU/Zrel: [{self.target_position[0]:6.2f}, {self.target_position[1]:6.2f}, {self.target_position[2]:6.2f}] m\n"
            f"Current ENU/Zrel: [{self.position[0]:6.2f}, {self.position[1]:6.2f}, {pos_curr_rel_z:6.2f}] m\n"
            f"Keyboard trajectory: active={self.auto_traj_mode} | pending={self.pending_auto_traj_mode}\n"
            f"RC: vx_b={self._last_manual_cmd['vx_b']:+4.2f}, vy_b={self._last_manual_cmd['vy_b']:+4.2f}, "
            f"vz={self._last_manual_cmd['vz']:+4.2f}, yaw_rate={self._last_manual_cmd['yaw_rate']:+4.2f}, "
            f"LT={self._last_manual_cmd.get('lt', 0.0):4.2f}, RT={self._last_manual_cmd.get('rt', 0.0):4.2f}\n"
            f"RollCmd: des={np.degrees(self.manual_des_roll):+5.1f}° | Pitch: current={current_pitch_deg:+5.1f}° | "
            f"roll_rate={np.degrees(self._last_manual_cmd.get('roll_rate', 0.0)):+5.1f}°/s\n"
            f"IEBC: enabled={self.iebc.debug.get('enabled', False)} | barrier={self.iebc.debug.get('barrier_active', False)} | "
            f"infeasible={self.iebc.debug.get('infeasible', False)}\n"
            f"  E: K={self.iebc.debug.get('k_i', 0.0):.3f} + Vc={self.iebc.debug.get('v_c', 0.0):.3f} + "
            f"Sbar={self.iebc.debug.get('s_bar', 0.0):.3f} = {self.iebc.debug.get('e_i', 0.0):.3f}/"
            f"{self.iebc.e_max:.3f} J | h={self.iebc.debug.get('h_i', 0.0):+.3f} J\n"
            f"  P: Pe_hat={self.iebc.debug.get('p_hat', 0.0):+.3f} W | Pallow={self.iebc.debug.get('p_allow', 0.0):+.3f} W | "
            f"gE={self.iebc.debug.get('g_e', 0.0):+.3f} N | piE={self.iebc.debug.get('pi_e', 0.0):+.3f} W\n"
            f"  ref: e={self.iebc.debug.get('e_ref', 0.0):+.3f} m | vI={self.iebc.debug.get('v_i', 0.0):+.3f} | "
            f"vnom={self.iebc.debug.get('v_nom_i', 0.0):+.3f} -> vsafe={self.iebc.debug.get('v_safe_i', 0.0):+.3f} m/s | "
            f"slack={self.iebc.debug.get('qp_slack_w', 0.0):.3f} W\n"
            f"{'=' * 72}"
        )

    def destroy_node(self):
        try:
            self.keyboard.close()
        except Exception:
            pass
        try:
            self.rc_input.close()
        except Exception:
            pass
        super().destroy_node()


# Gazebo-only experiment layer. It deliberately remains inside this single
# simulation entrypoint and is guarded against real-aircraft execution.

import csv
import math
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

# Conservative, explicit defaults for this one-dimensional contact experiment.
# They must be set before constructing the IEBC controller because its
# constructor reads the environment.
os.environ.setdefault('HNUTER_IEBC_ENABLE', '1')
os.environ.setdefault('HNUTER_IEBC_MASS_KG', '4.5')
os.environ.setdefault('HNUTER_IEBC_LAMBDA_BAR_KG', '4.5')
# The legacy experiment used 1.2 J for K_I + Sbar only. The revised
# certificate also includes V_c; replaying the two geometry-valid legacy 7 N
# successes gives about 1.66/1.87 J under K_I + V_c + Sbar. Keep this
# Gazebo-only default above those traces with explicit margin. Real hardware
# must select E_max from its own certified interaction-energy limit.
os.environ.setdefault('HNUTER_IEBC_E_MAX_J', '2.5')
os.environ.setdefault('HNUTER_IEBC_AXIS_X', '1.0')
os.environ.setdefault('HNUTER_IEBC_AXIS_Y', '0.0')
os.environ.setdefault('HNUTER_IEBC_AXIS_Z', '0.0')
os.environ.setdefault('HNUTER_IEBC_CBF_GAMMA', '4.0')
os.environ.setdefault('HNUTER_IEBC_KC_NPM', '11.25')
os.environ.setdefault('HNUTER_IEBC_DC_NSPM', '11.25')
os.environ.setdefault('HNUTER_IEBC_REF_SYNC_GAIN', '0.0')
os.environ.setdefault('HNUTER_IEBC_MAX_REF_SPEED_MPS', '0.12')
os.environ.setdefault('HNUTER_IEBC_MAX_REF_ACCEL_MPS2', '3.0')
os.environ.setdefault('HNUTER_IEBC_POWER_MARGIN_W', '0.05')
os.environ.setdefault('HNUTER_IEBC_FORCE_ERROR_BOUND_N', '0.20')
os.environ.setdefault('HNUTER_IEBC_RESIDUAL_POWER_BOUND_W', '0.0')
os.environ.setdefault('HNUTER_IEBC_STORAGE_INITIAL_J', '0.0')
os.environ.setdefault('HNUTER_IEBC_ACCEL_FF_MODE', 'nominal')
os.environ.setdefault('HNUTER_IEBC_WRENCH_SOURCE', 'proxy')

from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GazeboNode

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import VehicleCommand

from hnuter_log_paths import diagnostic_csv_path


def smoothstep01(value: float) -> tuple:
    """Return cubic smooth-step position, first and second derivatives."""
    u = float(np.clip(value, 0.0, 1.0))
    return (3.0 * u ** 2 - 2.0 * u ** 3,
            6.0 * u * (1.0 - u),
            6.0 * (1.0 - 2.0 * u))


def wrap_pi(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return float(math.atan2(math.sin(angle_rad), math.cos(angle_rad)))


class ContactForceFilter:
    """Age-gated first-order filter for Gazebo's contact-wrench samples."""

    def __init__(self, tau_s: float = 0.08, timeout_s: float = 0.15):
        self.tau_s = max(float(tau_s), 0.0)
        self.timeout_s = max(float(timeout_s), 0.01)
        self.raw_n = 0.0
        self.filtered_n = 0.0
        self.last_sample_monotonic = -math.inf

    def feed(self, force_n: float, received_s: float = None) -> None:
        self.raw_n = max(float(force_n), 0.0)
        self.last_sample_monotonic = (
            time.monotonic() if received_s is None else float(received_s))

    def update(self, dt: float, now_s: float = None) -> float:
        now_s = time.monotonic() if now_s is None else float(now_s)
        target = self.raw_n if now_s - self.last_sample_monotonic <= self.timeout_s else 0.0
        alpha = 1.0 if self.tau_s <= 1e-6 else float(np.clip(dt / (self.tau_s + dt), 0.0, 1.0))
        self.filtered_n += alpha * (target - self.filtered_n)
        return self.filtered_n


class SustainedForceThreshold:
    """Debounce a filtered force threshold without using object motion."""

    def __init__(self, threshold_n: float, hold_s: float):
        self.threshold_n = max(float(threshold_n), 0.0)
        self.hold_s = max(float(hold_s), 0.0)
        self.since_s = None

    def reset(self) -> None:
        self.since_s = None

    def update(self, force_n: float, now_s: float) -> bool:
        force_n = float(force_n)
        now_s = float(now_s)
        if not math.isfinite(force_n) or force_n < self.threshold_n:
            self.since_s = None
            return False
        if self.since_s is None:
            self.since_s = now_s
        return now_s - self.since_s >= self.hold_s


class HnuterIebcSimulation(HnuterIebcSimulationController):
    """SITL-only controller and Gazebo experiment coordinator."""

    EXPECTED_WORLD = 'hnuter_cube_contact'
    CUBE_MODEL = 'interaction_cube'
    VEHICLE_MODEL_PREFIX = 'hnuter_contact_'
    CONTACT_TOPIC = '/hnuter/cube_contact'
    FORCE_MARKER_NAMESPACE = 'hnuter_virtual_resistance'

    STAGE_WAIT = 'WAIT_CONTROL'
    STAGE_TAKEOFF = 'TAKEOFF'
    STAGE_ALIGN = 'ALIGN_YAW'
    STAGE_APPROACH = 'APPROACH'
    STAGE_LOAD_SETTLE = 'LOAD_SETTLE'
    STAGE_PUSH = 'PUSH_RAMP'
    STAGE_RELEASE = 'RELEASE_OBSERVE'
    STAGE_COMPLETE = 'COMPLETE'
    STAGE_FAILED = 'FAILED'

    def __init__(self):
        if os.environ.get('HNUTER_IEBC_CUBE_SIM', '0') != '1':
            raise RuntimeError(
                'Refusing to start: set HNUTER_IEBC_CUBE_SIM=1 only for the '
                'HNUTER Gazebo cube-contact experiment.')

        self.world_name = os.environ.get('HNUTER_GZ_WORLD', self.EXPECTED_WORLD)
        if self.world_name != self.EXPECTED_WORLD:
            raise RuntimeError(
                f'Expected Gazebo world {self.EXPECTED_WORLD!r}, got {self.world_name!r}.')

        super().__init__()

        # Free-flight actuator power is not environment interaction. Keep the
        # reference filter transparent through takeoff and approach, then arm
        # and reset IEBC exactly at measured probe contact.
        self._iebc_requested = bool(self.iebc.enabled)
        self.iebc.enabled = False
        self.iebc.reset()

        qos_command = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_command)

        self.cmd_set_mode = getattr(VehicleCommand, 'VEHICLE_CMD_DO_SET_MODE', 176)
        self.cmd_arm_disarm = getattr(VehicleCommand, 'VEHICLE_CMD_COMPONENT_ARM_DISARM', 400)
        self._startup_ticks = 0
        self._last_mode_request_s = -math.inf
        self._last_arm_request_s = -math.inf

        self.virtual_force_n = abs(float(os.environ.get('HNUTER_CUBE_FORCE_N', '2.0')))
        self.release_mode = os.environ.get(
            'HNUTER_CUBE_RELEASE_MODE', 'force').strip().lower()
        if self.release_mode not in ('force', 'time'):
            raise ValueError(
                'HNUTER_CUBE_RELEASE_MODE must be "force" or "time", got '
                f'{self.release_mode!r}.')
        self.release_time_s = max(
            float(os.environ.get('HNUTER_CUBE_RELEASE_TIME_S', '85.0')), 0.1)
        # This is a force-triggered breakaway latch, not a displacement test.
        # The low-pass filter rejects one-step collision impulses; a short
        # continuous hold prevents a single filtered sample from releasing the
        # load. The default margin is zero so the virtual resistance disappears
        # as soon as measured force genuinely reaches it.
        self.release_force_margin_n = max(
            float(os.environ.get('HNUTER_CUBE_RELEASE_FORCE_MARGIN_N', '0.0')), 0.0)
        self.release_force_hold_s = max(
            float(os.environ.get('HNUTER_CUBE_RELEASE_FORCE_HOLD_S', '0.04')), 0.0)
        self.release_force_threshold_n = self.virtual_force_n + self.release_force_margin_n
        self.force_release_latch = SustainedForceThreshold(
            self.release_force_threshold_n, self.release_force_hold_s)
        self.barrier_tolerance_j = max(float(os.environ.get('HNUTER_CUBE_BARRIER_TOL_J', '0.02')), 0.0)
        self.qp_slack_tolerance_w = max(
            float(os.environ.get('HNUTER_CUBE_QP_SLACK_TOL_W', '0.02')), 0.0)
        self.qp_infeasible_hold_s = max(
            float(os.environ.get('HNUTER_CUBE_QP_INFEASIBLE_HOLD_S', '0.10')), 0.0)
        self.stop_barrier_tolerance_m = max(float(os.environ.get(
            'HNUTER_CUBE_STOP_BARRIER_TOL_M', '0.02')), 0.0)
        self.require_barrier_active = os.environ.get(
            'HNUTER_CUBE_REQUIRE_BARRIER_ACTIVE', '0').strip().lower() in (
                '1', 'true', 'yes', 'on')
        self.intervention_velocity_tolerance_mps = max(float(os.environ.get(
            'HNUTER_CUBE_INTERVENTION_VEL_TOL_MPS', '0.001')), 0.0)
        self.intervention_hold_s = max(float(os.environ.get(
            'HNUTER_CUBE_INTERVENTION_HOLD_S', '1.0')), 0.0)
        self.takeoff_height_m = max(float(os.environ.get('HNUTER_CUBE_TAKEOFF_M', '1.10')), 0.3)
        self.takeoff_time_s = max(float(os.environ.get('HNUTER_CUBE_TAKEOFF_TIME_S', '5.0')), 1.0)
        # The shortened probe tip is 0.75 m ahead of the vehicle origin.  The
        # cube's near face is at world X=2.6 m, so retain enough travel to make
        # contact without restoring the artificial one-metre lever arm.
        self.approach_distance_m = max(float(os.environ.get('HNUTER_CUBE_APPROACH_M', '2.10')), 0.5)
        self.approach_speed_mps = max(float(os.environ.get('HNUTER_CUBE_APPROACH_MPS', '0.12')), 0.02)
        # The long nose probe turns a 35 mm/s push into a dynamic contact test:
        # 10 N repeatedly lost yaw before force/allocation saturation, while the
        # same load completed at 20 mm/s.  Use the validated quasi-static rate;
        # force-sweep overrides remain explicit through the environment.
        self.push_speed_mps = max(float(os.environ.get('HNUTER_CUBE_PUSH_MPS', '0.050')), 0.005)
        self.max_push_distance_m = max(float(os.environ.get('HNUTER_CUBE_MAX_PUSH_M', '4.50')), 0.1)
        self.load_settle_s = max(float(os.environ.get('HNUTER_CUBE_LOAD_SETTLE_S', '1.5')), 0.2)
        self.release_observe_s = max(float(os.environ.get('HNUTER_CUBE_OBSERVE_S', '7.0')), 1.0)
        self.release_settle_position_tol_m = max(float(os.environ.get(
            'HNUTER_CUBE_SETTLE_POS_TOL_M', '0.05')), 0.01)
        self.release_settle_hold_s = max(float(os.environ.get(
            'HNUTER_CUBE_SETTLE_HOLD_S', '1.0')), 0.1)
        self.max_push_time_s = max(float(os.environ.get('HNUTER_CUBE_MAX_PUSH_TIME_S', '110.0')), 2.0)
        self.yaw_tolerance_rad = math.radians(max(
            float(os.environ.get('HNUTER_CUBE_YAW_TOL_DEG', '3.0')), 0.5))
        self.yaw_hold_s = max(float(os.environ.get('HNUTER_CUBE_YAW_HOLD_S', '1.0')), 0.2)
        self.yaw_timeout_s = max(float(os.environ.get('HNUTER_CUBE_YAW_TIMEOUT_S', '12.0')), 2.0)
        self.yaw_loss_tolerance_rad = math.radians(max(
            float(os.environ.get('HNUTER_CUBE_YAW_LOSS_TOL_DEG', '5.0')), 1.0))
        self.yaw_loss_hold_s = max(
            float(os.environ.get('HNUTER_CUBE_YAW_LOSS_HOLD_S', '0.25')), 0.05)
        # This is an outer world-heading loop around PX4's geometric yaw loop.
        # Gain 2.0 excited the KR_Y=20 inner-loop mode at about 1.3 Hz even
        # before contact.  Keep the outer correction deliberately slower.
        self.yaw_align_gain = max(
            float(os.environ.get('HNUTER_CUBE_YAW_ALIGN_GAIN', '0.6')), 0.0)
        self.yaw_align_max_rate_rad_s = math.radians(max(
            float(os.environ.get('HNUTER_CUBE_YAW_ALIGN_RATE_DPS', '15.0')), 1.0))
        self.yaw_command_bias_rad = math.radians(
            float(os.environ.get('HNUTER_CUBE_YAW_CMD_BIAS_DEG', '0.0')))

        # Gazebo and the base controller's ENU representation share world X.
        # This was confirmed from /world/.../pose/info against PX4 odometry;
        # using ENU Y makes the vehicle pass beside the cube.
        self.interaction_axis_enu = np.array([1.0, 0.0, 0.0], dtype=float)
        # The probe is fixed to body +X and the cube rail lies on Gazebo world
        # +X. This HNUTER SITL model's physical Gazebo yaw was calibrated
        # against the position-controller input: its world yaw follows the
        # controller's ENU yaw value directly (the PX4 bridge handles the
        # internal NED conversion). Keep this model-specific mapping here,
        # isolated from the real-aircraft controller.
        self.desired_world_yaw = 0.0
        # The PX4 local-yaw zero is initialized independently on every SITL
        # run.  This value is therefore only an optional initial trim;
        # Gazebo's world-heading outer loop continuously maps it to the physical
        # probe heading while valid contact geometry is required.
        self.desired_controller_yaw = self.yaw_command_bias_rad
        self.stage = self.STAGE_WAIT
        self.stage_start_s = 0.0
        self.experiment_origin_enu = None
        self.contact_origin_enu = None
        self.nominal_reference_enu = None
        self.release_target_enu = None
        self.terminal_hold_enu = None
        self.release_vehicle_position_enu = None
        self.release_vehicle_velocity_enu = None
        self.release_cube_x = math.nan
        self.release_contact_force_n = math.nan
        self.release_contact_force_raw_n = math.nan
        self.loaded_cube_x = math.nan
        self.yaw_aligned_since_s = None
        self.yaw_loss_since_s = None
        self.virtual_force_active = False
        self.release_event_seen = False
        self.peak_post_release_speed_mps = 0.0
        self.peak_post_release_position_delta_m = 0.0
        self.min_interaction_barrier_j = math.inf
        self.max_qp_slack_w = 0.0
        self.qp_infeasible_since_s = None
        self.barrier_active_seen = False
        self.reference_intervention_since_s = None
        self.max_reference_intervention_duration_s = 0.0
        self.max_reference_velocity_reduction_mps = 0.0
        self.min_stop_distance_barrier_m = math.inf
        self.max_release_excursion_m = 0.0
        self.recovery_complete_seen = False
        self.recovery_complete_time_s = math.nan
        self.release_settle_since_s = None
        self.release_settled = False
        self.release_settle_time_s = math.nan
        self.release_settle_anchor_enu = None
        self.release_position_change_m = math.inf
        self.max_recovery_dissipation_slack_w = 0.0
        self._latest_contact_sample = (0.0, 0.0, math.nan)

        self._transport_lock = threading.Lock()
        self.contact_filter = ContactForceFilter()
        self.cube_x_m = math.nan
        self.cube_y_m = math.nan
        self.vehicle_gz_position = np.full(3, math.nan)
        self.vehicle_gz_yaw = math.nan
        self.gz_node = GazeboNode()
        self.gz_node.subscribe(Contacts, self.CONTACT_TOPIC, self._contact_callback)
        self.gz_node.subscribe(
            Pose_V, f'/world/{self.world_name}/pose/info', self._pose_callback)
        self._persistent_wrench_pub = self.gz_node.advertise(
            f'/world/{self.world_name}/wrench/persistent', EntityWrench)
        self._clear_wrench_pub = self.gz_node.advertise(
            f'/world/{self.world_name}/wrench/clear', Entity)

        self.csv_path = Path(diagnostic_csv_path(
            'hnuter_external_controller_px4_position_iebc_simulation'))
        self._csv_file = self.csv_path.open('w', newline='', encoding='utf-8')
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow([
            'px4_time_s', 'stage', 'vehicle_enu_x_m', 'vehicle_enu_y_m',
            'vehicle_enu_z_m', 'vehicle_enu_vx_mps', 'vehicle_enu_vy_mps',
            'vehicle_enu_vz_mps', 'target_enu_x_m', 'target_enu_y_m',
            'target_z_relative_m', 'vehicle_gz_yaw_deg', 'target_gz_yaw_deg',
            'controller_yaw_cmd_deg', 'yaw_error_deg', 'cube_world_x_m', 'contact_force_raw_n',
            'contact_force_filtered_n', 'release_force_threshold_n',
            'force_threshold_hold_s', 'release_mode', 'scheduled_release_time_s',
            'cube_breakaway_m', 'virtual_force_active',
            'virtual_force_n', 'release_event_seen', 'iebc_active', 'iebc_barrier_active',
            'iebc_infeasible', 'iebc_storage_update_enabled',
            'barrier_active_seen', 'reference_intervention_duration_s',
            'max_reference_velocity_reduction_mps',
            'iebc_power_w', 'iebc_power_error_bound_w',
            'iebc_storage_rate_w', 'iebc_storage_j', 'iebc_kinetic_j',
            'iebc_controller_storage_j', 'iebc_energy_j', 'iebc_barrier_j',
            'iebc_constraint_barrier_j', 'iebc_energy_reserve_j',
            'min_interaction_barrier_j', 'iebc_reference_error_m',
            'iebc_nominal_reference_position_m', 'iebc_safe_reference_position_m',
            'iebc_velocity_mps', 'iebc_nominal_reference_velocity_mps',
            'iebc_task_reference_velocity_mps', 'iebc_safe_reference_velocity_mps',
            'iebc_energy_gradient_n', 'iebc_uncontrolled_power_w',
            'iebc_allowed_power_w', 'iebc_reference_power_nominal_w',
            'iebc_reference_power_safe_w', 'equivalent_stiffness_force_n',
            'contact_power_gt_w', 'iebc_qp_slack_w', 'max_iebc_qp_slack_w',
            'iebc_accel_safe_mps2',
            'iebc_mode', 'iebc_recoverable_energy_j',
            'iebc_release_excursion_m', 'iebc_stop_distance_barrier_m',
            'iebc_reserved_stop_distance_m', 'iebc_rho',
            'iebc_release_position_m', 'recovery_complete_seen',
            'recovery_complete_time_s', 'iebc_recovery_dissipation_slack_w',
            'release_position_change_m', 'release_settled', 'release_settle_time_s',
            'max_recovery_dissipation_slack_w', 'iebc_recovery_phase',
            'iebc_recovery_reference_velocity_mps',
            'iebc_recovery_rate_infeasible', 'iebc_recovery_terminal_position_m',
            'iebc_recovery_stop_candidate_s', 'iebc_recovery_stop_latched',
            'iebc_recovery_rebase_energy_j',
        ])
        self._last_csv_flush_s = time.monotonic()

        self.get_logger().warn(
            'GAZEBO-ONLY IEBC cube experiment enabled: this node may Arm and '
            'enter Offboard automatically. Never run it with a real flight controller.')
        self.get_logger().info(
            f'Virtual cube load={self.virtual_force_n:.2f} N; release mode={self.release_mode}; '
            f'scheduled release={self.release_time_s:.2f} s; force release threshold='
            f'{self.release_force_threshold_n:.2f} N for {self.release_force_hold_s:.3f} s; '
            f'cube displacement is diagnostic only; yaw tolerance='
            f'{math.degrees(self.yaw_tolerance_rad):.1f} deg; initial yaw trim='
            f'{math.degrees(self.yaw_command_bias_rad):+.1f} deg; auto-align gain='
            f'{self.yaw_align_gain:.1f}; yaw loss limit='
            f'{math.degrees(self.yaw_loss_tolerance_rad):.1f} deg for '
            f'{self.yaw_loss_hold_s:.2f} s; Kc={self.iebc.k_c:.2f} N/m; '
            f'Dc={self.iebc.d_c:.2f} N s/m; Emax={self.iebc.e_max:.2f} J; '
            f'energy reserve={self.iebc.energy_reserve_j:.3f} J; '
            f'stop distance={self.iebc.stop_distance:.2f} m; certified brake force='
            f'{self.iebc.brake_force_cert:.2f} N; '
            f'barrier-active required={self.require_barrier_active}; '
            f'closed-loop static certificate ceiling='
            f'{math.sqrt(2.0 * self.iebc.k_c * self.iebc.e_max):.2f} N; CSV={self.csv_path}')

    # Gazebo transport -------------------------------------------------
    @staticmethod
    def _vector_x(vector) -> float:
        return float(getattr(vector, 'x', 0.0))

    def _contact_callback(self, message: Contacts) -> None:
        max_force_x = 0.0
        for contact in message.contact:
            force_body_1 = 0.0
            force_body_2 = 0.0
            for wrench in contact.wrench:
                if wrench.HasField('body_1_wrench'):
                    force_body_1 += self._vector_x(wrench.body_1_wrench.force)
                if wrench.HasField('body_2_wrench'):
                    force_body_2 += self._vector_x(wrench.body_2_wrench.force)
            max_force_x = max(max_force_x, abs(force_body_1), abs(force_body_2))

        with self._transport_lock:
            self.contact_filter.feed(max_force_x)

    def _pose_callback(self, message: Pose_V) -> None:
        cube_x = math.nan
        cube_y = math.nan
        vehicle_position = None
        vehicle_yaw = math.nan
        for pose in message.pose:
            name = str(pose.name)
            if name == self.CUBE_MODEL or name.endswith(f'::{self.CUBE_MODEL}'):
                cube_x = float(pose.position.x)
                cube_y = float(pose.position.y)
            # /pose/info contains the model and all of its scoped child links.
            # A startswith() match also selected e.g.
            # ``hnuter_contact_0::contact_probe``.  That link has a fixed
            # +90 deg pitch relative to base_link, so extracting its Euler yaw
            # produces an apparent yaw drift during contact even when the
            # vehicle model remains aligned.  Only the unscoped model pose is
            # the physical heading used by the geometry gate.
            elif (name.startswith(self.VEHICLE_MODEL_PREFIX)
                  and '::' not in name):
                vehicle_position = np.array([
                    float(pose.position.x), float(pose.position.y), float(pose.position.z)])
                q = pose.orientation
                vehicle_yaw = math.atan2(
                    2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
                    1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2))
        if math.isfinite(cube_x):
            with self._transport_lock:
                self.cube_x_m = cube_x
                self.cube_y_m = cube_y
                if vehicle_position is not None:
                    self.vehicle_gz_position = vehicle_position
                    self.vehicle_gz_yaw = vehicle_yaw

    def _set_virtual_force(self) -> None:
        message = EntityWrench()
        message.entity.name = self.CUBE_MODEL
        message.entity.type = Entity.MODEL
        message.wrench.force.x = -self.virtual_force_n
        published = bool(self._persistent_wrench_pub.publish(message))
        self.virtual_force_active = True
        marker_visible = self._set_virtual_force_marker()
        self.get_logger().info(
            f'Applied persistent cube virtual force Fx={-self.virtual_force_n:.2f} N '
            f'(Gazebo publish={published}, GUI marker={marker_visible}).')

    def _clear_virtual_force(self) -> None:
        message = Entity()
        message.name = self.CUBE_MODEL
        message.type = Entity.MODEL
        published = bool(self._clear_wrench_pub.publish(message))
        self.virtual_force_active = False
        self._clear_virtual_force_marker()
        self.get_logger().warn(
            f'CLEARED cube virtual force (Gazebo publish={published}); observing vehicle response.')

    @staticmethod
    def _red_marker(marker_id: int, marker_type: int, action: int = Marker.ADD_MODIFY) -> Marker:
        marker = Marker()
        marker.ns = HnuterIebcSimulation.FORCE_MARKER_NAMESPACE
        marker.id = marker_id
        marker.action = action
        marker.type = marker_type
        marker.visibility = Marker.GUI
        for color in (marker.material.ambient, marker.material.diffuse, marker.material.emissive):
            color.r = 0.96
            color.g = 0.05
            color.b = 0.03
            color.a = 1.0
        marker.material.lighting = True
        return marker

    @classmethod
    def _virtual_force_markers(cls, force_n: float, cube_x: float, cube_y: float) -> tuple:
        """Build a -world-X arrow whose length encodes the active load."""
        length_m = float(np.clip(0.05 * abs(force_n), 0.40, 1.20))
        marker_y = cube_y - 1.35
        marker_z = 2.75
        tail_x = cube_x - 0.15
        shaft_end_x = tail_x - length_m

        shaft = cls._red_marker(1, Marker.LINE_LIST)
        shaft.scale.x = 0.065
        for x in (tail_x, shaft_end_x):
            point = shaft.point.add()
            point.x = x
            point.y = marker_y
            point.z = marker_z

        head = cls._red_marker(2, Marker.CONE)
        head.pose.position.x = shaft_end_x - 0.15
        head.pose.position.y = marker_y
        head.pose.position.z = marker_z
        # Marker cone axis is local +Z; -90 deg about Y points it along -X.
        head.pose.orientation.w = math.cos(-math.pi / 4.0)
        head.pose.orientation.y = math.sin(-math.pi / 4.0)
        head.scale.x = 0.28
        head.scale.y = 0.28
        head.scale.z = 0.35
        return shaft, head

    def _set_virtual_force_marker(self) -> bool:
        with self._transport_lock:
            cube_x = self.cube_x_m
            cube_y = self.cube_y_m
        if not (math.isfinite(cube_x) and math.isfinite(cube_y)):
            return False

        # MarkerManager's /marker service uses an Empty response. The Python
        # transport binding reports ``False`` for that void response even when
        # the request is accepted and the marker is created. Service discovery
        # is therefore the meaningful availability check here.
        try:
            marker_service_available = '/marker' in self.gz_node.service_list()
        except Exception:
            marker_service_available = False
        if not marker_service_available:
            return False

        for marker in self._virtual_force_markers(self.virtual_force_n, cube_x, cube_y):
            try:
                self.gz_node.request('/marker', marker, Marker, Empty, 100)
            except Exception:
                return False
        return True

    def _clear_virtual_force_marker(self) -> None:
        try:
            if '/marker' not in self.gz_node.service_list():
                return
        except Exception:
            return
        for marker_id in (1, 2):
            marker = self._red_marker(marker_id, Marker.NONE, Marker.DELETE_MARKER)
            try:
                self.gz_node.request('/marker', marker, Marker, Empty, 50)
            except Exception:
                pass

    # PX4 simulation-only authority -----------------------------------
    def _publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        message = VehicleCommand()
        message.command = int(command)
        message.param1 = float(param1)
        message.param2 = float(param2)
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        message.timestamp = self.timestamp_now_us()
        self.vehicle_command_pub.publish(message)

    def offboard_startup_tick(self):
        self.publish_offboard_control_mode()
        self._startup_ticks += 1
        self._update_hardware_control_gate()

        if self.data_received and self.px4_timestamp > 0:
            if not self._hardware_control_active:
                self._hold_current_position()
            self.publish_px4_trajectory_setpoint()

        if not self.data_received or self._startup_ticks < 30:
            return

        now_s = time.monotonic()
        if not self.is_offboard() and now_s - self._last_mode_request_s >= 1.0:
            self._publish_vehicle_command(self.cmd_set_mode, param1=1.0, param2=6.0)
            self._last_mode_request_s = now_s
            self.get_logger().info('SITL experiment requesting Offboard mode.')

        if not self.armed and now_s - self._last_arm_request_s >= 1.0:
            self._publish_vehicle_command(self.cmd_arm_disarm, param1=1.0)
            self._last_arm_request_s = now_s
            self.get_logger().info('SITL experiment requesting Arm.')

    # Experiment state machine ----------------------------------------
    def _set_stage(self, stage: str, current_time: float) -> None:
        previous = self.stage
        self.stage = stage
        self.stage_start_s = float(current_time)
        if stage in (self.STAGE_COMPLETE, self.STAGE_FAILED):
            self.terminal_hold_enu = self.position.copy()
        self.get_logger().warn(f'Experiment stage: {previous} -> {stage}')

    def _begin_hardware_control(self):
        super()._begin_hardware_control()
        self.desired_controller_yaw = wrap_pi(
            self.initial_yaw + self.yaw_command_bias_rad)
        self.experiment_origin_enu = self.position.copy()
        self.contact_origin_enu = None
        self.nominal_reference_enu = self.position.copy()
        self.release_target_enu = None
        self.force_release_latch.reset()
        self.release_event_seen = False
        self.release_contact_force_n = math.nan
        self.release_contact_force_raw_n = math.nan
        self.loaded_cube_x = math.nan
        self.max_qp_slack_w = 0.0
        self.qp_infeasible_since_s = None
        self.barrier_active_seen = False
        self.reference_intervention_since_s = None
        self.max_reference_intervention_duration_s = 0.0
        self.max_reference_velocity_reduction_mps = 0.0
        self.min_stop_distance_barrier_m = math.inf
        self.max_release_excursion_m = 0.0
        self.recovery_complete_seen = False
        self.recovery_complete_time_s = math.nan
        self.release_settle_since_s = None
        self.release_settled = False
        self.release_settle_time_s = math.nan
        self.release_settle_anchor_enu = None
        self.release_position_change_m = math.inf
        self.max_recovery_dissipation_slack_w = 0.0
        self.yaw_aligned_since_s = None
        self.yaw_loss_since_s = None
        self._set_virtual_force()
        self._set_stage(self.STAGE_TAKEOFF, self.px4_timestamp / 1_000_000.0 - self.sim_start_time_s)

    def _set_reference(
            self, position_enu: np.ndarray, velocity_enu=None, acceleration_enu=None,
            yaw_enu: float = None) -> None:
        position_enu = np.asarray(position_enu, dtype=float).reshape(3)
        self.nominal_reference_enu = position_enu.copy()
        self.target_position = position_enu.copy()
        self.target_position[2] -= self._z0
        self.target_velocity = (np.zeros(3) if velocity_enu is None
                                else np.asarray(velocity_enu, dtype=float).reshape(3))
        self.target_acceleration = (np.zeros(3) if acceleration_enu is None
                                    else np.asarray(acceleration_enu, dtype=float).reshape(3))
        commanded_yaw = (self.desired_controller_yaw if yaw_enu is None
                         else float(yaw_enu))
        self.target_attitude = np.array([0.0, 0.0, commanded_yaw], dtype=float)
        # PX4 position mode publishes manual_des_yaw, not target_attitude[2].
        # Both must be updated or the aircraft preserves its arbitrary startup
        # heading while translating toward the cube.
        self.manual_des_yaw = commanded_yaw
        self.target_attitude_rate = np.zeros(3)

    def _gazebo_yaw_error(self) -> float:
        with self._transport_lock:
            vehicle_yaw = self.vehicle_gz_yaw
        return (wrap_pi(self.desired_world_yaw - vehicle_yaw)
                if math.isfinite(vehicle_yaw) else math.nan)

    def _update_alignment_yaw_command(self, yaw_error: float, dt: float) -> None:
        """Calibrate PX4's local-yaw command to the Gazebo world heading.

        This Gazebo-only outer loop maintains the physical head-on geometry.
        The independent five-degree gate still aborts a trial that cannot
        track it.
        """
        if not math.isfinite(yaw_error) or dt <= 0.0:
            return
        command_rate = float(np.clip(
            self.yaw_align_gain * yaw_error,
            -self.yaw_align_max_rate_rad_s,
            self.yaw_align_max_rate_rad_s))
        self.desired_controller_yaw = wrap_pi(
            self.desired_controller_yaw + command_rate * dt)

    def _contact_force(self, dt: float) -> tuple:
        with self._transport_lock:
            filtered = self.contact_filter.update(dt)
            raw = self.contact_filter.raw_n
            cube_x = self.cube_x_m
        return raw, filtered, cube_x

    def _should_release(
            self, elapsed: float, contact_force: float, current_time: float) -> bool:
        if self.release_mode == 'time':
            return elapsed >= self.release_time_s
        return self.force_release_latch.update(contact_force, current_time)

    def update_trajectory(self, current_time: float, dt: float):
        if not self._hardware_control_active or self.experiment_origin_enu is None:
            self._hold_current_position()
            return

        elapsed = max(0.0, current_time - self.stage_start_s)
        raw_force, contact_force, cube_x = self._contact_force(dt)
        self._latest_contact_sample = (raw_force, contact_force, cube_x)
        origin = self.experiment_origin_enu
        hover_position = origin + np.array([0.0, 0.0, self.takeoff_height_m])

        # A valid force-limit trial must remain a head-on push, not merely be
        # aligned once before approach. Continue the slow world-heading outer
        # loop because PX4 local yaw and Gazebo model Euler yaw are not a
        # constant offset once contact introduces roll/pitch. Abort if physical
        # yaw still leaves the allowed cone long enough.
        if self.stage in (self.STAGE_APPROACH, self.STAGE_LOAD_SETTLE, self.STAGE_PUSH):
            yaw_error = self._gazebo_yaw_error()
            self._update_alignment_yaw_command(yaw_error, dt)
            yaw_lost = (not math.isfinite(yaw_error)
                        or abs(yaw_error) > self.yaw_loss_tolerance_rad)
            if yaw_lost:
                if self.yaw_loss_since_s is None:
                    self.yaw_loss_since_s = current_time
                elif current_time - self.yaw_loss_since_s >= self.yaw_loss_hold_s:
                    if self.virtual_force_active:
                        self._clear_virtual_force()
                    self._set_stage(self.STAGE_FAILED, current_time)
                    self.get_logger().error(
                        'Physical probe yaw alignment was lost during interaction; '
                        f'error={math.degrees(yaw_error):.2f} deg, limit='
                        f'{math.degrees(self.yaw_loss_tolerance_rad):.2f} deg for '
                        f'{self.yaw_loss_hold_s:.2f} s.')
                    self._write_csv(current_time, raw_force, contact_force, cube_x)
                    return
            else:
                self.yaw_loss_since_s = None

        if self.stage in (
                self.STAGE_LOAD_SETTLE, self.STAGE_PUSH, self.STAGE_RELEASE
        ) and self.iebc.enabled:
            barrier_j = float(self.iebc.debug.get('h_i', self.iebc.e_max))
            self.min_interaction_barrier_j = min(self.min_interaction_barrier_j, barrier_j)
            if barrier_j < -self.barrier_tolerance_j:
                self._clear_virtual_force()
                self._set_stage(self.STAGE_FAILED, current_time)
                self.get_logger().error(
                    f'IEBC barrier violated during interaction: h={barrier_j:.3f} J, '
                    f'tolerance={self.barrier_tolerance_j:.3f} J.')
                self._write_csv(current_time, raw_force, contact_force, cube_x)
                return

            qp_slack_w = float(self.iebc.debug.get('qp_slack_w', 0.0))
            self.max_qp_slack_w = max(self.max_qp_slack_w, qp_slack_w)
            qp_infeasible = (bool(self.iebc.debug.get('infeasible', False))
                             and qp_slack_w > self.qp_slack_tolerance_w)
            if qp_infeasible:
                if self.qp_infeasible_since_s is None:
                    self.qp_infeasible_since_s = current_time
                elif current_time - self.qp_infeasible_since_s >= self.qp_infeasible_hold_s:
                    self._clear_virtual_force()
                    self._set_stage(self.STAGE_FAILED, current_time)
                    self.get_logger().error(
                        'Closed-loop IEBC QP remained infeasible; strict energy '
                        f'guarantee was lost: slack={qp_slack_w:.3f} W for '
                        f'{self.qp_infeasible_hold_s:.2f} s.')
                    self._write_csv(current_time, raw_force, contact_force, cube_x)
                    return
            else:
                self.qp_infeasible_since_s = None

        if self.stage == self.STAGE_TAKEOFF:
            u, du, ddu = smoothstep01(elapsed / self.takeoff_time_s)
            position = origin + np.array([0.0, 0.0, self.takeoff_height_m * u])
            velocity = np.array([0.0, 0.0, self.takeoff_height_m * du / self.takeoff_time_s])
            acceleration = np.array([0.0, 0.0, self.takeoff_height_m * ddu / self.takeoff_time_s ** 2])
            self._set_reference(position, velocity, acceleration, yaw_enu=self.initial_yaw)
            if elapsed >= self.takeoff_time_s and abs(self.position[2] - hover_position[2]) < 0.20:
                self.yaw_aligned_since_s = None
                self._set_stage(self.STAGE_ALIGN, current_time)

        elif self.stage == self.STAGE_ALIGN:
            yaw_error = self._gazebo_yaw_error()
            self._update_alignment_yaw_command(yaw_error, dt)
            self._set_reference(hover_position)
            if math.isfinite(yaw_error) and abs(yaw_error) <= self.yaw_tolerance_rad:
                if self.yaw_aligned_since_s is None:
                    self.yaw_aligned_since_s = current_time
                elif current_time - self.yaw_aligned_since_s >= self.yaw_hold_s:
                    self.yaw_loss_since_s = None
                    self._set_stage(self.STAGE_APPROACH, current_time)
            else:
                self.yaw_aligned_since_s = None

            if elapsed >= self.yaw_timeout_s:
                self._set_stage(self.STAGE_FAILED, current_time)
                self.get_logger().error(
                    'Physical probe yaw did not align with the cube before timeout; '
                    f'error={math.degrees(yaw_error):.2f} deg.')

        elif self.stage == self.STAGE_APPROACH:
            distance = min(self.approach_speed_mps * elapsed, self.approach_distance_m)
            position = hover_position + self.interaction_axis_enu * distance
            velocity = (self.interaction_axis_enu * self.approach_speed_mps
                        if distance < self.approach_distance_m else np.zeros(3))
            self._set_reference(position, velocity)

            if contact_force >= 0.20:
                self.contact_origin_enu = self.position.copy()
                self.iebc.enabled = self._iebc_requested
                self.iebc.reset()
                self.contact_filter.filtered_n = 0.0
                self.force_release_latch.reset()
                self._set_stage(self.STAGE_LOAD_SETTLE, current_time)
            elif distance >= self.approach_distance_m and elapsed > (
                    self.approach_distance_m / self.approach_speed_mps + 3.0):
                self._set_stage(self.STAGE_FAILED, current_time)
                self.get_logger().error('No probe/cube contact detected within the configured approach distance.')

        elif self.stage == self.STAGE_LOAD_SETTLE:
            self._set_reference(self.contact_origin_enu)
            if elapsed >= self.load_settle_s:
                self.loaded_cube_x = cube_x
                self._set_stage(self.STAGE_PUSH, current_time)

        elif self.stage == self.STAGE_PUSH:
            push_distance = min(self.push_speed_mps * elapsed, self.max_push_distance_m)
            nominal = self.contact_origin_enu + self.interaction_axis_enu * push_distance
            velocity = (self.interaction_axis_enu * self.push_speed_mps
                        if push_distance < self.max_push_distance_m else np.zeros(3))
            self._set_reference(nominal, velocity)

            if self._should_release(elapsed, contact_force, current_time):
                # The old forward nominal point is not a braking target.  Keep
                # the ordinary nominal layer at the measured release pose;
                # RECOVERY ignores its interaction-axis coordinate and evolves
                # the continuous safe_s state under the two hard barriers.
                self.release_target_enu = self.position.copy()
                self.release_vehicle_position_enu = self.position.copy()
                self.release_vehicle_velocity_enu = self.velocity.copy()
                self.release_cube_x = cube_x
                self.release_contact_force_n = contact_force
                self.release_contact_force_raw_n = raw_force
                self._clear_virtual_force()
                if self.iebc.enabled:
                    measured_s = float(np.dot(
                        self.interaction_axis_enu, self.position))
                    self.iebc.enter_recovery(measured_s)
                self.release_event_seen = True
                self._set_stage(self.STAGE_RELEASE, current_time)

            elif elapsed >= self.max_push_time_s:
                self._clear_virtual_force()
                self._set_stage(self.STAGE_FAILED, current_time)
                if self.release_mode == 'time':
                    self.get_logger().error(
                        f'Scheduled release at {self.release_time_s:.2f} s was not executed '
                        f'before push timeout {self.max_push_time_s:.2f} s.')
                else:
                    self.get_logger().error(
                        f'Contact force did not reach {self.release_force_threshold_n:.2f} N '
                        f'for {self.release_force_hold_s:.3f} s before the push timeout; '
                        f'filtered contact force={contact_force:.2f} N, '
                        f'cube travel={cube_x - self.loaded_cube_x:.3f} m (diagnostic only).')

        elif self.stage == self.STAGE_RELEASE:
            self._set_reference(
                self.release_target_enu, np.zeros(3), np.zeros(3))
            speed = float(np.linalg.norm(self.velocity))
            displacement = float(np.linalg.norm(self.position - self.release_vehicle_position_enu))
            self.peak_post_release_speed_mps = max(self.peak_post_release_speed_mps, speed)
            self.peak_post_release_position_delta_m = max(
                self.peak_post_release_position_delta_m, displacement)
            evaluation_due = self.release_settled or elapsed >= self.release_observe_s
            if evaluation_due:
                failed_reasons = []
                if not self.release_event_seen:
                    failed_reasons.append('scheduled release was not observed')
                if self.iebc.enabled:
                    if self.min_interaction_barrier_j < -self.barrier_tolerance_j:
                        failed_reasons.append(
                            f'minimum barrier {self.min_interaction_barrier_j:.3f} J')
                    if self.max_qp_slack_w > self.qp_slack_tolerance_w:
                        failed_reasons.append(
                            f'maximum QP slack {self.max_qp_slack_w:.3f} W')
                    if self.min_stop_distance_barrier_m < -self.stop_barrier_tolerance_m:
                        failed_reasons.append(
                            'minimum stopping-distance barrier '
                            f'{self.min_stop_distance_barrier_m:.3f} m')
                if not self.release_settled:
                    failed_reasons.append(
                        'position did not settle before timeout: '
                        f'position change={self.release_position_change_m:.3f} m '
                        f'(limit {self.release_settle_position_tol_m:.3f} m)')
                if self.require_barrier_active:
                    if not self.barrier_active_seen:
                        failed_reasons.append('barrier never became active before release')
                    if self.max_reference_intervention_duration_s < self.intervention_hold_s:
                        failed_reasons.append(
                            'safe reference was not below nominal for the required '
                            f'{self.intervention_hold_s:.2f} s')

                if failed_reasons:
                    self._set_stage(self.STAGE_FAILED, current_time)
                    self.get_logger().error(
                        'EXPERIMENT FAILED: ' + '; '.join(failed_reasons) +
                        f'; CSV={self.csv_path}')
                    return

                self._set_stage(self.STAGE_COMPLETE, current_time)
                cube_delta = cube_x - self.release_cube_x if (
                    math.isfinite(cube_x) and math.isfinite(self.release_cube_x)) else math.nan
                self.get_logger().warn(
                    f'EXPERIMENT COMPLETE: virtual load was cleared by {self.release_mode} release; '
                    f'release force={self.release_contact_force_n:.3f} N '
                    f'(raw={self.release_contact_force_raw_n:.3f} N); '
                    f'barrier_seen={self.barrier_active_seen}, '
                    f'max intervention={self.max_reference_intervention_duration_s:.3f} s; '
                    f'recovery mode={self.iebc.mode}, stop time='
                    f'{self.recovery_complete_time_s:.3f} s, min h_D='
                    f'{self.min_stop_distance_barrier_m:.3f} m; '
                    f'settled in {self.release_settle_time_s:.3f} s with '
                    f'1 s position change={self.release_position_change_m:.3f} m; '
                    f'post-release peak vehicle speed={self.peak_post_release_speed_mps:.3f} m/s, '
                    f'peak vehicle displacement={self.peak_post_release_position_delta_m:.3f} m, '
                    f'cube dx={cube_delta:.3f} m, CSV={self.csv_path}')

        elif self.stage in (self.STAGE_COMPLETE, self.STAGE_FAILED):
            hold = (self.release_target_enu if self.release_target_enu is not None
                    else self.terminal_hold_enu)
            self._set_reference(hold)

        # CSV is written by _after_iebc_reference_update(), after the nominal
        # reference has been filtered, so nominal and safe values share a sample.

    def _after_iebc_reference_update(self, current_time: float, dt: float) -> None:
        del dt
        debug = self.iebc.debug
        if self.stage == self.STAGE_PUSH and self.iebc.enabled:
            self.barrier_active_seen = (
                self.barrier_active_seen
                or bool(debug.get('barrier_active', False)))
            velocity_reduction = max(
                0.0,
                float(debug.get('v_task_i', 0.0))
                - float(debug.get('v_safe_i', 0.0)))
            self.max_reference_velocity_reduction_mps = max(
                self.max_reference_velocity_reduction_mps, velocity_reduction)
            if velocity_reduction > self.intervention_velocity_tolerance_mps:
                if self.reference_intervention_since_s is None:
                    self.reference_intervention_since_s = current_time
                duration = current_time - self.reference_intervention_since_s
                self.max_reference_intervention_duration_s = max(
                    self.max_reference_intervention_duration_s, duration)
            else:
                self.reference_intervention_since_s = None
        else:
            self.reference_intervention_since_s = None

        if self.stage == self.STAGE_RELEASE and self.iebc.enabled:
            stop_barrier = float(debug.get('stop_distance_barrier', math.inf))
            excursion = float(debug.get('release_excursion', 0.0))
            self.min_stop_distance_barrier_m = min(
                self.min_stop_distance_barrier_m, stop_barrier)
            self.max_release_excursion_m = max(
                self.max_release_excursion_m, excursion)
            self.max_recovery_dissipation_slack_w = max(
                self.max_recovery_dissipation_slack_w,
                float(debug.get('recovery_dissipation_slack_w', 0.0)))
            if (not self.recovery_complete_seen
                    and debug.get('mode') == self.iebc.MODE_HOLD):
                self.recovery_complete_seen = True
                self.recovery_complete_time_s = max(
                    0.0, current_time - self.stage_start_s)

            if debug.get('mode') == self.iebc.MODE_HOLD:
                if self.release_settle_anchor_enu is None:
                    self.release_settle_anchor_enu = self.position.copy()
                    self.release_settle_since_s = current_time
                    self.release_position_change_m = 0.0
                else:
                    self.release_position_change_m = float(np.linalg.norm(
                        self.position - self.release_settle_anchor_enu))
                    if (self.release_position_change_m
                            > self.release_settle_position_tol_m):
                        self.release_settle_anchor_enu = self.position.copy()
                        self.release_settle_since_s = current_time
                        self.release_position_change_m = 0.0
                    elif (not self.release_settled
                          and current_time - self.release_settle_since_s
                          >= self.release_settle_hold_s):
                        self.release_settled = True
                        self.release_settle_time_s = max(
                            0.0, current_time - self.stage_start_s)
            else:
                self.release_settle_since_s = None
                self.release_settle_anchor_enu = None
                self.release_position_change_m = math.inf

        raw_force, filtered_force, cube_x = self._latest_contact_sample
        self._write_csv(current_time, raw_force, filtered_force, cube_x)

    def _write_csv(self, current_time: float, raw_force: float, filtered_force: float, cube_x: float) -> None:
        debug = self.iebc.debug
        yaw_error = self._gazebo_yaw_error()
        cube_breakaway_m = (cube_x - self.loaded_cube_x if
                            math.isfinite(cube_x) and math.isfinite(self.loaded_cube_x)
                            else math.nan)
        contact_power_gt_w = filtered_force * float(
            np.dot(self.interaction_axis_enu, self.velocity))
        self._csv.writerow([
            f'{current_time:.6f}', self.stage,
            *(f'{value:.6f}' for value in self.position),
            *(f'{value:.6f}' for value in self.velocity),
            *(f'{value:.6f}' for value in self.target_position),
            f'{math.degrees(self.vehicle_gz_yaw):.6f}',
            f'{math.degrees(self.desired_world_yaw):.6f}',
            f'{math.degrees(self.desired_controller_yaw):.6f}',
            f'{math.degrees(yaw_error):.6f}',
            f'{cube_x:.6f}', f'{raw_force:.6f}', f'{filtered_force:.6f}',
            f'{self.release_force_threshold_n:.6f}', f'{self.release_force_hold_s:.6f}',
            self.release_mode, f'{self.release_time_s:.6f}',
            f'{cube_breakaway_m:.6f}', int(self.virtual_force_active), f'{self.virtual_force_n:.6f}',
            int(self.release_event_seen),
            int(bool(debug.get('active', False))),
            int(bool(debug.get('barrier_active', False))),
            int(bool(debug.get('infeasible', False))),
            int(bool(debug.get('storage_update_enabled', False))),
            int(self.barrier_active_seen),
            f'{self.max_reference_intervention_duration_s:.6f}',
            f'{self.max_reference_velocity_reduction_mps:.6f}',
            f"{debug.get('p_hat', 0.0):.6f}",
            f"{debug.get('p_bar_e', 0.0):.6f}",
            f"{debug.get('s_dot_bar', 0.0):.6f}",
            f"{debug.get('s_bar', 0.0):.6f}",
            f"{debug.get('k_i', 0.0):.6f}",
            f"{debug.get('v_c', 0.0):.6f}",
            f"{debug.get('e_i', 0.0):.6f}",
            f"{debug.get('h_i', 0.0):.6f}",
            f"{debug.get('h_constraint', 0.0):.6f}",
            f"{debug.get('energy_reserve_j', 0.0):.6f}",
            f'{self.min_interaction_barrier_j:.6f}',
            f"{debug.get('e_ref', 0.0):.6f}",
            f"{debug.get('s_nom_i', 0.0):.6f}",
            f"{debug.get('s_safe_i', 0.0):.6f}",
            f"{debug.get('v_i', 0.0):.6f}",
            f"{debug.get('v_nom_i', 0.0):.6f}",
            f"{debug.get('v_task_i', 0.0):.6f}",
            f"{debug.get('v_safe_i', 0.0):.6f}",
            f"{debug.get('g_e', 0.0):.6f}",
            f"{debug.get('pi_e', 0.0):.6f}",
            f"{debug.get('p_allow', 0.0):.6f}",
            f"{debug.get('p_ref_nominal', 0.0):.6f}",
            f"{debug.get('p_ref_safe', 0.0):.6f}",
            f"{debug.get('equivalent_stiffness_force_n', 0.0):.6f}",
            f'{contact_power_gt_w:.6f}',
            f"{debug.get('qp_slack_w', 0.0):.6f}", f'{self.max_qp_slack_w:.6f}',
            f"{debug.get('a_safe_i', 0.0):.6f}",
            str(debug.get('mode', 'disabled')),
            f"{debug.get('recoverable_energy', 0.0):.6f}",
            f"{debug.get('release_excursion', 0.0):.6f}",
            f"{debug.get('stop_distance_barrier', math.inf):.6f}",
            f"{debug.get('reserved_stop_distance', 0.0):.6f}",
            f"{debug.get('rho', 0.0):.6f}",
            f"{debug.get('release_s', math.nan):.6f}",
            int(self.recovery_complete_seen),
            f'{self.recovery_complete_time_s:.6f}',
            f"{debug.get('recovery_dissipation_slack_w', 0.0):.6f}",
            f'{self.release_position_change_m:.6f}',
            int(self.release_settled),
            f'{self.release_settle_time_s:.6f}',
            f'{self.max_recovery_dissipation_slack_w:.6f}',
            str(debug.get('recovery_phase', 'inactive')),
            f"{debug.get('recovery_reference_velocity', 0.0):.6f}",
            int(bool(debug.get('recovery_rate_infeasible', False))),
            f"{debug.get('recovery_terminal_s', math.nan):.6f}",
            f"{debug.get('recovery_stop_candidate_s', 0.0):.6f}",
            int(bool(debug.get('recovery_stop_latched', False))),
            f"{debug.get('recovery_rebase_energy_j', 0.0):.6f}",
        ])
        now_s = time.monotonic()
        if now_s - self._last_csv_flush_s >= 1.0:
            self._csv_file.flush()
            self._last_csv_flush_s = now_s

    def print_status(self):
        super().print_status()
        if self.data_received:
            _, filtered_force, cube_x = self._contact_force(0.02)
            self.get_logger().info(
                f'Cube experiment: stage={self.stage} | contact={filtered_force:.2f} N | '
                f'virtual_load={self.virtual_force_active} | cube_x={cube_x:.3f} m | '
                f'yaw={math.degrees(self.vehicle_gz_yaw):+.1f} deg -> '
                f'{math.degrees(self.desired_world_yaw):+.1f} deg | '
                f'release_seen={self.release_event_seen} | IEBC mode={self.iebc.mode}')

    def destroy_node(self):
        try:
            if self.virtual_force_active:
                self._clear_virtual_force()
        except Exception:
            pass
        try:
            self._csv_file.flush()
            self._csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = HnuterIebcSimulation()
    exit_code = 1
    try:
        while rclpy.ok() and controller.stage not in (
                controller.STAGE_COMPLETE, controller.STAGE_FAILED):
            rclpy.spin_once(controller, timeout_sec=0.1)
        exit_code = 0 if controller.stage == controller.STAGE_COMPLETE else 1
    except KeyboardInterrupt:
        controller.get_logger().info('Cube-contact experiment interrupted.')
        exit_code = 130
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
