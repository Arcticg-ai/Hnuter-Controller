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
* ``/hnuter/iebc/in/actuator_wrench`` (geometry_msgs/WrenchStamped): actual
  actuator force estimate in the ENU world frame.  This is not contact force
  and not a commanded wrench.
* ``/hnuter/iebc/in/recovery`` (std_msgs/Bool): a rising ``True`` edge marks
  physical load release and enters the certified stopping controller.
* ``/hnuter/iebc/in/reset`` (std_msgs/Empty): reset IEBC storage and reference
  state at the current flight-session origin.

Outputs are the inherited PX4 Offboard heartbeat and trajectory-setpoint
topics plus ``/hnuter/iebc/out/status`` (std_msgs/String, JSON).

For hardware, IEBC requires external wrench reconstruction.  The Gazebo-only
software proxy is rejected whenever IEBC is enabled.  Stale nominal commands
or stale force estimates latch a zero-velocity hold instead of failing open.
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
from px4_msgs.msg import RcChannels, TrajectorySetpoint
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String

from hnuter_external_controller_px4_position_hardware import (
    HnuterController as ValidatedHardwareController,
)
from hnuter_external_controller_px4_position_hardware_iebc_closed_loop import (
    InteractionEnergyBarrierFilter,
)


def env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name, '1' if default else '0')
    return str(raw).strip().lower() in ('1', 'true', 'yes', 'on')


def env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


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

        self.command_timeout_s = max(
            env_float('HNUTER_IEBC_COMMAND_TIMEOUT_S', 0.30), 0.05)
        self.initial_command_radius_m = max(
            env_float('HNUTER_IEBC_INITIAL_COMMAND_RADIUS_M', 0.75), 0.0)
        self.require_explicit_enu_frame = env_bool(
            'HNUTER_IEBC_REQUIRE_WRENCH_FRAME', True)
        self.nominal_topic = os.environ.get(
            'HNUTER_IEBC_NOMINAL_TOPIC', self.DEFAULT_NOMINAL_TOPIC)
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
            f'wrench={self.wrench_topic}, recovery={self.recovery_topic}, '
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

    def _wrench_age_s(self, now_s: Optional[float] = None) -> float:
        now_s = time.monotonic() if now_s is None else float(now_s)
        return now_s - self._external_force_received_s

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
            self._failsafe_reason = 'task_start_rejected_actuator_wrench_stale'
            self.get_logger().warn(
                'Task switch ignored: actual actuator wrench is missing or stale')
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
            self._latch_current_hold('actuator_wrench_stale_during_task')
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
        if self._wrench_age_s() > self.iebc.wrench_timeout_s:
            return None
        return self._external_force_enu.copy()

    def _filter_current_reference(self, dt: float) -> bool:
        if not self.iebc.enabled:
            return True
        actuator_force = self._fresh_external_force()
        if actuator_force is None:
            self._latch_current_hold('actuator_wrench_stale')
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
            self._latch_current_hold('actuator_wrench_stale')
            return
        if not self._activate_topic_reference(self._nominal_reference):
            return

        self._failsafe_hold_latched = False
        self._failsafe_reason = ''
        self._set_topic_nominal_reference(self._nominal_reference)
        self._filter_current_reference(dt)

    def publish_iebc_status(self) -> None:
        debug = self.iebc.debug
        payload = {
            'hardware_gate_active': bool(self._hardware_control_active),
            'nominal_source': self.nominal_source,
            'nominal_age_s': self._reference_age_s(),
            'wrench_age_s': self._wrench_age_s(),
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
