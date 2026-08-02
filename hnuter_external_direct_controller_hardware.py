#!/usr/bin/env python3
"""Hardware entry point for the Hnuter direct external controller.

PX4 and the RC transmitter remain responsible for arming and selecting
Offboard.  This node only publishes the Offboard proof-of-life and actuator
setpoints after PX4 reports both states active.  RC sticks are converted to
the same body-velocity, vertical-velocity, and yaw-rate references used by the
debug controller's gamepad path.
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import ManualControlSetpoint, RcChannels

from hnuter_external_direct_controller_debug import (
    GamepadManager,
    HnuterController as DebugController,
)


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return float(default)


@dataclass
class _StickSample:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    throttle: float = 0.0


class RCCommandManager:
    """Convert PX4 RC topics into the debug controller's manual commands."""

    def __init__(
        self,
        max_vxy_body_mps,
        max_vz: float,
        max_yaw_rate: float,
        deadzone: float,
        expo: float,
        filter_tau: float,
        filter_tau_body_xy_s,
        max_acc_body_xy_mps2,
        timeout_s: float = 0.5,
        logger=None,
    ) -> None:
        self.logger = logger
        self.max_vxy = float(np.max(np.asarray(max_vxy_body_mps)))
        self.max_vxy_body_mps = np.asarray(
            max_vxy_body_mps, dtype=float
        ).reshape(2)
        self.max_vz = float(max_vz)
        self.max_yaw_rate = float(max_yaw_rate)
        self.deadzone = float(deadzone)
        self.expo = float(expo)
        self.filter_tau = float(filter_tau)
        self.filter_tau_body_xy_s = np.asarray(
            filter_tau_body_xy_s, dtype=float
        ).reshape(2)
        self.max_acc_body_xy_mps2 = np.asarray(
            max_acc_body_xy_mps2, dtype=float
        ).reshape(2)
        self.timeout_s = max(float(timeout_s), 0.05)
        self.attitude_control_axis = 'roll'

        self.pitch_sign = _env_float('HNUTER_RC_PITCH_SIGN', 1.0)
        self.roll_sign = _env_float('HNUTER_RC_ROLL_SIGN', -1.0)
        self.throttle_sign = _env_float('HNUTER_RC_THROTTLE_SIGN', 1.0)
        self.yaw_sign = _env_float('HNUTER_RC_YAW_SIGN', -1.0)

        self._manual_sample = _StickSample()
        self._manual_valid = False
        self._manual_received_s = -math.inf
        self._channels_sample = _StickSample()
        self._channels_valid = False
        self._channels_received_s = -math.inf
        self._source = 'none'
        self.filtered_cmds = self._zero_commands()

    def _zero_commands(self) -> dict:
        return {
            'raw_vx_b': 0.0,
            'raw_vy_b': 0.0,
            'vx_b': 0.0,
            'vy_b': 0.0,
            'vz': 0.0,
            'yaw_rate': 0.0,
            'roll_rate': 0.0,
            'pitch_rate': 0.0,
            'lt': 0.0,
            'rt': 0.0,
            'rb_pressed': False,
            'attitude_axis': self.attitude_control_axis,
        }

    @staticmethod
    def _finite_sticks(*values: float) -> bool:
        return bool(np.all(np.isfinite(np.asarray(values, dtype=float))))

    def feed_manual_control(self, message) -> None:
        now = time.monotonic()
        sample = _StickSample(
            roll=float(getattr(message, 'roll', math.nan)),
            pitch=float(getattr(message, 'pitch', math.nan)),
            yaw=float(getattr(message, 'yaw', math.nan)),
            throttle=float(getattr(message, 'throttle', math.nan)),
        )
        source = int(getattr(
            message, 'data_source', ManualControlSetpoint.SOURCE_RC
        ))
        self._manual_valid = (
            bool(getattr(message, 'valid', False))
            and source == ManualControlSetpoint.SOURCE_RC
            and self._finite_sticks(
                sample.roll, sample.pitch, sample.yaw, sample.throttle
            )
        )
        if self._manual_valid:
            self._manual_sample = sample
        self._manual_received_s = now

    @staticmethod
    def _mapped_channel(message, function_id: int) -> float | None:
        mapping = tuple(getattr(message, 'function', ()))
        channels = tuple(getattr(message, 'channels', ()))
        channel_count = min(
            int(getattr(message, 'channel_count', 0)), len(channels)
        )
        if not 0 <= function_id < len(mapping):
            return None
        channel_index = int(mapping[function_id])
        if not 0 <= channel_index < channel_count:
            return None
        value = float(channels[channel_index])
        return value if math.isfinite(value) else None

    def feed_rc_channels(self, message) -> None:
        now = time.monotonic()
        roll = self._mapped_channel(message, RcChannels.FUNCTION_ROLL)
        pitch = self._mapped_channel(message, RcChannels.FUNCTION_PITCH)
        yaw = self._mapped_channel(message, RcChannels.FUNCTION_YAW)
        throttle = self._mapped_channel(message, RcChannels.FUNCTION_THROTTLE)
        values = (roll, pitch, yaw, throttle)
        self._channels_valid = (
            not bool(getattr(message, 'signal_lost', True))
            and all(value is not None for value in values)
        )
        if self._channels_valid:
            # rc_channels throttle is normalized to [0, 1]. Recenter it so
            # the physical throttle stick behaves like the gamepad axis.
            self._channels_sample = _StickSample(
                roll=float(roll),
                pitch=float(pitch),
                yaw=float(yaw),
                throttle=2.0 * float(throttle) - 1.0,
            )
        self._channels_received_s = now

    def _active_sample(self) -> tuple[_StickSample, str]:
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
        magnitude = (abs(value) - self.deadzone) / max(
            1.0 - self.deadzone, 1e-6
        )
        magnitude = self.expo * magnitude ** 3 + (1.0 - self.expo) * magnitude
        return math.copysign(magnitude, value)

    def get_velocity_commands(self, dt: float) -> dict:
        previous_source = self._source
        sample, self._source = self._active_sample()
        if self.logger is not None and self._source != previous_source:
            if self._source == 'stale':
                self.logger.warn(
                    'RC input timed out; manual references are returning to zero.'
                )
            else:
                self.logger.info(f'RC input source: {self._source}')
        pitch = self._shape(sample.pitch)
        roll = self._shape(sample.roll)
        throttle = self._shape(sample.throttle)
        yaw = self._shape(sample.yaw)

        target_vx_b = self.pitch_sign * pitch * self.max_vxy_body_mps[0]
        target_vy_b = self.roll_sign * roll * self.max_vxy_body_mps[1]
        target_vz = self.throttle_sign * throttle * self.max_vz
        target_yaw_rate = self.yaw_sign * yaw * self.max_yaw_rate

        self.filtered_cmds['raw_vx_b'] = float(target_vx_b)
        self.filtered_cmds['raw_vy_b'] = float(target_vy_b)
        self.filtered_cmds['vx_b'] = GamepadManager._filter_command(
            self.filtered_cmds['vx_b'], target_vx_b, dt,
            self.filter_tau_body_xy_s[0], self.max_acc_body_xy_mps2[0],
        )
        self.filtered_cmds['vy_b'] = GamepadManager._filter_command(
            self.filtered_cmds['vy_b'], target_vy_b, dt,
            self.filter_tau_body_xy_s[1], self.max_acc_body_xy_mps2[1],
        )
        self.filtered_cmds['vz'] = GamepadManager._filter_command(
            self.filtered_cmds['vz'], target_vz, dt, self.filter_tau
        )
        self.filtered_cmds['yaw_rate'] = GamepadManager._filter_command(
            self.filtered_cmds['yaw_rate'], target_yaw_rate, dt,
            self.filter_tau,
        )
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


class OffboardTaskRestartTracker:
    """Remember only a task interrupted by an Offboard falling edge."""

    def __init__(self) -> None:
        self._task: str | None = None
        self._reentered = False

    def observe(self, was_offboard: bool, is_offboard: bool, task: str) -> None:
        if was_offboard and not is_offboard:
            self._task = task if task != 'hover' else None
            self._reentered = False
        elif not was_offboard and is_offboard:
            self._reentered = True

    def consume(self) -> str | None:
        if not self._reentered:
            return None
        task = self._task
        self._task = None
        self._reentered = False
        return task


class HnuterHardwareController(DebugController):
    """Direct controller gated exclusively by PX4 Arm and Offboard states."""

    def _node_name(self):
        return 'hnuter_controller_direct_hardware'

    def __init__(self) -> None:
        self._hardware_control_active = False
        self._restart_tracker = OffboardTaskRestartTracker()
        super().__init__()

        try:
            self.gamepad.close()
        except Exception:
            pass
        self.rc_input = RCCommandManager(
            max_vxy_body_mps=self.gamepad_max_vxy_body_mps,
            max_vz=_env_float('HNUTER_RC_MAX_VZ_MPS', 0.30),
            max_yaw_rate=_env_float('HNUTER_RC_MAX_YAW_RATE_RPS', 0.40),
            deadzone=self.gamepad_deadzone,
            expo=self.gamepad_expo,
            filter_tau=self.gamepad_filter_tau_s,
            filter_tau_body_xy_s=self.gamepad_filter_tau_body_xy_s,
            max_acc_body_xy_mps2=self.gamepad_max_acc_body_xy_mps2,
            timeout_s=_env_float('HNUTER_RC_TIMEOUT_S', 0.50),
            logger=self.get_logger(),
        )
        # Reuse the established manual trajectory path without duplicating its
        # filtering, position-lead limiting, or yaw integration.
        self.gamepad = self.rc_input

        qos_rc = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.manual_control_sub = self.create_subscription(
            ManualControlSetpoint,
            '/fmu/out/manual_control_setpoint',
            self.manual_control_callback,
            qos_rc,
        )
        self.rc_channels_sub = self.create_subscription(
            RcChannels,
            '/fmu/out/rc_channels',
            self.rc_channels_callback,
            qos_rc,
        )

        self.auto_arm_enabled = False
        self.arm_after_takeoff_request = False
        self.takeoff_requested = False
        self.takeoff_height = 0.0
        self.attitude_test_altitude_m = 0.0
        self.takeoff_tilt_suppress_time_s = 0.0
        self.takeoff_xy_lock_time_s = 0.0
        self.direct_takeoff_vertical_only_time_s = 0.0
        self.preflight_tilt_test_enabled = False
        self.get_logger().warn(
            'HARDWARE MODE: 本节点不会 Arm/Disarm，也不会切换 Offboard。'
            '请使用遥控器完成解锁和模式切换；Arm 与 Offboard 同时有效后在当前位置接管。'
        )

    def _apply_tuning(self, data: dict):
        super()._apply_tuning(data)
        # A task may change attitude or position, but hardware startup must
        # never inject an altitude step merely because the node started.
        self.attitude_test_altitude_m = 0.0

    def manual_control_callback(self, message) -> None:
        self.rc_input.feed_manual_control(message)

    def rc_channels_callback(self, message) -> None:
        self.rc_input.feed_rc_channels(message)

    def status_callback(self, message):
        super().status_callback(message)
        self._update_hardware_control_gate()

    def control_mode_callback(self, message):
        was_offboard = self.is_offboard()
        interrupted_task = self.auto_traj_mode
        super().control_mode_callback(message)
        is_offboard = self.is_offboard()
        self._restart_tracker.observe(
            was_offboard, is_offboard, interrupted_task
        )
        self._update_hardware_control_gate()

    def _begin_hardware_control(self) -> None:
        self._hardware_control_active = True
        self.takeoff_requested = True
        self.manual_pos_initialized = False
        self.pending_auto_traj_mode = self._restart_tracker.consume()
        self.auto_traj_mode = 'hover'
        self.integral_pos_error[:] = 0.0
        self.integral_e_R[:] = 0.0
        self._takeoff_lock_start_time_s = None
        self._takeoff_start_z_rel = 0.0
        self._xy_lock_active = False
        self.manual_des_roll = 0.0
        self.manual_des_pitch = 0.0
        self.sim_start_time_s = 0.0
        self._last_timestamp_s = 0.0
        restart_text = (
            f'，将从当前位置重新开始任务 {self.pending_auto_traj_mode}'
            if self.pending_auto_traj_mode else ''
        )
        self.get_logger().info(
            f'检测到 Armed + Offboard，实机外部控制已接管{restart_text}。'
        )

    def _end_hardware_control(self) -> None:
        if self._hardware_control_active:
            self.get_logger().warn(
                'Armed 或 Offboard 条件失效，外部执行会话已停止。'
            )
        self._hardware_control_active = False
        self.takeoff_requested = False
        self.manual_pos_initialized = False
        self.pending_auto_traj_mode = None
        self.auto_traj_mode = 'hover'
        self.integral_pos_error[:] = 0.0
        self.integral_e_R[:] = 0.0
        self._last_manual_cmd = self._zero_manual_cmd()

    def _update_hardware_control_gate(self) -> None:
        should_control = bool(
            self.data_received and self.armed and self.is_offboard()
        )
        if should_control and not self._hardware_control_active:
            self._begin_hardware_control()
        elif not should_control and self._hardware_control_active:
            self._end_hardware_control()

    def offboard_startup_tick(self):
        # Required proof-of-life only. There are deliberately no VehicleCommand
        # publications in this hardware startup path.
        self.publish_offboard_control_mode()
        self._update_hardware_control_gate()
        if (
            self.data_received
            and not self._hardware_control_active
            and not self.armed
        ):
            self.publish_idle_direct_actuator_setpoint()

    def arm(self):
        self.get_logger().warn('实机模式忽略程序 Arm 请求，请使用遥控器。')

    def disarm(self):
        self.get_logger().warn('实机模式忽略程序 Disarm 请求，请使用遥控器。')

    def set_offboard_mode(self):
        self.get_logger().warn('实机模式忽略程序 Offboard 请求，请使用遥控器开关。')

    def poll_keyboard_commands(self):
        for key in self.keyboard.get_commands():
            if key in ('o', 'O'):
                self.get_logger().warn(
                    '实机模式下键盘 o 无效；Arm 与 Offboard 只能由遥控器/PX4 控制。'
                )
            elif key == '1':
                self.pending_auto_traj_mode = 'rectangle'
                self.get_logger().info('矩形轨迹已排队。')
            elif key == '2':
                self.pending_auto_traj_mode = 'lissajous'
                self.get_logger().info('三维李萨如轨迹已排队。')
            elif key == '3':
                self.pending_auto_traj_mode = 'attitude'
                self.get_logger().info('姿态角轨迹已排队。')

    def _trajectory_ready(self, current_time: float) -> bool:
        del current_time
        return bool(
            self._hardware_control_active
            and self.is_offboard()
            and self.armed
            and self.manual_pos_initialized
        )

    def _start_auto_trajectory(self, mode: str, current_time: float):
        # The debug implementation raises attitude tests to a configured test
        # altitude. Hardware mode starts every requested task at current height.
        configured_altitude = self.attitude_test_altitude_m
        self.attitude_test_altitude_m = float(self.manual_des_pos[2])
        try:
            super()._start_auto_trajectory(mode, current_time)
        finally:
            self.attitude_test_altitude_m = configured_altitude

    def control_loop(self):
        self._update_hardware_control_gate()
        if not self._hardware_control_active:
            return
        super().control_loop()

    def _diagnostic_file_prefix(self):
        return 'hnuter_direct_hardware'

    def _diagnostic_extra_header(self):
        return ['rc_source', 'rc_age_s', 'rc_valid', 'hardware_control_active']

    def _diagnostic_extra_values(self):
        rc_input = getattr(self, 'rc_input', None)
        if rc_input is None:
            return ['none', math.inf, 0, 0]
        return [
            rc_input.source,
            rc_input.age_s,
            int(rc_input.valid),
            int(self._hardware_control_active),
        ]

    def print_status(self):
        super().print_status()
        rc_input = getattr(self, 'rc_input', None)
        if rc_input is not None:
            self.get_logger().info(
                f'Hardware gate={self._hardware_control_active} | '
                f'RC source={rc_input.source} | age={rc_input.age_s:.3f}s | '
                f'valid={rc_input.valid}'
            )


def main(args=None):
    rclpy.init(args=args)
    controller = HnuterHardwareController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('实机外部控制节点已停止。')
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
