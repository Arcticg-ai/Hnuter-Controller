#!/usr/bin/env python3
"""Standalone hardware direct-actuator controller for Hnuter.

The transmitter and PX4 retain exclusive Arm, Disarm, and Offboard mode
authority. The node runs continuously while Position mode performs the manual
takeoff, starts control only while PX4 reports Armed + Offboard, captures the
measured pose at the mode transition, and converts physical RC sticks to the
established body-velocity and yaw-rate references.

This file intentionally contains its controller helpers, RC parser, logging
paths, and task restart state machine so it can run without importing another
repository-local Python module.
"""

import sys
import os
import time
import math
import queue
import csv
import json
import select
import termios
import threading
import tty
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import numpy as np

def workspace_root() -> Path:
    return Path(__file__).resolve().parent


def log_root() -> Path:
    root = Path(os.environ.get('HNUTER_LOG_DIR', workspace_root() / 'hnuter_logs')).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    return root


def log_path(*parts: str) -> Path:
    path = log_root().joinpath(*parts)
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def stamp() -> str:
    return time.strftime('%Y%m%d_%H%M%S')


def configure_ros_log_dir() -> Path:
    ros_dir = log_path('ros', '.keep').parent
    os.environ.setdefault('ROS_LOG_DIR', str(ros_dir))
    return ros_dir


def diagnostic_csv_path(prefix: str) -> Path:
    return log_path('external_control', f'{prefix}_{int(time.time())}.csv')


def tuning_csv_path(prefix: str) -> Path:
    return log_path('tuning', f'{prefix}_{stamp()}.csv')


def update_attitude_axis_toggle(
    current_axis: str,
    rb_pressed: bool,
    rb_was_pressed: bool,
) -> tuple[str, bool, bool]:
    """Toggle roll/pitch control once on each RB rising edge."""
    axis = 'pitch' if str(current_axis).lower() == 'pitch' else 'roll'
    pressed = bool(rb_pressed)
    toggled = pressed and not bool(rb_was_pressed)
    if toggled:
        axis = 'pitch' if axis == 'roll' else 'roll'
    return axis, pressed, toggled


def large_tilt_yaw_scale(
    tilt_rad: float,
    start_rad: float,
    full_rad: float,
    minimum_scale: float,
) -> float:
    """Smoothly reduce yaw authority as the vehicle approaches a large tilt."""
    minimum = float(np.clip(minimum_scale, 0.0, 1.0))
    start = max(float(start_rad), 0.0)
    full = max(float(full_rad), start + 1e-6)
    progress = float(np.clip((abs(float(tilt_rad)) - start) / (full - start), 0.0, 1.0))
    smooth_progress = progress * progress * (3.0 - 2.0 * progress)
    return 1.0 - (1.0 - minimum) * smooth_progress


def reduced_tilt_attitude_error(
    desired_rotation: np.ndarray,
    current_rotation: np.ndarray,
    full_error: np.ndarray,
    antipodal_start: float = -0.80,
    antipodal_full: float = -0.98,
) -> tuple[np.ndarray, float, float]:
    """Prioritize thrust-axis alignment while retaining half-turn recovery.

    Away from the antipodal singularity, the first two components align the
    current body-Z axis with the desired body-Z axis. Near a 180-degree tilt,
    they blend back to the nonsingular full quaternion error.
    """
    desired = np.asarray(desired_rotation, dtype=float)
    current = np.asarray(current_rotation, dtype=float)
    error = np.asarray(full_error, dtype=float).reshape(3).copy()
    if desired.shape != (3, 3) or current.shape != (3, 3):
        raise ValueError('desired_rotation and current_rotation must be 3x3')

    desired_z = desired[:, 2]
    current_z = current[:, 2]
    alignment = float(np.clip(np.dot(desired_z, current_z), -1.0, 1.0))
    reduced_error = current.T @ np.cross(desired_z, current_z)

    start = float(np.clip(antipodal_start, -1.0, 1.0))
    full = min(float(antipodal_full), start - 1e-6)
    progress = float(np.clip((start - alignment) / (start - full), 0.0, 1.0))
    full_error_blend = progress * progress * (3.0 - 2.0 * progress)
    error[:2] = (
        (1.0 - full_error_blend) * reduced_error[:2]
        + full_error_blend * error[:2]
    )
    return error, alignment, full_error_blend


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """Return a normalized scalar-first quaternion for a rotation matrix."""
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError('rotation must have shape (3, 3)')

    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(max(trace + 1.0, 0.0))
        quaternion = np.array([
            0.25 * scale,
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
        ])
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = 2.0 * np.sqrt(
                max(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2], 0.0)
            )
            quaternion = np.array([
                (matrix[2, 1] - matrix[1, 2]) / scale,
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
            ])
        elif index == 1:
            scale = 2.0 * np.sqrt(
                max(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2], 0.0)
            )
            quaternion = np.array([
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
            ])
        else:
            scale = 2.0 * np.sqrt(
                max(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1], 0.0)
            )
            quaternion = np.array([
                (matrix[1, 0] - matrix[0, 1]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
            ])

    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError('rotation produced a degenerate quaternion')
    return quaternion / norm


def quaternion_attitude_error(
    desired_rotation: np.ndarray,
    current_rotation: np.ndarray,
    previous_quaternion: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Return a nonsingular body-frame attitude error and its angle.

    The vector error is ``2 * q_xyz`` for the shortest relative quaternion.
    Unlike the usual skew-symmetric SO(3) error, its magnitude remains nonzero
    at 180 degrees. ``previous_quaternion`` resolves the sign exactly at the
    half-turn boundary.
    """
    desired = np.asarray(desired_rotation, dtype=float)
    current = np.asarray(current_rotation, dtype=float)
    if desired.shape != (3, 3) or current.shape != (3, 3):
        raise ValueError('desired_rotation and current_rotation must be 3x3')

    quaternion = rotation_matrix_to_quaternion(desired.T @ current)
    epsilon = 1e-9
    if quaternion[0] < -epsilon:
        quaternion = -quaternion
    elif abs(float(quaternion[0])) <= epsilon and previous_quaternion is not None:
        previous = np.asarray(previous_quaternion, dtype=float).reshape(4)
        if float(np.dot(quaternion, previous)) < 0.0:
            quaternion = -quaternion

    vector_norm = float(np.linalg.norm(quaternion[1:]))
    angle_rad = 2.0 * float(np.arctan2(vector_norm, abs(float(quaternion[0]))))
    return 2.0 * quaternion[1:], quaternion, angle_rad

# PX4 uses fixed DDS topic names. Keep SITL telemetry local unless remote DDS
# access is explicitly requested, otherwise another PX4 on the LAN can mix in.
if os.environ.get('HNUTER_ALLOW_REMOTE_DDS', '0') != '1':
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_STATIC_PEERS', None)
configure_ros_log_dir()

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float64

from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleAngularVelocity
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import ActuatorServos
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleCommandAck
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleLandDetected
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleThrustSetpoint
from px4_msgs.msg import VehicleTorqueSetpoint
from px4_msgs.msg import ManualControlSetpoint
from px4_msgs.msg import RcChannels

try:
    import pygame
except Exception:  # 允许没有手柄/没有 pygame 时保持悬停
    pygame = None


def env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default

    try:
        return float(raw)
    except ValueError:
        return default


# ============================================================
# 手柄管理器：从 hnuter104.py 移植，加入异常保护
# ============================================================
class GamepadManager:
    def __init__(self,
                 max_vxy: float = 1.0,
                 max_vz: float = 0.5,
                 max_yaw_rate: float = 0.6,
                 max_roll_rate: float = math.radians(20.0),
                 max_pitch_rate: float = math.radians(20.0),
                 deadzone: float = 0.10,
                 expo: float = 0.40,
                 filter_tau: float = 0.20,
                 max_vxy_body_mps=None,
                 filter_tau_body_xy_s=None,
                 max_acc_body_xy_mps2=None,
                 lt_axis: int = 2,
                 rt_axis: int = 5,
                 rb_button: int = 5,
                 attitude_axis_toggle_enabled: bool = False,
                 trigger_mode: str = 'minus_one_to_one',
                 logger=None):
        self.logger = logger
        self.joystick = None
        self.max_vxy = float(max_vxy)
        self.max_vz = float(max_vz)
        self.max_yaw_rate = float(max_yaw_rate)
        self.max_roll_rate = float(max_roll_rate)
        self.max_pitch_rate = float(max_pitch_rate)
        self.deadzone = float(deadzone)
        self.expo = float(expo)
        self.filter_tau = float(filter_tau)
        self.max_vxy_body_mps = self._axis_pair(
            max_vxy_body_mps, self.max_vxy
        )
        self.filter_tau_body_xy_s = self._axis_pair(
            filter_tau_body_xy_s, self.filter_tau
        )
        self.max_acc_body_xy_mps2 = self._axis_pair(
            max_acc_body_xy_mps2, float('inf')
        )
        self.lt_axis = int(lt_axis)
        self.rt_axis = int(rt_axis)
        self.rb_button = int(rb_button)
        self.attitude_axis_toggle_enabled = bool(attitude_axis_toggle_enabled)
        self.attitude_control_axis = 'roll'
        self._rb_was_pressed = False
        # 常见 Xbox/XInput 手柄 LT/RT: 未按=-1，按满=+1。
        # 若你的手柄是未按=0，按满=1，把 trigger_mode 改为 'zero_to_one'。
        # 若你的手柄是未按=+1，按满=-1，把 trigger_mode 改为 'one_to_minus_one'。
        self.trigger_mode = str(trigger_mode)
        self.filtered_cmds = {
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

        if pygame is None:
            self._log_warn('未导入 pygame，手柄不可用，控制器将保持悬停。')
            return

        try:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self._log_info(f'🎮 成功连接控制外设: {self.joystick.get_name()}')
            else:
                self._log_warn('⚠️ 未检测到手柄，控制器将保持悬停。')
        except Exception as exc:
            self._log_warn(f'⚠️ 手柄初始化失败: {exc}，控制器将保持悬停。')
            self.joystick = None

    @staticmethod
    def _axis_pair(value, fallback: float) -> np.ndarray:
        if value is None:
            return np.full(2, float(fallback), dtype=float)
        array = np.asarray(value, dtype=float).reshape(-1)
        if array.size != 2:
            return np.full(2, float(fallback), dtype=float)
        return array.copy()

    @staticmethod
    def _filter_command(
        current: float,
        target: float,
        dt: float,
        filter_tau_s: float,
        max_rate: float = float('inf'),
    ) -> float:
        dt = max(float(dt), 0.0)
        tau = max(float(filter_tau_s), 0.0)
        alpha = dt / (tau + dt) if tau > 1e-3 else 1.0
        delta = float(np.clip(alpha, 0.0, 1.0)) * (
            float(target) - float(current)
        )
        if np.isfinite(max_rate):
            max_delta = max(float(max_rate), 0.0) * dt
            delta = float(np.clip(delta, -max_delta, max_delta))
        return float(current) + delta

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

    def close(self):
        if pygame is not None:
            try:
                pygame.quit()
            except Exception:
                pass

    def _apply_deadzone(self, val: float) -> float:
        return float(val) if abs(float(val)) > self.deadzone else 0.0

    def _apply_expo(self, val: float) -> float:
        return self.expo * (val ** 3) + (1.0 - self.expo) * val

    def _trigger_to_unit(self, raw: float) -> float:
        """将 LT/RT 原始轴值转换为 [0, 1]，并施加死区与 EXPO。"""
        raw = float(raw)
        if self.trigger_mode == 'zero_to_one':
            val = raw
        elif self.trigger_mode == 'one_to_minus_one':
            val = 0.5 * (1.0 - raw)
        else:
            # 默认 Xbox/XInput: -1 未按，+1 按满
            val = 0.5 * (raw + 1.0)

        val = float(np.clip(val, 0.0, 1.0))
        if val <= self.deadzone:
            return 0.0

        # 把死区之后的行程重新归一化到 [0, 1]
        val = (val - self.deadzone) / max(1.0 - self.deadzone, 1e-6)
        return float(np.clip(self._apply_expo(val), 0.0, 1.0))

    def get_velocity_commands(self, dt: float) -> dict:
        if pygame is None or self.joystick is None:
            return self.filtered_cmds.copy()

        try:
            pygame.event.pump()
            num_axes = self.joystick.get_numaxes()

            # Xbox/PS 常用轴映射：0 左摇杆左右；1 左摇杆上下；3 右摇杆左右；4 右摇杆上下
            raw_yaw = self.joystick.get_axis(0) if num_axes > 0 else 0.0
            raw_throttle = self.joystick.get_axis(1) if num_axes > 1 else 0.0
            raw_roll = self.joystick.get_axis(3) if num_axes > 3 else 0.0
            raw_pitch = self.joystick.get_axis(4) if num_axes > 4 else 0.0
            raw_lt = self.joystick.get_axis(self.lt_axis) if num_axes > self.lt_axis else -1.0
            raw_rt = self.joystick.get_axis(self.rt_axis) if num_axes > self.rt_axis else -1.0
            rb_pressed = False
            if self.attitude_axis_toggle_enabled:
                num_buttons = self.joystick.get_numbuttons()
                rb_pressed = bool(
                    self.joystick.get_button(self.rb_button)
                    if 0 <= self.rb_button < num_buttons else False
                )
                (
                    self.attitude_control_axis,
                    self._rb_was_pressed,
                    attitude_axis_toggled,
                ) = update_attitude_axis_toggle(
                    self.attitude_control_axis,
                    rb_pressed,
                    self._rb_was_pressed,
                )
                if attitude_axis_toggled:
                    self.filtered_cmds['roll_rate'] = 0.0
                    self.filtered_cmds['pitch_rate'] = 0.0
                    axis_text = '俯仰' if self.attitude_control_axis == 'pitch' else '横滚'
                    self._log_info(f'RB 姿态控制轴切换为：{axis_text}')

            yaw_expo = self._apply_expo(self._apply_deadzone(raw_yaw))
            thr_expo = self._apply_expo(self._apply_deadzone(raw_throttle))
            roll_expo = self._apply_expo(self._apply_deadzone(raw_roll))
            pitch_expo = self._apply_expo(self._apply_deadzone(raw_pitch))
            lt_expo = self._trigger_to_unit(raw_lt)
            rt_expo = self._trigger_to_unit(raw_rt)

            # FLU 机体系：x 前，y 左，z 上；上推为正向前/上升
            target_vx_b = -pitch_expo * self.max_vxy_body_mps[0]
            target_vy_b = -roll_expo * self.max_vxy_body_mps[1]
            target_vz_w = -thr_expo * self.max_vz
            target_yaw_rate = -yaw_expo * self.max_yaw_rate

            # LT/RT 控制 RB 当前选中的姿态轴。
            trigger_direction = lt_expo - rt_expo
            target_roll_rate = (
                trigger_direction * self.max_roll_rate
                if self.attitude_control_axis == 'roll' else 0.0
            )
            target_pitch_rate = (
                trigger_direction * self.max_pitch_rate
                if self.attitude_control_axis == 'pitch' else 0.0
            )

            self.filtered_cmds['raw_vx_b'] = float(target_vx_b)
            self.filtered_cmds['raw_vy_b'] = float(target_vy_b)
            self.filtered_cmds['vx_b'] = self._filter_command(
                self.filtered_cmds['vx_b'],
                target_vx_b,
                dt,
                self.filter_tau_body_xy_s[0],
                self.max_acc_body_xy_mps2[0],
            )
            self.filtered_cmds['vy_b'] = self._filter_command(
                self.filtered_cmds['vy_b'],
                target_vy_b,
                dt,
                self.filter_tau_body_xy_s[1],
                self.max_acc_body_xy_mps2[1],
            )
            self.filtered_cmds['vz'] = self._filter_command(
                self.filtered_cmds['vz'], target_vz_w, dt, self.filter_tau
            )
            self.filtered_cmds['yaw_rate'] = self._filter_command(
                self.filtered_cmds['yaw_rate'],
                target_yaw_rate,
                dt,
                self.filter_tau,
            )
            self.filtered_cmds['roll_rate'] = self._filter_command(
                self.filtered_cmds['roll_rate'],
                target_roll_rate,
                dt,
                self.filter_tau,
            )
            self.filtered_cmds['pitch_rate'] = self._filter_command(
                self.filtered_cmds['pitch_rate'],
                target_pitch_rate,
                dt,
                self.filter_tau,
            )
            self.filtered_cmds['lt'] = lt_expo
            self.filtered_cmds['rt'] = rt_expo
            self.filtered_cmds['rb_pressed'] = rb_pressed
            self.filtered_cmds['attitude_axis'] = self.attitude_control_axis
            return self.filtered_cmds.copy()
        except Exception as exc:
            self._log_warn(f'读取手柄失败: {exc}，本周期保持上一指令。')
            return self.filtered_cmds.copy()


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
            self._log_info('键盘已启用：按 o 起飞悬停；按 1/2/3 分别执行矩形/李萨如/姿态角轨迹。')
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


class HnuterController(Node):
    def __init__(self):
        super().__init__(self._node_name())

        qos_profile_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_profile_command = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.actuator_motors_pub = self.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', qos_profile_in)
        self.actuator_servos_pub = self.create_publisher(
            ActuatorServos, '/fmu/in/actuator_servos', qos_profile_in)
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile_command)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile_command)
        self.vehicle_thrust_setpoint_pub = self.create_publisher(
            VehicleThrustSetpoint, '/fmu/in/vehicle_thrust_setpoint', qos_profile_in)
        self.vehicle_torque_setpoint_pub = self.create_publisher(
            VehicleTorqueSetpoint, '/fmu/in/vehicle_torque_setpoint', qos_profile_in)
        self.vehicle_command_pub = None
        if self._vehicle_command_publication_enabled():
            self.vehicle_command_pub = self.create_publisher(
                VehicleCommand, '/fmu/in/vehicle_command', qos_profile_command)

        self.gz_servo0_pub = self.create_publisher(Float64, '/model/hnuter_0/servo_0', 10)
        self.gz_servo1_pub = self.create_publisher(Float64, '/model/hnuter_0/servo_1', 10)
        self.gz_servo2_pub = self.create_publisher(Float64, '/model/hnuter_0/servo_2', 10)
        self.gz_servo3_pub = self.create_publisher(Float64, '/model/hnuter_0/servo_3', 10)
        self.publish_gz_servos_direct = False

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_position_callback, qos_profile_out)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile_out)
        self.angular_velocity_sub = self.create_subscription(
            VehicleAngularVelocity, '/fmu/out/vehicle_angular_velocity', self.angular_velocity_callback, qos_profile_out)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_callback, qos_profile_out)
        self.vehicle_control_mode_sub = self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self.control_mode_callback, qos_profile_out)
        self.vehicle_land_detected_sub = self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.land_detected_callback, qos_profile_out)
        self.vehicle_command_ack_sub = self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.vehicle_command_ack_callback, qos_profile_out)
        self.px4_actuator_motors_out_sub = self.create_subscription(
            ActuatorMotors, '/fmu/out/actuator_motors', self.px4_actuator_motors_callback, qos_profile_out)
        self.px4_actuator_servos_out_sub = self.create_subscription(
            ActuatorServos, '/fmu/out/actuator_servos', self.px4_actuator_servos_callback, qos_profile_out)

        # PX4 常量，兼容不同 px4_msgs 版本
        self.CMD_DO_SET_MODE = getattr(VehicleCommand, 'VEHICLE_CMD_DO_SET_MODE', 176)
        self.CMD_COMPONENT_ARM_DISARM = getattr(VehicleCommand, 'VEHICLE_CMD_COMPONENT_ARM_DISARM', 400)
        self.NAVIGATION_STATE_OFFBOARD = getattr(VehicleStatus, 'NAVIGATION_STATE_OFFBOARD', 14)
        self.ARMING_STATE_ARMED = getattr(VehicleStatus, 'ARMING_STATE_ARMED', 2)
        self.command_ack_result_names = {
            getattr(VehicleCommandAck, 'VEHICLE_CMD_RESULT_ACCEPTED', 0): 'ACCEPTED',
            getattr(VehicleCommandAck, 'VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED', 1): 'TEMPORARILY_REJECTED',
            getattr(VehicleCommandAck, 'VEHICLE_CMD_RESULT_DENIED', 2): 'DENIED',
            getattr(VehicleCommandAck, 'VEHICLE_CMD_RESULT_UNSUPPORTED', 3): 'UNSUPPORTED',
            getattr(VehicleCommandAck, 'VEHICLE_CMD_RESULT_FAILED', 4): 'FAILED',
            getattr(VehicleCommandAck, 'VEHICLE_CMD_RESULT_IN_PROGRESS', 5): 'IN_PROGRESS',
            getattr(VehicleCommandAck, 'VEHICLE_CMD_RESULT_CANCELLED', 6): 'CANCELLED',
        }

        # State variables
        self.position = np.zeros(3)       # ENU: x East, y North, z Up
        self.velocity = np.zeros(3)       # ENU
        self.attitude_q = np.array([1.0, 0.0, 0.0, 0.0])
        self.angular_velocity = np.zeros(3)  # FLU body angular velocity
        self.angular_velocity_frd = np.zeros(3)
        self.R = np.eye(3)                # ENU <- FLU
        self.R_ned_frd = np.eye(3)
        self.R_ned_frd_raw = np.eye(3)
        self._attitude_axis_transform = np.eye(3)
        self._attitude_canonical_initialized = False
        self.nav_state = None
        self.control_offboard_enabled = False
        self.armed = False
        self._armed_from_control_mode = False
        self.land_detected = {
            'landed': True,
            'maybe_landed': True,
            'ground_contact': True,
            'freefall': False,
            'has_low_throttle': True,
        }
        self.data_received = False
        self.local_position_received = False
        self.attitude_received = False
        self.px4_timestamp = 0

        # Offboard/Arm 启动状态机
        self.offboard_setpoint_counter = 0
        self._last_offboard_cmd_time = 0.0
        self._last_arm_cmd_time = 0.0
        self._offboard_request_sent = False
        self._arm_request_sent = False

        # ====== 启动策略配置：防止 PX4 自动 disarm 后被程序反复 arm ======
        # True : 节点启动后自动尝试切 Offboard，并只自动 Arm 一次。
        # False: 节点只维持 OffboardControlMode 心跳，需要你用 QGC/遥控器手动 Arm。
        self.auto_arm_enabled = True

        # 强烈建议 False。PX4 如果因为预起飞超时、落地检测或 failsafe disarm，
        # 程序不应立刻再次解锁，否则会出现“反复 arm / 反复起落”的循环。
        self.rearm_after_auto_disarm = False

        # 自动 Arm 最多尝试次数。调试期建议 1；若想完全手动解锁，设 auto_arm_enabled=False。
        self.max_auto_arm_attempts = 1
        self.arm_after_takeoff_request = os.environ.get('HNUTER_ARM_AFTER_O', '1').strip().lower() not in (
            '0', 'false', 'no', 'off'
        )
        self.auto_arm_attempts = 0
        self.was_armed_once = False
        self._last_armed_state = False
        self._optimistic_armed_until = 0.0
        self._last_arm_disarm_command_is_arm = None
        self.startup_blocked_after_disarm = False
        self.preflight_disarm_waiting_for_o = False

        # PX4 要求进入 Offboard 前先连续发送 >1s 的 OffboardControlMode。
        # 这里 20Hz * 30 = 1.5s，留出裕量。
        self.offboard_warmup_ticks = 30
        self.mode_request_period_s = 1.0
        self.arm_request_period_s = 1.0

        # Debug variables
        self.last_motor_cmd = np.zeros(12)
        self.last_servo_cmd = np.zeros(8)
        self.px4_out_motor_cmd = np.full(12, np.nan)
        self.px4_out_servo_cmd = np.full(8, np.nan)
        self.px4_out_motor_timestamp = 0
        self.px4_out_servo_timestamp = 0
        self.last_F1 = 0.0
        self.last_F2 = 0.0
        self.last_F3 = 0.0
        self.control_loop_count = 0
        self.last_W = np.zeros(6)
        self._last_manual_cmd = {
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
            'attitude_axis': 'roll',
        }

        # Physical parameters
        self.mass = 4.5
        self.gravity = 9.81
        self.J = np.diag([0.2456, 0.1276, 0.3264])
        self.l1 = 0.33
        self.l2 = 0.664
        self.allow_tail_reverse = os.environ.get('HNUTER_ALLOW_TAIL_REVERSE', '1').strip().lower() in (
            '1', 'true', 'yes', 'on'
        )
        self.allocator_force_x_sign = env_float('HNUTER_ALLOCATOR_FORCE_X_SIGN', 1.0)
        self.allocator_force_y_sign = env_float('HNUTER_ALLOCATOR_FORCE_Y_SIGN', -1.0)
        # Match the no-delay PX4 main-branch Hnuter allocator parameters.
        # Pitch bias is normalized torque, not a raw Motor5 command offset.
        # The 2026-08-04 hardware flight used 0.09. Airframe observation shows
        # that a smaller value lowers the tail, so start the next test at 0.10.
        self.pitch_torque_bias = float(np.clip(
            env_float(
                'HNUTER_PITCH_BIAS',
                env_float('HNTR_PITCH_BIAS', 0.10),
            ),
            -1.0,
            1.0,
        ))
        self.tail_torque_sign = (
            -1.0
            if env_float(
                'HNUTER_TAIL_SIGN',
                env_float('HNTR_TAIL_SIGN', 1.0),
            ) < 0.0
            else 1.0
        )
        self.tail_collective_comp = float(np.clip(
            env_float(
                'HNUTER_TAIL_COMP',
                env_float('HNTR_TAIL_COMP', 0.0),
            ),
            0.0,
            1.0,
        ))

        # Default to the firmware/profile actually used by flight log 113.
        # The newer 3131ddd4 mapping remains available in a separate config.
        self.hardware_firmware_profile = os.environ.get(
            'HNUTER_HARDWARE_FIRMWARE_PROFILE',
            '3131ddd4_500_2500_gear2',
        ).strip()
        self.primary_servo_angle_max_rad = math.radians(max(
            1.0, env_float('HNUTER_PRIMARY_SERVO_MAX_DEG', 180.0)
        ))
        self.secondary_servo_angle_max_rad = math.radians(max(
            1.0, env_float('HNUTER_SECONDARY_SERVO_MAX_DEG', 180.0)
        ))
        self.secondary_servo_gear_ratio = float(np.clip(
            env_float(
                'HNUTER_S2_GEAR',
                env_float('HNTR_S2_GEAR', 2.0),
            ),
            0.1,
            10.0,
        ))
        self.servo_pwm_min_us = int(np.clip(
            env_float('HNUTER_SERVO_PWM_MIN_US', 500.0), 500.0, 1500.0
        ))
        self.servo_pwm_trim_us = int(np.clip(
            env_float('HNUTER_SERVO_PWM_TRIM_US', 1500.0), 500.0, 2500.0
        ))
        self.servo_pwm_max_us = int(np.clip(
            env_float('HNUTER_SERVO_PWM_MAX_US', 2500.0), 1500.0, 2500.0
        ))

        # Actuator limits are physical joint limits, not servo-shaft limits.
        self.pitch_command_limit_rad = np.radians(180.0)
        self.alpha_limit_rad = min(
            math.radians(abs(env_float('HNUTER_ALPHA_LIMIT_DEG', 180.0))),
            self.primary_servo_angle_max_rad,
        )
        self.theta_limit_rad = min(
            math.radians(abs(env_float('HNUTER_THETA_LIMIT_DEG', 90.0))),
            self.secondary_servo_angle_max_rad / self.secondary_servo_gear_ratio,
        )
        self.servo_rate_limit_rad_s = 50.0
        self.takeoff_tilt_suppress_time_s = 1.0
        self.takeoff_tilt_limit_rad = np.radians(20.0)
        self.takeoff_xy_lock_time_s = 3.0
        self.xy_lock_max_acc_xy = 3.0
        self.xy_lock_tilt_limit_rad = np.radians(30.0)
        self._xy_lock_initialized = False
        self._xy_lock_position = np.zeros(2)
        self._xy_lock_active = False
        self._takeoff_lock_start_time_s = None
        self._takeoff_start_z_rel = 0.0
        self._last_control_dt_s = 0.0

        # 不再永久 hover_only，否则会覆盖手柄 XY 目标点
        self.hover_only = False

        # Yaw variables
        self._yaw_initialized = False
        self.initial_yaw = 0.0

        self._alpha1_cmd = 0.0
        self._alpha2_cmd = 0.0
        self._theta1_cmd = 0.0
        self._theta2_cmd = 0.0

        self.integral_pos_error = np.zeros(3)
        self.integral_e_R = np.zeros(3)
        self._attitude_error_quaternion = None
        self.last_attitude_error = np.zeros(3)
        self.last_attitude_error_angle_rad = 0.0
        self.last_omega_error = np.zeros(3)
        self.last_tau_c = np.zeros(3)
        self.last_yaw_authority_scale = 1.0
        self.last_thrust_axis_alignment = 1.0
        self.last_full_error_blend = 0.0

        # Direct-actuator mode cannot rely on PX4's inner-loop guards. Keep
        # takeoff torque gentle so a small pitch error cannot saturate tail
        # thrust and flip the aircraft before the altitude loop settles.
        self.direct_takeoff_KR = np.array([1.5, 1.5, 1.5])
        self.direct_takeoff_Domega = np.array([1.2, 1.2, 1.2])
        self.direct_xy_lock_KR = np.array([1.5, 1.5, 1.5])
        self.direct_xy_lock_Domega = np.array([1.2, 1.2, 1.2])
        self.direct_KR = np.array([2.1, 2.1, env_float('HNUTER_DIRECT_KR_YAW', 4.2)])
        self.direct_Domega = np.array([1.4, 1.4, env_float('HNUTER_DIRECT_DOMEGA_YAW', 2.6)])
        self.direct_attitude_Ki = np.array([0.15, 0.18, 0.50])
        self.direct_attitude_integral_limit = np.array([0.6, 0.6, 0.4])
        self.direct_attitude_integral_activation_error_rad = math.radians(35.0)
        self.direct_quaternion_error_enabled = True
        self.direct_attitude_gyro_compensation_enabled = True
        self.direct_reduced_tilt_error_enabled = True
        self.direct_large_tilt_yaw_scheduling_enabled = True
        self.direct_large_tilt_yaw_start_rad = math.radians(45.0)
        self.direct_large_tilt_yaw_full_rad = math.radians(80.0)
        self.direct_large_tilt_yaw_min_scale = 0.10
        self.direct_takeoff_tau_limit = np.array([0.90, 0.90, 0.50])
        self.direct_xy_lock_tau_limit = np.array([0.90, 0.90, 0.50])
        self.direct_tau_limit = np.array([0.90, 0.90, env_float('HNUTER_DIRECT_TAU_YAW_LIMIT', 1.80)])
        self.direct_yaw_control_enabled = os.environ.get(
            'HNUTER_DIRECT_YAW_CONTROL', '1'
        ).strip().lower() in ('1', 'true', 'yes', 'on')
        self.direct_takeoff_vertical_only_time_s = self.takeoff_tilt_suppress_time_s
        self.direct_takeoff_vertical_only_height_m = 0.0
        self.direct_takeoff_vertical_only_height_enabled = True
        self.direct_takeoff_thrust_floor_enabled = True

        self.max_acc_xy = 20.0
        self.max_acc_z = 20.0
        # Position-loop arrays use NED axis order: north, east, down.
        self.direct_pos_Kp_ned = np.array([3.0, 3.0, 8.0])
        self.direct_pos_Kd_ned = np.array([2.1, 2.1, 4.0])
        self.direct_pos_Ki_ned = np.array([0.0, 0.0, 3.0])
        self.direct_pos_integral_limit_ned = np.array([1.0, 1.0, 2.0])
        self.max_climb_rate = 0.35
        self.manual_max_position_lead_xy = 0.6
        self.manual_max_position_lead_z = 0.45
        self.manual_max_yaw_lead_rad = np.radians(25.0)
        self.gamepad_max_vxy_mps = env_float(
            'HNUTER_PAD_MAX_VXY_MPS', 0.6
        )
        self.gamepad_deadzone = env_float(
            'HNUTER_PAD_DEADZONE', 0.10
        )
        self.gamepad_expo = env_float('HNUTER_PAD_EXPO', 0.40)
        self.gamepad_filter_tau_s = env_float(
            'HNUTER_PAD_FILTER_TAU_S', 0.25
        )
        self.gamepad_max_vxy_body_mps = np.full(
            2, self.gamepad_max_vxy_mps, dtype=float
        )
        self.gamepad_filter_tau_body_xy_s = np.full(
            2, self.gamepad_filter_tau_s, dtype=float
        )
        self.gamepad_max_acc_body_xy_mps2 = np.array([1.0, 0.70])
        # Direct debug: 按 o 后不交给 PX4 位置控制器，而是直接发布 actuator_motors/servos。
        # 若要用同一份日志结构记录 PX4 position baseline，启动前设置 HNUTER_CONTROL_MODE=px4。
        control_mode_env = os.environ.get('HNUTER_CONTROL_MODE', 'direct').strip().lower()
        self.use_px4_position_takeoff = control_mode_env in ('px4', 'px4_position', 'position')
        self.debug_control_mode = 'px4_position' if self.use_px4_position_takeoff else 'direct'
        # Direct mode should have a single actuator source. Keep actuator_motors/servos
        # external, but still publish thrust setpoint so PX4 land_detector does not
        # treat direct actuator flight as low-throttle landed flight.
        self.publish_land_detector_thrust_setpoint = os.environ.get(
            'HNUTER_DIRECT_PUBLISH_THRUST_SETPOINT', '1'
        ).strip().lower() in ('1', 'true', 'yes', 'on')
        # Only enable this for allocator comparison: torque setpoints can wake PX4
        # ControlAllocator, which is not the normal direct-actuator path.
        self.publish_allocator_setpoints_in_direct = os.environ.get(
            'HNUTER_DIRECT_PUBLISH_ALLOCATOR_SETPOINTS', '0'
        ).strip().lower() in ('1', 'true', 'yes', 'on')
        self.direct_safety_shutdown_enabled = os.environ.get(
            'HNUTER_DIRECT_SAFETY_SHUTDOWN', '0'
        ).strip().lower() in ('1', 'true', 'yes', 'on')
        self.direct_safety_attitude_check_enabled = os.environ.get(
            'HNUTER_DIRECT_SAFETY_ATTITUDE_CHECK', '0'
        ).strip().lower() in ('1', 'true', 'yes', 'on')
        self.direct_prearm_level_check_enabled = os.environ.get(
            'HNUTER_DIRECT_PREARM_LEVEL_CHECK', '0'
        ).strip().lower() in ('1', 'true', 'yes', 'on')
        self.direct_safety_cutoff = False
        self.direct_safety_cutoff_reason = ''
        self.direct_safety_pitch_limit_rad = np.radians(env_float('HNUTER_DIRECT_SAFETY_ATTITUDE_LIMIT_DEG', 55.0))
        self.direct_prearm_level_limit_rad = np.radians(env_float('HNUTER_DIRECT_PREARM_LEVEL_DEG', 20.0))
        self.direct_safety_speed_xy_limit = 8.0
        self._last_direct_safety_log_time = 0.0
        self._last_prearm_reject_log_time = 0.0

        self.target_position = np.array([0.0, 0.0, 1.3])
        self.target_velocity = np.zeros(3)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([0.0, 0.0, 0.0])
        self.target_attitude_rate = np.zeros(3)
        self.target_R_des_ned_frd = None

        self.takeoff_height = 1.3
        self.max_altitude = 5.0
        self.min_altitude = 0.25
        self.manual_enabled = True
        self.takeoff_requested = False
        self.manual_pos_initialized = False
        self.manual_des_pos = np.zeros(3)   # [x_enu, y_enu, z_relative]
        self.manual_des_yaw = 0.0
        # LT/RT 积分得到当前 RB 所选轴的姿态期望，初始控制横滚。
        # 正号沿当前 ENU 姿态约定；若实机观察方向相反，
        # 只需要在 GamepadManager 中反转 trigger_direction。
        self.manual_des_roll = 0.0
        self.manual_roll_limit_rad = np.radians(90.0)
        self.manual_des_pitch = 0.0
        # The primary tilt has +/-185 deg travel, leaving margin at a +/-180 deg attitude command.
        self.manual_pitch_limit_rad = np.radians(180.0)
        self._z0_initialized = False
        self._z0 = 0.0
        self._z_sp = 0.0

        # 解锁进入 Offboard 后，默认先不产生升力，只做地面倾转自检。
        # 按键盘 o 后才允许进入起飞/悬停控制。
        self.preflight_tilt_test_enabled = os.environ.get(
            'HNUTER_PREFLIGHT_TILT_TEST', '0'
        ).strip().lower() in ('1', 'true', 'yes', 'on')
        self.preflight_tilt_test_started = False
        self.preflight_tilt_test_finished = False
        self.preflight_tilt_start_time_s = None
        self.preflight_tilt_axis_duration_s = 2.0
        self.preflight_tilt_amplitude_rad = math.radians(8.0)

        # Keyboard-triggered auto trajectories. 轨迹在当前 yaw 坐标系下生成，位置仍发布为 ENU。
        self.auto_traj_mode = 'hover'
        self.pending_auto_traj_mode = None
        self.auto_traj_start_time = 0.0
        self.auto_traj_start_pos = np.zeros(3)
        self.auto_traj_origin_xy = np.zeros(2)
        self.auto_traj_z = self.takeoff_height
        self.auto_traj_yaw = 0.0
        self.auto_traj_start_attitude = np.zeros(3)
        self.auto_traj_ready_margin = 0.08
        self.rectangle_size_x = 2.0
        self.rectangle_size_y = 1.5
        self.rectangle_segment_time_s = 5.0
        self.lissajous_amp_x = env_float('HNUTER_LISSAJOUS_AMP_X_M', 1.0)
        self.lissajous_amp_y = env_float('HNUTER_LISSAJOUS_AMP_Y_M', 0.75)
        self.lissajous_amp_z = env_float('HNUTER_LISSAJOUS_AMP_Z_M', 0.35)
        self.lissajous_a = max(1, int(env_float('HNUTER_LISSAJOUS_FREQ_X', 2)))
        self.lissajous_b = max(1, int(env_float('HNUTER_LISSAJOUS_FREQ_Y', 3)))
        self.lissajous_c = max(1, int(env_float('HNUTER_LISSAJOUS_FREQ_Z', 1)))
        self.lissajous_period_s = max(
            8.0, env_float('HNUTER_LISSAJOUS_PERIOD_S', 24.0)
        )
        # Trajectory 3 validates large-attitude holding: roll +/-90 deg, pitch +/-180 deg, no yaw step.
        # Per-axis environment variables and the live tuning JSON can still override these defaults.
        self.attitude_step_angle_rad = math.radians(env_float('HNUTER_ATTITUDE_STEP_DEG', 180.0))
        self.attitude_step_axis_rad = np.array([
            math.radians(env_float('HNUTER_ATTITUDE_STEP_ROLL_DEG', 90.0)),
            math.radians(env_float('HNUTER_ATTITUDE_STEP_PITCH_DEG', 180.0)),
            math.radians(env_float('HNUTER_ATTITUDE_STEP_YAW_DEG', 0.0)),
        ], dtype=float)
        self.attitude_step_axis_rad[1] = float(np.clip(
            self.attitude_step_axis_rad[1],
            -self.pitch_command_limit_rad,
            self.pitch_command_limit_rad,
        ))
        self.attitude_segment_time_s = env_float('HNUTER_ATTITUDE_SEGMENT_S', 5.0)
        self.attitude_peak_hold_s = env_float('HNUTER_ATTITUDE_PEAK_HOLD_S', 1.0)
        self.attitude_test_bidirectional = True
        self.attitude_test_altitude_only = os.environ.get(
            'HNUTER_ATTITUDE_TEST_ALTITUDE_ONLY', '0'
        ).strip().lower() in ('1', 'true', 'yes', 'on')
        self.attitude_test_max_acc_xy = env_float('HNUTER_ATTITUDE_TEST_MAX_ACC_XY', 3.0)
        self.attitude_test_altitude_m = env_float('HNUTER_ATTITUDE_TEST_ALTITUDE_M', self.takeoff_height)

        default_tuning_path = (
            Path(__file__).resolve().parent
            / 'config'
            / self._default_tuning_filename()
        )
        self.tuning_path = os.path.expanduser(os.environ.get(
            'HNUTER_TUNING_FILE', str(default_tuning_path)
        ))
        self.tuning_status_path = Path(self.tuning_path).with_suffix(
            '.applied.json'
        )
        self._last_tuning_mtime_ns = None
        self._last_tuning_log_time = 0.0
        self._write_tuning_file_if_missing()
        self._load_tuning_file(force=True)

        # Time
        self.sim_start_time_s = 0.0
        self._last_timestamp_s = 0.0

        # Timers: Offboard heartbeat should be comfortably > 2 Hz
        self.offboard_timer = self.create_timer(0.05, self.offboard_startup_tick)
        self.status_timer = self.create_timer(1.0, self.print_status)
        self.debug_print_period_s = 1.0
        self._last_debug_print_time = 0.0

        self.gamepad = self._create_manual_input()
        self.keyboard = KeyboardCommandReader(logger=self.get_logger())
        self.keyboard_timer = self.create_timer(0.1, self.poll_keyboard_commands)
        self.tuning_timer = self.create_timer(0.5, self._load_tuning_file)

        # CSV diagnostics
        self.diagnostic_enabled = True
        self.diagnostic_period_s = 0.10
        self._last_diagnostic_log_time = -1.0
        self.diagnostic_path = diagnostic_csv_path(self._diagnostic_file_prefix())
        self._diagnostic_file = None
        self._diagnostic_writer = None
        if self.diagnostic_enabled:
            self._diagnostic_file = self.diagnostic_path.open('w', newline='', buffering=1)
            self._diagnostic_writer = csv.writer(self._diagnostic_file)
            self._diagnostic_writer.writerow(self._diagnostic_header())

        self.get_logger().info(
            f'Hnuter direct controller core initialized: mode={self.debug_control_mode}'
        )
        if self.use_px4_position_takeoff:
            self.get_logger().info(
                f'PX4 baseline 记录模式：使用 PX4 position Offboard；诊断日志写入 {self.diagnostic_path}'
            )
        else:
            self.get_logger().warn(
                f'DIRECT 模式：px4_equiv actuator direct；诊断日志写入 {self.diagnostic_path}'
            )

    def _create_manual_input(self):
        return GamepadManager(
            max_vxy=self.gamepad_max_vxy_mps,
            max_vz=0.3,
            max_yaw_rate=0.4,
            max_roll_rate=math.radians(20.0),
            max_pitch_rate=math.radians(20.0),
            deadzone=self.gamepad_deadzone,
            expo=self.gamepad_expo,
            filter_tau=self.gamepad_filter_tau_s,
            max_vxy_body_mps=self.gamepad_max_vxy_body_mps,
            filter_tau_body_xy_s=self.gamepad_filter_tau_body_xy_s,
            max_acc_body_xy_mps2=self.gamepad_max_acc_body_xy_mps2,
            lt_axis=2,
            rt_axis=5,
            rb_button=int(env_float('HNUTER_PAD_RB_BUTTON', 5)),
            attitude_axis_toggle_enabled=self._gamepad_attitude_axis_toggle_enabled(),
            trigger_mode='minus_one_to_one',
            logger=self.get_logger()
        )

    def _node_name(self):
        return 'hnuter_controller_direct_debug'

    def _default_tuning_filename(self):
        return 'hnuter_direct_tuning.json'

    def _vehicle_command_publication_enabled(self):
        return True

    def _gamepad_attitude_axis_toggle_enabled(self):
        return False

    def _direct_position_acceleration_ned(
        self,
        acc_ff_ned: np.ndarray,
        pos_error_ned: np.ndarray,
        vel_error_ned: np.ndarray,
        xy_lock_active: bool,
    ) -> np.ndarray:
        kp = self.direct_pos_Kp_ned.copy()
        if xy_lock_active:
            kp[:2] *= 0.8
        return (
            acc_ff_ned
            + kp * pos_error_ned
            + self.direct_pos_Kd_ned * vel_error_ned
            + self.direct_pos_Ki_ned * self.integral_pos_error
        )

    def _limit_direct_horizontal_acceleration_ned(
        self,
        acceleration_xy_ned: np.ndarray,
        max_acc_xy: float,
    ) -> np.ndarray:
        return np.clip(acceleration_xy_ned, -max_acc_xy, max_acc_xy)

    def _apply_direct_body_force_trim(self, force_body: np.ndarray) -> np.ndarray:
        return force_body

    # ============================================================
    # Live tuning
    # ============================================================
    def _tuning_snapshot(self) -> dict:
        return {
            "attitude_step_angle_deg": float(math.degrees(self.attitude_step_angle_rad)),
            "attitude_step_roll_deg": float(math.degrees(self.attitude_step_axis_rad[0])),
            "attitude_step_pitch_deg": float(math.degrees(self.attitude_step_axis_rad[1])),
            "attitude_step_yaw_deg": float(math.degrees(self.attitude_step_axis_rad[2])),
            "attitude_segment_time_s": float(self.attitude_segment_time_s),
            "attitude_peak_hold_s": float(self.attitude_peak_hold_s),
            "attitude_test_bidirectional": bool(self.attitude_test_bidirectional),
            "attitude_test_altitude_only": bool(self.attitude_test_altitude_only),
            "attitude_test_altitude_m": float(self.attitude_test_altitude_m),
            "attitude_test_max_acc_xy": float(self.attitude_test_max_acc_xy),
            "alpha_limit_deg": float(math.degrees(self.alpha_limit_rad)),
            "theta_limit_deg": float(math.degrees(self.theta_limit_rad)),
            "hardware_firmware_profile": self.hardware_firmware_profile,
            "primary_servo_angle_max_deg": float(
                math.degrees(self.primary_servo_angle_max_rad)
            ),
            "secondary_servo_angle_max_deg": float(
                math.degrees(self.secondary_servo_angle_max_rad)
            ),
            "HNTR_S2_GEAR": float(self.secondary_servo_gear_ratio),
            "servo_pwm_min_us": int(self.servo_pwm_min_us),
            "servo_pwm_trim_us": int(self.servo_pwm_trim_us),
            "servo_pwm_max_us": int(self.servo_pwm_max_us),
            "manual_roll_limit_deg": float(math.degrees(self.manual_roll_limit_rad)),
            "manual_pitch_limit_deg": float(math.degrees(self.manual_pitch_limit_rad)),
            "direct_safety_attitude_check_enabled": bool(self.direct_safety_attitude_check_enabled),
            "direct_safety_attitude_limit_deg": float(math.degrees(self.direct_safety_pitch_limit_rad)),
            "allocator_force_x_sign": float(self.allocator_force_x_sign),
            "allocator_force_y_sign": float(self.allocator_force_y_sign),
            "HNTR_PITCH_BIAS": float(self.pitch_torque_bias),
            "HNTR_TAIL_SIGN": float(self.tail_torque_sign),
            "HNTR_TAIL_COMP": float(self.tail_collective_comp),
            "direct_KR": self.direct_KR.tolist(),
            "direct_Domega": self.direct_Domega.tolist(),
            "direct_attitude_Ki": self.direct_attitude_Ki.tolist(),
            "direct_attitude_integral_limit": self.direct_attitude_integral_limit.tolist(),
            "direct_attitude_integral_activation_error_deg": float(
                math.degrees(self.direct_attitude_integral_activation_error_rad)
            ),
            "direct_quaternion_error_enabled": bool(self.direct_quaternion_error_enabled),
            "direct_attitude_gyro_compensation_enabled": bool(
                self.direct_attitude_gyro_compensation_enabled
            ),
            "direct_reduced_tilt_error_enabled": bool(
                self.direct_reduced_tilt_error_enabled
            ),
            "direct_large_tilt_yaw_scheduling_enabled": bool(
                self.direct_large_tilt_yaw_scheduling_enabled
            ),
            "direct_large_tilt_yaw_start_deg": float(
                math.degrees(self.direct_large_tilt_yaw_start_rad)
            ),
            "direct_large_tilt_yaw_full_deg": float(
                math.degrees(self.direct_large_tilt_yaw_full_rad)
            ),
            "direct_large_tilt_yaw_min_scale": float(
                self.direct_large_tilt_yaw_min_scale
            ),
            "direct_tau_limit": self.direct_tau_limit.tolist(),
            "direct_takeoff_KR": self.direct_takeoff_KR.tolist(),
            "direct_takeoff_Domega": self.direct_takeoff_Domega.tolist(),
            "direct_takeoff_tau_limit": self.direct_takeoff_tau_limit.tolist(),
            "direct_xy_lock_KR": self.direct_xy_lock_KR.tolist(),
            "direct_xy_lock_Domega": self.direct_xy_lock_Domega.tolist(),
            "direct_xy_lock_tau_limit": self.direct_xy_lock_tau_limit.tolist(),
            "direct_pos_Kp_ned": self.direct_pos_Kp_ned.tolist(),
            "direct_pos_Kd_ned": self.direct_pos_Kd_ned.tolist(),
            "direct_pos_Ki_ned": self.direct_pos_Ki_ned.tolist(),
            "direct_pos_integral_limit_ned": self.direct_pos_integral_limit_ned.tolist(),
            "manual_max_position_lead_xy": float(
                self.manual_max_position_lead_xy
            ),
            "gamepad_max_vxy_mps": float(self.gamepad_max_vxy_mps),
            "gamepad_max_vxy_body_mps": self.gamepad_max_vxy_body_mps.tolist(),
            "gamepad_deadzone": float(self.gamepad_deadzone),
            "gamepad_expo": float(self.gamepad_expo),
            "gamepad_filter_tau_s": float(self.gamepad_filter_tau_s),
            "gamepad_filter_tau_body_xy_s":
                self.gamepad_filter_tau_body_xy_s.tolist(),
            "gamepad_max_acc_body_xy_mps2":
                self.gamepad_max_acc_body_xy_mps2.tolist(),
            "rc_attitude_rate_deg_s": np.degrees(getattr(
                self, 'rc_attitude_rate_rad_s', np.radians([20.0, 20.0])
            )).tolist(),
            "rc_attitude_angle_limit_deg": float(math.degrees(getattr(
                self, 'rc_attitude_angle_limit_rad', math.radians(45.0)
            ))),
            "rc_attitude_sign": np.asarray(getattr(
                self, 'rc_attitude_sign', np.array([-1.0, -1.0])
            ), dtype=float).tolist(),
            "max_acc_xy": float(self.max_acc_xy),
            "max_acc_z": float(self.max_acc_z),
            "xy_lock_max_acc_xy": float(self.xy_lock_max_acc_xy),
            "direct_yaw_control_enabled": bool(self.direct_yaw_control_enabled),
        }

    def _write_tuning_file_if_missing(self):
        path = Path(self.tuning_path)
        if path.exists():
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', encoding='utf-8') as f:
            json.dump(self._tuning_snapshot(), f, indent=2, sort_keys=True)
            f.write('\n')

    @staticmethod
    def _tuning_array(data: dict, key: str, current: np.ndarray) -> np.ndarray:
        value = data.get(key)
        if value is None:
            return current
        array = np.asarray(value, dtype=float).reshape(-1)
        if array.size != current.size:
            return current
        return array

    @staticmethod
    def _tuning_float(data: dict, key: str, current: float) -> float:
        value = data.get(key)
        if value is None:
            return float(current)
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(current)

    @staticmethod
    def _tuning_bool(data: dict, key: str, current: bool) -> bool:
        value = data.get(key)
        if value is None:
            return bool(current)
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    @staticmethod
    def _tuning_string(data: dict, key: str, current: str) -> str:
        value = data.get(key)
        if value is None:
            return str(current)
        value = str(value).strip()
        return value if value else str(current)

    @staticmethod
    def _bumpless_integral_gain_change(
        integral_state: np.ndarray,
        previous_gain: np.ndarray,
        new_gain: np.ndarray,
        integral_limit: np.ndarray,
    ) -> np.ndarray:
        state = np.asarray(integral_state, dtype=float).copy()
        previous_gain = np.asarray(previous_gain, dtype=float)
        new_gain = np.asarray(new_gain, dtype=float)
        enabled = new_gain > 1e-8
        retained = enabled & (previous_gain > 1e-8)
        state[~enabled] = 0.0
        state[enabled & ~retained] = 0.0
        state[retained] *= previous_gain[retained] / new_gain[retained]
        return np.clip(state, -integral_limit, integral_limit)

    def _update_attitude_integral(
        self,
        attitude_error: np.ndarray,
        attitude_error_angle: float,
        attitude_ki: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        previous = self.integral_e_R.copy()
        enabled = np.asarray(attitude_ki) > 1e-8
        self.integral_e_R[~enabled] = 0.0
        previous[~enabled] = 0.0
        if attitude_error_angle <= self.direct_attitude_integral_activation_error_rad:
            self.integral_e_R[enabled] += np.asarray(attitude_error)[enabled] * dt
        else:
            self.integral_e_R[enabled] *= math.exp(-dt / 0.5)
        self.integral_e_R = np.clip(
            self.integral_e_R,
            -self.direct_attitude_integral_limit,
            self.direct_attitude_integral_limit,
        )
        return previous

    def _reject_saturating_attitude_integration(
        self,
        previous_integral: np.ndarray,
        attitude_ki: np.ndarray,
        unconstrained_torque: np.ndarray,
        torque_limit: np.ndarray,
    ) -> bool:
        integral_torque_change = -np.asarray(attitude_ki) * (
            self.integral_e_R - np.asarray(previous_integral)
        )
        pushing_positive = (
            unconstrained_torque > torque_limit
        ) & (integral_torque_change > 0.0)
        pushing_negative = (
            unconstrained_torque < -torque_limit
        ) & (integral_torque_change < 0.0)
        reject = pushing_positive | pushing_negative
        if np.any(reject):
            self.integral_e_R[reject] = np.asarray(previous_integral)[reject]
            return True
        return False

    def _apply_tuning(self, data: dict):
        self.attitude_step_angle_rad = math.radians(
            self._tuning_float(data, 'attitude_step_angle_deg', math.degrees(self.attitude_step_angle_rad))
        )
        self.attitude_step_axis_rad = np.array([
            math.radians(self._tuning_float(data, 'attitude_step_roll_deg', math.degrees(self.attitude_step_angle_rad))),
            math.radians(self._tuning_float(data, 'attitude_step_pitch_deg', math.degrees(self.attitude_step_angle_rad))),
            math.radians(self._tuning_float(data, 'attitude_step_yaw_deg', math.degrees(self.attitude_step_angle_rad))),
        ], dtype=float)
        self.attitude_step_axis_rad[1] = float(np.clip(
            self.attitude_step_axis_rad[1],
            -self.pitch_command_limit_rad,
            self.pitch_command_limit_rad,
        ))
        self.attitude_segment_time_s = self._tuning_float(data, 'attitude_segment_time_s', self.attitude_segment_time_s)
        self.attitude_peak_hold_s = max(
            0.0,
            self._tuning_float(data, 'attitude_peak_hold_s', self.attitude_peak_hold_s),
        )
        self.attitude_test_bidirectional = self._tuning_bool(
            data,
            'attitude_test_bidirectional',
            self.attitude_test_bidirectional,
        )
        self.attitude_test_altitude_only = self._tuning_bool(
            data, 'attitude_test_altitude_only', self.attitude_test_altitude_only
        )
        self.attitude_test_altitude_m = self._tuning_float(
            data, 'attitude_test_altitude_m', self.attitude_test_altitude_m
        )
        self.attitude_test_max_acc_xy = self._tuning_float(data, 'attitude_test_max_acc_xy', self.attitude_test_max_acc_xy)
        self.hardware_firmware_profile = self._tuning_string(
            data,
            'hardware_firmware_profile',
            self.hardware_firmware_profile,
        )
        self.primary_servo_angle_max_rad = math.radians(max(
            1.0,
            self._tuning_float(
                data,
                'primary_servo_angle_max_deg',
                math.degrees(self.primary_servo_angle_max_rad),
            ),
        ))
        self.secondary_servo_angle_max_rad = math.radians(max(
            1.0,
            self._tuning_float(
                data,
                'secondary_servo_angle_max_deg',
                math.degrees(self.secondary_servo_angle_max_rad),
            ),
        ))
        self.secondary_servo_gear_ratio = float(np.clip(
            self._tuning_float(
                data, 'HNTR_S2_GEAR', self.secondary_servo_gear_ratio
            ),
            0.1,
            10.0,
        ))
        pwm_min = int(np.clip(
            self._tuning_float(data, 'servo_pwm_min_us', self.servo_pwm_min_us),
            500.0,
            1500.0,
        ))
        pwm_trim = int(np.clip(
            self._tuning_float(data, 'servo_pwm_trim_us', self.servo_pwm_trim_us),
            500.0,
            2500.0,
        ))
        pwm_max = int(np.clip(
            self._tuning_float(data, 'servo_pwm_max_us', self.servo_pwm_max_us),
            1500.0,
            2500.0,
        ))
        if not pwm_min < pwm_trim < pwm_max:
            raise ValueError(
                'servo PWM calibration must satisfy min < trim < max'
            )
        self.servo_pwm_min_us = pwm_min
        self.servo_pwm_trim_us = pwm_trim
        self.servo_pwm_max_us = pwm_max
        requested_alpha_limit_rad = abs(math.radians(
            self._tuning_float(
                data, 'alpha_limit_deg', math.degrees(self.alpha_limit_rad)
            )
        ))
        requested_theta_limit_rad = abs(math.radians(
            self._tuning_float(
                data, 'theta_limit_deg', math.degrees(self.theta_limit_rad)
            )
        ))
        self.alpha_limit_rad = min(
            requested_alpha_limit_rad,
            self.primary_servo_angle_max_rad,
        )
        self.theta_limit_rad = min(
            requested_theta_limit_rad,
            self.secondary_servo_angle_max_rad / self.secondary_servo_gear_ratio,
        )
        requested_manual_roll_limit = abs(math.radians(
            self._tuning_float(data, 'manual_roll_limit_deg', math.degrees(self.manual_roll_limit_rad))
        ))
        self.manual_roll_limit_rad = min(requested_manual_roll_limit, math.radians(90.0))
        requested_manual_pitch_limit = abs(math.radians(
            self._tuning_float(data, 'manual_pitch_limit_deg', math.degrees(self.manual_pitch_limit_rad))
        ))
        self.manual_pitch_limit_rad = min(requested_manual_pitch_limit, self.pitch_command_limit_rad)
        self.direct_safety_attitude_check_enabled = self._tuning_bool(
            data, 'direct_safety_attitude_check_enabled', self.direct_safety_attitude_check_enabled
        )
        self.direct_safety_pitch_limit_rad = math.radians(
            self._tuning_float(
                data,
                'direct_safety_attitude_limit_deg',
                math.degrees(self.direct_safety_pitch_limit_rad),
            )
        )
        self.allocator_force_x_sign = self._tuning_float(
            data, 'allocator_force_x_sign', self.allocator_force_x_sign
        )
        self.allocator_force_y_sign = self._tuning_float(
            data, 'allocator_force_y_sign', self.allocator_force_y_sign
        )
        self.pitch_torque_bias = float(np.clip(
            self._tuning_float(
                data, 'HNTR_PITCH_BIAS', self.pitch_torque_bias
            ),
            -1.0,
            1.0,
        ))
        self.tail_torque_sign = (
            -1.0
            if self._tuning_float(
                data, 'HNTR_TAIL_SIGN', self.tail_torque_sign
            ) < 0.0
            else 1.0
        )
        self.tail_collective_comp = float(np.clip(
            self._tuning_float(
                data, 'HNTR_TAIL_COMP', self.tail_collective_comp
            ),
            0.0,
            1.0,
        ))

        self.direct_KR = self._tuning_array(data, 'direct_KR', self.direct_KR)
        self.direct_Domega = self._tuning_array(data, 'direct_Domega', self.direct_Domega)
        previous_attitude_ki = self.direct_attitude_Ki.copy()
        self.direct_attitude_Ki = np.maximum(
            self._tuning_array(data, 'direct_attitude_Ki', self.direct_attitude_Ki),
            0.0,
        )
        self.direct_attitude_integral_limit = np.maximum(
            self._tuning_array(
                data,
                'direct_attitude_integral_limit',
                self.direct_attitude_integral_limit,
            ),
            0.0,
        )
        self.integral_e_R = self._bumpless_integral_gain_change(
            self.integral_e_R,
            previous_attitude_ki,
            self.direct_attitude_Ki,
            self.direct_attitude_integral_limit,
        )
        self.direct_attitude_integral_activation_error_rad = math.radians(max(
            0.0,
            self._tuning_float(
                data,
                'direct_attitude_integral_activation_error_deg',
                math.degrees(self.direct_attitude_integral_activation_error_rad),
            ),
        ))
        self.direct_quaternion_error_enabled = self._tuning_bool(
            data,
            'direct_quaternion_error_enabled',
            self.direct_quaternion_error_enabled,
        )
        self.direct_attitude_gyro_compensation_enabled = self._tuning_bool(
            data,
            'direct_attitude_gyro_compensation_enabled',
            self.direct_attitude_gyro_compensation_enabled,
        )
        self.direct_reduced_tilt_error_enabled = self._tuning_bool(
            data,
            'direct_reduced_tilt_error_enabled',
            self.direct_reduced_tilt_error_enabled,
        )
        self.direct_large_tilt_yaw_scheduling_enabled = self._tuning_bool(
            data,
            'direct_large_tilt_yaw_scheduling_enabled',
            self.direct_large_tilt_yaw_scheduling_enabled,
        )
        self.direct_large_tilt_yaw_start_rad = math.radians(max(
            0.0,
            self._tuning_float(
                data,
                'direct_large_tilt_yaw_start_deg',
                math.degrees(self.direct_large_tilt_yaw_start_rad),
            ),
        ))
        self.direct_large_tilt_yaw_full_rad = math.radians(max(
            math.degrees(self.direct_large_tilt_yaw_start_rad) + 0.1,
            self._tuning_float(
                data,
                'direct_large_tilt_yaw_full_deg',
                math.degrees(self.direct_large_tilt_yaw_full_rad),
            ),
        ))
        self.direct_large_tilt_yaw_min_scale = float(np.clip(
            self._tuning_float(
                data,
                'direct_large_tilt_yaw_min_scale',
                self.direct_large_tilt_yaw_min_scale,
            ),
            0.0,
            1.0,
        ))
        self.direct_tau_limit = self._tuning_array(data, 'direct_tau_limit', self.direct_tau_limit)
        self.direct_takeoff_KR = self._tuning_array(data, 'direct_takeoff_KR', self.direct_takeoff_KR)
        self.direct_takeoff_Domega = self._tuning_array(data, 'direct_takeoff_Domega', self.direct_takeoff_Domega)
        self.direct_takeoff_tau_limit = self._tuning_array(data, 'direct_takeoff_tau_limit', self.direct_takeoff_tau_limit)
        self.direct_xy_lock_KR = self._tuning_array(data, 'direct_xy_lock_KR', self.direct_xy_lock_KR)
        self.direct_xy_lock_Domega = self._tuning_array(data, 'direct_xy_lock_Domega', self.direct_xy_lock_Domega)
        self.direct_xy_lock_tau_limit = self._tuning_array(data, 'direct_xy_lock_tau_limit', self.direct_xy_lock_tau_limit)
        self.direct_pos_Kp_ned = np.maximum(
            self._tuning_array(data, 'direct_pos_Kp_ned', self.direct_pos_Kp_ned),
            0.0,
        )
        self.direct_pos_Kd_ned = np.maximum(
            self._tuning_array(data, 'direct_pos_Kd_ned', self.direct_pos_Kd_ned),
            0.0,
        )
        previous_pos_ki = self.direct_pos_Ki_ned.copy()
        self.direct_pos_Ki_ned = np.maximum(
            self._tuning_array(data, 'direct_pos_Ki_ned', self.direct_pos_Ki_ned),
            0.0,
        )
        self.direct_pos_integral_limit_ned = np.maximum(
            self._tuning_array(
                data,
                'direct_pos_integral_limit_ned',
                self.direct_pos_integral_limit_ned,
            ),
            0.0,
        )
        self.integral_pos_error = self._bumpless_integral_gain_change(
            self.integral_pos_error,
            previous_pos_ki,
            self.direct_pos_Ki_ned,
            self.direct_pos_integral_limit_ned,
        )
        self.manual_max_position_lead_xy = float(np.clip(
            self._tuning_float(
                data,
                'manual_max_position_lead_xy',
                self.manual_max_position_lead_xy,
            ),
            0.05,
            2.0,
        ))
        self.gamepad_max_vxy_mps = float(np.clip(
            self._tuning_float(
                data, 'gamepad_max_vxy_mps', self.gamepad_max_vxy_mps
            ),
            0.05,
            3.0,
        ))
        self.gamepad_deadzone = float(np.clip(
            self._tuning_float(
                data, 'gamepad_deadzone', self.gamepad_deadzone
            ),
            0.0,
            0.4,
        ))
        self.gamepad_expo = float(np.clip(
            self._tuning_float(data, 'gamepad_expo', self.gamepad_expo),
            0.0,
            1.0,
        ))
        self.gamepad_filter_tau_s = float(np.clip(
            self._tuning_float(
                data, 'gamepad_filter_tau_s', self.gamepad_filter_tau_s
            ),
            0.0,
            2.0,
        ))
        self.gamepad_max_vxy_body_mps = np.clip(
            self._tuning_array(
                data,
                'gamepad_max_vxy_body_mps',
                self.gamepad_max_vxy_body_mps,
            ),
            0.05,
            3.0,
        )
        self.gamepad_filter_tau_body_xy_s = np.clip(
            self._tuning_array(
                data,
                'gamepad_filter_tau_body_xy_s',
                self.gamepad_filter_tau_body_xy_s,
            ),
            0.0,
            2.0,
        )
        self.gamepad_max_acc_body_xy_mps2 = np.clip(
            self._tuning_array(
                data,
                'gamepad_max_acc_body_xy_mps2',
                self.gamepad_max_acc_body_xy_mps2,
            ),
            0.05,
            5.0,
        )
        if hasattr(self, 'gamepad'):
            self.gamepad.max_vxy = self.gamepad_max_vxy_mps
            self.gamepad.deadzone = self.gamepad_deadzone
            self.gamepad.expo = self.gamepad_expo
            self.gamepad.filter_tau = self.gamepad_filter_tau_s
            self.gamepad.max_vxy_body_mps = (
                self.gamepad_max_vxy_body_mps.copy()
            )
            self.gamepad.filter_tau_body_xy_s = (
                self.gamepad_filter_tau_body_xy_s.copy()
            )
            self.gamepad.max_acc_body_xy_mps2 = (
                self.gamepad_max_acc_body_xy_mps2.copy()
            )

        self.max_acc_xy = self._tuning_float(data, 'max_acc_xy', self.max_acc_xy)
        self.max_acc_z = self._tuning_float(data, 'max_acc_z', self.max_acc_z)
        self.xy_lock_max_acc_xy = self._tuning_float(data, 'xy_lock_max_acc_xy', self.xy_lock_max_acc_xy)
        self.direct_yaw_control_enabled = self._tuning_bool(
            data, 'direct_yaw_control_enabled', self.direct_yaw_control_enabled
        )

        # Convenience scalar aliases for quick yaw tuning.
        self.direct_KR[2] = self._tuning_float(data, 'direct_KR_yaw', self.direct_KR[2])
        self.direct_Domega[2] = self._tuning_float(data, 'direct_Domega_yaw', self.direct_Domega[2])
        self.direct_tau_limit[2] = self._tuning_float(data, 'direct_tau_yaw_limit', self.direct_tau_limit[2])

    def _load_tuning_file(self, force: bool = False):
        try:
            stat = os.stat(self.tuning_path)
        except FileNotFoundError:
            self._write_tuning_file_if_missing()
            stat = os.stat(self.tuning_path)

        if not force and self._last_tuning_mtime_ns == stat.st_mtime_ns:
            return

        try:
            with open(self.tuning_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            if not isinstance(data, dict):
                raise ValueError('tuning file must contain a JSON object')
            self._apply_tuning(data)
            self._last_tuning_mtime_ns = stat.st_mtime_ns
        except Exception as exc:
            now = time.time()
            if force or now - self._last_tuning_log_time > 2.0:
                self.get_logger().warn(f'无法读取在线调参文件 {self.tuning_path}: {exc}')
                self._last_tuning_log_time = now
            return

        self._write_tuning_apply_status(data, stat.st_mtime_ns)

        now = time.time()
        if force or now - self._last_tuning_log_time > 1.0:
            self.get_logger().info(
                '在线调参已加载: '
                f'att_step={np.round(np.degrees(self.attitude_step_axis_rad), 1).tolist()}deg, '
                f'peak_hold={self.attitude_peak_hold_s:.1f}s, '
                f'att_z={self.attitude_test_altitude_m:.1f}m, '
                f'alpha_lim={math.degrees(self.alpha_limit_rad):.1f}deg, '
                f'manual_roll_lim={math.degrees(self.manual_roll_limit_rad):.1f}deg, '
                f'manual_pitch_lim={math.degrees(self.manual_pitch_limit_rad):.1f}deg, '
                f'att_safety={self.direct_safety_attitude_check_enabled}, '
                f'theta_lim={math.degrees(self.theta_limit_rad):.1f}deg, '
                f'servo_cal=[profile={self.hardware_firmware_profile}, '
                f'primary={math.degrees(self.primary_servo_angle_max_rad):.1f}deg, '
                f'secondary={math.degrees(self.secondary_servo_angle_max_rad):.1f}deg, '
                f'gear={self.secondary_servo_gear_ratio:.3f}, '
                f'pwm={self.servo_pwm_min_us}/{self.servo_pwm_trim_us}/{self.servo_pwm_max_us}us], '
                f'force_sign=[{self.allocator_force_x_sign:+.0f}, {self.allocator_force_y_sign:+.0f}], '
                f'tail=[bias={self.pitch_torque_bias:+.3f}, '
                f'sign={self.tail_torque_sign:+.0f}, '
                f'collective={self.tail_collective_comp:.2f}], '
                f'KR={np.round(self.direct_KR, 3).tolist()}, '
                f'D={np.round(self.direct_Domega, 3).tolist()}, '
                f'att_Ki={np.round(self.direct_attitude_Ki, 3).tolist()}, '
                f'tau_lim={np.round(self.direct_tau_limit, 3).tolist()}, '
                f'rc_filter={np.round(self.gamepad_filter_tau_body_xy_s, 3).tolist()}s, '
                f'rc_acc={np.round(self.gamepad_max_acc_body_xy_mps2, 3).tolist()}m/s2, '
                f'pos_Kp_ned={np.round(self.direct_pos_Kp_ned, 3).tolist()}, '
                f'pos_Kd_ned={np.round(self.direct_pos_Kd_ned, 3).tolist()}, '
                f'pos_Ki_ned={np.round(self.direct_pos_Ki_ned, 3).tolist()}'
            )
            self._last_tuning_log_time = now

    def _write_tuning_apply_status(
        self,
        tuning_data: dict,
        source_mtime_ns: int,
    ) -> None:
        """Atomically acknowledge a successfully applied live-tuning file."""
        status = {
            'ok': True,
            'node': self._node_name(),
            'pid': os.getpid(),
            'source_path': str(Path(self.tuning_path).resolve()),
            'source_mtime_ns': int(source_mtime_ns),
            'revision': tuning_data.get('_web_revision'),
            'applied_at_unix_s': time.time(),
        }
        path = self.tuning_status_path
        temporary = path.with_name(
            f'.{path.name}.{os.getpid()}.{time.time_ns()}.tmp'
        )
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with temporary.open('x', encoding='utf-8') as stream:
                json.dump(status, stream, indent=2, sort_keys=True)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        except OSError as exc:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass
            now = time.time()
            if now - self._last_tuning_log_time > 2.0:
                self.get_logger().warn(
                    f'无法写入在线调参加载回执 {path}: {exc}'
                )
                self._last_tuning_log_time = now

    # ============================================================
    # PX4 callbacks
    # ============================================================
    def _current_yaw_enu(self) -> float:
        return float(np.arctan2(self.R[1, 0], self.R[0, 0]))

    @staticmethod
    def _rotation_enu_flu_from_euler(attitude_enu: np.ndarray) -> np.ndarray:
        roll, pitch, yaw = [float(v) for v in attitude_enu]
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ], dtype=float)

    @staticmethod
    def _rotation_x(angle_rad: float) -> np.ndarray:
        c = math.cos(float(angle_rad))
        s = math.sin(float(angle_rad))
        return np.array([
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ], dtype=float)

    @staticmethod
    def _rotation_y(angle_rad: float) -> np.ndarray:
        c = math.cos(float(angle_rad))
        s = math.sin(float(angle_rad))
        return np.array([
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ], dtype=float)

    @staticmethod
    def _rotation_z(angle_rad: float) -> np.ndarray:
        c = math.cos(float(angle_rad))
        s = math.sin(float(angle_rad))
        return np.array([
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=float)

    @staticmethod
    def _enu_flu_to_ned_frd_rotation(r_enu_flu: np.ndarray) -> np.ndarray:
        r_ned_enu = np.array([
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ], dtype=float)
        r_flu_frd = np.diag([1.0, -1.0, -1.0])
        return r_ned_enu @ r_enu_flu @ r_flu_frd

    def _direct_desired_attitude_ned_frd(self, attitude_enu: np.ndarray) -> np.ndarray:
        return self._enu_flu_to_ned_frd_rotation(self._rotation_enu_flu_from_euler(attitude_enu))

    def _continuous_attitude_test_pitch_rad(self) -> float:
        yaw_reference = float(
            self.auto_traj_yaw
            if self.auto_traj_mode == 'attitude'
            else self.target_attitude[2]
        )
        relative_rotation = self._rotation_z(yaw_reference).T @ self.R
        return float(math.atan2(-relative_rotation[2, 0], relative_rotation[0, 0]))

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
        self.attitude_q = np.array([w, x, y, z], dtype=float)
        R_ned_frd_raw = np.array([
            [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x ** 2 + y ** 2)]
        ])
        self.R_ned_frd_raw = R_ned_frd_raw
        identity_transform = np.eye(3)
        flip_transform = np.diag([1.0, -1.0, -1.0])
        candidates = (
            (R_ned_frd_raw, identity_transform, False),
            (R_ned_frd_raw @ flip_transform, flip_transform, True),
        )
        if not self._attitude_canonical_initialized:
            selected = max(candidates, key=lambda item: float(item[0][2, 2]))
            self._attitude_canonical_initialized = True
        else:
            selected = min(
                candidates,
                key=lambda item: float(np.linalg.norm(item[0] - self.R_ned_frd))
            )
        R_ned_frd, self._attitude_axis_transform, _ = selected
        self.R_ned_frd = R_ned_frd
        R_enu_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        R_frd_flu = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        self.R = R_enu_ned @ R_ned_frd @ R_frd_flu
        current_yaw = self._current_yaw_enu()

        if not self._yaw_initialized:
            self.initial_yaw = current_yaw
            self.target_attitude[2] = self.initial_yaw
            self.manual_des_yaw = self.initial_yaw
            self._yaw_initialized = True
        elif not self.armed and not self.takeoff_requested and not self.manual_pos_initialized:
            self.initial_yaw = current_yaw
            self.target_attitude[2] = current_yaw
            self.manual_des_yaw = current_yaw

        self.attitude_received = True
        self.data_received = self.local_position_received and self.attitude_received
        if self.data_received:
            self.control_loop()

    def angular_velocity_callback(self, msg):
        # PX4 FRD -> canonical FRD -> FLU
        omega_frd = self._attitude_axis_transform @ np.array(msg.xyz, dtype=float)
        self.angular_velocity_frd = omega_frd
        self.angular_velocity = np.array([omega_frd[0], -omega_frd[1], -omega_frd[2]], dtype=float)

    def status_callback(self, msg):
        self.nav_state = int(getattr(msg, 'nav_state', -1))
        if int(getattr(msg, 'arming_state', -1)) == self.ARMING_STATE_ARMED:
            self.armed = True

    def control_mode_callback(self, msg):
        self.control_offboard_enabled = bool(getattr(msg, 'flag_control_offboard_enabled', False))
        self._armed_from_control_mode = bool(getattr(msg, 'flag_armed', False))
        if self._armed_from_control_mode:
            self.armed = True
        elif time.time() > self._optimistic_armed_until:
            self.armed = False

    def land_detected_callback(self, msg):
        self.land_detected = {
            'landed': bool(getattr(msg, 'landed', False)),
            'maybe_landed': bool(getattr(msg, 'maybe_landed', False)),
            'ground_contact': bool(getattr(msg, 'ground_contact', False)),
            'freefall': bool(getattr(msg, 'freefall', False)),
            'has_low_throttle': bool(getattr(msg, 'has_low_throttle', False)),
        }

    def vehicle_command_ack_callback(self, msg):
        command = int(msg.command)
        if command not in (self.CMD_DO_SET_MODE, self.CMD_COMPONENT_ARM_DISARM):
            return

        result = int(msg.result)
        result_name = self.command_ack_result_names.get(result, f'UNKNOWN({result})')
        command_name = 'DO_SET_MODE' if command == self.CMD_DO_SET_MODE else 'ARM_DISARM'
        text = (
            f'PX4 command ack: {command_name} -> {result_name} '
            f'(result_param1={int(msg.result_param1)}, result_param2={int(msg.result_param2)})'
        )
        if result == getattr(VehicleCommandAck, 'VEHICLE_CMD_RESULT_ACCEPTED', 0):
            self.get_logger().info(text)
            if command == self.CMD_COMPONENT_ARM_DISARM:
                if self._last_arm_disarm_command_is_arm:
                    self._optimistic_armed_until = time.time() + 1.5
                    self.armed = True
                    self.was_armed_once = True
                else:
                    self._optimistic_armed_until = 0.0
                    self.armed = False
        else:
            self.get_logger().warn(text)

    def px4_actuator_motors_callback(self, msg):
        self.px4_out_motor_timestamp = int(getattr(msg, 'timestamp', 0))
        self.px4_out_motor_cmd = np.array(msg.control, dtype=float)

    def px4_actuator_servos_callback(self, msg):
        self.px4_out_servo_timestamp = int(getattr(msg, 'timestamp', 0))
        self.px4_out_servo_cmd = np.array(msg.control, dtype=float)

    # ============================================================
    # Offboard/Arm startup logic
    # ============================================================
    def is_offboard(self) -> bool:
        return bool(self.control_offboard_enabled)

    def timestamp_now_us(self) -> int:
        return int(self.px4_timestamp) if self.px4_timestamp > 0 else int(self.get_clock().now().nanoseconds / 1000)

    def offboard_startup_tick(self):
        # 1) 始终发送 OffboardControlMode 作为 proof-of-life，频率 20Hz。
        #    这是维持 Offboard 的心跳，不等价于重复 arm。
        self.publish_offboard_control_mode()

        # 2) 未收到状态数据前不切模式、不解锁。
        if not self.data_received or self.px4_timestamp <= 0:
            return

        # Offboard 切换前也要持续发送对应 setpoint，避免 commander 因 setpoint 不完整而拒绝。
        if self._use_px4_position_mode():
            self.publish_px4_trajectory_setpoint()
        elif not self.is_offboard():
            self.publish_idle_direct_actuator_setpoint()

        self.offboard_setpoint_counter += 1
        now = time.time()

        # 3) 检测 PX4 是否从 armed 变成 disarmed。
        #    如果已经成功 arm 过一次，之后又被 PX4 自动上锁，默认禁止自动二次 arm。
        if self._last_armed_state and not self.armed:
            takeoff_was_requested = self.takeoff_requested
            self.was_armed_once = True
            self.takeoff_requested = False
            self.manual_pos_initialized = False
            self.integral_pos_error[:] = 0.0
            self.integral_e_R[:] = 0.0
            if not takeoff_was_requested:
                self.startup_blocked_after_disarm = True
                self.preflight_disarm_waiting_for_o = True
                self.auto_arm_attempts = 0
                self.was_armed_once = False
                self.get_logger().warn(
                    'PX4 在起飞许可前已自动上锁，可能是 COM_DISARM_PRFLT 预起飞超时。'
                    '已停止自动二次 Arm；按键盘 o 后会重新请求 Offboard/Arm 并起飞悬停。'
                )
            elif not self.rearm_after_auto_disarm:
                self.startup_blocked_after_disarm = True
                self.preflight_disarm_waiting_for_o = True
                self.get_logger().warn(
                    'PX4 已从 armed 变为 disarmed。已阻止自动二次 Arm。'
                    '请检查是否触发 COM_DISARM_PRFLT、COM_DISARM_LAND、land detector 或 failsafe；'
                    '确认安全后按键盘 o 重新请求 Offboard/Arm。'
                )
        self._last_armed_state = self.armed

        if self.startup_blocked_after_disarm:
            return

        # 4) 至少连续发送 1s 以上 OffboardControlMode 后，再请求 Offboard。
        stream_ready = self.offboard_setpoint_counter >= self.offboard_warmup_ticks
        if stream_ready and not self.is_offboard():
            if now - self._last_offboard_cmd_time > self.mode_request_period_s:
                self.set_offboard_mode()
                self._last_offboard_cmd_time = now
                self._offboard_request_sent = True
                self.get_logger().info('请求切换到 Offboard 模式...')
            return

        # 5) 已进入 Offboard 后再 Arm；默认等键盘 o 作为 direct 解锁/起飞许可。
        if self.is_offboard() and not self.armed:
            if self.arm_after_takeoff_request and not self.takeoff_requested:
                return
            if not self.auto_arm_enabled:
                return
            if self.was_armed_once and not self.rearm_after_auto_disarm:
                return
            if self.auto_arm_attempts >= self.max_auto_arm_attempts:
                return
            prearm_failure = self._direct_prearm_failure_reason()
            if prearm_failure:
                if now - self._last_prearm_reject_log_time > 1.0:
                    self.get_logger().warn(
                        f'Direct Arm 已阻止：{prearm_failure}。请先重置仿真/扶正机体后再按 o。'
                    )
                    self._last_prearm_reject_log_time = now
                return
            if now - self._last_arm_cmd_time > self.arm_request_period_s:
                self.arm()
                self.auto_arm_attempts += 1
                self._last_arm_cmd_time = now
                self._arm_request_sent = True
                self.get_logger().info(
                    f'请求 Arm 解锁... ({self.auto_arm_attempts}/{self.max_auto_arm_attempts})'
                )

        if self.armed:
            self.was_armed_once = True

        if self.is_offboard() and self.armed and not self.takeoff_requested:
            now_s = self.px4_timestamp / 1_000_000.0
            current_time = max(0.0, now_s - self.sim_start_time_s) if self.sim_start_time_s > 0.0 else 0.0
            self.publish_preflight_tilt_test_setpoint(current_time, 0.05)

    def _use_px4_position_mode(self) -> bool:
        return bool(self.use_px4_position_takeoff)

    def publish_offboard_control_mode(self):
        position_mode = self._use_px4_position_mode()
        msg = OffboardControlMode()
        msg.position = position_mode
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        # 兼容不同 px4_msgs 版本
        if hasattr(msg, 'thrust_and_torque'):
            msg.thrust_and_torque = False
        if hasattr(msg, 'direct_actuator'):
            msg.direct_actuator = not position_mode
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

    def publish_idle_direct_actuator_setpoint(self):
        self._alpha1_cmd = 0.0
        self._alpha2_cmd = 0.0
        self._theta1_cmd = 0.0
        self._theta2_cmd = 0.0
        self.publish_direct_actuator_setpoint(
            motor_controls=[0.0, 0.0, 0.0, 0.0, 0.0],
            alpha1=0.0,
            alpha2=0.0,
            theta1=0.0,
            theta2=0.0
        )

    def publish_direct_actuator_setpoint(self, motor_controls, alpha1, alpha2, theta1, theta2):
        servo_controls = [
            self._primary_joint_angle_to_normalized(alpha2),
            self._primary_joint_angle_to_normalized(alpha1),
            self._secondary_joint_angle_to_normalized(theta2),
            self._secondary_joint_angle_to_normalized(theta1),
        ]
        self._publish_normalized_direct_actuator_setpoint(
            motor_controls,
            servo_controls,
        )

    def _publish_normalized_direct_actuator_setpoint(
        self,
        motor_controls,
        servo_controls,
    ):
        """Publish already-normalized actuator commands.

        Keeping this final publication step separate lets the hardware path
        blend the last PX4 Position-mode outputs into the first external
        controller outputs without converting normalized servo commands back
        and forth through angles.
        """
        timestamp = self.timestamp_now_us()

        motor_msg = ActuatorMotors()
        motor_msg.timestamp = timestamp
        if hasattr(motor_msg, 'timestamp_sample'):
            motor_msg.timestamp_sample = timestamp
        if hasattr(motor_msg, 'reversible_flags'):
            motor_msg.reversible_flags = (1 << 4) if self.allow_tail_reverse else 0
        motor_msg.control = [float('nan')] * 12
        for index, value in enumerate(motor_controls[:12]):
            lower = -1.0 if (index == 4 and self.allow_tail_reverse) else 0.0
            motor_msg.control[index] = float(np.clip(value, lower, 1.0))
        self.last_motor_cmd = np.array(motor_msg.control)
        self.actuator_motors_pub.publish(motor_msg)

        servo_msg = ActuatorServos()
        servo_msg.timestamp = timestamp
        if hasattr(servo_msg, 'timestamp_sample'):
            servo_msg.timestamp_sample = timestamp
        servo_msg.control = [float('nan')] * 8
        for index, value in enumerate(servo_controls[:8]):
            servo_msg.control[index] = float(np.clip(value, -1.0, 1.0))
        self.last_servo_cmd = np.array(servo_msg.control)
        self.actuator_servos_pub.publish(servo_msg)

    def _primary_joint_angle_to_normalized(self, joint_angle_rad: float) -> float:
        return float(np.clip(
            float(joint_angle_rad) / max(self.primary_servo_angle_max_rad, 1e-8),
            -1.0,
            1.0,
        ))

    def _secondary_joint_angle_to_normalized(self, joint_angle_rad: float) -> float:
        servo_shaft_angle = float(joint_angle_rad) * self.secondary_servo_gear_ratio
        return float(np.clip(
            servo_shaft_angle / max(self.secondary_servo_angle_max_rad, 1e-8),
            -1.0,
            1.0,
        ))

    def _normalized_servo_to_expected_pwm_us(self, normalized: float) -> float:
        normalized = float(np.clip(normalized, -1.0, 1.0))
        if normalized >= 0.0:
            span = self.servo_pwm_max_us - self.servo_pwm_trim_us
        else:
            span = self.servo_pwm_trim_us - self.servo_pwm_min_us
        return float(self.servo_pwm_trim_us + normalized * span)

    def _secondary_joint_rate_limit_rad_s(self) -> float:
        return self.servo_rate_limit_rad_s / self.secondary_servo_gear_ratio

    def _preflight_tilt_targets(self, current_time: float):
        if not self.preflight_tilt_test_enabled:
            return 0.0, 0.0, 0.0, 0.0

        if self.preflight_tilt_start_time_s is None:
            self.preflight_tilt_start_time_s = current_time
            self.preflight_tilt_test_started = True
            self.preflight_tilt_test_finished = False
            self.get_logger().info('已进入 Offboard 且已解锁：电机保持零输出，开始地面倾转舵机自检。')

        elapsed = max(0.0, current_time - self.preflight_tilt_start_time_s)
        total_time = 4.0 * self.preflight_tilt_axis_duration_s
        if elapsed >= total_time:
            if not self.preflight_tilt_test_finished:
                self.preflight_tilt_test_finished = True
                self.get_logger().info('倾转舵机自检完成，继续零油门等待；按键盘 o 后起飞悬停。')
            return 0.0, 0.0, 0.0, 0.0

        axis_idx = int(elapsed / self.preflight_tilt_axis_duration_s)
        axis_elapsed = elapsed - axis_idx * self.preflight_tilt_axis_duration_s
        phase = axis_elapsed / self.preflight_tilt_axis_duration_s
        value = self.preflight_tilt_amplitude_rad * math.sin(2.0 * math.pi * phase)

        alpha1 = alpha2 = theta1 = theta2 = 0.0
        if axis_idx == 0:
            alpha1 = value
        elif axis_idx == 1:
            alpha2 = value
        elif axis_idx == 2:
            theta1 = value
        else:
            theta2 = value
        return alpha1, alpha2, theta1, theta2

    def publish_preflight_tilt_test_setpoint(self, current_time: float, dt: float):
        alpha1, alpha2, theta1, theta2 = self._preflight_tilt_targets(current_time)
        dt = float(np.clip(dt, 0.01, 0.1))
        self._alpha1_cmd = self._slew_limit(self._alpha1_cmd, alpha1, self.servo_rate_limit_rad_s, dt)
        self._alpha2_cmd = self._slew_limit(self._alpha2_cmd, alpha2, self.servo_rate_limit_rad_s, dt)
        secondary_rate_limit = self._secondary_joint_rate_limit_rad_s()
        self._theta1_cmd = self._slew_limit(self._theta1_cmd, theta1, secondary_rate_limit, dt)
        self._theta2_cmd = self._slew_limit(self._theta2_cmd, theta2, secondary_rate_limit, dt)

        self.last_F1 = 0.0
        self.last_F2 = 0.0
        self.last_F3 = 0.0
        self.last_W = np.zeros(6)
        self._last_manual_cmd = self._zero_manual_cmd()
        self.publish_direct_actuator_setpoint(
            motor_controls=[0.0, 0.0, 0.0, 0.0, 0.0],
            alpha1=self._alpha1_cmd,
            alpha2=self._alpha2_cmd,
            theta1=self._theta1_cmd,
            theta2=self._theta2_cmd
        )

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0,
                                param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.command = int(command)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.timestamp_now_us()
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self._last_arm_disarm_command_is_arm = True
        self.publish_vehicle_command(self.CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        self._last_arm_disarm_command_is_arm = False
        self._optimistic_armed_until = 0.0
        self.armed = False
        self.publish_vehicle_command(self.CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def set_offboard_mode(self):
        # VEHICLE_CMD_DO_SET_MODE: param1=1(custom), param2=6(OFFBOARD)
        self.publish_vehicle_command(self.CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    # ============================================================
    # Keyboard trajectory commands
    # ============================================================
    def _zero_manual_cmd(self) -> dict:
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
            'attitude_axis': self.gamepad.attitude_control_axis,
        }

    def poll_keyboard_commands(self):
        for key in self.keyboard.get_commands():
            if key in ('o', 'O'):
                self.armed = False
                self._last_armed_state = False
                self._optimistic_armed_until = 0.0
                self.startup_blocked_after_disarm = False
                self.preflight_disarm_waiting_for_o = False
                self.was_armed_once = False
                self.auto_arm_attempts = 0
                self._offboard_request_sent = False
                self._arm_request_sent = False
                self._last_offboard_cmd_time = 0.0
                self._last_arm_cmd_time = 0.0
                self.takeoff_requested = True
                self.manual_pos_initialized = False
                self._takeoff_lock_start_time_s = None
                self._takeoff_start_z_rel = 0.0
                self._xy_lock_active = False
                self._z0_initialized = False
                self._z0 = 0.0
                self._alpha1_cmd = 0.0
                self._alpha2_cmd = 0.0
                self._theta1_cmd = 0.0
                self._theta2_cmd = 0.0
                self.integral_pos_error[:] = 0.0
                self.integral_e_R[:] = 0.0
                self.direct_safety_cutoff = False
                self.direct_safety_cutoff_reason = ''
                self._last_direct_safety_log_time = 0.0
                current_yaw = self._current_yaw_enu() if self.attitude_received else self.initial_yaw
                self.initial_yaw = current_yaw
                self.manual_des_yaw = current_yaw
                self.target_attitude[2] = current_yaw
                self.get_logger().info('收到键盘 o：起飞许可已打开，开始爬升到悬停高度。')
            elif key == '1':
                self.pending_auto_traj_mode = 'rectangle'
                self.get_logger().info('收到键盘 1：矩形轨迹已排队，悬停稳定后开始。')
            elif key == '2':
                self.pending_auto_traj_mode = 'lissajous'
                self.get_logger().info('收到键盘 2：三维李萨如轨迹已排队，悬停稳定后开始。')
            elif key == '3':
                self.pending_auto_traj_mode = 'attitude'
                self.get_logger().info('收到键盘 3：姿态角轨迹已排队，悬停稳定后开始。')

    def _trajectory_ready(self, current_time: float) -> bool:
        if not (self.is_offboard() and self.armed and self.manual_pos_initialized):
            return False
        if current_time < self.takeoff_xy_lock_time_s:
            return False

        if self.pending_auto_traj_mode == 'attitude':
            test_z = float(np.clip(
                max(self.takeoff_height, self.attitude_test_altitude_m),
                self.min_altitude,
                self.max_altitude,
            ))
            if self.manual_des_pos[2] < test_z:
                self.manual_des_pos[2] = test_z
                self.integral_pos_error[:] = 0.0
                return False

            pos_rel_z = self.position[2] - self._z0 if self._z0_initialized else self.position[2]
            return pos_rel_z >= test_z - self.auto_traj_ready_margin

        return self.manual_des_pos[2] >= self.takeoff_height - self.auto_traj_ready_margin

    def _yaw_rotation_2d(self, yaw: float) -> np.ndarray:
        c = math.cos(yaw)
        s = math.sin(yaw)
        return np.array([[c, -s], [s, c]], dtype=float)

    def _wrap_angle_rad(self, angle: float) -> float:
        return float(math.atan2(math.sin(angle), math.cos(angle)))

    def _start_auto_trajectory(self, mode: str, current_time: float):
        self.auto_traj_mode = mode
        self.auto_traj_start_time = current_time
        self.auto_traj_yaw = float(self.manual_des_yaw)
        self.auto_traj_start_attitude = np.array([0.0, 0.0, self.auto_traj_yaw], dtype=float)
        self.auto_traj_start_pos = self.manual_des_pos.copy()
        if mode == 'attitude':
            self.auto_traj_start_pos[2] = max(self.auto_traj_start_pos[2], self.attitude_test_altitude_m)
        self.auto_traj_start_pos[2] = float(np.clip(
            max(self.auto_traj_start_pos[2], self.takeoff_height),
            self.min_altitude,
            self.max_altitude
        ))
        self.auto_traj_z = float(self.auto_traj_start_pos[2])

        R_yaw = self._yaw_rotation_2d(self.auto_traj_yaw)
        if mode == 'lissajous':
            first_rel_xy = np.array([self.lissajous_amp_x, self.lissajous_amp_y], dtype=float)
            self.auto_traj_origin_xy = self.auto_traj_start_pos[:2] - R_yaw @ first_rel_xy
            mode_text = '三维李萨如'
        elif mode == 'attitude':
            self.auto_traj_origin_xy = self.auto_traj_start_pos[:2].copy()
            mode_text = '姿态角'
        else:
            self.auto_traj_origin_xy = self.auto_traj_start_pos[:2].copy()
            mode_text = '矩形'

        self.manual_des_pos = self.auto_traj_start_pos.copy()
        self.manual_des_roll = 0.0
        self.manual_des_pitch = 0.0
        self.integral_pos_error[:] = 0.0
        self.integral_e_R[:] = 0.0
        self._attitude_error_quaternion = None
        finish_text = '完成后回到该点悬停。'
        self.get_logger().info(
            f'开始执行{mode_text}轨迹：起点 [{self.auto_traj_start_pos[0]:.2f}, '
            f'{self.auto_traj_start_pos[1]:.2f}, {self.auto_traj_start_pos[2]:.2f}]，'
            f'{finish_text}'
        )

    def _finish_auto_trajectory(self):
        finished_mode = self.auto_traj_mode
        if finished_mode == 'lissajous':
            mode_text = '三维李萨如'
        elif finished_mode == 'attitude':
            mode_text = '姿态角'
        else:
            mode_text = '矩形'
        self.auto_traj_mode = 'hover'
        if finished_mode == 'attitude':
            self.manual_des_pos = self.auto_traj_start_pos.copy()
            self.manual_des_pos[2] = self.auto_traj_z
        else:
            self.manual_des_pos = self.auto_traj_start_pos.copy()
        self.manual_des_yaw = self.auto_traj_yaw
        self.manual_des_roll = 0.0
        self.manual_des_pitch = 0.0
        self.target_position = self.manual_des_pos.copy()
        self.target_velocity = np.zeros(3)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([0.0, 0.0, self.manual_des_yaw], dtype=float)
        self.target_attitude_rate = np.zeros(3)
        self.target_R_des_ned_frd = None
        self.integral_pos_error[:] = 0.0
        self.integral_e_R[:] = 0.0
        self._attitude_error_quaternion = None
        if finished_mode == 'attitude':
            self.get_logger().info(
                f'{mode_text}轨迹完成，已回到并保持原悬停点 '
                f'[{self.manual_des_pos[0]:.2f}, {self.manual_des_pos[1]:.2f}, {self.manual_des_pos[2]:.2f}]。'
            )
        else:
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
        cz = float(self.lissajous_c)

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
        local_z = self.lissajous_amp_z * (1.0 - math.cos(cz * theta))
        local_vel_z = self.lissajous_amp_z * cz * theta_dot * math.sin(cz * theta)
        local_acc_z = (
            self.lissajous_amp_z
            * (cz * theta_dot) ** 2
            * math.cos(cz * theta)
        )

        R_yaw = self._yaw_rotation_2d(self.auto_traj_yaw)
        pos = np.array([
            *(self.auto_traj_origin_xy + R_yaw @ local_xy),
            float(np.clip(
                self.auto_traj_z + local_z,
                self.min_altitude,
                self.max_altitude,
            )),
        ], dtype=float)
        vel = np.array([*(R_yaw @ local_vel_xy), local_vel_z], dtype=float)
        acc = np.array([*(R_yaw @ local_acc_xy), local_acc_z], dtype=float)
        return pos, vel, acc, False

    def _attitude_reference(self, elapsed: float):
        segment_time = float(self.attitude_segment_time_s)
        peak_hold_time = float(self.attitude_peak_hold_s)
        cycle_time = 2.0 * segment_time + peak_hold_time
        active_axes = np.flatnonzero(np.abs(self.attitude_step_axis_rad) > math.radians(0.01))
        if active_axes.size == 0:
            return self.auto_traj_start_attitude.copy(), np.zeros(3), None, True

        axis_steps = []
        for axis in active_axes:
            step = float(self.attitude_step_axis_rad[int(axis)])
            if abs(step) > math.radians(0.01):
                axis_steps.append((int(axis), step))

        # Run one axis at a time and always return to level before switching axes:
        # roll +, pitch +, yaw +, then roll -, pitch -, yaw - for whichever axes are enabled.
        if self.attitude_test_bidirectional:
            signed_sequence = (
                [(axis, abs(step)) for axis, step in axis_steps]
                + [(axis, -abs(step)) for axis, step in axis_steps]
            )
        else:
            signed_sequence = axis_steps

        if not signed_sequence:
            return self.auto_traj_start_attitude.copy(), np.zeros(3), None, True

        total_time = float(len(signed_sequence)) * cycle_time
        if elapsed >= total_time:
            return self.auto_traj_start_attitude.copy(), np.zeros(3), None, True

        sequence_index = min(int(elapsed / cycle_time), len(signed_sequence) - 1)
        axis_idx, step_rad = signed_sequence[sequence_index]
        cycle_elapsed = elapsed - sequence_index * cycle_time

        if cycle_elapsed < segment_time:
            u = float(np.clip(cycle_elapsed / segment_time, 0.0, 1.0))
            smooth_u = 3.0 * u ** 2 - 2.0 * u ** 3
            smooth_du = (6.0 * u * (1.0 - u)) / segment_time
            offset = step_rad * smooth_u
            offset_rate = step_rad * smooth_du
        elif cycle_elapsed < segment_time + peak_hold_time:
            offset = step_rad
            offset_rate = 0.0
        else:
            segment_elapsed = cycle_elapsed - segment_time - peak_hold_time
            u = float(np.clip(segment_elapsed / segment_time, 0.0, 1.0))
            smooth_u = 3.0 * u ** 2 - 2.0 * u ** 3
            smooth_du = (6.0 * u * (1.0 - u)) / segment_time
            offset = step_rad * (1.0 - smooth_u)
            offset_rate = -step_rad * smooth_du

        attitude = self.auto_traj_start_attitude.copy()
        attitude_rate = np.zeros(3)
        attitude[axis_idx] += offset
        attitude_rate[axis_idx] = offset_rate
        attitude[2] = self._wrap_angle_rad(attitude[2])
        yaw0 = float(self.auto_traj_start_attitude[2])

        if axis_idx == 0:
            r_enu_flu = self._rotation_z(yaw0) @ self._rotation_x(offset)
        elif axis_idx == 1:
            r_enu_flu = self._rotation_z(yaw0) @ self._rotation_y(offset)
        else:
            r_enu_flu = self._rotation_z(self._wrap_angle_rad(yaw0 + offset))

        return attitude, attitude_rate, self._enu_flu_to_ned_frd_rotation(r_enu_flu), False

    def _update_auto_trajectory(self, current_time: float):
        elapsed = max(0.0, current_time - self.auto_traj_start_time)
        if self.auto_traj_mode == 'attitude':
            attitude, attitude_rate, r_des_ned_frd, done = self._attitude_reference(elapsed)
            if done:
                self._finish_auto_trajectory()
                return True

            self.manual_des_pos = self.auto_traj_start_pos.copy()
            self.manual_des_yaw = attitude[2]
            self.manual_des_roll = attitude[0]
            self.manual_des_pitch = attitude[1]
            self._last_manual_cmd = self._zero_manual_cmd()
            self.target_position = self.auto_traj_start_pos.copy()
            self.target_velocity = np.zeros(3)
            self.target_acceleration = np.zeros(3)
            self.target_attitude = attitude
            self.target_attitude_rate = attitude_rate
            self.target_R_des_ned_frd = r_des_ned_frd
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
        self.manual_des_pitch = 0.0
        self._last_manual_cmd = self._zero_manual_cmd()
        self.target_position = pos
        self.target_velocity = vel
        self.target_acceleration = acc
        self.target_attitude = np.array([0.0, 0.0, self.auto_traj_yaw], dtype=float)
        self.target_attitude_rate = np.zeros(3)
        self.target_R_des_ned_frd = None
        return True

    def _limit_manual_position_lead(self, clamp_xy=True, clamp_z=True):
        current_rel = np.array([
            float(self.position[0]),
            float(self.position[1]),
            float(self.position[2] - self._z0 if self._z0_initialized else self.position[2]),
        ], dtype=float)

        if clamp_xy:
            xy_error = self.manual_des_pos[:2] - current_rel[:2]
            xy_error_norm = float(np.linalg.norm(xy_error))
            if xy_error_norm > self.manual_max_position_lead_xy and xy_error_norm > 1e-6:
                self.manual_des_pos[:2] = (
                    current_rel[:2]
                    + xy_error * (self.manual_max_position_lead_xy / xy_error_norm)
                )

        if clamp_z:
            z_error = float(self.manual_des_pos[2] - current_rel[2])
            self.manual_des_pos[2] = float(
                current_rel[2] + np.clip(z_error, -self.manual_max_position_lead_z, self.manual_max_position_lead_z)
            )
        self.manual_des_pos[2] = float(np.clip(self.manual_des_pos[2], 0.0, self.max_altitude))

    # ============================================================
    # Manual trajectory: gamepad velocity -> desired position/yaw
    # ============================================================
    def update_trajectory(self, current_time: float, dt: float):
        if not self._z0_initialized:
            self._z0 = float(self.position[2])
            self._z0_initialized = True

        # 未进入 offboard 或未解锁前，目标点贴住当前点，清积分，避免一解锁就猛冲
        if (not self.is_offboard()) or (not self.armed):
            self.integral_pos_error[:] = 0.0
            self.integral_e_R[:] = 0.0
            self.manual_pos_initialized = False
            self.auto_traj_mode = 'hover'
            self._z_sp = 0.0
            self._takeoff_lock_start_time_s = None
            self._takeoff_start_z_rel = 0.0
            self._xy_lock_active = False
            self.preflight_tilt_test_started = False
            self.preflight_tilt_test_finished = False
            self.preflight_tilt_start_time_s = None
            self.manual_des_roll = 0.0
            self.manual_des_pitch = 0.0
            self.target_position = np.array([self.position[0], self.position[1], 0.0])
            self.target_velocity = np.zeros(3)
            self.target_acceleration = np.zeros(3)
            self.target_attitude = np.array([0.0, 0.0, self.initial_yaw])
            self.target_attitude_rate = np.zeros(3)
            self.target_R_des_ned_frd = None
            return

        if not self.takeoff_requested:
            self.integral_pos_error[:] = 0.0
            self.integral_e_R[:] = 0.0
            self.manual_pos_initialized = False
            self.auto_traj_mode = 'hover'
            self._z0_initialized = False
            self._z0 = 0.0
            self._z_sp = 0.0
            self._takeoff_lock_start_time_s = None
            self._takeoff_start_z_rel = 0.0
            self._xy_lock_active = False
            self.manual_des_roll = 0.0
            self.manual_des_pitch = 0.0
            self._last_manual_cmd = self._zero_manual_cmd()
            self.target_position = np.array([self.position[0], self.position[1], 0.0])
            self.target_velocity = np.zeros(3)
            self.target_acceleration = np.zeros(3)
            self.target_attitude = np.array([0.0, 0.0, self.initial_yaw])
            self.target_attitude_rate = np.zeros(3)
            self.target_R_des_ned_frd = None
            return

        if not self.manual_pos_initialized:
            self.manual_des_pos = np.array([self.position[0], self.position[1], max(0.0, self.position[2] - self._z0)])
            self.initial_yaw = self._current_yaw_enu()
            self.manual_des_yaw = self.initial_yaw
            self.manual_des_roll = 0.0
            self.manual_des_pitch = 0.0
            self._z_sp = float(self.manual_des_pos[2])
            self._takeoff_start_z_rel = float(self.manual_des_pos[2])
            self._xy_lock_position = self.position[:2].copy()
            self._xy_lock_initialized = True
            self._takeoff_lock_start_time_s = current_time
            self.manual_pos_initialized = True

        if self.pending_auto_traj_mode is not None and self._trajectory_ready(current_time):
            self._start_auto_trajectory(self.pending_auto_traj_mode, current_time)
            self.pending_auto_traj_mode = None

        if self.auto_traj_mode != 'hover':
            if self._update_auto_trajectory(current_time):
                return

        cmds = (
            self.gamepad.get_velocity_commands(dt)
            if self._manual_command_input_enabled()
            else self._zero_manual_cmd()
        )
        self._last_manual_cmd = cmds.copy()

        yaw_rate = float(cmds['yaw_rate'])
        current_yaw_enu = self._current_yaw_enu()
        manual_yaw_active = abs(yaw_rate) > 1e-5

        yaw_ref = self.manual_des_yaw
        vx_w = cmds['vx_b'] * math.cos(yaw_ref) - cmds['vy_b'] * math.sin(yaw_ref)
        vy_w = cmds['vx_b'] * math.sin(yaw_ref) + cmds['vy_b'] * math.cos(yaw_ref)
        roll_rate = cmds.get('roll_rate', 0.0)
        pitch_rate = cmds.get('pitch_rate', 0.0)
        command_attitude_limit = max(
            float(cmds.get('attitude_limit_rad', float('inf'))), 0.0
        )
        roll_limit = min(self.manual_roll_limit_rad, command_attitude_limit)
        pitch_limit = min(self.manual_pitch_limit_rad, command_attitude_limit)
        prev_xy = self.manual_des_pos[:2].copy()
        manual_xy_active = abs(cmds['vx_b']) > 1e-5 or abs(cmds['vy_b']) > 1e-5

        takeoff_elapsed_s = (
            current_time - self._takeoff_lock_start_time_s
            if self._takeoff_lock_start_time_s is not None else float('inf')
        )
        if takeoff_elapsed_s < self.takeoff_xy_lock_time_s:
            vx_w = 0.0
            vy_w = 0.0
            self.manual_des_pos[0] = float(self._xy_lock_position[0])
            self.manual_des_pos[1] = float(self._xy_lock_position[1])

        prev_z = float(self.manual_des_pos[2])
        manual_vz = float(cmds['vz'])
        auto_climb_active = abs(manual_vz) < 1e-5 and prev_z < self.takeoff_height
        if abs(manual_vz) < 1e-5 and prev_z < self.takeoff_height:
            z_ramp_sp = min(
                float(self.takeoff_height),
                float(self._takeoff_start_z_rel + self.max_climb_rate * max(takeoff_elapsed_s, 0.0))
            )
            self.manual_des_pos[2] = max(prev_z, z_ramp_sp)
        else:
            self.manual_des_pos[2] += manual_vz * dt

        self.manual_des_pos[2] = float(np.clip(self.manual_des_pos[2], 0.0, self.max_altitude))
        vz_w = float(np.clip(
            (self.manual_des_pos[2] - prev_z) / max(dt, 1e-3),
            -self.max_climb_rate,
            self.max_climb_rate
        ))

        self.manual_des_pos[0] += vx_w * dt
        self.manual_des_pos[1] += vy_w * dt
        if manual_yaw_active:
            self.manual_des_yaw = self._wrap_angle_rad(self.manual_des_yaw + yaw_rate * dt)
            yaw_error = self._wrap_angle_rad(self.manual_des_yaw - current_yaw_enu)
            if abs(yaw_error) > self.manual_max_yaw_lead_rad:
                yaw_error = float(np.clip(yaw_error, -self.manual_max_yaw_lead_rad, self.manual_max_yaw_lead_rad))
                self.manual_des_yaw = self._wrap_angle_rad(current_yaw_enu + yaw_error)
        previous_roll = self.manual_des_roll
        self.manual_des_roll = float(np.clip(
            previous_roll + roll_rate * dt,
            -roll_limit,
            roll_limit,
        ))
        realized_roll_rate = (self.manual_des_roll - previous_roll) / max(dt, 1e-3)
        previous_pitch = self.manual_des_pitch
        self.manual_des_pitch = float(np.clip(
            previous_pitch + pitch_rate * dt,
            -pitch_limit,
            pitch_limit,
        ))
        realized_pitch_rate = (
            self.manual_des_pitch - previous_pitch
        ) / max(dt, 1e-3)
        self._limit_manual_position_lead(
            clamp_xy=manual_xy_active,
            clamp_z=(abs(manual_vz) > 1e-5 or auto_climb_active),
        )
        realized_vxy = (self.manual_des_pos[:2] - prev_xy) / max(dt, 1e-3)

        self.target_position = self.manual_des_pos.copy()
        self.target_velocity = np.array([realized_vxy[0], realized_vxy[1], vz_w], dtype=float)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([
            self.manual_des_roll,
            self.manual_des_pitch,
            self.manual_des_yaw,
        ], dtype=float)
        self.target_attitude_rate = np.array([
            realized_roll_rate,
            realized_pitch_rate,
            yaw_rate,
        ], dtype=float)
        self.target_R_des_ned_frd = None

    def _manual_command_input_enabled(self) -> bool:
        return bool(self.manual_enabled)

    def control_loop(self):
        if not self.data_received or self.px4_timestamp <= 0:
            return

        now_s = self.px4_timestamp / 1_000_000.0
        if self.sim_start_time_s == 0.0:
            self.sim_start_time_s = now_s
            self._last_timestamp_s = now_s
            return

        raw_dt = now_s - self._last_timestamp_s
        if raw_dt <= 0.0001 or raw_dt > 0.2:
            self._last_timestamp_s = now_s
            return

        self._last_timestamp_s = now_s
        dt = float(np.clip(raw_dt, 0.000125, 0.02))
        self._last_control_dt_s = float(dt)
        current_time = now_s - self.sim_start_time_s

        self.update_trajectory(current_time, dt)

        # 没进入 Offboard 时不向 actuator topics 写入外部 setpoint，避免和 PX4 内部控制器冲突
        if not self.is_offboard():
            return

        if not self.takeoff_requested:
            self.control_loop_count += 1
            if self.armed:
                self.publish_preflight_tilt_test_setpoint(current_time, dt)
            else:
                self.publish_idle_direct_actuator_setpoint()
            now = time.time()
            if now - self._last_debug_print_time >= self.debug_print_period_s:
                self.get_logger().info(
                    f'地面自检中：Offboard={self.is_offboard()} | Armed={self.armed} | '
                    'motors=0，按 o 后 Arm 并起飞悬停'
                )
                self._last_debug_print_time = now
            self._write_diagnostic_row(current_time, 'preflight_direct')
            return

        if not self.use_px4_position_takeoff and not self.armed:
            self.control_loop_count += 1
            self.last_F1 = 0.0
            self.last_F2 = 0.0
            self.last_F3 = 0.0
            self.last_W = np.zeros(6)
            self.publish_idle_direct_actuator_setpoint()
            self._write_diagnostic_row(current_time, 'takeoff_wait_arm_direct')
            now = time.time()
            if now - self._last_debug_print_time >= self.debug_print_period_s:
                self.get_logger().info(
                    '已收到起飞许可，正在等待 PX4 armed 状态确认；direct 电机保持零输出。'
                )
                self._last_debug_print_time = now
            return

        if self.use_px4_position_takeoff:
            self.control_loop_count += 1
            self.last_F1 = 0.0
            self.last_F2 = 0.0
            self.last_F3 = 0.0
            self.last_W = np.zeros(6)
            self.publish_px4_trajectory_setpoint()
            self._write_diagnostic_row(current_time, 'px4_position')
            now = time.time()
            if now - self._last_debug_print_time >= self.debug_print_period_s:
                self.get_logger().info(
                    f'PX4 位置 Offboard 起飞/悬停 dt={dt * 1000:.1f}ms | '
                    f'z={self.position[2] - self._z0:+.2f}m -> {self.target_position[2]:.2f}m'
                )
                self._last_debug_print_time = now
            return

        safety_reason = self._direct_safety_violation_reason()
        if safety_reason:
            self.direct_safety_cutoff_reason = safety_reason
            now = time.time()
            if now - self._last_direct_safety_log_time > 1.0:
                level = self.get_logger().error if self.direct_safety_shutdown_enabled else self.get_logger().warn
                level(f'DIRECT SAFETY: {safety_reason}')
                self._last_direct_safety_log_time = now

            if self.direct_safety_shutdown_enabled:
                self.control_loop_count += 1
                self._publish_direct_safety_cutoff(current_time)
                return
        else:
            self.direct_safety_cutoff_reason = ''

        self.control_loop_count += 1
        self.publish_px4_equivalent_direct_commands(current_time, dt)
        self._write_diagnostic_row(current_time, 'direct_px4_equiv')

        now = time.time()
        if now - self._last_debug_print_time >= self.debug_print_period_s:
            pos_rel_z = self.position[2] - self._z0 if self._z0_initialized else self.position[2]
            self.get_logger().info(
                f'Direct px4_equiv dt={dt * 1000:.1f}ms | Offboard={self.is_offboard()} | '
                f'Armed={self.armed} | z={pos_rel_z:+.2f}m -> {self.target_position[2]:.2f}m'
            )
            self._last_debug_print_time = now
        return

    @staticmethod
    def _slew_limit(current, target, rate_limit, dt):
        delta = target - current
        max_delta = rate_limit * dt
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target

    @staticmethod
    def _continuous_bounded_angle(angle, reference, limit):
        unwrapped = reference + math.atan2(
            math.sin(angle - reference),
            math.cos(angle - reference),
        )
        return float(np.clip(unwrapped, -limit, limit))

    @staticmethod
    def _thrust_to_normalized_motor_control(
        thrust,
        max_thrust,
        motor_constant=8.54858e-05,
        min_velocity=10.0,
    ):
        if thrust <= 0.0 or max_thrust <= 0.0 or motor_constant <= 0.0:
            return 0.0
        max_velocity = math.sqrt(max_thrust / motor_constant)
        velocity_range = max_velocity - min_velocity
        if velocity_range <= 1e-3:
            return 0.0
        velocity = math.sqrt(max(thrust, 0.0) / motor_constant)
        return float(np.clip((velocity - min_velocity) / velocity_range, 0.0, 1.0))

    @staticmethod
    def _thrust_to_normalized_bidirectional_motor_control(
        thrust,
        max_thrust,
    ):
        if abs(thrust) <= 1e-8 or max_thrust <= 1e-8:
            return 0.0
        control = math.sqrt(abs(thrust) / max_thrust)
        return float(np.clip(control if thrust > 0.0 else -control, -1.0, 1.0))

    def _allocator_wrench_from_body_force_torque(self, f_body: np.ndarray, tau_c: np.ndarray) -> np.ndarray:
        """Map PX4 FRD body force/torque to the Hnuter allocator wrench.

        The custom Hnuter airframe reports two equivalent hover attitude branches
        during SITL. attitude_callback canonicalizes them to the normal FRD
        branch before control, so the allocator mapping can stay single-valued.
        """
        return np.array([
            float(self.allocator_force_x_sign * f_body[0]),
            float(self.allocator_force_y_sign * f_body[1]),
            float(-f_body[2]),
            float(tau_c[0]),
            float(-tau_c[1]),
            float(-tau_c[2]),
        ], dtype=float)

    @staticmethod
    def _apply_firmware_tail_pitch_bias(
        allocator_pitch_torque: float,
        max_tail_thrust: float,
        tail_arm: float,
        pitch_torque_bias: float,
        tail_torque_sign: float,
    ) -> float:
        """Apply PX4 HNTR_PITCH_BIAS and HNTR_TAIL_SIGN semantics."""
        torque_scale = max(
            float(max_tail_thrust) * float(tail_arm), 1e-8
        )
        normalized_torque = float(np.clip(
            float(allocator_pitch_torque) / torque_scale,
            -1.0,
            1.0,
        ))
        biased_torque = float(np.clip(
            normalized_torque
            + float(np.clip(pitch_torque_bias, -1.0, 1.0)),
            -1.0,
            1.0,
        ))
        direction = -1.0 if float(tail_torque_sign) < 0.0 else 1.0
        return direction * biased_torque * torque_scale

    def _direct_prearm_failure_reason(self) -> str:
        if (self.use_px4_position_takeoff
                or not self.takeoff_requested
                or not self.direct_prearm_level_check_enabled):
            return ''

        roll = float(np.arctan2(self.R[2, 1], self.R[2, 2]))
        pitch = float(np.arcsin(np.clip(-self.R[2, 0], -1.0, 1.0)))

        if abs(roll) > self.direct_prearm_level_limit_rad:
            return (
                f'roll={math.degrees(roll):.1f}deg exceeds prearm level limit '
                f'{math.degrees(self.direct_prearm_level_limit_rad):.1f}deg'
            )

        if abs(pitch) > self.direct_prearm_level_limit_rad:
            return (
                f'pitch={math.degrees(pitch):.1f}deg exceeds prearm level limit '
                f'{math.degrees(self.direct_prearm_level_limit_rad):.1f}deg'
            )

        if self.direct_safety_cutoff and self.direct_safety_shutdown_enabled:
            return 'direct safety cutoff is latched'

        return ''

    def _direct_safety_violation_reason(self) -> str:
        if self.use_px4_position_takeoff or not (self.takeoff_requested and self.armed):
            return ''

        pos_rel_z = self.position[2] - self._z0 if self._z0_initialized else self.position[2]
        speed_xy = float(np.linalg.norm(self.velocity[:2]))

        if self.direct_safety_attitude_check_enabled and self.auto_traj_mode != 'attitude':
            roll = float(np.arctan2(self.R[2, 1], self.R[2, 2]))
            pitch = float(np.arcsin(np.clip(-self.R[2, 0], -1.0, 1.0)))
            if pos_rel_z > 0.15 and abs(roll) > self.direct_safety_pitch_limit_rad:
                return f'roll={math.degrees(roll):.1f}deg exceeds direct safety limit'
            if pos_rel_z > 0.15 and abs(pitch) > self.direct_safety_pitch_limit_rad:
                return f'pitch={math.degrees(pitch):.1f}deg exceeds direct safety limit'
        if pos_rel_z > 0.15 and speed_xy > self.direct_safety_speed_xy_limit:
            return f'xy speed={speed_xy:.1f}m/s exceeds direct safety limit'
        return ''

    def _direct_vertical_only_active(
        self,
        takeoff_elapsed_s: float,
        pos_rel_z: float,
    ) -> bool:
        """Return whether the direct controller must suppress horizontal force.

        The height condition is useful for the ground-takeoff debug path, but
        must be explicitly disabled when Hardware mode takes over in flight:
        its relative-Z origin is the handover altitude, so even millimetres of
        descent must not re-enable a takeoff-only vertical-force guard.
        """
        time_guard_active = (
            float(takeoff_elapsed_s)
            < float(self.direct_takeoff_vertical_only_time_s)
        )
        height_guard_active = bool(
            self.direct_takeoff_vertical_only_height_enabled
            and float(pos_rel_z)
            < float(self.direct_takeoff_vertical_only_height_m)
        )
        return bool(time_guard_active or height_guard_active)

    def _publish_direct_safety_cutoff(self, current_time: float):
        if not self.direct_safety_cutoff:
            reason = self._direct_safety_violation_reason()
            self.direct_safety_cutoff_reason = reason or 'direct safety cutoff requested'
            self.direct_safety_cutoff = True
            self.get_logger().error(
                f'DIRECT SAFETY CUTOFF: {self.direct_safety_cutoff_reason}. '
                'Motors are forced to idle; automatic in-air disarm is disabled.'
            )

        self.last_F1 = 0.0
        self.last_F2 = 0.0
        self.last_F3 = 0.0
        self.last_W = np.zeros(6)
        self.integral_pos_error[:] = 0.0
        self.integral_e_R[:] = 0.0
        self.takeoff_requested = False
        self.startup_blocked_after_disarm = True
        self.preflight_disarm_waiting_for_o = True
        self.was_armed_once = True
        self.auto_arm_attempts = self.max_auto_arm_attempts
        self.publish_idle_direct_actuator_setpoint()

        self._write_diagnostic_row(current_time, 'direct_safety_cutoff')

    def publish_px4_equivalent_direct_commands(self, current_time: float, dt: float):
        if not self.armed:
            self.integral_pos_error[:] = 0.0
            self.integral_e_R[:] = 0.0
            self.last_F1 = 0.0
            self.last_F2 = 0.0
            self.last_F3 = 0.0
            self.last_W = np.zeros(6)
            self.publish_idle_direct_actuator_setpoint()
            return

        target_abs_z_enu = float(self._z0 + self.target_position[2]) if self._z0_initialized else float(self.position[2])
        pos_ned = np.array([self.position[1], self.position[0], -self.position[2]], dtype=float)
        vel_ned = np.array([self.velocity[1], self.velocity[0], -self.velocity[2]], dtype=float)
        pos_sp_ned = np.array([self.target_position[1], self.target_position[0], -target_abs_z_enu], dtype=float)
        vel_sp_ned = np.array([self.target_velocity[1], self.target_velocity[0], -self.target_velocity[2]], dtype=float)
        acc_ff_ned = np.array([self.target_acceleration[1], self.target_acceleration[0], -self.target_acceleration[2]], dtype=float)

        takeoff_elapsed_s = (
            current_time - self._takeoff_lock_start_time_s
            if self._takeoff_lock_start_time_s is not None else 100.0
        )
        tilt_suppress_active = takeoff_elapsed_s < self.takeoff_tilt_suppress_time_s
        xy_lock_active = (
            takeoff_elapsed_s >= self.takeoff_tilt_suppress_time_s
            and takeoff_elapsed_s < self.takeoff_xy_lock_time_s
        )
        self._xy_lock_active = bool(xy_lock_active)
        pos_rel_z = self.position[2] - self._z0 if self._z0_initialized else self.position[2]
        vertical_only_active = self._direct_vertical_only_active(
            takeoff_elapsed_s,
            pos_rel_z,
        )

        auto_attitude_active = self.auto_traj_mode == 'attitude'
        attitude_altitude_only_active = auto_attitude_active and self.attitude_test_altitude_only

        if xy_lock_active and self._xy_lock_initialized:
            pos_sp_ned[0] = float(self._xy_lock_position[1])
            pos_sp_ned[1] = float(self._xy_lock_position[0])
            vel_sp_ned[0] = 0.0
            vel_sp_ned[1] = 0.0
            acc_ff_ned[0] = 0.0
            acc_ff_ned[1] = 0.0

        pos_error = pos_sp_ned - pos_ned
        vel_error = vel_sp_ned - vel_ned

        if attitude_altitude_only_active:
            pos_error[0] = 0.0
            pos_error[1] = 0.0
            vel_error[0] = 0.0
            vel_error[1] = 0.0
            acc_ff_ned[0] = 0.0
            acc_ff_ned[1] = 0.0
            self.integral_pos_error[0] = 0.0
            self.integral_pos_error[1] = 0.0

        position_integral_enabled = self.direct_pos_Ki_ned > 1e-8
        self.integral_pos_error[~position_integral_enabled] = 0.0
        self.integral_pos_error[position_integral_enabled] += (
            pos_error[position_integral_enabled] * dt
        )
        self.integral_pos_error = np.clip(
            self.integral_pos_error,
            -self.direct_pos_integral_limit_ned,
            self.direct_pos_integral_limit_ned,
        )

        acc_des = self._direct_position_acceleration_ned(
            acc_ff_ned,
            pos_error,
            vel_error,
            xy_lock_active,
        )
        if xy_lock_active:
            max_acc_xy = self.xy_lock_max_acc_xy
        elif auto_attitude_active:
            max_acc_xy = self.attitude_test_max_acc_xy
        else:
            max_acc_xy = self.max_acc_xy
        acc_des[:2] = self._limit_direct_horizontal_acceleration_ned(
            acc_des[:2],
            max_acc_xy,
        )
        acc_des[2] = float(np.clip(acc_des[2], -self.max_acc_z, self.max_acc_z))

        f_world = self.mass * (acc_des - np.array([0.0, 0.0, self.gravity], dtype=float))
        f_body = self._apply_direct_body_force_trim(self.R_ned_frd.T @ f_world)
        if tilt_suppress_active or vertical_only_active:
            f_body[0] = 0.0
            f_body[1] = 0.0

        tilt_limit = (
            self.takeoff_tilt_limit_rad if tilt_suppress_active
            else (self.xy_lock_tilt_limit_rad if xy_lock_active else self.alpha_limit_rad)
        )
        if tilt_limit < math.radians(89.0):
            fz_abs = abs(float(f_body[2]))
            if fz_abs > 1e-3:
                max_xy = fz_abs * math.tan(tilt_limit)
                fxy_norm = float(np.linalg.norm(f_body[:2]))
                if fxy_norm > max_xy and fxy_norm > 1e-5:
                    f_body[0] *= max_xy / fxy_norm
                    f_body[1] *= max_xy / fxy_norm
            else:
                f_body[0] = 0.0
                f_body[1] = 0.0
        # Outside takeoff protection, do not pre-clip body-Y force. The
        # secondary tilt realizes this force and must retain enough authority
        # to hold position while the vehicle is rolled.

        manual_yaw_active = abs(float(self._last_manual_cmd.get('yaw_rate', 0.0))) > 1e-5

        R_des = (
            self.target_R_des_ned_frd
            if self.target_R_des_ned_frd is not None
            else self._direct_desired_attitude_ned_frd(self.target_attitude)
        )
        relative_rotation = R_des.T @ self.R_ned_frd
        if self.direct_quaternion_error_enabled:
            e_R, error_quaternion, attitude_error_angle = quaternion_attitude_error(
                R_des,
                self.R_ned_frd,
                self._attitude_error_quaternion,
            )
            self._attitude_error_quaternion = error_quaternion
        else:
            e_rm = 0.5 * (relative_rotation - relative_rotation.T)
            e_R = np.array(
                [e_rm[2, 1], e_rm[0, 2], e_rm[1, 0]],
                dtype=float,
            )
            attitude_error_angle = math.acos(float(np.clip(
                0.5 * (np.trace(relative_rotation) - 1.0),
                -1.0,
                1.0,
            )))
            self._attitude_error_quaternion = None

        self.last_thrust_axis_alignment = float(np.clip(
            np.dot(R_des[:, 2], self.R_ned_frd[:, 2]), -1.0, 1.0
        ))
        self.last_full_error_blend = 0.0
        if auto_attitude_active and self.direct_reduced_tilt_error_enabled:
            e_R, self.last_thrust_axis_alignment, self.last_full_error_blend = (
                reduced_tilt_attitude_error(R_des, self.R_ned_frd, e_R)
            )

        if auto_attitude_active:
            target_rate = np.array([
                float(self.target_attitude_rate[0]),
                float(-self.target_attitude_rate[1]),
                float(-self.target_attitude_rate[2]),
            ], dtype=float)
        elif manual_yaw_active:
            target_rate = np.array([0.0, 0.0, float(-self.target_attitude_rate[2])], dtype=float)
        else:
            target_rate = np.zeros(3)
        omega_error = self.angular_velocity_frd - self.R_ned_frd.T @ R_des @ target_rate
        if tilt_suppress_active:
            KR = self.direct_takeoff_KR
            Domega = self.direct_takeoff_Domega
            attitude_Ki = np.zeros(3)
            tau_limit = self.direct_takeoff_tau_limit
        elif xy_lock_active:
            KR = self.direct_xy_lock_KR
            Domega = self.direct_xy_lock_Domega
            attitude_Ki = np.zeros(3)
            tau_limit = self.direct_xy_lock_tau_limit
        else:
            KR = self.direct_KR.copy()
            Domega = self.direct_Domega.copy()
            attitude_Ki = self.direct_attitude_Ki.copy()
            tau_limit = self.direct_tau_limit.copy()
        if not self.direct_yaw_control_enabled:
            attitude_Ki[2] = 0.0

        yaw_authority_scale = 1.0
        if (
            auto_attitude_active
            and self.direct_large_tilt_yaw_scheduling_enabled
            and not (tilt_suppress_active or xy_lock_active)
        ):
            desired_tilt = math.acos(float(np.clip(R_des[2, 2], -1.0, 1.0)))
            current_tilt = math.acos(float(np.clip(self.R_ned_frd[2, 2], -1.0, 1.0)))
            yaw_authority_scale = large_tilt_yaw_scale(
                max(desired_tilt, current_tilt),
                self.direct_large_tilt_yaw_start_rad,
                self.direct_large_tilt_yaw_full_rad,
                self.direct_large_tilt_yaw_min_scale,
            )
            KR[2] *= yaw_authority_scale
            Domega[2] *= math.sqrt(yaw_authority_scale)
            attitude_Ki[2] *= yaw_authority_scale
            tau_limit[2] *= yaw_authority_scale
            self.integral_e_R[2] *= math.exp(-dt * (1.0 - yaw_authority_scale) / 0.3)
        self.last_yaw_authority_scale = yaw_authority_scale

        previous_integral = self._update_attitude_integral(
            e_R,
            attitude_error_angle,
            attitude_Ki,
            dt,
        )

        gyro_torque = np.zeros(3)
        if self.direct_attitude_gyro_compensation_enabled:
            gyro_torque = np.cross(
                self.angular_velocity_frd,
                self.J @ self.angular_velocity_frd,
            )
        tau_c = (
            -KR * e_R
            - Domega * omega_error
            - attitude_Ki * self.integral_e_R
            + gyro_torque
        )
        if not self.direct_yaw_control_enabled:
            tau_c[2] = 0.0
        if self._reject_saturating_attitude_integration(
            previous_integral,
            attitude_Ki,
            tau_c,
            tau_limit,
        ):
            tau_c = (
                -KR * e_R
                - Domega * omega_error
                - attitude_Ki * self.integral_e_R
                + gyro_torque
            )
            if not self.direct_yaw_control_enabled:
                tau_c[2] = 0.0
        tau_c = np.clip(tau_c, -tau_limit, tau_limit)
        self.last_attitude_error = e_R.copy()
        self.last_attitude_error_angle_rad = float(attitude_error_angle)
        self.last_omega_error = omega_error.copy()
        self.last_tau_c = tau_c.copy()

        # Physical allocator limits. F1/F2 are the total thrust of the two
        # motors on each front arm; F3 is the single tail-motor thrust.
        max_thrust_per_arm = 100.0
        max_tail_thrust = 50.0
        r_x = 0.105
        r_z = -0.013

        W = self._allocator_wrench_from_body_force_torque(f_body, tau_c)
        if tilt_suppress_active or vertical_only_active:
            W[0] = 0.0
            W[1] = 0.0
        W[4] = self._apply_firmware_tail_pitch_bias(
            W[4],
            max_tail_thrust,
            self.l2,
            self.pitch_torque_bias,
            self.tail_torque_sign,
        )
        if self.direct_takeoff_thrust_floor_enabled and takeoff_elapsed_s < 12.0:
            W[2] = max(float(W[2]), self.mass * self.gravity * 0.70)

        if self.publish_land_detector_thrust_setpoint or self.publish_allocator_setpoints_in_direct:
            timestamp = self.timestamp_now_us()
            thrust_msg = VehicleThrustSetpoint()
            thrust_msg.timestamp = timestamp
            if hasattr(thrust_msg, 'timestamp_sample'):
                thrust_msg.timestamp_sample = timestamp
            if self.publish_allocator_setpoints_in_direct:
                thrust_msg.xyz = [
                    float(W[0] / max_thrust_per_arm),
                    float(-W[1] / max_thrust_per_arm),
                    float(-W[2] / (self.mass * self.gravity * 2.0)),
                ]
            else:
                # This message is only a land-detector hint in direct mode.
                # Preserve thrust magnitude when body-Z thrust crosses zero at
                # a 90-degree attitude, otherwise PX4 can report landed in air.
                normalized_thrust = float(np.clip(
                    np.linalg.norm(W[:3]) / (self.mass * self.gravity * 2.0),
                    0.0,
                    1.0,
                ))
                thrust_msg.xyz = [0.0, 0.0, -normalized_thrust]
            self.vehicle_thrust_setpoint_pub.publish(thrust_msg)

        if self.publish_allocator_setpoints_in_direct:
            timestamp = self.timestamp_now_us()
            torque_msg = VehicleTorqueSetpoint()
            torque_msg.timestamp = timestamp
            if hasattr(torque_msg, 'timestamp_sample'):
                torque_msg.timestamp_sample = timestamp
            torque_msg.xyz = [
                float(W[3] / (max_thrust_per_arm * self.l1)),
                float(W[4] / (max_tail_thrust * self.l2)),
                float(-W[5] / (max_thrust_per_arm * self.l1)),
            ]
            self.vehicle_torque_setpoint_pub.publish(torque_msg)

        u1 = W[0] / 2.0 - W[5] / (2.0 * self.l1)
        u4 = W[0] / 2.0 + W[5] / (2.0 * self.l1)
        ty_parasitic = r_z * W[0] - r_x * W[2]
        F3 = (
            W[4] - self.tail_collective_comp * ty_parasitic
        ) / (r_x + self.l2)
        Fz_front = W[2] - F3
        tx_parasitic = -r_z * W[1]
        tx_comp = W[3] - tx_parasitic
        u2 = Fz_front / 2.0 + tx_comp / (2.0 * self.l1)
        u5 = Fz_front / 2.0 - tx_comp / (2.0 * self.l1)
        u3 = -W[1] / 2.0
        u6 = -W[1] / 2.0

        F1 = math.sqrt(u1 * u1 + u2 * u2 + u3 * u3)
        F2 = math.sqrt(u4 * u4 + u5 * u5 + u6 * u6)
        eps = 1e-8
        alpha1 = math.atan2(u1, u2)
        alpha2 = math.atan2(u4, u5)
        theta1 = math.asin(float(np.clip(u3 / max(F1, eps), -0.99, 0.99)))
        theta2 = math.asin(float(np.clip(u6 / max(F2, eps), -0.99, 0.99)))

        F1 = float(np.clip(F1, 0.0, max_thrust_per_arm))
        F2 = float(np.clip(F2, 0.0, max_thrust_per_arm))
        F3 = float(np.clip(
            F3,
            -max_tail_thrust if self.allow_tail_reverse else 0.0,
            max_tail_thrust,
        ))
        alpha_limit = self.alpha_limit_rad
        theta_limit = self.theta_limit_rad
        if tilt_suppress_active:
            alpha_limit = self.takeoff_tilt_limit_rad
            theta_limit = self.takeoff_tilt_limit_rad
        elif xy_lock_active:
            alpha_limit = self.xy_lock_tilt_limit_rad
            theta_limit = self.xy_lock_tilt_limit_rad
        if alpha_limit >= math.radians(179.0):
            alpha1 = self._continuous_bounded_angle(alpha1, self._alpha1_cmd, alpha_limit)
            alpha2 = self._continuous_bounded_angle(alpha2, self._alpha2_cmd, alpha_limit)
        else:
            alpha1 = float(np.clip(alpha1, -alpha_limit, alpha_limit))
            alpha2 = float(np.clip(alpha2, -alpha_limit, alpha_limit))
        theta1 = float(np.clip(theta1, -theta_limit, theta_limit))
        theta2 = float(np.clip(theta2, -theta_limit, theta_limit))

        right_single = 0.5 * F2
        left_single = 0.5 * F1
        max_front_motor_thrust = 0.5 * max_thrust_per_arm
        motor_controls = [
            self._thrust_to_normalized_motor_control(
                right_single, max_front_motor_thrust
            ),
            self._thrust_to_normalized_motor_control(
                right_single, max_front_motor_thrust
            ),
            self._thrust_to_normalized_motor_control(
                left_single, max_front_motor_thrust
            ),
            self._thrust_to_normalized_motor_control(
                left_single, max_front_motor_thrust
            ),
            (
                self._thrust_to_normalized_bidirectional_motor_control(
                    F3, max_tail_thrust
                )
                if self.allow_tail_reverse
                else self._thrust_to_normalized_motor_control(
                    F3, max_tail_thrust
                )
            ),
        ]

        dt_slew = float(np.clip(dt, 0.0, 0.2))
        self._alpha1_cmd = self._slew_limit(self._alpha1_cmd, alpha1, self.servo_rate_limit_rad_s, dt_slew)
        self._alpha2_cmd = self._slew_limit(self._alpha2_cmd, alpha2, self.servo_rate_limit_rad_s, dt_slew)
        secondary_rate_limit = self._secondary_joint_rate_limit_rad_s()
        self._theta1_cmd = self._slew_limit(self._theta1_cmd, theta1, secondary_rate_limit, dt_slew)
        self._theta2_cmd = self._slew_limit(self._theta2_cmd, theta2, secondary_rate_limit, dt_slew)

        self.last_F1 = F1
        self.last_F2 = F2
        self.last_F3 = F3
        self.last_W = W
        self.publish_direct_actuator_setpoint(
            motor_controls=motor_controls,
            alpha1=self._alpha1_cmd,
            alpha2=self._alpha2_cmd,
            theta1=self._theta1_cmd,
            theta2=self._theta2_cmd,
        )

    def _diagnostic_file_prefix(self):
        return f'hnuter_{self.debug_control_mode}_debug'

    def _diagnostic_extra_header(self):
        return []

    def _diagnostic_extra_values(self):
        return []

    def _diagnostic_header(self):
        columns = [
            'time_s', 'mode', 'offboard', 'armed', 'nav_state',
            'landed', 'maybe_landed', 'ground_contact', 'freefall', 'land_has_low_throttle',
            'position_x_enu_m', 'position_y_enu_m', 'position_z_rel_m',
            'velocity_x_enu_mps', 'velocity_y_enu_mps', 'velocity_z_enu_mps',
            'roll_deg', 'pitch_deg', 'yaw_deg', 'continuous_test_pitch_deg',
            'target_x_enu_m', 'target_y_enu_m', 'target_z_rel_m',
            'target_vx_enu_mps', 'target_vy_enu_mps', 'target_vz_enu_mps',
            'target_ax_enu_mps2', 'target_ay_enu_mps2', 'target_az_enu_mps2',
            'target_roll_deg', 'target_pitch_deg', 'target_yaw_deg',
            'wrench_fx_body_n', 'wrench_fy_body_n', 'wrench_fz_body_n',
            'wrench_tau_x_nm', 'wrench_tau_y_nm', 'wrench_tau_z_nm',
            'tail_pitch_bias_normalized', 'tail_torque_sign',
            'tail_collective_comp',
            'allocated_F1_n', 'allocated_F2_n', 'allocated_F3_n',
            'px4_out_motor_age_s', 'px4_out_servo_age_s', 'auto_traj_mode',
        ]
        columns += [f'cmd_motor_{i}' for i in range(5)]
        columns += [f'cmd_servo_{i}' for i in range(4)]
        columns += [f'px4_out_motor_{i}' for i in range(5)]
        columns += [f'px4_out_servo_{i}' for i in range(4)]
        columns += [
            'takeoff_elapsed_s', 'takeoff_start_z_rel_m', 'control_dt_s',
            'xy_lock_active', 'direct_safety_cutoff',
            'angular_p_frd_rps', 'angular_q_frd_rps', 'angular_r_frd_rps',
            'direct_safety_reason',
            'direct_pos_kp_n', 'direct_pos_kp_e', 'direct_pos_kp_d',
            'direct_pos_kd_n', 'direct_pos_kd_e', 'direct_pos_kd_d',
            'direct_pos_ki_n', 'direct_pos_ki_e', 'direct_pos_ki_d',
            'attitude_error_angle_deg',
            'attitude_error_x', 'attitude_error_y', 'attitude_error_z',
            'omega_error_p_rps', 'omega_error_q_rps', 'omega_error_r_rps',
            'attitude_integral_x', 'attitude_integral_y', 'attitude_integral_z',
            'commanded_tau_x_nm', 'commanded_tau_y_nm', 'commanded_tau_z_nm',
            'yaw_authority_scale', 'thrust_axis_alignment', 'full_error_blend',
            'manual_attitude_axis', 'manual_des_roll_deg', 'manual_des_pitch_deg',
            'manual_roll_rate_dps', 'manual_pitch_rate_dps', 'gamepad_rb_pressed',
            'gamepad_raw_vx_body_mps', 'gamepad_raw_vy_body_mps',
            'gamepad_vx_body_mps', 'gamepad_vy_body_mps',
        ]
        columns += self._diagnostic_extra_header()
        return columns

    @staticmethod
    def _diag_values(values, count):
        array = np.asarray(values, dtype=float).reshape(-1)
        if array.size < count:
            array = np.pad(array, (0, count - array.size), constant_values=np.nan)
        return [float(v) for v in array[:count]]

    def _px4_topic_age_s(self, timestamp_us):
        if self.px4_timestamp <= 0 or int(timestamp_us) <= 0:
            return float('nan')
        return float((self.px4_timestamp - int(timestamp_us)) / 1_000_000.0)

    def _write_diagnostic_row(self, current_time, mode):
        if not self.diagnostic_enabled or self._diagnostic_writer is None:
            return
        if (
            self._last_diagnostic_log_time >= 0.0 and
            current_time - self._last_diagnostic_log_time < self.diagnostic_period_s
        ):
            return

        self._last_diagnostic_log_time = float(current_time)
        pos_curr_rel_z = self.position[2] - self._z0 if self._z0_initialized else self.position[2]
        roll = np.arctan2(self.R[2, 1], self.R[2, 2])
        pitch = np.arcsin(np.clip(-self.R[2, 0], -1.0, 1.0))
        yaw = np.arctan2(self.R[1, 0], self.R[0, 0])
        row = [
            float(current_time), mode, int(self.is_offboard()), int(self.armed),
            -1 if self.nav_state is None else int(self.nav_state),
            int(self.land_detected.get('landed', False)),
            int(self.land_detected.get('maybe_landed', False)),
            int(self.land_detected.get('ground_contact', False)),
            int(self.land_detected.get('freefall', False)),
            int(self.land_detected.get('has_low_throttle', False)),
            float(self.position[0]), float(self.position[1]), float(pos_curr_rel_z),
            float(self.velocity[0]), float(self.velocity[1]), float(self.velocity[2]),
            float(np.degrees(roll)), float(np.degrees(pitch)), float(np.degrees(yaw)),
            float(np.degrees(self._continuous_attitude_test_pitch_rad())),
            float(self.target_position[0]), float(self.target_position[1]), float(self.target_position[2]),
            float(self.target_velocity[0]), float(self.target_velocity[1]), float(self.target_velocity[2]),
            float(self.target_acceleration[0]), float(self.target_acceleration[1]), float(self.target_acceleration[2]),
            float(np.degrees(self.target_attitude[0])),
            float(np.degrees(self.target_attitude[1])),
            float(np.degrees(self.target_attitude[2])),
            *self._diag_values(self.last_W, 6),
            float(self.pitch_torque_bias),
            float(self.tail_torque_sign),
            float(self.tail_collective_comp),
            float(self.last_F1), float(self.last_F2), float(self.last_F3),
            self._px4_topic_age_s(self.px4_out_motor_timestamp),
            self._px4_topic_age_s(self.px4_out_servo_timestamp),
            self.auto_traj_mode,
        ]
        row += self._diag_values(self.last_motor_cmd, 5)
        row += self._diag_values(self.last_servo_cmd, 4)
        row += self._diag_values(self.px4_out_motor_cmd, 5)
        row += self._diag_values(self.px4_out_servo_cmd, 4)
        takeoff_elapsed_s = (
            current_time - self._takeoff_lock_start_time_s
            if self._takeoff_lock_start_time_s is not None else float('nan')
        )
        row += [
            float(takeoff_elapsed_s),
            float(self._takeoff_start_z_rel),
            float(self._last_control_dt_s),
            int(self._xy_lock_active),
            int(self.direct_safety_cutoff),
            float(self.angular_velocity_frd[0]),
            float(self.angular_velocity_frd[1]),
            float(self.angular_velocity_frd[2]),
            self.direct_safety_cutoff_reason,
            *self._diag_values(self.direct_pos_Kp_ned, 3),
            *self._diag_values(self.direct_pos_Kd_ned, 3),
            *self._diag_values(self.direct_pos_Ki_ned, 3),
            float(math.degrees(self.last_attitude_error_angle_rad)),
            *self._diag_values(self.last_attitude_error, 3),
            *self._diag_values(self.last_omega_error, 3),
            *self._diag_values(self.integral_e_R, 3),
            *self._diag_values(self.last_tau_c, 3),
            float(self.last_yaw_authority_scale),
            float(self.last_thrust_axis_alignment),
            float(self.last_full_error_blend),
            str(self._last_manual_cmd.get('attitude_axis', 'roll')),
            float(math.degrees(self.manual_des_roll)),
            float(math.degrees(self.manual_des_pitch)),
            float(math.degrees(self._last_manual_cmd.get('roll_rate', 0.0))),
            float(math.degrees(self._last_manual_cmd.get('pitch_rate', 0.0))),
            int(bool(self._last_manual_cmd.get('rb_pressed', False))),
            float(self._last_manual_cmd.get('raw_vx_b', 0.0)),
            float(self._last_manual_cmd.get('raw_vy_b', 0.0)),
            float(self._last_manual_cmd.get('vx_b', 0.0)),
            float(self._last_manual_cmd.get('vy_b', 0.0)),
        ]
        row += self._diagnostic_extra_values()
        self._diagnostic_writer.writerow(row)

    # ============================================================
    # Status/shutdown
    # ============================================================
    def print_status(self):
        if not self.data_received:
            self.get_logger().info('等待 PX4 odometry/attitude/status 数据...')
            return

        control_hz = self.control_loop_count
        self.control_loop_count = 0
        pos_curr_rel_z = self.position[2] - self._z0 if self._z0_initialized else self.position[2]
        current_roll_deg = float(np.degrees(np.arctan2(self.R[2, 1], self.R[2, 2])))
        current_pitch_deg = float(np.degrees(np.arcsin(np.clip(-self.R[2, 0], -1.0, 1.0))))
        current_yaw_deg = float(np.degrees(self._current_yaw_enu()))
        target_roll_deg = float(np.degrees(self.target_attitude[0]))
        target_pitch_deg = float(np.degrees(self.target_attitude[1]))
        target_yaw_deg = float(np.degrees(self.target_attitude[2]))
        continuous_test_pitch_deg = float(np.degrees(self._continuous_attitude_test_pitch_rad()))
        self.get_logger().info(
            f"\n{'=' * 72}\n"
            f"Mode: Offboard={self.is_offboard()} | Armed={self.armed} | nav_state={self.nav_state} | ctrl≈{control_hz}Hz\n"
            f"Land: landed={self.land_detected.get('landed', False)} | maybe={self.land_detected.get('maybe_landed', False)} | "
            f"ground={self.land_detected.get('ground_contact', False)} | low_thr={self.land_detected.get('has_low_throttle', False)}\n"
            f"Takeoff gate: requested={self.takeoff_requested} | preflight_done={self.preflight_tilt_test_finished} | "
            f"waiting_rearm_o={self.preflight_disarm_waiting_for_o}\n"
            f"Target ENU/Zrel: [{self.target_position[0]:6.2f}, {self.target_position[1]:6.2f}, {self.target_position[2]:6.2f}] m\n"
            f"Current ENU/Zrel: [{self.position[0]:6.2f}, {self.position[1]:6.2f}, {pos_curr_rel_z:6.2f}] m\n"
            f"Attitude ENU/FLU deg: target R/P/Y=[{target_roll_deg:+5.1f}, {target_pitch_deg:+5.1f}, {target_yaw_deg:+5.1f}] | "
            f"current R/P/Y=[{current_roll_deg:+5.1f}, {current_pitch_deg:+5.1f}, {current_yaw_deg:+5.1f}]\n"
            f"Continuous test pitch: target={target_pitch_deg:+6.1f}° | current={continuous_test_pitch_deg:+6.1f}°\n"
            f"SO(3) error={math.degrees(self.last_attitude_error_angle_rad):5.1f}° | "
            f"eR={np.round(self.last_attitude_error, 3).tolist()} | "
            f"omega_err={np.round(self.last_omega_error, 3).tolist()} | "
            f"thrust_align={self.last_thrust_axis_alignment:.3f}\n"
            f"Tune: step={np.round(np.degrees(self.attitude_step_axis_rad), 1).tolist()}deg | KR={np.round(self.direct_KR, 2).tolist()} | "
            f"D={np.round(self.direct_Domega, 2).tolist()} | "
            f"Ki={np.round(self.direct_attitude_Ki, 2).tolist()} | "
            f"tau_lim={np.round(self.direct_tau_limit, 2).tolist()} | "
            f"yaw_scale={self.last_yaw_authority_scale:.2f}\n"
            f"Keyboard trajectory: active={self.auto_traj_mode} | pending={self.pending_auto_traj_mode}\n"
            f"Gamepad XY raw=[{self._last_manual_cmd.get('raw_vx_b', 0.0):+4.2f}, "
            f"{self._last_manual_cmd.get('raw_vy_b', 0.0):+4.2f}] -> "
            f"filtered=[{self._last_manual_cmd['vx_b']:+4.2f}, "
            f"{self._last_manual_cmd['vy_b']:+4.2f}], "
            f"vz={self._last_manual_cmd['vz']:+4.2f}, yaw_rate={self._last_manual_cmd['yaw_rate']:+4.2f}, "
            f"LT={self._last_manual_cmd.get('lt', 0.0):4.2f}, RT={self._last_manual_cmd.get('rt', 0.0):4.2f}, "
            f"RB={int(bool(self._last_manual_cmd.get('rb_pressed', False)))}\n"
            f"Manual attitude: axis={self._last_manual_cmd.get('attitude_axis', 'roll')} | "
            f"R/P=[{np.degrees(self.manual_des_roll):+5.1f}, {np.degrees(self.manual_des_pitch):+5.1f}]° | "
            f"rates=[{np.degrees(self._last_manual_cmd.get('roll_rate', 0.0)):+5.1f}, "
            f"{np.degrees(self._last_manual_cmd.get('pitch_rate', 0.0)):+5.1f}]°/s\n"
            f"Wrench: Fx={self.last_W[0]:+5.2f}N, Fy={self.last_W[1]:+5.2f}N, Fz={self.last_W[2]:+5.2f}N\n"
            f"Thrust: F1={self.last_F1:5.2f}N | F2={self.last_F2:5.2f}N | F3={self.last_F3:5.2f}N\n"
            f"Tilt: A1={np.degrees(self._alpha1_cmd):+5.1f}° | A2={np.degrees(self._alpha2_cmd):+5.1f}° | "
            f"T1={np.degrees(self._theta1_cmd):+5.1f}° | T2={np.degrees(self._theta2_cmd):+5.1f}°\n"
            f"{'=' * 72}"
        )

    def destroy_node(self):
        try:
            self.keyboard.close()
        except Exception:
            pass
        try:
            self.gamepad.close()
        except Exception:
            pass
        try:
            if self._diagnostic_file is not None:
                self._diagnostic_file.close()
        except Exception:
            pass
        super().destroy_node()


@dataclass
class _StickSample:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    throttle: float = 0.0
    aux1: float = 0.0
    aux2: float = 0.0


class RCCommandManager:
    """Convert PX4 RC topics into the debug controller's manual commands."""

    def __init__(
        self,
        max_vxy_body_mps,
        max_vz: float,
        max_yaw_rate: float,
        max_attitude_rate_rad_s,
        max_attitude_angle_rad: float,
        attitude_sign,
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
        self.max_attitude_rate_rad_s = np.asarray(
            max_attitude_rate_rad_s, dtype=float
        ).reshape(2)
        self.max_attitude_angle_rad = max(float(max_attitude_angle_rad), 0.0)
        self.attitude_sign = np.sign(
            np.asarray(attitude_sign, dtype=float).reshape(2)
        )
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
        self.attitude_control_axis = 'roll+pitch'

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
            'attitude_limit_rad': self.max_attitude_angle_rad,
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
            aux1=float(getattr(message, 'aux1', 0.0)),
            aux2=float(getattr(message, 'aux2', 0.0)),
        )
        source = int(getattr(
            message, 'data_source', ManualControlSetpoint.SOURCE_RC
        ))
        self._manual_valid = (
            bool(getattr(message, 'valid', False))
            and source == ManualControlSetpoint.SOURCE_RC
            and self._finite_sticks(
                sample.roll, sample.pitch, sample.yaw, sample.throttle,
                sample.aux1, sample.aux2,
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
        aux1 = self._mapped_channel(message, RcChannels.FUNCTION_AUX_1)
        aux2 = self._mapped_channel(message, RcChannels.FUNCTION_AUX_2)
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
                aux1=0.0 if aux1 is None else float(aux1),
                aux2=0.0 if aux2 is None else float(aux2),
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
        aux1 = self._shape(sample.aux1)
        aux2 = self._shape(sample.aux2)

        target_vx_b = self.pitch_sign * pitch * self.max_vxy_body_mps[0]
        target_vy_b = self.roll_sign * roll * self.max_vxy_body_mps[1]
        target_vz = self.throttle_sign * throttle * self.max_vz
        target_yaw_rate = self.yaw_sign * yaw * self.max_yaw_rate
        target_roll_rate = (
            self.attitude_sign[0] * aux1 * self.max_attitude_rate_rad_s[0]
        )
        target_pitch_rate = (
            self.attitude_sign[1] * aux2 * self.max_attitude_rate_rad_s[1]
        )

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
        self.filtered_cmds['roll_rate'] = GamepadManager._filter_command(
            self.filtered_cmds['roll_rate'], target_roll_rate, dt,
            self.filter_tau,
        )
        self.filtered_cmds['pitch_rate'] = GamepadManager._filter_command(
            self.filtered_cmds['pitch_rate'], target_pitch_rate, dt,
            self.filter_tau,
        )
        self.filtered_cmds['attitude_limit_rad'] = self.max_attitude_angle_rad
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

    def takeoff_throttle(self) -> tuple[float, str]:
        """Return the shaped physical throttle-stick position and source."""
        sample, source = self._active_sample()
        if source == 'stale':
            return 0.0, source
        # Do not apply throttle_sign here. The takeoff interlock follows the
        # physical RC convention: -1 is stick low and +1 is stick high.
        return float(self._shape(sample.throttle)), source

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


class HnuterHardwareController(HnuterController):
    """Direct controller gated exclusively by PX4 Arm and Offboard states."""

    def _node_name(self):
        return 'hnuter_controller_direct_hardware'

    def _default_tuning_filename(self):
        return 'hnuter_direct_hardware_tuning.json'

    def _vehicle_command_publication_enabled(self):
        return False

    def _create_manual_input(self):
        self.rc_input = RCCommandManager(
            max_vxy_body_mps=self.gamepad_max_vxy_body_mps,
            max_vz=env_float('HNUTER_RC_MAX_VZ_MPS', 0.30),
            max_yaw_rate=env_float('HNUTER_RC_MAX_YAW_RATE_RPS', 0.40),
            max_attitude_rate_rad_s=self.rc_attitude_rate_rad_s,
            max_attitude_angle_rad=self.rc_attitude_angle_limit_rad,
            attitude_sign=self.rc_attitude_sign,
            deadzone=self.gamepad_deadzone,
            expo=self.gamepad_expo,
            filter_tau=self.gamepad_filter_tau_s,
            filter_tau_body_xy_s=self.gamepad_filter_tau_body_xy_s,
            max_acc_body_xy_mps2=self.gamepad_max_acc_body_xy_mps2,
            timeout_s=env_float('HNUTER_RC_TIMEOUT_S', 0.50),
            logger=self.get_logger(),
        )
        return self.rc_input

    def __init__(self) -> None:
        self._hardware_control_active = False
        self._restart_tracker = OffboardTaskRestartTracker()
        super().__init__()

        # Position mode is responsible for takeoff. When the pilot switches to
        # Offboard in the air, blend the final PX4 actuator command into the
        # external controller command while holding the measured pose.
        self.hardware_handover_duration_s = max(
            0.0, env_float('HNUTER_HARDWARE_HANDOVER_S', 0.80)
        )
        self.hardware_handover_snapshot_max_age_s = max(
            0.05, env_float('HNUTER_HARDWARE_HANDOVER_MAX_AGE_S', 0.30)
        )
        self._hardware_handover_start_timestamp_us = 0
        self._hardware_handover_motor_start = np.full(5, np.nan)
        self._hardware_handover_servo_start = np.full(4, np.nan)
        self._hardware_handover_snapshot_valid = False
        self._hardware_handover_blend = 0.0
        self._hardware_handover_state = 'inactive'

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
        self.direct_takeoff_vertical_only_height_enabled = False
        self.direct_takeoff_thrust_floor_enabled = False
        self.preflight_tilt_test_enabled = False
        self.get_logger().warn(
            'HARDWARE MODE: 本节点不会 Arm/Disarm，也不会切换 Offboard。'
            '请在 Position 模式手动解锁并起飞，再在空中切换 Offboard；'
            '节点常驻发送心跳，检测到 Armed + Offboard 后在当前位置无扰接管。'
        )
        self.get_logger().warn(
            '实机舵机映射要求 PX4 MAIN8--11 参数与控制器一致: '
            f'profile={self.hardware_firmware_profile}, '
            f'PWM={self.servo_pwm_min_us}/{self.servo_pwm_trim_us}/'
            f'{self.servo_pwm_max_us}us, '
            f'primary=+/-{math.degrees(self.primary_servo_angle_max_rad):.1f}deg, '
            f'secondary_servo=+/-{math.degrees(self.secondary_servo_angle_max_rad):.1f}deg, '
            f'HNTR_S2_GEAR={self.secondary_servo_gear_ratio:.3f}, '
            f'secondary_joint=+/-{math.degrees(self.theta_limit_rad):.1f}deg。'
        )
        self.get_logger().info(
            f'空中接管过渡时间 {self.hardware_handover_duration_s:.2f}s；'
            '过渡期间保持切换瞬间的位置和航向，并暂时忽略遥控运动指令。'
        )

    def _apply_tuning(self, data: dict):
        super()._apply_tuning(data)
        current_rate_deg_s = np.degrees(getattr(
            self, 'rc_attitude_rate_rad_s', np.radians([20.0, 20.0])
        ))
        self.rc_attitude_rate_rad_s = np.radians(np.clip(
            self._tuning_array(
                data, 'rc_attitude_rate_deg_s', current_rate_deg_s
            ),
            0.0,
            90.0,
        ))
        self.rc_attitude_angle_limit_rad = math.radians(float(np.clip(
            self._tuning_float(
                data,
                'rc_attitude_angle_limit_deg',
                math.degrees(getattr(
                    self, 'rc_attitude_angle_limit_rad', math.radians(45.0)
                )),
            ),
            0.0,
            90.0,
        )))
        self.rc_attitude_sign = np.sign(self._tuning_array(
            data,
            'rc_attitude_sign',
            getattr(self, 'rc_attitude_sign', np.array([-1.0, -1.0])),
        ))
        if hasattr(self, 'rc_input'):
            self.rc_input.max_attitude_rate_rad_s = self.rc_attitude_rate_rad_s.copy()
            self.rc_input.max_attitude_angle_rad = self.rc_attitude_angle_limit_rad
            self.rc_input.attitude_sign = self.rc_attitude_sign.copy()
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
        # This flag only enables the shared direct-control path. Hardware mode
        # never uses it to request takeoff or Arm.
        self.takeoff_requested = True
        self.pending_auto_traj_mode = self._restart_tracker.consume()
        self.auto_traj_mode = 'hover'
        self.integral_pos_error[:] = 0.0
        self.integral_e_R[:] = 0.0
        self._attitude_error_quaternion = None
        self._takeoff_lock_start_time_s = None
        self._takeoff_start_z_rel = 0.0
        self._xy_lock_active = False
        self.manual_des_roll = 0.0
        self.manual_des_pitch = 0.0

        # Capture the actual state at the Offboard rising edge. Relative Z is
        # reset to zero here, so a mode change never injects the old ground
        # takeoff height or a stale target from an earlier control session.
        current_yaw = self._current_yaw_enu()
        self._z0 = float(self.position[2])
        self._z0_initialized = True
        self.manual_des_pos = np.array([
            float(self.position[0]),
            float(self.position[1]),
            0.0,
        ])
        self.manual_des_yaw = current_yaw
        self.initial_yaw = current_yaw
        self.target_position = self.manual_des_pos.copy()
        self.target_velocity = np.zeros(3)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([0.0, 0.0, current_yaw])
        self.target_attitude_rate = np.zeros(3)
        self.target_R_des_ned_frd = None
        self.manual_pos_initialized = True
        self._xy_lock_position = self.position[:2].copy()
        self._xy_lock_initialized = True
        self._last_manual_cmd = self._zero_manual_cmd()

        now_s = float(self.px4_timestamp) / 1_000_000.0
        self.sim_start_time_s = now_s
        self._last_timestamp_s = now_s
        self._capture_hardware_handover_snapshot()
        restart_text = (
            f'，将从当前位置重新开始任务 {self.pending_auto_traj_mode}'
            if self.pending_auto_traj_mode else ''
        )
        snapshot_text = (
            '已捕获 Position 模式执行器输出并开始平滑混合'
            if self._hardware_handover_snapshot_valid
            else '未取得新鲜的 Position 执行器输出，仅执行目标点无扰初始化'
        )
        self.get_logger().warn(
            f'检测到 Armed + Offboard：锁定当前位置 '
            f'[{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}]m '
            f'和当前航向 {math.degrees(current_yaw):.1f}deg；{snapshot_text}{restart_text}。'
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
        self._attitude_error_quaternion = None
        self._last_manual_cmd = self._zero_manual_cmd()
        self._hardware_handover_start_timestamp_us = 0
        self._hardware_handover_snapshot_valid = False
        self._hardware_handover_blend = 0.0
        self._hardware_handover_state = 'inactive'

    def _capture_hardware_handover_snapshot(self) -> None:
        motor_age = self._px4_topic_age_s(self.px4_out_motor_timestamp)
        servo_age = self._px4_topic_age_s(self.px4_out_servo_timestamp)
        motors = np.asarray(self.px4_out_motor_cmd[:5], dtype=float)
        servos = np.asarray(self.px4_out_servo_cmd[:4], dtype=float)
        fresh = bool(
            np.all(np.isfinite(motors))
            and np.all(np.isfinite(servos))
            and math.isfinite(motor_age)
            and math.isfinite(servo_age)
            # These callbacks are asynchronous. A few milliseconds of negative
            # age only means the actuator sample arrived after the latest local
            # position sample, while both still use the same PX4 clock.
            and abs(motor_age) <= self.hardware_handover_snapshot_max_age_s
            and abs(servo_age) <= self.hardware_handover_snapshot_max_age_s
        )
        self._hardware_handover_snapshot_valid = fresh
        self._hardware_handover_start_timestamp_us = int(self.px4_timestamp)
        self._hardware_handover_blend = (
            1.0 if self.hardware_handover_duration_s <= 0.0 else 0.0
        )
        self._hardware_handover_state = (
            'active' if self._hardware_handover_blend >= 1.0 else 'blending'
        )
        if not fresh:
            self._hardware_handover_motor_start[:] = np.nan
            self._hardware_handover_servo_start[:] = np.nan
            return

        self._hardware_handover_motor_start = motors.copy()
        self._hardware_handover_servo_start = servos.copy()

        # Start the allocator's servo slew limiter at the actual Position-mode
        # command instead of at zero or a command left by an older session.
        self._alpha2_cmd = float(servos[0]) * self.primary_servo_angle_max_rad
        self._alpha1_cmd = float(servos[1]) * self.primary_servo_angle_max_rad
        secondary_scale = (
            self.secondary_servo_angle_max_rad
            / max(self.secondary_servo_gear_ratio, 1e-8)
        )
        self._theta2_cmd = float(servos[2]) * secondary_scale
        self._theta1_cmd = float(servos[3]) * secondary_scale

        # Publish the captured command immediately after the mode rising edge;
        # the first controller callback then continues from the same values.
        self._publish_normalized_direct_actuator_setpoint(motors, servos)

    def _current_hardware_handover_blend(self) -> float:
        if self.hardware_handover_duration_s <= 0.0:
            return 1.0
        if self._hardware_handover_start_timestamp_us <= 0:
            return 1.0
        elapsed = max(
            0.0,
            (
                int(self.px4_timestamp)
                - self._hardware_handover_start_timestamp_us
            ) / 1_000_000.0,
        )
        phase = float(np.clip(
            elapsed / self.hardware_handover_duration_s,
            0.0,
            1.0,
        ))
        # Smoothstep avoids a step in both command and command slope.
        blend = phase * phase * (3.0 - 2.0 * phase)
        if phase >= 1.0 and self._hardware_handover_state != 'active':
            self._hardware_handover_state = 'active'
            self.get_logger().info('Position -> Offboard 无扰接管完成。')
        self._hardware_handover_blend = float(blend)
        return float(blend)

    def publish_direct_actuator_setpoint(
        self, motor_controls, alpha1, alpha2, theta1, theta2
    ):
        target_motors = np.asarray(motor_controls[:5], dtype=float)
        target_servos = np.array([
            self._primary_joint_angle_to_normalized(alpha2),
            self._primary_joint_angle_to_normalized(alpha1),
            self._secondary_joint_angle_to_normalized(theta2),
            self._secondary_joint_angle_to_normalized(theta1),
        ], dtype=float)
        blend = self._current_hardware_handover_blend()
        if self._hardware_handover_snapshot_valid and blend < 1.0:
            target_motors = (
                (1.0 - blend) * self._hardware_handover_motor_start
                + blend * target_motors
            )
            target_servos = (
                (1.0 - blend) * self._hardware_handover_servo_start
                + blend * target_servos
            )
        self._publish_normalized_direct_actuator_setpoint(
            target_motors,
            target_servos,
        )

    def _manual_command_input_enabled(self) -> bool:
        # Suppress motion references during the short actuator blend so RC
        # switch transients cannot move the captured hold point.
        return bool(
            super()._manual_command_input_enabled()
            and self._hardware_control_active
            and self._hardware_handover_blend >= 1.0
        )

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
        return [
            'rc_source',
            'rc_age_s',
            'rc_valid',
            'hardware_control_active',
            'hardware_handover_state',
            'hardware_handover_snapshot_valid',
            'hardware_handover_blend',
        ]

    def _diagnostic_extra_values(self):
        rc_input = getattr(self, 'rc_input', None)
        if rc_input is None:
            return [
                'none', math.inf, 0, 0,
                getattr(self, '_hardware_handover_state', 'uninitialized'),
                0,
                0.0,
            ]
        return [
            rc_input.source,
            rc_input.age_s,
            int(rc_input.valid),
            int(self._hardware_control_active),
            self._hardware_handover_state,
            int(self._hardware_handover_snapshot_valid),
            self._hardware_handover_blend,
        ]

    def print_status(self):
        super().print_status()
        rc_input = getattr(self, 'rc_input', None)
        if rc_input is not None:
            self.get_logger().info(
                f'Hardware gate={self._hardware_control_active} | '
                f'RC source={rc_input.source} | age={rc_input.age_s:.3f}s | '
                f'valid={rc_input.valid} | '
                f'handover={self._hardware_handover_state} | '
                f'snapshot={self._hardware_handover_snapshot_valid} | '
                f'blend={self._hardware_handover_blend:.2f}'
            )


# DRCDA actuator layout: four tilt angles followed by five motor thrusts.
ANGLE_COUNT = 4
THRUST_COUNT = 5
ACTUATOR_COUNT = ANGLE_COUNT + THRUST_COUNT
ALLOCATOR_VARIANTS = (
    'full',
    'basic_da',
    'no_delay',
    'no_horizon',
    'no_rate_limits',
)


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
        [math.pi, math.pi, math.pi, math.pi],
        dtype=float,
    ))
    servo_command_limit_rad: np.ndarray = field(default_factory=lambda: np.array(
        [math.pi, math.pi, math.pi, math.pi],
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
    elif variant == 'no_rate_limits':
        config.servo_rate_positive_rad_s[:] = 1.0e6
        config.servo_rate_negative_rad_s[:] = 1.0e6
        config.servo_command_rate_rad_s[:] = 1.0e6
        config.thrust_command_rate_n_s[:] = 1.0e6
        config.motor_force_rate_floor_n_s = 1.0e6
        config.motor_force_rate_cap_n_s = 1.0e6
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

    def synchronize_wrench_reference(self, desired_wrench: Iterable[float]) -> None:
        """Drop derivative history after an estimator reference-frame reset."""
        self._previous_desired_wrench = _array(
            desired_wrench, 6, 'desired_wrench'
        ).copy()
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


class BasicDifferentialAllocator(DRCDAAllocator):
    """One-step differential allocator with fixed actuator-rate bounds.

    This baseline uses the current nominal command as actuator state. It has no
    actuator delay queue, lag model, or reachable-set prediction.
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

        self.state = self.command.copy()
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
            cfg.servo_command_rate_rad_s,
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
            (preferred - self.command) / (dt * rate_scale),
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
                self.command + dt * rate_scale * normalized_rate,
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
        self.state = command.copy()
        predicted_wrench = self.model.wrench(self.state)
        predicted_rate = (predicted_wrench - estimated_wrench) / dt
        result = DRCDAResult(
            command=command.copy(),
            predicted_state=self.state.copy(),
            estimated_wrench=estimated_wrench,
            predicted_wrench=predicted_wrench,
            desired_wrench=desired,
            jerk_reference=jerk_reference,
            wrench_rate_residual=jerk_reference - predicted_rate,
            wrench_residual=predicted_wrench - desired,
            solve_time_ms=(time.perf_counter() - start_time) * 1000.0,
            iterations=1,
            status=status,
        )
        self.last_result = result
        return result


class HnuterHardwareDRCDAController(HnuterHardwareController):
    """Hardware safety state machine with rate-limited DRCDA allocation."""

    def _node_name(self):
        variant = getattr(self, '_drcda_variant', 'full')
        return f'hnuter_hardware_drcda_{variant}'

    def _default_tuning_filename(self):
        return 'hnuter_drcda_hardware_tuning.json'

    def _gamepad_attitude_axis_toggle_enabled(self):
        return True

    def __init__(self) -> None:
        self._drcda_variant = os.environ.get(
            'HNUTER_DRCDA_VARIANT', 'full'
        ).strip().lower()
        if self._drcda_variant not in ALLOCATOR_VARIANTS:
            choices = ', '.join(ALLOCATOR_VARIANTS)
            raise ValueError(
                f'unknown HNUTER_DRCDA_VARIANT={self._drcda_variant!r}; '
                f'choose from {choices}'
            )
        self._drcda_ready = False
        self._drcda_active_call = False
        self._drcda_current_time_s = 0.0
        self._drcda_current_dt_s = 0.01
        self._drcda_accumulated_dt_s = 0.0
        super().__init__()

        model_name = os.environ.get(
            'HNUTER_DRCDA_SERVO_MODEL', 'hardware_rate_limited'
        ).strip().lower()
        config_kwargs = {
            'prediction_dt_s': env_float(
                'HNUTER_DRCDA_PREDICTION_DT_S', 0.01
            ),
            'horizon_s': env_float('HNUTER_DRCDA_HORIZON_S', 0.10),
            'gauss_newton_iterations': int(env_float(
                'HNUTER_DRCDA_ITERATIONS', 2
            )),
            'wrench_error_gain': env_float('HNUTER_DRCDA_WRENCH_GAIN', 6.0),
        }
        self._drcda_primary_rate_rad_s = max(
            0.1, env_float('HNUTER_DRCDA_PRIMARY_RATE_RAD_S', 6.0)
        )
        self._drcda_secondary_servo_rate_rad_s = max(
            0.1,
            env_float(
                'HNUTER_DRCDA_SECONDARY_SERVO_RATE_RAD_S',
                self._drcda_primary_rate_rad_s,
            ),
        )
        if model_name in ('ideal', 'instant'):
            config = DRCDAConfig.ideal_servos(**config_kwargs)
        elif model_name in ('hardware', 'hardware_rate_limited'):
            config = DRCDAConfig.ideal_servos(**config_kwargs)
            secondary_joint_rate = (
                self._drcda_secondary_servo_rate_rad_s
                / self.secondary_servo_gear_ratio
            )
            joint_rates = np.array([
                self._drcda_primary_rate_rad_s,
                secondary_joint_rate,
                self._drcda_primary_rate_rad_s,
                secondary_joint_rate,
            ])
            config.servo_rate_positive_rad_s[:] = joint_rates
            config.servo_rate_negative_rad_s[:] = joint_rates
            config.servo_command_rate_rad_s[:] = joint_rates
        else:
            raise ValueError(
                'unknown HNUTER_DRCDA_SERVO_MODEL='
                f'{model_name!r}; choose hardware_rate_limited or ideal'
            )
        configure_allocator_variant(config, self._drcda_variant)

        front_thrust_max = env_float(
            'HNUTER_DRCDA_FRONT_MOTOR_MAX_N', 50.0
        )
        tail_thrust_max = env_float(
            'HNUTER_DRCDA_TAIL_MOTOR_MAX_N', 50.0
        )
        config.thrust_max_n[:4] = front_thrust_max
        config.thrust_max_n[4] = tail_thrust_max
        config.thrust_min_n[4] = (
            -tail_thrust_max if self.allow_tail_reverse else 0.0
        )
        angle_limits = np.array([
            self.primary_servo_angle_max_rad,
            self.secondary_servo_angle_max_rad
            / self.secondary_servo_gear_ratio,
            self.primary_servo_angle_max_rad,
            self.secondary_servo_angle_max_rad
            / self.secondary_servo_gear_ratio,
        ])
        config.servo_state_limit_rad[:] = angle_limits
        config.servo_command_limit_rad[:] = angle_limits
        config.command_scale[:ANGLE_COUNT] = angle_limits
        config.command_scale[ANGLE_COUNT:8] = front_thrust_max
        config.command_scale[8] = tail_thrust_max
        config.antiwindup_gain = env_float(
            'HNUTER_DRCDA_ANTIWINDUP_GAIN', 0.20
        )

        wrench_model = HnuterWrenchModel(
            arm_half_span_m=self.l1,
            tail_x_m=-self.l2,
            reaction_torque_ratio_m=env_float(
                'HNUTER_DRCDA_REACTION_RATIO_M', 0.016
            ),
        )
        allocator_type = (
            BasicDifferentialAllocator
            if self._drcda_variant == 'basic_da'
            else DRCDAAllocator
        )
        self.drcda = allocator_type(wrench_model, config)
        self._drcda_update_period_s = float(np.clip(
            env_float('HNUTER_DRCDA_UPDATE_PERIOD_S', 0.01),
            0.002,
            0.05,
        ))
        self._drcda_model_name = model_name
        self._drcda_front_motor_max_n = float(front_thrust_max)
        self._drcda_tail_motor_max_n = float(tail_thrust_max)
        self._drcda_ready = True
        self._synchronize_drcda_hardware_limits()
        self._load_tuning_file(force=True)
        self.get_logger().warn(
            'EXPERIMENTAL HARDWARE DRCDA: 首次验证必须拆桨。'
            f'variant={self._drcda_variant}, model={model_name}, '
            f'horizon={config.horizon_s:.3f}s, '
            f'joint_limits={np.round(np.degrees(angle_limits), 1).tolist()}deg, '
            f'gear={self.secondary_servo_gear_ratio:.3f}, '
            f'log={self.diagnostic_path}'
        )

    def _apply_tuning(self, data: dict):
        super()._apply_tuning(data)
        if not getattr(self, '_drcda_ready', False):
            return
        config = self.drcda.config
        config.horizon_s = max(
            self._tuning_float(data, 'drcda_horizon_s', config.horizon_s),
            config.prediction_dt_s,
        )
        config.wrench_error_gain = max(self._tuning_float(
            data, 'drcda_wrench_error_gain', config.wrench_error_gain
        ), 0.0)
        config.wrench_ff_tau_s = max(self._tuning_float(
            data, 'drcda_wrench_ff_tau_s', config.wrench_ff_tau_s
        ), 0.0)
        config.wrench_rate_weight = max(self._tuning_float(
            data, 'drcda_wrench_rate_weight', config.wrench_rate_weight
        ), 0.0)
        config.wrench_weight = np.maximum(self._tuning_array(
            data, 'drcda_wrench_weight', config.wrench_weight
        ), 0.0)
        config.command_move_weight = np.maximum(self._tuning_array(
            data, 'drcda_command_move_weight', config.command_move_weight
        ), 0.0)
        config.command_preference_weight = np.maximum(self._tuning_array(
            data,
            'drcda_command_preference_weight',
            config.command_preference_weight,
        ), 0.0)
        config.antiwindup_gain = max(self._tuning_float(
            data, 'drcda_antiwindup_gain', config.antiwindup_gain
        ), 0.0)
        self._synchronize_drcda_hardware_limits()

    def _synchronize_drcda_hardware_limits(self) -> None:
        config = self.drcda.config
        limits = np.array([
            self.primary_servo_angle_max_rad,
            self.secondary_servo_angle_max_rad
            / self.secondary_servo_gear_ratio,
            self.primary_servo_angle_max_rad,
            self.secondary_servo_angle_max_rad
            / self.secondary_servo_gear_ratio,
        ])
        config.servo_state_limit_rad[:] = limits
        config.servo_command_limit_rad[:] = limits
        config.command_scale[:ANGLE_COUNT] = limits
        if self._drcda_model_name in ('hardware', 'hardware_rate_limited'):
            secondary_joint_rate = (
                self._drcda_secondary_servo_rate_rad_s
                / self.secondary_servo_gear_ratio
            )
            rates = np.array([
                self._drcda_primary_rate_rad_s,
                secondary_joint_rate,
                self._drcda_primary_rate_rad_s,
                secondary_joint_rate,
            ])
            config.servo_rate_positive_rad_s[:] = rates
            config.servo_rate_negative_rad_s[:] = rates
            config.servo_command_rate_rad_s[:] = rates
        self.drcda.state[:ANGLE_COUNT] = np.clip(
            self.drcda.state[:ANGLE_COUNT], -limits, limits
        )
        self.drcda.command[:ANGLE_COUNT] = np.clip(
            self.drcda.command[:ANGLE_COUNT], -limits, limits
        )

    def _tuning_snapshot(self) -> dict:
        snapshot = super()._tuning_snapshot()
        if not getattr(self, '_drcda_ready', False):
            return snapshot
        config = self.drcda.config
        snapshot.update({
            'drcda_variant': self._drcda_variant,
            'drcda_servo_model': self._drcda_model_name,
            'drcda_horizon_s': float(config.horizon_s),
            'drcda_prediction_dt_s': float(config.prediction_dt_s),
            'drcda_wrench_error_gain': float(config.wrench_error_gain),
            'drcda_wrench_ff_tau_s': float(config.wrench_ff_tau_s),
            'drcda_wrench_rate_weight': float(config.wrench_rate_weight),
            'drcda_wrench_weight': config.wrench_weight.tolist(),
            'drcda_command_move_weight': config.command_move_weight.tolist(),
            'drcda_command_preference_weight': (
                config.command_preference_weight.tolist()
            ),
            'drcda_antiwindup_gain': float(config.antiwindup_gain),
            'drcda_servo_state_limit_deg': np.degrees(
                config.servo_state_limit_rad
            ).tolist(),
        })
        return snapshot

    def _diagnostic_file_prefix(self):
        return (
            f'hardware/drcda/{self._drcda_variant}/'
            f'hnuter_hardware_{self._drcda_variant}'
        )

    def _diagnostic_extra_header(self):
        columns = list(super()._diagnostic_extra_header())
        columns += ['drcda_status', 'drcda_solve_ms', 'drcda_iterations']
        columns += [
            'drcda_q_alpha_left_rad', 'drcda_q_beta_left_rad',
            'drcda_q_alpha_right_rad', 'drcda_q_beta_right_rad',
            'drcda_q_f_left_1_n', 'drcda_q_f_left_2_n',
            'drcda_q_f_right_1_n', 'drcda_q_f_right_2_n',
            'drcda_q_f_tail_n',
        ]
        columns += [f'drcda_predicted_wrench_{name}' for name in (
            'fx_n', 'fy_n', 'fz_n', 'tx_nm', 'ty_nm', 'tz_nm'
        )]
        columns += [f'drcda_wrench_residual_{name}' for name in (
            'fx_n', 'fy_n', 'fz_n', 'tx_nm', 'ty_nm', 'tz_nm'
        )]
        columns += ['drcda_wrench_rate_residual_norm']
        return columns

    def _diagnostic_extra_values(self):
        values = list(super()._diagnostic_extra_values())
        if not self._drcda_ready or self.drcda.last_result is None:
            return values + ['', float('nan'), 0] + [float('nan')] * 22
        result = self.drcda.last_result
        return values + [
            result.status,
            float(result.solve_time_ms),
            int(result.iterations),
            *[float(value) for value in result.predicted_state],
            *[float(value) for value in result.predicted_wrench],
            *[float(value) for value in result.wrench_residual],
            float(np.linalg.norm(result.wrench_rate_residual)),
        ]

    @staticmethod
    def _motor_control_to_thrust(
        control: float,
        max_thrust: float,
        bidirectional: bool = False,
        motor_constant: float = 8.54858e-05,
        min_velocity: float = 10.0,
    ) -> float:
        if (
            not np.isfinite(control)
            or abs(control) <= 1e-9
            or max_thrust <= 0.0
        ):
            return 0.0
        if bidirectional:
            thrust = float(control) * abs(float(control)) * max_thrust
            return float(np.clip(thrust, -max_thrust, max_thrust))
        max_velocity = math.sqrt(max_thrust / motor_constant)
        velocity = min_velocity + float(np.clip(control, 0.0, 1.0)) * (
            max_velocity - min_velocity
        )
        return float(np.clip(
            motor_constant * velocity * velocity, 0.0, max_thrust
        ))

    def _preferred_drcda_command(
        self,
        motor_controls,
        alpha1: float,
        alpha2: float,
        theta1: float,
        theta2: float,
    ) -> np.ndarray:
        controls = np.asarray(motor_controls, dtype=float)
        if controls.size < 5:
            controls = np.pad(controls, (0, 5 - controls.size))
        front_max = self._drcda_front_motor_max_n
        tail_max = self._drcda_tail_motor_max_n
        return np.array([
            alpha1,
            theta1,
            alpha2,
            theta2,
            self._motor_control_to_thrust(controls[2], front_max),
            self._motor_control_to_thrust(controls[3], front_max),
            self._motor_control_to_thrust(controls[0], front_max),
            self._motor_control_to_thrust(controls[1], front_max),
            self._motor_control_to_thrust(
                controls[4], tail_max, self.allow_tail_reverse
            ),
        ], dtype=float)

    def _active_drcda_angle_limits(self) -> np.ndarray:
        elapsed_s = (
            self._drcda_current_time_s - self._takeoff_lock_start_time_s
            if self._takeoff_lock_start_time_s is not None else 100.0
        )
        if elapsed_s < self.takeoff_tilt_suppress_time_s:
            alpha_limit = self.takeoff_tilt_limit_rad
            beta_limit = min(
                self.takeoff_tilt_limit_rad, self.theta_limit_rad
            )
        elif elapsed_s < self.takeoff_xy_lock_time_s:
            alpha_limit = self.xy_lock_tilt_limit_rad
            beta_limit = min(self.xy_lock_tilt_limit_rad, self.theta_limit_rad)
        else:
            alpha_limit = self.alpha_limit_rad
            beta_limit = self.theta_limit_rad
        return np.array([
            alpha_limit, beta_limit, alpha_limit, beta_limit
        ], dtype=float)

    def _apply_drcda_antiwindup(
        self, wrench_residual: np.ndarray, dt: float
    ) -> None:
        config = self.drcda.config
        normalized_error = wrench_residual / config.wrench_scale
        if np.linalg.norm(normalized_error) < 0.02:
            return
        delta_force_body = np.array([
            wrench_residual[0] / max(abs(self.allocator_force_x_sign), 1e-6),
            wrench_residual[1] / (
                self.allocator_force_y_sign
                if abs(self.allocator_force_y_sign) > 1e-6 else 1.0
            ),
            -wrench_residual[2],
        ])
        delta_acceleration_ned = (
            self.R_ned_frd @ delta_force_body / self.mass
        )
        active = self.direct_pos_Ki_ned > 1e-6
        correction = np.zeros(3)
        correction[active] = (
            -config.antiwindup_gain
            * delta_acceleration_ned[active]
            / self.direct_pos_Ki_ned[active]
            * dt
        )
        self.integral_pos_error += correction
        self.integral_pos_error = np.clip(
            self.integral_pos_error,
            -self.direct_pos_integral_limit_ned,
            self.direct_pos_integral_limit_ned,
        )

    def publish_px4_equivalent_direct_commands(
        self, current_time: float, dt: float
    ):
        self._drcda_active_call = True
        self._drcda_current_time_s = float(current_time)
        self._drcda_current_dt_s = float(dt)
        try:
            return super().publish_px4_equivalent_direct_commands(
                current_time, dt
            )
        finally:
            self._drcda_active_call = False

    def publish_idle_direct_actuator_setpoint(self):
        if self._drcda_ready:
            self.drcda.reset()
            self._drcda_accumulated_dt_s = 0.0
        return super().publish_idle_direct_actuator_setpoint()

    def publish_direct_actuator_setpoint(
        self,
        motor_controls,
        alpha1,
        alpha2,
        theta1,
        theta2,
    ):
        drcda_active = (
            self._drcda_ready
            and self._drcda_active_call
            and self.armed
            and self.takeoff_requested
            and self._hardware_control_active
        )
        if not drcda_active:
            if self._drcda_ready:
                self.drcda.reset(
                    angle_state=[alpha1, theta1, alpha2, theta2],
                    thrust_state=[0.0] * 5,
                )
            return super().publish_direct_actuator_setpoint(
                motor_controls, alpha1, alpha2, theta1, theta2
            )

        preferred = self._preferred_drcda_command(
            motor_controls, alpha1, alpha2, theta1, theta2
        )
        self._drcda_accumulated_dt_s += self._drcda_current_dt_s
        solve_due = (
            self.drcda.last_result is None
            or self._drcda_accumulated_dt_s >= self._drcda_update_period_s
        )
        if solve_due:
            allocation_dt = max(
                self._drcda_accumulated_dt_s, self._drcda_current_dt_s
            )
            result = self.drcda.allocate(
                desired_wrench=self.last_W,
                dt=allocation_dt,
                preferred_command=preferred,
                active_angle_limits=self._active_drcda_angle_limits(),
            )
            self._drcda_accumulated_dt_s = 0.0
            self._apply_drcda_antiwindup(
                result.wrench_residual, allocation_dt
            )
        command = self.drcda.command
        logical_thrust = command[ANGLE_COUNT:]
        front_max = self._drcda_front_motor_max_n
        tail_max = self._drcda_tail_motor_max_n
        output_motor_controls = [
            self._thrust_to_normalized_motor_control(
                logical_thrust[2], front_max
            ),
            self._thrust_to_normalized_motor_control(
                logical_thrust[3], front_max
            ),
            self._thrust_to_normalized_motor_control(
                logical_thrust[0], front_max
            ),
            self._thrust_to_normalized_motor_control(
                logical_thrust[1], front_max
            ),
            (
                self._thrust_to_normalized_bidirectional_motor_control(
                    logical_thrust[4], tail_max
                )
                if self.allow_tail_reverse
                else self._thrust_to_normalized_motor_control(
                    logical_thrust[4], tail_max
                )
            ),
        ]
        self._alpha1_cmd = float(command[0])
        self._theta1_cmd = float(command[1])
        self._alpha2_cmd = float(command[2])
        self._theta2_cmd = float(command[3])
        self.last_F1 = float(logical_thrust[0] + logical_thrust[1])
        self.last_F2 = float(logical_thrust[2] + logical_thrust[3])
        self.last_F3 = float(logical_thrust[4])
        return super().publish_direct_actuator_setpoint(
            output_motor_controls,
            self._alpha1_cmd,
            self._alpha2_cmd,
            self._theta1_cmd,
            self._theta2_cmd,
        )


def main(args=None):
    rclpy.init(args=args)
    controller = HnuterHardwareDRCDAController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('实机 DRCDA 外部控制节点已停止。')
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
