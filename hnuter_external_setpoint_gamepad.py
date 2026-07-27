#!/usr/bin/env python3
"""
Hnuter PX4 setpoint-only gamepad controller.

This node keeps the control loop inside the PX4 firmware. It publishes only
Offboard setpoints:

- TrajectorySetpoint: position, velocity, yaw and yaw rate.
- VehicleAttitudeSetpoint: roll/pitch/yaw attitude reference for the Hnuter
  firmware controller or for logging/inspection.
- VehicleRatesSetpoint: optional body-rate reference when
  HNUTER_SP_PUBLISH_RATES=1.

Gamepad mapping:
- Left stick X: yaw rate.
- Left stick Y: vertical speed.
- Right stick X/Y: horizontal velocity in body frame.
- A/B: roll setpoint negative/positive step.
- X/Y: pitch setpoint negative/positive step.
- Keyboard o: allow Offboard + Arm + takeoff hover.
"""

import math
import os
import queue
import select
import sys
import termios
import threading
import time
import tty

from hnuter_log_paths import configure_ros_log_dir

if os.environ.get('HNUTER_ALLOW_REMOTE_DDS', '0') != '1':
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_STATIC_PEERS', None)
configure_ros_log_dir()

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleAngularVelocity
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleCommandAck
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleRatesSetpoint
from px4_msgs.msg import VehicleStatus

try:
    import pygame
except Exception:
    pygame = None


def env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.lower() in ('1', 'true', 'yes', 'on')


class KeyboardReader:
    def __init__(self, logger=None):
        self.logger = logger
        self.commands = queue.Queue()
        self._stop = threading.Event()
        self._old_termios = None
        self._stdin_fd = None
        self._thread = None

        try:
            if not sys.stdin or not sys.stdin.isatty():
                self._warn('stdin is not a TTY, keyboard takeoff key is disabled.')
                return

            self._stdin_fd = sys.stdin.fileno()
            self._old_termios = termios.tcgetattr(self._stdin_fd)
            tty.setcbreak(self._stdin_fd)
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
        except Exception as exc:
            self._warn(f'keyboard init failed: {exc}')
            self.close()

    def _info(self, text: str):
        if self.logger:
            self.logger.info(text)
        else:
            print(text)

    def _warn(self, text: str):
        if self.logger:
            self.logger.warn(text)
        else:
            print(text)

    def _loop(self):
        while not self._stop.is_set():
            try:
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if readable:
                    key = sys.stdin.read(1)
                    if key in ('o', 'O', 'h', 'H'):
                        self.commands.put(key.lower())
            except Exception as exc:
                if not self._stop.is_set():
                    self._warn(f'keyboard read failed: {exc}')
                break

    def get_commands(self):
        result = []
        while True:
            try:
                result.append(self.commands.get_nowait())
            except queue.Empty:
                return result

    def close(self):
        self._stop.set()
        if self._old_termios is not None and self._stdin_fd is not None:
            try:
                termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_termios)
            except Exception:
                pass
        self._old_termios = None
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.2)


class GamepadManager:
    def __init__(self, logger=None):
        self.logger = logger
        self.joystick = None
        self.deadzone = env_float('HNUTER_PAD_DEADZONE', 0.10)
        self.expo = env_float('HNUTER_PAD_EXPO', 0.35)
        self.filter_tau = env_float('HNUTER_PAD_FILTER_TAU', 0.12)
        self.max_vxy = env_float('HNUTER_PAD_MAX_VXY', 0.8)
        self.max_vz = env_float('HNUTER_PAD_MAX_VZ', 0.45)
        self.max_yaw_rate = math.radians(env_float('HNUTER_PAD_MAX_YAW_RATE_DEG', 45.0))
        self.step_deg = env_float('HNUTER_PAD_ATT_STEP_DEG', 5.0)
        self.repeat_s = env_float('HNUTER_PAD_REPEAT_S', 0.25)
        self.roll_limit = math.radians(env_float('HNUTER_PAD_ROLL_LIMIT_DEG', 75.0))
        self.pitch_limit = math.radians(env_float('HNUTER_PAD_PITCH_LIMIT_DEG', 180.0))
        self.axis_yaw = int(env_float('HNUTER_PAD_AXIS_YAW', 0))
        self.axis_z = int(env_float('HNUTER_PAD_AXIS_Z', 1))
        self.axis_xy_y = int(env_float('HNUTER_PAD_AXIS_XY_Y', 3))
        self.axis_xy_x = int(env_float('HNUTER_PAD_AXIS_XY_X', 4))
        self.debug_axes = env_bool('HNUTER_PAD_DEBUG', False)
        self.button_roll_minus = int(env_float('HNUTER_PAD_BTN_ROLL_MINUS', 0))   # A
        self.button_roll_plus = int(env_float('HNUTER_PAD_BTN_ROLL_PLUS', 1))     # B
        self.button_pitch_minus = int(env_float('HNUTER_PAD_BTN_PITCH_MINUS', 2)) # X
        self.button_pitch_plus = int(env_float('HNUTER_PAD_BTN_PITCH_PLUS', 3))   # Y
        self.filtered_vx_b = 0.0
        self.filtered_vy_b = 0.0
        self.filtered_z = 0.0
        self.filtered_yaw_rate = 0.0
        self._last_button_step = {}
        self.last_event = ''
        self.last_raw_axes = {}
        self._last_debug_print_t = 0.0

        if pygame is None:
            self._warn('pygame is unavailable, gamepad commands stay zero.')
            return

        try:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self._info(
                    f'gamepad connected: {self.joystick.get_name()}, '
                    f'axes={self.joystick.get_numaxes()}, buttons={self.joystick.get_numbuttons()}, '
                    f'mapping yaw={self.axis_yaw}, z={self.axis_z}, '
                    f'xy_x={self.axis_xy_x}, xy_y={self.axis_xy_y}'
                )
            else:
                self._warn('no gamepad detected, commands stay zero.')
        except Exception as exc:
            self._warn(f'gamepad init failed: {exc}')
            self.joystick = None

    def _info(self, text: str):
        if self.logger:
            self.logger.info(text)
        else:
            print(text)

    def _warn(self, text: str):
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

    def _shape(self, value: float) -> float:
        value = float(value)
        if abs(value) <= self.deadzone:
            return 0.0
        shaped = math.copysign((abs(value) - self.deadzone) / max(1.0 - self.deadzone, 1e-6), value)
        return self.expo * shaped ** 3 + (1.0 - self.expo) * shaped

    def read(self, dt: float):
        delta_roll = 0.0
        delta_pitch = 0.0
        event = ''

        if pygame is None or self.joystick is None:
            return {
                'vx_b': self.filtered_vx_b,
                'vy_b': self.filtered_vy_b,
                'vz_enu': self.filtered_z,
                'yaw_rate_enu': self.filtered_yaw_rate,
                'delta_roll': delta_roll,
                'delta_pitch': delta_pitch,
                'event': event,
            }

        try:
            pygame.event.pump()
            num_axes = self.joystick.get_numaxes()
            raw_yaw = self.joystick.get_axis(self.axis_yaw) if num_axes > self.axis_yaw else 0.0
            raw_z = self.joystick.get_axis(self.axis_z) if num_axes > self.axis_z else 0.0
            raw_xy_y = self.joystick.get_axis(self.axis_xy_y) if num_axes > self.axis_xy_y else 0.0
            raw_xy_x = self.joystick.get_axis(self.axis_xy_x) if num_axes > self.axis_xy_x else 0.0
            self.last_raw_axes = {
                'yaw': raw_yaw,
                'z': raw_z,
                'xy_x': raw_xy_x,
                'xy_y': raw_xy_y,
            }

            target_yaw_rate = -self._shape(raw_yaw) * self.max_yaw_rate
            target_vz = -self._shape(raw_z) * self.max_vz
            target_vx_b = -self._shape(raw_xy_x) * self.max_vxy
            target_vy_b = -self._shape(raw_xy_y) * self.max_vxy

            alpha = dt / (self.filter_tau + dt) if self.filter_tau > 1e-4 else 1.0
            alpha = float(np.clip(alpha, 0.0, 1.0))
            self.filtered_vx_b += alpha * (target_vx_b - self.filtered_vx_b)
            self.filtered_vy_b += alpha * (target_vy_b - self.filtered_vy_b)
            self.filtered_yaw_rate += alpha * (target_yaw_rate - self.filtered_yaw_rate)
            self.filtered_z += alpha * (target_vz - self.filtered_z)

            now = time.monotonic()
            step = math.radians(self.step_deg)
            button_actions = (
                (self.button_roll_minus, -step, 0.0, 'A roll-'),
                (self.button_roll_plus, step, 0.0, 'B roll+'),
                (self.button_pitch_minus, 0.0, -step, 'X pitch-'),
                (self.button_pitch_plus, 0.0, step, 'Y pitch+'),
            )

            for button, d_roll, d_pitch, event_text in button_actions:
                if button < 0 or button >= self.joystick.get_numbuttons():
                    continue
                if self.joystick.get_button(button):
                    last_t = self._last_button_step.get(button, -1e9)
                    if now - last_t >= self.repeat_s:
                        delta_roll += d_roll
                        delta_pitch += d_pitch
                        event = event_text
                        self.last_event = event_text
                        self._last_button_step[button] = now
                else:
                    self._last_button_step.pop(button, None)

            if self.debug_axes and now - self._last_debug_print_t > 1.0:
                self._last_debug_print_t = now
                self._info(
                    'raw axes '
                    f'yaw={raw_yaw:+.2f}, z={raw_z:+.2f}, '
                    f'xy_x={raw_xy_x:+.2f}, xy_y={raw_xy_y:+.2f}; '
                    f'cmd body vx={self.filtered_vx_b:+.2f}, vy={self.filtered_vy_b:+.2f}'
                )

        except Exception as exc:
            self._warn(f'gamepad read failed: {exc}')

        return {
            'vx_b': self.filtered_vx_b,
            'vy_b': self.filtered_vy_b,
            'vz_enu': self.filtered_z,
            'yaw_rate_enu': self.filtered_yaw_rate,
            'delta_roll': delta_roll,
            'delta_pitch': delta_pitch,
            'event': event,
        }


def wrap_pi(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def quat_from_matrix(rot: np.ndarray) -> np.ndarray:
    m = np.asarray(rot, dtype=float)
    tr = float(np.trace(m))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return np.array([
            0.25 * s,
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
        ])
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(max(1.0 + m[0, 0] - m[1, 1] - m[2, 2], 1e-12)) * 2.0
        return np.array([
            (m[2, 1] - m[1, 2]) / s,
            0.25 * s,
            (m[0, 1] + m[1, 0]) / s,
            (m[0, 2] + m[2, 0]) / s,
        ])
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(max(1.0 + m[1, 1] - m[0, 0] - m[2, 2], 1e-12)) * 2.0
        return np.array([
            (m[0, 2] - m[2, 0]) / s,
            (m[0, 1] + m[1, 0]) / s,
            0.25 * s,
            (m[1, 2] + m[2, 1]) / s,
        ])
    s = math.sqrt(max(1.0 + m[2, 2] - m[0, 0] - m[1, 1], 1e-12)) * 2.0
    return np.array([
        (m[1, 0] - m[0, 1]) / s,
        (m[0, 2] + m[2, 0]) / s,
        (m[1, 2] + m[2, 1]) / s,
        0.25 * s,
    ])


def rotation_enu_flu_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def enu_flu_euler_to_ned_frd_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    r_enu_ned = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    r_frd_flu = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
    r_ned_frd = r_enu_ned.T @ rotation_enu_flu_from_euler(roll, pitch, yaw) @ r_frd_flu.T
    q = quat_from_matrix(r_ned_frd)
    norm = float(np.linalg.norm(q))
    return q / norm if norm > 1e-9 else np.array([1.0, 0.0, 0.0, 0.0])


class HnuterSetpointGamepad(Node):
    def __init__(self):
        super().__init__('hnuter_setpoint_gamepad')

        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_cmd)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_cmd)
        self.att_sp_pub = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_cmd)
        self.rates_sp_pub = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_cmd)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_cmd)

        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.on_local_position, qos_out)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.on_attitude, qos_out)
        self.create_subscription(VehicleAngularVelocity, '/fmu/out/vehicle_angular_velocity', self.on_rates, qos_out)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.on_status, qos_out)
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.on_control_mode, qos_out)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.on_command_ack, qos_out)

        self.cmd_do_set_mode = getattr(VehicleCommand, 'VEHICLE_CMD_DO_SET_MODE', 176)
        self.cmd_arm_disarm = getattr(VehicleCommand, 'VEHICLE_CMD_COMPONENT_ARM_DISARM', 400)
        self.nav_offboard = getattr(VehicleStatus, 'NAVIGATION_STATE_OFFBOARD', 14)
        self.arming_armed = getattr(VehicleStatus, 'ARMING_STATE_ARMED', 2)

        self.position_enu = np.zeros(3)
        self.velocity_enu = np.zeros(3)
        self.angular_velocity_frd = np.zeros(3)
        self.yaw_enu = 0.0
        self.px4_timestamp = 0
        self.have_position = False
        self.have_attitude = False
        self.armed = False
        self.nav_state = -1
        self.control_offboard = False

        self.takeoff_height = env_float('HNUTER_SP_TAKEOFF_HEIGHT', 1.3)
        self.max_altitude = env_float('HNUTER_SP_MAX_ALTITUDE', 4.0)
        self.min_altitude = env_float('HNUTER_SP_MIN_ALTITUDE', 0.15)
        self.position_hold = np.zeros(3)
        self.position_target = np.array([0.0, 0.0, self.takeoff_height], dtype=float)
        self.velocity_target = np.zeros(3)
        self.attitude_target = np.zeros(3)
        self.attitude_rate_target = np.zeros(3)
        self.last_manual = {
            'vx_b': 0.0,
            'vy_b': 0.0,
            'vz_enu': 0.0,
            'yaw_rate_enu': 0.0,
            'delta_roll': 0.0,
            'delta_pitch': 0.0,
            'event': '',
        }
        self._last_gamepad_event = ''
        self.takeoff_requested = False
        self.target_initialized = False

        self.auto_arm = env_bool('HNUTER_SP_AUTO_ARM', True)
        self.publish_rates = env_bool('HNUTER_SP_PUBLISH_RATES', False)
        self.position_mode = env_bool('HNUTER_SP_POSITION_MODE', True)
        self.attitude_mode = env_bool('HNUTER_SP_ATTITUDE_MODE', True)
        self.body_rate_mode = env_bool('HNUTER_SP_BODY_RATE_MODE', False)
        self.offboard_warmup_ticks = int(env_float('HNUTER_SP_WARMUP_TICKS', 30))
        self.offboard_ticks = 0
        self.last_mode_request_t = 0.0
        self.last_arm_request_t = 0.0
        self.last_update_t = time.monotonic()

        self.gamepad = GamepadManager(logger=self.get_logger())
        self.keyboard = KeyboardReader(logger=self.get_logger())
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.print_status)

        self.get_logger().info(
            'Hnuter setpoint-only gamepad controller ready. '
            'Press o to request Offboard/Arm/takeoff. A/B roll -/+, X/Y pitch -/+.'
        )

    def timestamp_us(self) -> int:
        if self.px4_timestamp > 0:
            return int(self.px4_timestamp)
        return int(self.get_clock().now().nanoseconds / 1000)

    def on_local_position(self, msg):
        if not (bool(msg.xy_valid) and bool(msg.z_valid)):
            return
        self.px4_timestamp = int(msg.timestamp)
        self.position_enu = np.array([msg.y, msg.x, -msg.z], dtype=float)
        if bool(msg.v_xy_valid) and bool(msg.v_z_valid):
            self.velocity_enu = np.array([msg.vy, msg.vx, -msg.vz], dtype=float)
        self.have_position = True

    def on_attitude(self, msg):
        self.px4_timestamp = int(msg.timestamp)
        w, x, y, z = [float(v) for v in msg.q]
        r_ned_frd = np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ])
        r_enu_ned = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
        r_frd_flu = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
        r_enu_flu = r_enu_ned @ r_ned_frd @ r_frd_flu
        self.yaw_enu = math.atan2(float(r_enu_flu[1, 0]), float(r_enu_flu[0, 0]))
        if not self.target_initialized:
            self.attitude_target[2] = self.yaw_enu
        self.have_attitude = True

    def on_rates(self, msg):
        self.angular_velocity_frd = np.array(msg.xyz, dtype=float)

    def on_status(self, msg):
        self.nav_state = int(getattr(msg, 'nav_state', -1))
        self.armed = int(getattr(msg, 'arming_state', -1)) == self.arming_armed

    def on_control_mode(self, msg):
        self.control_offboard = bool(getattr(msg, 'flag_control_offboard_enabled', False))
        if hasattr(msg, 'flag_armed'):
            self.armed = bool(msg.flag_armed)

    def on_command_ack(self, msg):
        if int(msg.command) in (self.cmd_do_set_mode, self.cmd_arm_disarm):
            self.get_logger().info(f'command ack command={int(msg.command)} result={int(msg.result)}')

    def is_ready(self) -> bool:
        return self.have_position and self.have_attitude

    def is_offboard(self) -> bool:
        return self.control_offboard or self.nav_state == self.nav_offboard

    def init_targets(self):
        if self.target_initialized or not self.is_ready():
            return
        self.position_hold = self.position_enu.copy()
        self.position_target = np.array([
            self.position_enu[0],
            self.position_enu[1],
            self.position_enu[2],
        ], dtype=float)
        self.attitude_target = np.array([0.0, 0.0, self.yaw_enu], dtype=float)
        self.target_initialized = True

    def tick(self):
        now = time.monotonic()
        dt = float(np.clip(now - self.last_update_t, 0.01, 0.10))
        self.last_update_t = now

        for key in self.keyboard.get_commands():
            if key == 'o':
                self.takeoff_requested = True
                self.target_initialized = False
                self.get_logger().info('takeoff requested by keyboard o')
            elif key == 'h':
                self.attitude_target[0] = 0.0
                self.attitude_target[1] = 0.0
                self.get_logger().info('roll/pitch target reset to level by keyboard h')

        if not self.is_ready():
            self.publish_offboard_control_mode()
            return

        self.init_targets()

        manual = self.gamepad.read(dt)
        self.last_manual = manual
        self.attitude_target[0] = float(np.clip(
            self.attitude_target[0] + manual['delta_roll'],
            -self.gamepad.roll_limit,
            self.gamepad.roll_limit,
        ))
        self.attitude_target[1] = float(np.clip(
            self.attitude_target[1] + manual['delta_pitch'],
            -self.gamepad.pitch_limit,
            self.gamepad.pitch_limit,
        ))
        self.attitude_target[2] = wrap_pi(self.attitude_target[2] + manual['yaw_rate_enu'] * dt)
        self.attitude_rate_target[:] = [0.0, 0.0, manual['yaw_rate_enu']]
        if manual.get('event'):
            self._last_gamepad_event = manual['event']
            self.get_logger().info(
                f'gamepad button event: {manual["event"]}, '
                f'attitude target roll={math.degrees(self.attitude_target[0]):.1f}deg '
                f'pitch={math.degrees(self.attitude_target[1]):.1f}deg'
            )

        if self.takeoff_requested:
            cos_yaw = math.cos(self.attitude_target[2])
            sin_yaw = math.sin(self.attitude_target[2])
            vx_enu = cos_yaw * manual['vx_b'] - sin_yaw * manual['vy_b']
            vy_enu = sin_yaw * manual['vx_b'] + cos_yaw * manual['vy_b']
            self.velocity_target[0:2] = [vx_enu, vy_enu]
            self.position_target[0] += vx_enu * dt
            self.position_target[1] += vy_enu * dt
            target_z = self.position_hold[2] + self.takeoff_height
            self.position_target[2] = float(np.clip(
                self.position_target[2] + manual['vz_enu'] * dt,
                self.position_hold[2] + self.min_altitude,
                min(self.position_hold[2] + self.max_altitude, target_z + 1.0),
            ))
            if self.position_target[2] < target_z:
                self.position_target[2] = min(target_z, self.position_target[2] + 0.35 * dt)
            self.velocity_target[2] = manual['vz_enu']
        else:
            self.position_target = self.position_enu.copy()
            self.velocity_target[:] = 0.0

        self.publish_offboard_control_mode()
        self.publish_setpoints()
        self.offboard_ticks += 1

        if self.offboard_ticks < self.offboard_warmup_ticks:
            return

        if not self.is_offboard() and now - self.last_mode_request_t > 1.0:
            self.set_offboard()
            self.last_mode_request_t = now
            return

        if self.is_offboard() and self.takeoff_requested and self.auto_arm and not self.armed:
            if now - self.last_arm_request_t > 1.0:
                self.arm()
                self.last_arm_request_t = now

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_us()
        msg.position = bool(self.position_mode)
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = bool(self.attitude_mode)
        msg.body_rate = bool(self.body_rate_mode)
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_pub.publish(msg)

    def publish_setpoints(self):
        timestamp = self.timestamp_us()

        traj = TrajectorySetpoint()
        traj.timestamp = timestamp
        traj.position = [
            float(self.position_target[1]),
            float(self.position_target[0]),
            float(-self.position_target[2]),
        ]
        traj.velocity = [
            float(self.velocity_target[1]),
            float(self.velocity_target[0]),
            float(-self.velocity_target[2]),
        ]
        traj.acceleration = [float('nan'), float('nan'), float('nan')]
        # Hnuter firmware extension: jerk[0]/jerk[1] carry roll/pitch attitude
        # setpoints in radians while position/velocity fields keep translation.
        traj.jerk = [
            float(self.attitude_target[0]),
            float(self.attitude_target[1]),
            float('nan'),
        ]
        traj.yaw = float(-self.attitude_target[2])
        traj.yawspeed = float(-self.attitude_rate_target[2])
        self.traj_pub.publish(traj)

        att = VehicleAttitudeSetpoint()
        att.timestamp = timestamp
        q = enu_flu_euler_to_ned_frd_quat(
            self.attitude_target[0],
            self.attitude_target[1],
            self.attitude_target[2],
        )
        att.q_d = [float(v) for v in q]
        att.thrust_body = [float('nan'), float('nan'), float('nan')]
        att.yaw_sp_move_rate = float(-self.attitude_rate_target[2])
        self.att_sp_pub.publish(att)

        if self.publish_rates:
            rates = VehicleRatesSetpoint()
            rates.timestamp = timestamp
            rates.roll = float(self.attitude_rate_target[0])
            rates.pitch = float(-self.attitude_rate_target[1])
            rates.yaw = float(-self.attitude_rate_target[2])
            rates.thrust_body = [float('nan'), float('nan'), float('nan')]
            rates.reset_integral = False
            self.rates_sp_pub.publish(rates)

    def publish_vehicle_command(self, command: int, param1=0.0, param2=0.0, param3=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def set_offboard(self):
        self.publish_vehicle_command(self.cmd_do_set_mode, 1.0, 6.0)
        self.get_logger().info('requested Offboard mode')

    def arm(self):
        self.publish_vehicle_command(self.cmd_arm_disarm, 1.0)
        self.get_logger().info('requested Arm')

    def print_status(self):
        if not self.is_ready():
            self.get_logger().info('waiting for PX4 local position and attitude...')
            return
        self.get_logger().info(
            'sp pos ENU='
            f'{np.round(self.position_target, 2).tolist()} vel={np.round(self.velocity_target, 2).tolist()} '
            f'att_deg={np.round(np.degrees(self.attitude_target), 1).tolist()} '
            f'pad_body_vxy={[round(self.last_manual.get("vx_b", 0.0), 2), round(self.last_manual.get("vy_b", 0.0), 2)]} '
            f'pad_vz={self.last_manual.get("vz_enu", 0.0):.2f} '
            f'pad_yaw_rate={math.degrees(self.last_manual.get("yaw_rate_enu", 0.0)):.1f}deg/s '
            f'offboard={self.is_offboard()} armed={self.armed}'
        )

    def destroy_node(self):
        self.keyboard.close()
        self.gamepad.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HnuterSetpointGamepad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
