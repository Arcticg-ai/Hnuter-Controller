#!/usr/bin/env python3
"""Hnuter PX4 position-offboard controller for real-aircraft use.

The transmitter owns Arm and Offboard mode selection. This node never publishes
VehicleCommand. It continuously reads PX4 RC topics and publishes position,
velocity, acceleration, yaw, and Hnuter attitude-extension references only
while using the current local position as the origin of each flight session and
each keyboard-triggered trajectory. PX4 owns actuator mapping; this version is
intended for firmware profile 3131ddd4_500_2500_gear2, where 500/1500/2500 us
applies only to the four tilt-servo inputs and not to motor outputs.
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


def env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


@dataclass
class _StickSample:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    throttle: float = 0.0
    aux1: float = 0.0
    aux2: float = 0.0


class RCCommandManager:
    """Convert PX4 RC telemetry into body-frame velocity references."""

    def __init__(self, logger=None):
        self.logger = logger
        self.max_vxy = env_float('HNUTER_RC_MAX_VXY_MPS', 0.6)
        self.max_vz = env_float('HNUTER_RC_MAX_VZ_MPS', 0.3)
        self.max_yaw_rate = env_float('HNUTER_RC_MAX_YAW_RATE_RPS', 0.4)
        self.max_attitude_rate_rad_s = np.radians(np.array([35.0, 35.0]))
        self.max_attitude_angle_rad = math.radians(45.0)
        self.attitude_sign = np.array([-1.0, -1.0])
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
            'pitch_rate': 0.0,
            'lt': 0.0,
            'rt': 0.0,
        }

    @staticmethod
    def _finite_sticks(sample: _StickSample) -> bool:
        return bool(np.all(np.isfinite([
            sample.roll, sample.pitch, sample.yaw, sample.throttle,
            sample.aux1, sample.aux2,
        ])))

    def feed_manual_control(self, message) -> None:
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
        aux1 = self._mapped_channel(message, RcChannels.FUNCTION_AUX_1)
        aux2 = self._mapped_channel(message, RcChannels.FUNCTION_AUX_2)
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
                aux1=0.0 if aux1 is None else float(aux1),
                aux2=0.0 if aux2 is None else float(aux2),
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
            'roll_rate': self.attitude_sign[0] * self._shape(sample.aux1) * self.max_attitude_rate_rad_s[0],
            'pitch_rate': self.attitude_sign[1] * self._shape(sample.aux2) * self.max_attitude_rate_rad_s[1],
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


class HnuterController(Node):
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

        # Runtime status
        self.control_loop_count = 0
        self._last_manual_cmd = {
            'vx_b': 0.0,
            'vy_b': 0.0,
            'vz': 0.0,
            'yaw_rate': 0.0,
            'roll_rate': 0.0,
            'pitch_rate': 0.0,
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
        self.manual_des_pitch = 0.0
        self.manual_attitude_limit_rad = math.radians(45.0)
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
        self.manual_des_pitch = 0.0
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
        self.manual_des_pitch = 0.0
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
        self.manual_des_pitch = 0.0
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
        self.manual_des_pitch = 0.0
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
            self.manual_des_pitch = 0.0
            self._last_manual_cmd = self._zero_manual_cmd()
            self._hold_current_position()
            return

        if not self.manual_pos_initialized:
            self.manual_des_pos = np.array([
                self.position[0], self.position[1], self.position[2] - self._z0
            ])
            self.manual_des_yaw = self.initial_yaw if self._yaw_initialized else 0.0
            self.manual_des_roll = 0.0
            self.manual_des_pitch = 0.0
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
        pitch_rate = cmds.get('pitch_rate', 0.0)

        self.manual_des_pos[0] += vx_w * dt
        self.manual_des_pos[1] += vy_w * dt
        self.manual_des_pos[2] += vz_w * dt
        self.manual_des_pos[2] = float(np.clip(self.manual_des_pos[2], self.min_altitude, self.max_altitude))
        self.manual_des_yaw = float(np.arctan2(math.sin(self.manual_des_yaw + yaw_rate * dt), math.cos(self.manual_des_yaw + yaw_rate * dt)))
        self.manual_des_roll = float(np.clip(
            self.manual_des_roll + roll_rate * dt,
            -self.manual_attitude_limit_rad,
            self.manual_attitude_limit_rad
        ))
        self.manual_des_pitch = float(np.clip(
            self.manual_des_pitch + pitch_rate * dt,
            -self.manual_attitude_limit_rad,
            self.manual_attitude_limit_rad
        ))

        self.target_position = self.manual_des_pos.copy()
        self.target_velocity = np.array([vx_w, vy_w, vz_w], dtype=float)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([
            self.manual_des_roll, self.manual_des_pitch, self.manual_des_yaw
        ], dtype=float)
        self.target_attitude_rate = np.array([roll_rate, pitch_rate, yaw_rate], dtype=float)

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
            f"AttCmd: roll={np.degrees(self.manual_des_roll):+5.1f}°, "
            f"pitch={np.degrees(self.manual_des_pitch):+5.1f}° | Pitch: current={current_pitch_deg:+5.1f}° | "
            f"rate R/P=[{np.degrees(self._last_manual_cmd.get('roll_rate', 0.0)):+5.1f}, "
            f"{np.degrees(self._last_manual_cmd.get('pitch_rate', 0.0)):+5.1f}]°/s\n"
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


def main(args=None):
    rclpy.init(args=args)
    controller = HnuterController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('接收到终止信号，退出节点。')
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
