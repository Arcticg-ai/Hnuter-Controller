#!/usr/bin/env python3
"""Hnuter PX4 position-offboard controller.

The node publishes only PX4 Offboard mode, trajectory setpoint, and vehicle
command topics. PX4 keeps ownership of the position, attitude, rate, and
actuator control loops. Gamepad control and keyboard-triggered rectangle,
Lissajous, and attitude-reference trajectories are retained.
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

from controllers.common.hnuter_log_paths import configure_ros_log_dir

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
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleCommandAck
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleStatus

try:
    import pygame
except Exception:  # 允许没有手柄/没有 pygame 时保持悬停
    pygame = None


# ============================================================
# 手柄管理器：从 hnuter104.py 移植，加入异常保护
# ============================================================
class GamepadManager:
    def __init__(self,
                 max_vxy: float = 1.0,
                 max_vz: float = 0.5,
                 max_yaw_rate: float = 0.6,
                 max_roll_rate: float = math.radians(20.0),
                 deadzone: float = 0.10,
                 expo: float = 0.40,
                 filter_tau: float = 0.20,
                 lt_axis: int = 2,
                 rt_axis: int = 5,
                 trigger_mode: str = 'minus_one_to_one',
                 logger=None):
        self.logger = logger
        self.joystick = None
        self.max_vxy = float(max_vxy)
        self.max_vz = float(max_vz)
        self.max_yaw_rate = float(max_yaw_rate)
        self.max_roll_rate = float(max_roll_rate)
        self.deadzone = float(deadzone)
        self.expo = float(expo)
        self.filter_tau = float(filter_tau)
        self.lt_axis = int(lt_axis)
        self.rt_axis = int(rt_axis)
        # 常见 Xbox/XInput 手柄 LT/RT: 未按=-1，按满=+1。
        # 若你的手柄是未按=0，按满=1，把 trigger_mode 改为 'zero_to_one'。
        # 若你的手柄是未按=+1，按满=-1，把 trigger_mode 改为 'one_to_minus_one'。
        self.trigger_mode = str(trigger_mode)
        self.filtered_cmds = {
            'vx_b': 0.0,
            'vy_b': 0.0,
            'vz': 0.0,
            'yaw_rate': 0.0,
            'roll_rate': 0.0,
            'lt': 0.0,
            'rt': 0.0,
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

            yaw_expo = self._apply_expo(self._apply_deadzone(raw_yaw))
            thr_expo = self._apply_expo(self._apply_deadzone(raw_throttle))
            roll_expo = self._apply_expo(self._apply_deadzone(raw_roll))
            pitch_expo = self._apply_expo(self._apply_deadzone(raw_pitch))
            lt_expo = self._trigger_to_unit(raw_lt)
            rt_expo = self._trigger_to_unit(raw_rt)

            # FLU 机体系：x 前，y 左，z 上；上推为正向前/上升
            target_vx_b = -pitch_expo * self.max_vxy
            target_vy_b = -roll_expo * self.max_vxy
            target_vz_w = -thr_expo * self.max_vz
            target_yaw_rate = -yaw_expo * self.max_yaw_rate

            # LT 增大期望 roll，RT 减小期望 roll。
            # 输出是 roll 角速度，后面在 update_trajectory() 中积分为目标横滚角。
            target_roll_rate = (lt_expo - rt_expo) * self.max_roll_rate

            alpha = dt / (self.filter_tau + dt) if self.filter_tau > 1e-3 else 1.0
            alpha = float(np.clip(alpha, 0.0, 1.0))

            self.filtered_cmds['vx_b'] += alpha * (target_vx_b - self.filtered_cmds['vx_b'])
            self.filtered_cmds['vy_b'] += alpha * (target_vy_b - self.filtered_cmds['vy_b'])
            self.filtered_cmds['vz'] += alpha * (target_vz_w - self.filtered_cmds['vz'])
            self.filtered_cmds['yaw_rate'] += alpha * (target_yaw_rate - self.filtered_cmds['yaw_rate'])
            self.filtered_cmds['roll_rate'] += alpha * (target_roll_rate - self.filtered_cmds['roll_rate'])
            self.filtered_cmds['lt'] = lt_expo
            self.filtered_cmds['rt'] = rt_expo
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
        super().__init__('hnuter_controller_gamepad')

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
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile_command)

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_position_callback, qos_profile_out)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile_out)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_callback, qos_profile_out)
        self.vehicle_control_mode_sub = self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self.control_mode_callback, qos_profile_out)
        self.vehicle_command_ack_sub = self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.vehicle_command_ack_callback, qos_profile_out)

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
        self.R = np.eye(3)                # ENU <- FLU
        self.nav_state = None
        self.control_offboard_enabled = False
        self.armed = False
        self.data_received = False
        self.local_position_received = False
        self.attitude_received = False
        self.px4_timestamp = 0

        # Offboard/Arm 启动状态机
        self.offboard_setpoint_counter = 0
        self._last_offboard_cmd_time = 0.0
        self._last_arm_cmd_time = 0.0
        self._last_arm_command_param1 = None

        # ====== 启动策略配置：防止 PX4 自动 disarm 后被程序反复 arm ======
        # True : 节点启动后自动尝试切 Offboard，并只自动 Arm 一次。
        # False: 节点只维持 OffboardControlMode 心跳，需要你用 QGC/遥控器手动 Arm。
        self.auto_arm_enabled = True

        # 强烈建议 False。PX4 如果因为预起飞超时、落地检测或 failsafe disarm，
        # 程序不应立刻再次解锁，否则会出现“反复 arm / 反复起落”的循环。
        self.rearm_after_auto_disarm = False

        # 自动 Arm 最多尝试次数。调试期建议 1；若想完全手动解锁，设 auto_arm_enabled=False。
        self.max_auto_arm_attempts = 1
        self.auto_arm_attempts = 0
        self.was_armed_once = False
        self._last_armed_state = False
        self.startup_blocked_after_disarm = False

        # PX4 要求进入 Offboard 前先连续发送 >1s 的 OffboardControlMode。
        # 这里 20Hz * 30 = 1.5s，留出裕量。
        self.offboard_warmup_ticks = 30
        self.mode_request_period_s = 1.0
        self.arm_request_period_s = 1.0

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

        # Takeoff setpoint constraints
        self.takeoff_xy_lock_time_s = 3.0
        self._xy_lock_position = np.zeros(2)
        self._takeoff_lock_start_time_s = None

        # Yaw variables
        self._yaw_initialized = False
        self.initial_yaw = 0.0

        self.max_climb_rate = 1.0

        self.target_position = np.array([0.0, 0.0, 1.3])
        self.target_velocity = np.zeros(3)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([0.0, 0.0, 0.0])
        self.target_attitude_rate = np.zeros(3)

        self.takeoff_height = 1.3
        self.max_altitude = 5.0
        self.min_altitude = 0.25
        self.manual_enabled = True
        self.takeoff_requested = False
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
        self.auto_traj_z = self.takeoff_height
        self.auto_traj_yaw = 0.0
        self.auto_traj_start_attitude = np.zeros(3)
        self.auto_traj_ready_margin = 0.08
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

        # Gamepad: 实机建议先用低速度，确认方向后再加大
        self.gamepad = GamepadManager(
            max_vxy=1.0,
            max_vz=0.5,
            max_yaw_rate=0.6,
            max_roll_rate=math.radians(20.0),
            deadzone=0.10,
            expo=0.40,
            filter_tau=0.20,
            lt_axis=2,
            rt_axis=5,
            trigger_mode='minus_one_to_one',
            logger=self.get_logger()
        )
        self.keyboard = KeyboardCommandReader(logger=self.get_logger())
        self.keyboard_timer = self.create_timer(0.1, self.poll_keyboard_commands)

        self.get_logger().info(
            'Hnuter PX4 position-offboard controller initialized: '
            'gamepad + keyboard trajectories; actuator control remains inside PX4.'
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
        if int(getattr(msg, 'arming_state', -1)) == self.ARMING_STATE_ARMED:
            self.armed = True
        self.nav_state = int(getattr(msg, 'nav_state', -1))

    def control_mode_callback(self, msg):
        self.control_offboard_enabled = bool(getattr(msg, 'flag_control_offboard_enabled', False))
        if hasattr(msg, 'flag_armed'):
            self.armed = bool(msg.flag_armed)

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
        accepted = result == getattr(VehicleCommandAck, 'VEHICLE_CMD_RESULT_ACCEPTED', 0)
        if accepted:
            if command == self.CMD_DO_SET_MODE:
                self.nav_state = self.NAVIGATION_STATE_OFFBOARD
                self.control_offboard_enabled = True
            elif command == self.CMD_COMPONENT_ARM_DISARM and self._last_arm_command_param1 is not None:
                self.armed = self._last_arm_command_param1 > 0.5
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)

    # ============================================================
    # Offboard/Arm startup logic
    # ============================================================
    def is_offboard(self) -> bool:
        return bool(self.control_offboard_enabled) or self.nav_state == self.NAVIGATION_STATE_OFFBOARD

    def timestamp_now_us(self) -> int:
        return int(self.px4_timestamp) if self.px4_timestamp > 0 else int(self.get_clock().now().nanoseconds / 1000)

    def offboard_startup_tick(self):
        # 1) 始终发送 OffboardControlMode 作为 proof-of-life，频率 20Hz。
        #    这是维持 Offboard 的心跳，不等价于重复 arm。
        self.publish_offboard_control_mode()

        # 2) 未收到状态数据前不切模式、不解锁。
        if not self.data_received or self.px4_timestamp <= 0:
            return

        # Offboard 切换前也持续发送轨迹设定值，避免 commander 因设定值流不完整而拒绝。
        self.publish_px4_trajectory_setpoint()

        self.offboard_setpoint_counter += 1
        now = time.time()

        # 3) 检测 PX4 是否从 armed 变成 disarmed。
        #    如果已经成功 arm 过一次，之后又被 PX4 自动上锁，默认禁止自动二次 arm。
        if self._last_armed_state and not self.armed:
            takeoff_was_requested = self.takeoff_requested
            self.was_armed_once = True
            self.takeoff_requested = False
            self.manual_pos_initialized = False
            if not takeoff_was_requested:
                self.startup_blocked_after_disarm = True
                self.auto_arm_attempts = 0
                self.was_armed_once = False
                self.get_logger().warn(
                    'PX4 在起飞许可前已自动上锁，可能是 COM_DISARM_PRFLT 预起飞超时。'
                    '已停止自动二次 Arm；按键盘 o 后会重新请求 Offboard/Arm 并起飞悬停。'
                )
            elif not self.rearm_after_auto_disarm:
                self.startup_blocked_after_disarm = True
                self.get_logger().warn(
                    'PX4 已从 armed 变为 disarmed。已阻止自动二次 Arm。'
                    '请检查是否触发 COM_DISARM_PRFLT、COM_DISARM_LAND、land detector 或 failsafe；'
                    '确认安全后重启本节点或手动 Arm。'
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
                self.get_logger().info('请求切换到 Offboard 模式...')
            return

        # 5) 已进入 Offboard 后再 Arm；等待键盘 o 作为起飞/解锁许可。
        if self.is_offboard() and not self.armed:
            if not self.takeoff_requested:
                return
            if not self.auto_arm_enabled:
                return
            if self.was_armed_once and not self.rearm_after_auto_disarm:
                return
            if self.auto_arm_attempts >= self.max_auto_arm_attempts:
                return
            if now - self._last_arm_cmd_time > self.arm_request_period_s:
                self.arm()
                self.auto_arm_attempts += 1
                self._last_arm_cmd_time = now
                self.get_logger().info(
                    f'请求 Arm 解锁... ({self.auto_arm_attempts}/{self.max_auto_arm_attempts})'
                )

        if self.armed:
            self.was_armed_once = True

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
        self._last_arm_command_param1 = 1.0
        self.publish_vehicle_command(self.CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        self._last_arm_command_param1 = 0.0
        self.publish_vehicle_command(self.CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def set_offboard_mode(self):
        # VEHICLE_CMD_DO_SET_MODE: param1=1(custom), param2=6(OFFBOARD)
        self.publish_vehicle_command(self.CMD_DO_SET_MODE, param1=1.0, param2=6.0)

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
                if self.startup_blocked_after_disarm and not self.was_armed_once:
                    self.startup_blocked_after_disarm = False
                    self.was_armed_once = False
                    self.auto_arm_attempts = 0
                    self._last_offboard_cmd_time = 0.0
                    self._last_arm_cmd_time = 0.0
                self.takeoff_requested = True
                self.manual_pos_initialized = False
                self._takeoff_lock_start_time_s = None
                self._z0_initialized = False
                self._z0 = 0.0
                self.get_logger().info('收到键盘 o：起飞许可已打开，开始爬升到悬停高度。')
            elif key == '1':
                self.pending_auto_traj_mode = 'rectangle'
                self.get_logger().info('收到键盘 1：矩形轨迹已排队，悬停稳定后开始。')
            elif key == '2':
                self.pending_auto_traj_mode = 'lissajous'
                self.get_logger().info('收到键盘 2：李萨如轨迹已排队，悬停稳定后开始。')
            elif key == '3':
                self.pending_auto_traj_mode = 'attitude'
                self.get_logger().info('收到键盘 3：姿态角轨迹已排队，悬停稳定后开始。')

    def _trajectory_ready(self, current_time: float) -> bool:
        if not (self.is_offboard() and self.armed and self.manual_pos_initialized):
            return False
        if current_time < self.takeoff_xy_lock_time_s:
            return False
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
    # Manual trajectory: gamepad velocity -> desired position/yaw
    # ============================================================
    def update_trajectory(self, current_time: float, dt: float):
        if not self._z0_initialized:
            self._z0 = float(self.position[2])
            self._z0_initialized = True

        # 未进入 Offboard 或未解锁前，目标点贴住当前点，避免一解锁就猛冲。
        if (not self.is_offboard()) or (not self.armed):
            self.manual_pos_initialized = False
            self.auto_traj_mode = 'hover'
            self._takeoff_lock_start_time_s = None
            self.manual_des_roll = 0.0
            self.target_position = np.array([self.position[0], self.position[1], 0.0])
            self.target_velocity = np.zeros(3)
            self.target_acceleration = np.zeros(3)
            self.target_attitude = np.array([0.0, 0.0, self.initial_yaw])
            self.target_attitude_rate = np.zeros(3)
            return

        if not self.takeoff_requested:
            self.manual_pos_initialized = False
            self.auto_traj_mode = 'hover'
            self._z0_initialized = False
            self._z0 = 0.0
            self._takeoff_lock_start_time_s = None
            self.manual_des_roll = 0.0
            self._last_manual_cmd = self._zero_manual_cmd()
            self.target_position = np.array([self.position[0], self.position[1], 0.0])
            self.target_velocity = np.zeros(3)
            self.target_acceleration = np.zeros(3)
            self.target_attitude = np.array([0.0, 0.0, self.initial_yaw])
            self.target_attitude_rate = np.zeros(3)
            return

        if not self.manual_pos_initialized:
            self.manual_des_pos = np.array([self.position[0], self.position[1], max(0.0, self.position[2] - self._z0)])
            self.manual_des_yaw = self.initial_yaw if self._yaw_initialized else 0.0
            self.manual_des_roll = 0.0
            self._xy_lock_position = self.position[:2].copy()
            self._takeoff_lock_start_time_s = current_time
            self.manual_pos_initialized = True

        if self.pending_auto_traj_mode is not None and self._trajectory_ready(current_time):
            self._start_auto_trajectory(self.pending_auto_traj_mode, current_time)
            self.pending_auto_traj_mode = None

        if self.auto_traj_mode != 'hover':
            if self._update_auto_trajectory(current_time):
                return

        cmds = self.gamepad.get_velocity_commands(dt) if self.manual_enabled else self._zero_manual_cmd()
        self._last_manual_cmd = cmds.copy()

        # 初始爬升：若手柄不动，则自动缓慢爬到 takeoff_height；若手柄给 z，则叠加人工指令
        z_auto_vel = 0.0
        if self.manual_des_pos[2] < self.takeoff_height:
            z_err = self.takeoff_height - self.manual_des_pos[2]
            z_auto_vel = float(np.clip(z_err, 0.0, self.max_climb_rate))

        yaw_ref = self.manual_des_yaw
        vx_w = cmds['vx_b'] * math.cos(yaw_ref) - cmds['vy_b'] * math.sin(yaw_ref)
        vy_w = cmds['vx_b'] * math.sin(yaw_ref) + cmds['vy_b'] * math.cos(yaw_ref)
        vz_w = cmds['vz'] + z_auto_vel
        yaw_rate = cmds['yaw_rate']
        roll_rate = cmds.get('roll_rate', 0.0)

        takeoff_elapsed_s = (
            current_time - self._takeoff_lock_start_time_s
            if self._takeoff_lock_start_time_s is not None else float('inf')
        )
        if takeoff_elapsed_s < self.takeoff_xy_lock_time_s:
            vx_w = 0.0
            vy_w = 0.0
            self.manual_des_pos[0] = float(self._xy_lock_position[0])
            self.manual_des_pos[1] = float(self._xy_lock_position[1])

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
        self.control_loop_count += 1
        self.publish_px4_trajectory_setpoint()

        now = time.time()
        if now - self._last_debug_print_time >= self.debug_print_period_s:
            state = '起飞/轨迹控制' if self.takeoff_requested else '等待起飞许可'
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
            f"Takeoff gate: requested={self.takeoff_requested} | restart_blocked={self.startup_blocked_after_disarm}\n"
            f"Target ENU/Zrel: [{self.target_position[0]:6.2f}, {self.target_position[1]:6.2f}, {self.target_position[2]:6.2f}] m\n"
            f"Current ENU/Zrel: [{self.position[0]:6.2f}, {self.position[1]:6.2f}, {pos_curr_rel_z:6.2f}] m\n"
            f"Keyboard trajectory: active={self.auto_traj_mode} | pending={self.pending_auto_traj_mode}\n"
            f"Gamepad: vx_b={self._last_manual_cmd['vx_b']:+4.2f}, vy_b={self._last_manual_cmd['vy_b']:+4.2f}, "
            f"vz={self._last_manual_cmd['vz']:+4.2f}, yaw_rate={self._last_manual_cmd['yaw_rate']:+4.2f}, "
            f"LT={self._last_manual_cmd.get('lt', 0.0):4.2f}, RT={self._last_manual_cmd.get('rt', 0.0):4.2f}\n"
            f"RollCmd: des={np.degrees(self.manual_des_roll):+5.1f}° | Pitch: current={current_pitch_deg:+5.1f}° | "
            f"roll_rate={np.degrees(self._last_manual_cmd.get('roll_rate', 0.0)):+5.1f}°/s\n"
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
