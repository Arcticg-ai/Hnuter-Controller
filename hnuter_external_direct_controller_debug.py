#!/usr/bin/env python3
"""
Hnuter Tiltrotor Direct Actuator Debug Controller

核心修改：
1. 将 hnuter104.py 中的 GamepadManager 移植到 ROS2/PX4 外部控制节点。
2. 手柄输出为机体系速度 vx_b/vy_b、世界系垂直速度 vz、偏航角速度 yaw_rate。
3. LT/RT 触发器输出期望俯仰角速度，积分为目标 pitch 姿态。
4. 速度指令通过欧拉积分生成期望位置与期望偏航，送入 px4_equiv direct 控制器与 Hnuter 分配逆解。
5. 修复 Offboard/Arm 启动逻辑：先连续发布 OffboardControlMode，再切 Offboard，最后 Arm。
6. Offboard+Arm 后先零油门做倾转自检，按键盘 o 后才起飞悬停。
7. 避免 hover_only/xy_lock 永久覆盖手动目标点。
8. 键盘输入 1 执行一圈矩形轨迹，输入 2 执行一圈李萨如轨迹，
   输入 3 依次改变 roll/pitch/yaw 小角度后恢复，完成后回到悬停。
9. 关闭退出绘图与 /plot_data 发布。
10. 姿态反馈采用连续等价分支，避免 roll 0/180 表示跳变导致 direct 控制方向突变。
11. 本文件用于 direct actuator 调试：按 o 后直接控制 actuator_motors/servos，
   并记录 PX4 反馈、外部控制指令、/fmu/out actuator 输出到 CSV。
   默认 HNUTER_CONTROL_MODE=direct；若设为 px4，则用同一日志格式记录 PX4 position baseline。
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
from pathlib import Path

# PX4 uses fixed DDS topic names. Keep SITL telemetry local unless remote DDS
# access is explicitly requested, otherwise another PX4 on the LAN can mix in.
if os.environ.get('HNUTER_ALLOW_REMOTE_DDS', '0') != '1':
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_STATIC_PEERS', None)

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float64

from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleAttitudeSetpoint
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

try:
    import pygame
except Exception:  # 允许没有手柄/没有 pygame 时保持悬停
    pygame = None

from hnuter_gamepad import GamepadManager as RobustGamepadManager


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
                 max_pitch_rate: float = math.radians(20.0),
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
        self.max_pitch_rate = float(max_pitch_rate)
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
            'pitch_rate': 0.0,
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

            # LT 增大期望 pitch，RT 减小期望 pitch。
            # 输出是 pitch 角速度，后面在 update_trajectory() 中积分为目标俯仰角。
            target_pitch_rate = (lt_expo - rt_expo) * self.max_pitch_rate

            alpha = dt / (self.filter_tau + dt) if self.filter_tau > 1e-3 else 1.0
            alpha = float(np.clip(alpha, 0.0, 1.0))

            self.filtered_cmds['vx_b'] += alpha * (target_vx_b - self.filtered_cmds['vx_b'])
            self.filtered_cmds['vy_b'] += alpha * (target_vy_b - self.filtered_cmds['vy_b'])
            self.filtered_cmds['vz'] += alpha * (target_vz_w - self.filtered_cmds['vz'])
            self.filtered_cmds['yaw_rate'] += alpha * (target_yaw_rate - self.filtered_cmds['yaw_rate'])
            self.filtered_cmds['pitch_rate'] += alpha * (target_pitch_rate - self.filtered_cmds['pitch_rate'])
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
        super().__init__('hnuter_controller_direct_debug')

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
        self.vehicle_attitude_setpoint_pub = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint_v1', qos_profile_command)
        self.vehicle_thrust_setpoint_pub = self.create_publisher(
            VehicleThrustSetpoint, '/fmu/in/vehicle_thrust_setpoint', qos_profile_in)
        self.vehicle_torque_setpoint_pub = self.create_publisher(
            VehicleTorqueSetpoint, '/fmu/in/vehicle_torque_setpoint', qos_profile_in)
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
        self._vehicle_control_mode_received = False
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
            'vx_b': 0.0,
            'vy_b': 0.0,
            'vz': 0.0,
            'yaw_rate': 0.0,
            'pitch_rate': 0.0,
            'lt': 0.0,
            'rt': 0.0,
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

        # Actuator limits
        self.pitch_command_limit_rad = np.radians(180.0)
        self.alpha_limit_rad = np.radians(env_float('HNUTER_ALPHA_LIMIT_DEG', 185.0))
        self.theta_limit_rad = np.radians(env_float('HNUTER_THETA_LIMIT_DEG', 45.0))
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
        # Static transmission calibration only. Delay, lag and rate limits
        # belong to the plant and must not be applied again by this controller.
        self._tilt_static_gain = np.array([1.414, 1.414, 0.700, 0.700])
        self.direct_gain_ramp_time_s = 6.0

        self.integral_pos_error = np.zeros(3)
        self.integral_e_R = np.zeros(3)

        # Direct-actuator mode cannot rely on PX4's inner-loop guards. Keep
        # takeoff torque gentle so a small pitch error cannot saturate tail
        # thrust and flip the aircraft before the altitude loop settles.
        self.direct_takeoff_KR = np.array([1.5, 1.5, 1.5])
        self.direct_takeoff_KI = np.array([0.0, 0.0, 0.12])
        self.direct_takeoff_Domega = np.array([1.2, 1.2, 1.2])
        self.direct_xy_lock_KR = np.array([1.5, 1.5, 1.5])
        self.direct_xy_lock_KI = np.array([0.0, 0.0, 0.12])
        self.direct_xy_lock_Domega = np.array([1.2, 1.2, 1.2])
        self.direct_KR = np.array([1.5, 1.5, env_float('HNUTER_DIRECT_KR_YAW', 3.2)])
        self.direct_KI = np.array([0.02, 0.02, 0.12])
        self.direct_Domega = np.array([1.2, 1.2, env_float('HNUTER_DIRECT_DOMEGA_YAW', 2.2)])
        self.direct_integral_limit = np.array([0.5, 0.5, 1.5])
        self.direct_takeoff_tau_limit = np.array([0.90, 0.90, 0.50])
        self.direct_xy_lock_tau_limit = np.array([0.90, 0.90, 0.50])
        self.direct_tau_limit = np.array([0.90, 0.90, env_float('HNUTER_DIRECT_TAU_YAW_LIMIT', 1.80)])
        self.direct_yaw_control_enabled = os.environ.get(
            'HNUTER_DIRECT_YAW_CONTROL', '1'
        ).strip().lower() in ('1', 'true', 'yes', 'on')
        self.direct_takeoff_vertical_only_time_s = self.takeoff_tilt_suppress_time_s
        self.direct_takeoff_vertical_only_height_m = 0.0

        self.max_acc_xy = 20.0
        self.max_acc_z = 20.0
        self.max_climb_rate = 0.35
        self.manual_max_position_lead_xy = 0.6
        self.manual_max_position_lead_z = 0.45
        self.manual_max_yaw_lead_rad = np.radians(25.0)
        # Direct debug: 按 o 后不交给 PX4 位置控制器，而是直接发布 actuator_motors/servos。
        # 若要用同一份日志结构记录 PX4 position baseline，启动前设置 HNUTER_CONTROL_MODE=px4。
        control_mode_env = os.environ.get('HNUTER_CONTROL_MODE', 'direct').strip().lower()
        self.use_px4_position_takeoff = control_mode_env in ('px4', 'px4_position', 'position')
        self.use_px4_attitude_control = control_mode_env in ('px4_attitude', 'attitude')
        self.use_px4_closed_loop = self.use_px4_position_takeoff or self.use_px4_attitude_control
        if self.use_px4_attitude_control:
            self.debug_control_mode = 'px4_attitude'
        elif self.use_px4_position_takeoff:
            self.debug_control_mode = 'px4_position'
        else:
            self.debug_control_mode = 'direct'
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
        # LT/RT 积分得到的俯仰姿态期望。
        # 正号沿当前 ENU pitch 约定；若实机观察方向相反，
        # 只需要在 GamepadManager 中把 target_pitch_rate 改成 (rt_expo - lt_expo)。
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
        self.lissajous_amp_x = 1.0
        self.lissajous_amp_y = 0.75
        self.lissajous_a = 2
        self.lissajous_b = 3
        self.lissajous_period_s = 24.0
        # Trajectory 3 is intended to validate attitude-frame signs first. Large
        # forced attitude steps fight the position hold loop and can saturate the
        # direct controller, so keep the default conservative and override from
        # the shell when doing stress tests.
        self.attitude_step_angle_rad = math.radians(env_float('HNUTER_ATTITUDE_STEP_DEG', 35.0))
        self.attitude_step_axis_rad = np.array([
            math.radians(env_float('HNUTER_ATTITUDE_STEP_ROLL_DEG', math.degrees(self.attitude_step_angle_rad))),
            math.radians(env_float('HNUTER_ATTITUDE_STEP_PITCH_DEG', math.degrees(self.attitude_step_angle_rad))),
            math.radians(env_float('HNUTER_ATTITUDE_STEP_YAW_DEG', math.degrees(self.attitude_step_angle_rad))),
        ], dtype=float)
        self.attitude_step_axis_rad[1] = float(np.clip(
            self.attitude_step_axis_rad[1],
            -self.pitch_command_limit_rad,
            self.pitch_command_limit_rad,
        ))
        self.attitude_segment_time_s = env_float('HNUTER_ATTITUDE_SEGMENT_S', 5.0)
        self.attitude_peak_hold_s = env_float('HNUTER_ATTITUDE_PEAK_HOLD_S', 1.0)
        self.attitude_test_altitude_only = os.environ.get(
            'HNUTER_ATTITUDE_TEST_ALTITUDE_ONLY', '0'
        ).strip().lower() in ('1', 'true', 'yes', 'on')
        self.attitude_test_max_acc_xy = env_float('HNUTER_ATTITUDE_TEST_MAX_ACC_XY', 3.0)
        self.attitude_test_altitude_m = env_float('HNUTER_ATTITUDE_TEST_ALTITUDE_M', self.takeoff_height)

        self.tuning_path = os.path.expanduser(os.environ.get(
            'HNUTER_TUNING_FILE', '~/px4_ws_ros2/hnuter_direct_tuning.json'
        ))
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

        # Gamepad: 实机建议先用低速度，确认方向后再加大
        self.gamepad = RobustGamepadManager(
            max_vxy=0.6,
            max_vz=0.3,
            max_yaw_rate=0.4,
            max_pitch_rate=math.radians(20.0),
            deadzone=0.10,
            expo=0.40,
            filter_tau=0.35,
            lt_axis=2,
            rt_axis=5,
            trigger_mode='minus_one_to_one',
            logger=self.get_logger()
        )
        self.keyboard = KeyboardCommandReader(logger=self.get_logger())
        self.keyboard_timer = self.create_timer(0.1, self.poll_keyboard_commands)
        self.tuning_timer = self.create_timer(0.5, self._load_tuning_file)

        # CSV diagnostics
        self.diagnostic_enabled = True
        self.diagnostic_period_s = 0.10
        self._last_diagnostic_log_time = -1.0
        self.diagnostic_path = f'hnuter_{self.debug_control_mode}_debug_{int(time.time())}.csv'
        self._diagnostic_file = None
        self._diagnostic_writer = None
        if self.diagnostic_enabled:
            self._diagnostic_file = open(self.diagnostic_path, 'w', newline='', buffering=1)
            self._diagnostic_writer = csv.writer(self._diagnostic_file)
            self._diagnostic_writer.writerow(self._diagnostic_header())

        self.get_logger().info(
            f'Hnuter actuator debug controller initialized: mode={self.debug_control_mode}'
        )
        if self.use_px4_attitude_control:
            self.get_logger().info(
                f'PX4 Hnuter 姿态闭环模式：位置轨迹与姿态期望同时发布；诊断日志写入 {self.diagnostic_path}'
            )
        elif self.use_px4_position_takeoff:
            self.get_logger().info(
                f'PX4 baseline 记录模式：按 o 后使用 PX4 position Offboard；诊断日志写入 {self.diagnostic_path}'
            )
        else:
            self.get_logger().warn(
                f'DIRECT DEBUG 模式：px4_equiv actuator direct；诊断日志写入 {self.diagnostic_path}'
            )

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
            "attitude_test_altitude_only": bool(self.attitude_test_altitude_only),
            "attitude_test_altitude_m": float(self.attitude_test_altitude_m),
            "attitude_test_max_acc_xy": float(self.attitude_test_max_acc_xy),
            "alpha_limit_deg": float(math.degrees(self.alpha_limit_rad)),
            "theta_limit_deg": float(math.degrees(self.theta_limit_rad)),
            "manual_pitch_limit_deg": float(math.degrees(self.manual_pitch_limit_rad)),
            "direct_safety_attitude_check_enabled": bool(self.direct_safety_attitude_check_enabled),
            "direct_safety_attitude_limit_deg": float(math.degrees(self.direct_safety_pitch_limit_rad)),
            "allocator_force_x_sign": float(self.allocator_force_x_sign),
            "allocator_force_y_sign": float(self.allocator_force_y_sign),
            "direct_KR": self.direct_KR.tolist(),
            "direct_KI": self.direct_KI.tolist(),
            "direct_Domega": self.direct_Domega.tolist(),
            "direct_integral_limit": self.direct_integral_limit.tolist(),
            "direct_tau_limit": self.direct_tau_limit.tolist(),
            "direct_takeoff_KR": self.direct_takeoff_KR.tolist(),
            "direct_takeoff_KI": self.direct_takeoff_KI.tolist(),
            "direct_takeoff_Domega": self.direct_takeoff_Domega.tolist(),
            "direct_takeoff_tau_limit": self.direct_takeoff_tau_limit.tolist(),
            "direct_xy_lock_KR": self.direct_xy_lock_KR.tolist(),
            "direct_xy_lock_KI": self.direct_xy_lock_KI.tolist(),
            "direct_xy_lock_Domega": self.direct_xy_lock_Domega.tolist(),
            "direct_xy_lock_tau_limit": self.direct_xy_lock_tau_limit.tolist(),
            "direct_gain_ramp_time_s": float(self.direct_gain_ramp_time_s),
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
        self.attitude_test_altitude_only = self._tuning_bool(
            data, 'attitude_test_altitude_only', self.attitude_test_altitude_only
        )
        self.attitude_test_altitude_m = self._tuning_float(
            data, 'attitude_test_altitude_m', self.attitude_test_altitude_m
        )
        self.attitude_test_max_acc_xy = self._tuning_float(data, 'attitude_test_max_acc_xy', self.attitude_test_max_acc_xy)
        self.alpha_limit_rad = math.radians(
            self._tuning_float(data, 'alpha_limit_deg', math.degrees(self.alpha_limit_rad))
        )
        self.theta_limit_rad = math.radians(
            self._tuning_float(data, 'theta_limit_deg', math.degrees(self.theta_limit_rad))
        )
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

        self.direct_KR = self._tuning_array(data, 'direct_KR', self.direct_KR)
        self.direct_KI = self._tuning_array(data, 'direct_KI', self.direct_KI)
        self.direct_Domega = self._tuning_array(data, 'direct_Domega', self.direct_Domega)
        self.direct_integral_limit = np.maximum(
            self._tuning_array(data, 'direct_integral_limit', self.direct_integral_limit),
            0.0,
        )
        self.direct_tau_limit = self._tuning_array(data, 'direct_tau_limit', self.direct_tau_limit)
        self.direct_takeoff_KR = self._tuning_array(data, 'direct_takeoff_KR', self.direct_takeoff_KR)
        self.direct_takeoff_KI = self._tuning_array(data, 'direct_takeoff_KI', self.direct_takeoff_KI)
        self.direct_takeoff_Domega = self._tuning_array(data, 'direct_takeoff_Domega', self.direct_takeoff_Domega)
        self.direct_takeoff_tau_limit = self._tuning_array(data, 'direct_takeoff_tau_limit', self.direct_takeoff_tau_limit)
        self.direct_xy_lock_KR = self._tuning_array(data, 'direct_xy_lock_KR', self.direct_xy_lock_KR)
        self.direct_xy_lock_KI = self._tuning_array(data, 'direct_xy_lock_KI', self.direct_xy_lock_KI)
        self.direct_xy_lock_Domega = self._tuning_array(data, 'direct_xy_lock_Domega', self.direct_xy_lock_Domega)
        self.direct_xy_lock_tau_limit = self._tuning_array(data, 'direct_xy_lock_tau_limit', self.direct_xy_lock_tau_limit)
        self.direct_gain_ramp_time_s = max(
            0.5,
            self._tuning_float(data, 'direct_gain_ramp_time_s', self.direct_gain_ramp_time_s),
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

        now = time.time()
        if force or now - self._last_tuning_log_time > 1.0:
            self.get_logger().info(
                '在线调参已加载: '
                f'att_step={np.round(np.degrees(self.attitude_step_axis_rad), 1).tolist()}deg, '
                f'peak_hold={self.attitude_peak_hold_s:.1f}s, '
                f'att_z={self.attitude_test_altitude_m:.1f}m, '
                f'alpha_lim={math.degrees(self.alpha_limit_rad):.1f}deg, '
                f'manual_pitch_lim={math.degrees(self.manual_pitch_limit_rad):.1f}deg, '
                f'att_safety={self.direct_safety_attitude_check_enabled}, '
                f'theta_lim={math.degrees(self.theta_limit_rad):.1f}deg, '
                f'force_sign=[{self.allocator_force_x_sign:+.0f}, {self.allocator_force_y_sign:+.0f}], '
                f'KR={np.round(self.direct_KR, 3).tolist()}, '
                f'D={np.round(self.direct_Domega, 3).tolist()}, '
                f'tau_lim={np.round(self.direct_tau_limit, 3).tolist()}'
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
        # In the current Hnuter SITL bridge, vehicle_status.arming_state does
        # not match px4_msgs' ARMING_STATE_* constants. Use our own arm/disarm
        # command ACKs as the direct-controller armed latch instead.

    def control_mode_callback(self, msg):
        self.control_offboard_enabled = bool(getattr(msg, 'flag_control_offboard_enabled', False))
        actual_armed = bool(getattr(msg, 'flag_armed', False))
        self._vehicle_control_mode_received = True
        self._armed_from_control_mode = actual_armed
        self.armed = actual_armed

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
                    # An accepted command only means commander accepted the request.
                    # Start the takeoff ramp from VehicleControlMode.flag_armed so
                    # spool-up and DDS latency cannot accumulate a hidden z step.
                    self._optimistic_armed_until = time.time() + 1.5
                else:
                    self._optimistic_armed_until = 0.0
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
        if not self.data_received or self.px4_timestamp <= 0 or not self._vehicle_control_mode_received:
            return

        # Offboard 切换前也要持续发送对应 setpoint，避免 commander 因 setpoint 不完整而拒绝。
        if self._use_px4_position_mode():
            self.publish_px4_trajectory_setpoint()
        elif self.use_px4_attitude_control:
            self.publish_px4_trajectory_setpoint()
            self.publish_px4_attitude_setpoint()
        elif not self.is_offboard():
            self.publish_idle_direct_actuator_setpoint()

        self.offboard_setpoint_counter += 1
        now = time.time()

        # 3) 检测 PX4 是否从 armed 变成 disarmed。
        #    如果已经成功 arm 过一次，之后又被 PX4 自动上锁，默认禁止自动二次 arm。
        if self._last_armed_state and not self.armed:
            self.was_armed_once = True
            self.takeoff_requested = False
            self.manual_pos_initialized = False
            self.integral_pos_error[:] = 0.0
            self.integral_e_R[:] = 0.0
            if not self.rearm_after_auto_disarm:
                self.startup_blocked_after_disarm = True
                self.preflight_disarm_waiting_for_o = True
                self.get_logger().warn(
                    'PX4 已从 armed 变为 disarmed。无论当前落地检测状态如何，均已阻止自动二次 Arm。'
                    '请检查是否触发 COM_DISARM_PRFLT、COM_DISARM_LAND、land detector 或 failsafe；'
                    '确认安全后按 o 明确发起下一次 Offboard/Arm。'
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
        msg.attitude = self.use_px4_attitude_control
        msg.body_rate = False
        # 兼容不同 px4_msgs 版本
        if hasattr(msg, 'thrust_and_torque'):
            msg.thrust_and_torque = False
        if hasattr(msg, 'direct_actuator'):
            msg.direct_actuator = not self.use_px4_closed_loop
        msg.timestamp = self.timestamp_now_us()
        self.offboard_control_mode_pub.publish(msg)

    def _yaw_enu_to_ned(self, yaw_enu: float) -> float:
        yaw_ned = 0.5 * math.pi - float(yaw_enu)
        return float(math.atan2(math.sin(yaw_ned), math.cos(yaw_ned)))

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
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.yaw = self._yaw_enu_to_ned(self.manual_des_yaw)
        msg.yawspeed = float(-self.target_attitude_rate[2])
        self.trajectory_setpoint_pub.publish(msg)

    @staticmethod
    def _rotation_matrix_to_quaternion(rotation):
        matrix = np.asarray(rotation, dtype=float)
        trace = float(np.trace(matrix))
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            quaternion = np.array([
                0.25 * scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ])
        else:
            diagonal = np.diag(matrix)
            axis = int(np.argmax(diagonal))
            if axis == 0:
                scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
                quaternion = np.array([
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                ])
            elif axis == 1:
                scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
                quaternion = np.array([
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                ])
            else:
                scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
                quaternion = np.array([
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                ])
        return quaternion / max(float(np.linalg.norm(quaternion)), 1e-6)

    def publish_px4_attitude_setpoint(self):
        rotation = (
            self.target_R_des_ned_frd
            if self.target_R_des_ned_frd is not None
            else self._direct_desired_attitude_ned_frd(self.target_attitude)
        )
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = self.timestamp_now_us()
        msg.q_d = self._rotation_matrix_to_quaternion(rotation).astype(float).tolist()
        msg.yaw_sp_move_rate = float(-self.target_attitude_rate[2])
        msg.thrust_body = [float('nan'), float('nan'), float('nan')]
        self.vehicle_attitude_setpoint_pub.publish(msg)

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

    def _physical_tilt_to_normalized(self, angle: float, channel: int) -> float:
        """Invert static transmission gain without predicting plant dynamics."""
        gain = self._tilt_static_gain[channel]
        nominal_range = math.radians(185.0 if channel < 2 else 180.0)
        return float(np.clip(angle / max(gain * nominal_range, 1e-6), -1.0, 1.0))

    def publish_direct_actuator_setpoint(self, motor_controls, alpha1, alpha2, theta1, theta2):
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
        servo_msg.control[0] = self._physical_tilt_to_normalized(alpha2, 0)
        servo_msg.control[1] = self._physical_tilt_to_normalized(alpha1, 1)
        servo_msg.control[2] = self._physical_tilt_to_normalized(theta2, 2)
        servo_msg.control[3] = self._physical_tilt_to_normalized(theta1, 3)
        self.last_servo_cmd = np.array(servo_msg.control)
        self.actuator_servos_pub.publish(servo_msg)

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
        self._theta1_cmd = self._slew_limit(self._theta1_cmd, theta1, self.servo_rate_limit_rad_s, dt)
        self._theta2_cmd = self._slew_limit(self._theta2_cmd, theta2, self.servo_rate_limit_rad_s, dt)

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
            'vx_b': 0.0,
            'vy_b': 0.0,
            'vz': 0.0,
            'yaw_rate': 0.0,
            'pitch_rate': 0.0,
            'lt': 0.0,
            'rt': 0.0,
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
                self.get_logger().info('收到键盘 2：李萨如轨迹已排队，悬停稳定后开始。')
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
            mode_text = '李萨如'
        elif mode == 'attitude':
            self.auto_traj_origin_xy = self.auto_traj_start_pos[:2].copy()
            mode_text = '姿态角'
        else:
            self.auto_traj_origin_xy = self.auto_traj_start_pos[:2].copy()
            mode_text = '矩形'

        self.manual_des_pos = self.auto_traj_start_pos.copy()
        self.manual_des_pitch = 0.0
        self.integral_pos_error[:] = 0.0
        self.integral_e_R[:] = 0.0
        finish_text = '完成后回到该点悬停。'
        self.get_logger().info(
            f'开始执行{mode_text}轨迹：起点 [{self.auto_traj_start_pos[0]:.2f}, '
            f'{self.auto_traj_start_pos[1]:.2f}, {self.auto_traj_start_pos[2]:.2f}]，'
            f'{finish_text}'
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
        if finished_mode == 'attitude':
            self.manual_des_pos = self.auto_traj_start_pos.copy()
            self.manual_des_pos[2] = self.auto_traj_z
        else:
            self.manual_des_pos = self.auto_traj_start_pos.copy()
        self.manual_des_yaw = self.auto_traj_yaw
        self.manual_des_pitch = 0.0
        self.target_position = self.manual_des_pos.copy()
        self.target_velocity = np.zeros(3)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([0.0, 0.0, self.manual_des_yaw], dtype=float)
        self.target_attitude_rate = np.zeros(3)
        self.target_R_des_ned_frd = None
        self.integral_pos_error[:] = 0.0
        self.integral_e_R[:] = 0.0
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
        peak_hold_time = float(self.attitude_peak_hold_s)
        cycle_time = 2.0 * segment_time + peak_hold_time
        active_axes = np.flatnonzero(np.abs(self.attitude_step_axis_rad) > math.radians(0.01))
        if active_axes.size == 0:
            return self.auto_traj_start_attitude.copy(), np.zeros(3), None, True

        total_time = float(active_axes.size) * cycle_time
        if elapsed >= total_time:
            return self.auto_traj_start_attitude.copy(), np.zeros(3), None, True

        sequence_index = min(int(elapsed / cycle_time), active_axes.size - 1)
        axis_idx = int(active_axes[sequence_index])
        cycle_elapsed = elapsed - sequence_index * cycle_time
        step_rad = float(self.attitude_step_axis_rad[axis_idx])

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

        cmds = self.gamepad.get_velocity_commands(dt) if self.manual_enabled else self._zero_manual_cmd()
        self._last_manual_cmd = cmds.copy()

        yaw_rate = float(cmds['yaw_rate'])
        current_yaw_enu = self._current_yaw_enu()
        manual_yaw_active = abs(yaw_rate) > 1e-5

        yaw_ref = self.manual_des_yaw
        vx_w = cmds['vx_b'] * math.cos(yaw_ref) - cmds['vy_b'] * math.sin(yaw_ref)
        vy_w = cmds['vx_b'] * math.sin(yaw_ref) + cmds['vy_b'] * math.cos(yaw_ref)
        pitch_rate = cmds.get('pitch_rate', 0.0)
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
        previous_pitch = self.manual_des_pitch
        self.manual_des_pitch = float(np.clip(
            previous_pitch + pitch_rate * dt,
            -self.manual_pitch_limit_rad,
            self.manual_pitch_limit_rad
        ))
        realized_pitch_rate = (self.manual_des_pitch - previous_pitch) / max(dt, 1e-3)
        self._limit_manual_position_lead(
            clamp_xy=manual_xy_active,
            clamp_z=(abs(manual_vz) > 1e-5 or auto_climb_active),
        )
        realized_vxy = (self.manual_des_pos[:2] - prev_xy) / max(dt, 1e-3)

        self.target_position = self.manual_des_pos.copy()
        self.target_velocity = np.array([realized_vxy[0], realized_vxy[1], vz_w], dtype=float)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([0.0, self.manual_des_pitch, self.manual_des_yaw], dtype=float)
        self.target_attitude_rate = np.array([0.0, realized_pitch_rate, yaw_rate], dtype=float)
        self.target_R_des_ned_frd = None

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

        if not self.use_px4_closed_loop and not self.armed:
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

        if self.use_px4_closed_loop:
            self.control_loop_count += 1
            self.last_F1 = 0.0
            self.last_F2 = 0.0
            self.last_F3 = 0.0
            self.last_W = np.zeros(6)
            self.publish_px4_trajectory_setpoint()
            if self.use_px4_attitude_control:
                self.publish_px4_attitude_setpoint()
            self._write_diagnostic_row(current_time, self.debug_control_mode)
            now = time.time()
            if now - self._last_debug_print_time >= self.debug_print_period_s:
                self.get_logger().info(
                    f'PX4 {self.debug_control_mode} Offboard 起飞/悬停 dt={dt * 1000:.1f}ms | '
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
    def _thrust_to_normalized_motor_control(thrust, motor_constant=8.54858e-05,
                                            min_velocity=10.0, max_velocity=1000.0):
        if thrust <= 0.0:
            return 0.0
        velocity_range = max_velocity - min_velocity
        if velocity_range <= 1e-3:
            return 0.0
        velocity = math.sqrt(max(thrust, 0.0) / motor_constant)
        return float(np.clip((velocity - min_velocity) / velocity_range, 0.0, 1.0))

    @staticmethod
    def _thrust_to_normalized_bidirectional_motor_control(thrust, motor_constant=8.54858e-05,
                                                         max_velocity=1000.0):
        if abs(thrust) <= 1e-8 or motor_constant <= 1e-8 or max_velocity <= 1e-8:
            return 0.0
        velocity = math.sqrt(abs(thrust) / motor_constant)
        control = velocity / max_velocity
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
            # Match the custom PX4 Hnuter allocator convention. ULog comparison
            # confirms this sign produces the same servo0-servo1 differential
            # as the stable internal-controller path.
            float(-tau_c[2]),
        ], dtype=float)

    def _direct_prearm_failure_reason(self) -> str:
        if (self.use_px4_closed_loop
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
        if self.use_px4_closed_loop or not (self.takeoff_requested and self.armed):
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
        vertical_only_active = (
            takeoff_elapsed_s < self.direct_takeoff_vertical_only_time_s
            or pos_rel_z < self.direct_takeoff_vertical_only_height_m
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

        self.integral_pos_error += pos_error * dt
        self.integral_pos_error[0] = float(np.clip(self.integral_pos_error[0], -1.0, 1.0))
        self.integral_pos_error[1] = float(np.clip(self.integral_pos_error[1], -1.0, 1.0))
        self.integral_pos_error[2] = float(np.clip(self.integral_pos_error[2], -2.0, 2.0))

        Kp = np.diag([2.5, 2.5, 8.0])
        if xy_lock_active:
            Kp[0, 0] *= 0.8
            Kp[1, 1] *= 0.8
        Dp = np.diag([1.8, 1.8, 4.0])
        Ki = np.array([0.0, 0.0, 3.0], dtype=float)
        acc_des = acc_ff_ned + Kp @ pos_error + Dp @ vel_error + Ki * self.integral_pos_error
        if xy_lock_active:
            max_acc_xy = self.xy_lock_max_acc_xy
        elif auto_attitude_active:
            max_acc_xy = self.attitude_test_max_acc_xy
        else:
            max_acc_xy = self.max_acc_xy
        acc_des[0] = float(np.clip(acc_des[0], -max_acc_xy, max_acc_xy))
        acc_des[1] = float(np.clip(acc_des[1], -max_acc_xy, max_acc_xy))
        acc_des[2] = float(np.clip(acc_des[2], -self.max_acc_z, self.max_acc_z))

        f_world = self.mass * (acc_des - np.array([0.0, 0.0, self.gravity], dtype=float))
        f_body = self.R_ned_frd.T @ f_world
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
        else:
            # Full-circle primary tilt has no forward/backward cone boundary.
            # Only the secondary tilt limits the lateral force component.
            xz_force = float(np.linalg.norm(f_body[[0, 2]]))
            max_y = xz_force * math.tan(self.theta_limit_rad)
            f_body[1] = float(np.clip(f_body[1], -max_y, max_y))

        manual_yaw_active = abs(float(self._last_manual_cmd.get('yaw_rate', 0.0))) > 1e-5

        R_des = (
            self.target_R_des_ned_frd
            if self.target_R_des_ned_frd is not None
            else self._direct_desired_attitude_ned_frd(self.target_attitude)
        )
        e_rm = 0.5 * (R_des.T @ self.R_ned_frd - self.R_ned_frd.T @ R_des)
        e_R = np.array([e_rm[2, 1], e_rm[0, 2], e_rm[1, 0]], dtype=float)
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
            KI = self.direct_takeoff_KI
            Domega = self.direct_takeoff_Domega
            tau_limit = self.direct_takeoff_tau_limit
        elif xy_lock_active:
            KR = self.direct_xy_lock_KR
            KI = self.direct_xy_lock_KI
            Domega = self.direct_xy_lock_Domega
            tau_limit = self.direct_xy_lock_tau_limit
        else:
            # Do not step from takeoff gains to full gains when XY lock ends.
            # The identified joints still carry delayed commands at this point.
            blend = float(np.clip(
                (takeoff_elapsed_s - self.takeoff_xy_lock_time_s)
                / max(self.direct_gain_ramp_time_s, 1e-3),
                0.0,
                1.0,
            ))
            blend = blend * blend * (3.0 - 2.0 * blend)
            KR = (1.0 - blend) * self.direct_xy_lock_KR + blend * self.direct_KR
            KI = (1.0 - blend) * self.direct_xy_lock_KI + blend * self.direct_KI
            Domega = (1.0 - blend) * self.direct_xy_lock_Domega + blend * self.direct_Domega
            tau_limit = (
                (1.0 - blend) * self.direct_xy_lock_tau_limit
                + blend * self.direct_tau_limit
            )

        integral_before = self.integral_e_R.copy()
        integral_candidate = np.clip(
            integral_before + e_R * dt,
            -self.direct_integral_limit,
            self.direct_integral_limit,
        )
        tau_unsaturated = -KR * e_R - KI * integral_candidate - Domega * omega_error
        # Freeze an integrator only when its update would drive an already
        # saturated axis farther into saturation. This keeps bias rejection
        # without storing a large release transient during takeoff.
        for axis in range(3):
            integral_torque_delta = -KI[axis] * (integral_candidate[axis] - integral_before[axis])
            if (
                abs(tau_unsaturated[axis]) > tau_limit[axis]
                and tau_unsaturated[axis] * integral_torque_delta > 0.0
            ):
                integral_candidate[axis] = integral_before[axis]
        self.integral_e_R = integral_candidate
        tau_c = -KR * e_R - KI * self.integral_e_R - Domega * omega_error
        if not self.direct_yaw_control_enabled:
            tau_c[2] = 0.0
        tau_c = np.clip(tau_c, -tau_limit, tau_limit)

        max_thrust_per_arm = 85.48 * 2.0
        max_tail_thrust = 85.48
        r_x = 0.105
        r_z = -0.013

        W = self._allocator_wrench_from_body_force_torque(f_body, tau_c)
        if tilt_suppress_active or vertical_only_active:
            W[0] = 0.0
            W[1] = 0.0
        if takeoff_elapsed_s < 12.0:
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
        ty_comp = W[4] - ty_parasitic
        F3 = ty_comp / (r_x + self.l2)
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

        F1 = float(np.clip(F1, 0.0, 50.0))
        F2 = float(np.clip(F2, 0.0, 50.0))
        F3 = float(np.clip(F3, -50.0 if self.allow_tail_reverse else 0.0, 50.0))
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

        dt_slew = float(np.clip(dt, 0.0, 0.2))
        self._alpha1_cmd = self._slew_limit(self._alpha1_cmd, alpha1, self.servo_rate_limit_rad_s, dt_slew)
        self._alpha2_cmd = self._slew_limit(self._alpha2_cmd, alpha2, self.servo_rate_limit_rad_s, dt_slew)
        self._theta1_cmd = self._slew_limit(self._theta1_cmd, theta1, self.servo_rate_limit_rad_s, dt_slew)
        self._theta2_cmd = self._slew_limit(self._theta2_cmd, theta2, self.servo_rate_limit_rad_s, dt_slew)

        right_single = 0.5 * F2
        left_single = 0.5 * F1
        motor_controls = [
            self._thrust_to_normalized_motor_control(right_single),
            self._thrust_to_normalized_motor_control(right_single),
            self._thrust_to_normalized_motor_control(left_single),
            self._thrust_to_normalized_motor_control(left_single),
            (
                self._thrust_to_normalized_bidirectional_motor_control(F3)
                if self.allow_tail_reverse
                else self._thrust_to_normalized_motor_control(F3)
            ),
        ]

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
        ]
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
        ]
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
            f"Tune: step={np.round(np.degrees(self.attitude_step_axis_rad), 1).tolist()}deg | KR={np.round(self.direct_KR, 2).tolist()} | "
            f"D={np.round(self.direct_Domega, 2).tolist()} | tau_lim={np.round(self.direct_tau_limit, 2).tolist()}\n"
            f"Keyboard trajectory: active={self.auto_traj_mode} | pending={self.pending_auto_traj_mode}\n"
            f"Gamepad: vx_b={self._last_manual_cmd['vx_b']:+4.2f}, vy_b={self._last_manual_cmd['vy_b']:+4.2f}, "
            f"vz={self._last_manual_cmd['vz']:+4.2f}, yaw_rate={self._last_manual_cmd['yaw_rate']:+4.2f}, "
            f"LT={self._last_manual_cmd.get('lt', 0.0):4.2f}, RT={self._last_manual_cmd.get('rt', 0.0):4.2f}\n"
            f"Manual pitch: des={np.degrees(self.manual_des_pitch):+5.1f}° | "
            f"pitch_rate={np.degrees(self._last_manual_cmd.get('pitch_rate', 0.0)):+5.1f}°/s\n"
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


def main(args=None):
    rclpy.init(args=args)
    controller = HnuterController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('接收到终止信号，退出节点。绘图已关闭。')
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
