#!/usr/bin/env python3
"""
Hnuter Tiltrotor PX4 External Controller with Gamepad + Keyboard Trajectories

核心修改：
1. 将 hnuter104.py 中的 GamepadManager 移植到 ROS2/PX4 外部控制节点。
2. 手柄输出为机体系速度 vx_b/vy_b、世界系垂直速度 vz、偏航角速度 yaw_rate。
3. LT/RT 触发器输出期望俯仰角速度，积分为目标 pitch 姿态。
4. 速度指令通过欧拉积分生成期望位置与期望偏航，送入原几何控制器与执行器分配。
4. 修复 Offboard/Arm 启动逻辑：先连续发布 OffboardControlMode，再切 Offboard，最后 Arm。
5. Offboard+Arm 后先零油门做倾转自检，按键盘 o 后才起飞悬停。
6. 避免 hover_only/xy_lock 永久覆盖手动目标点。
7. 键盘输入 1 执行一圈矩形轨迹，输入 2 执行一圈李萨如轨迹，
   输入 3 依次改变 roll/pitch/yaw 50 度后恢复，完成后回到悬停。
8. 关闭退出绘图与 /plot_data 发布。
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
from px4_msgs.msg import VehicleAngularVelocity
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import ActuatorServos
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

from hnuter_gamepad import GamepadManager as RobustGamepadManager


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
        super().__init__('hnuter_controller_gamepad')

        qos_profile_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

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

        self.actuator_motors_pub = self.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', qos_profile_in)
        self.actuator_servos_pub = self.create_publisher(
            ActuatorServos, '/fmu/in/actuator_servos', qos_profile_in)
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile_command)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile_command)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile_command)

        self.gz_servo0_pub = self.create_publisher(Float64, '/model/hnuter_0/servo_0', 10)
        self.gz_servo1_pub = self.create_publisher(Float64, '/model/hnuter_0/servo_1', 10)
        self.gz_servo2_pub = self.create_publisher(Float64, '/model/hnuter_0/servo_2', 10)
        self.gz_servo3_pub = self.create_publisher(Float64, '/model/hnuter_0/servo_3', 10)
        self.publish_gz_servos_direct = False
        self.plotting_enabled = False
        self.plot_pub = None

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
        self.attitude_q = np.array([1.0, 0.0, 0.0, 0.0])
        self.angular_velocity = np.zeros(3)  # FLU body angular velocity
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
        self.auto_arm_attempts = 0
        self.was_armed_once = False
        self._last_armed_state = False
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
        self.last_F1 = 0.0
        self.last_F2 = 0.0
        self.last_F3 = 0.0
        self.last_velocity_left = 0.0
        self.last_velocity_right = 0.0
        self.last_velocity_tail = 0.0
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
        self.tail_thrust_scale = 0.08
        self.tail_control_limit = 1.0

        # Actuator limits
        self.alpha_limit_rad = np.radians(60.0)
        self.theta_limit_rad = np.radians(45.0)
        self.servo_rate_limit_rad_s = 50.0
        self.takeoff_tilt_suppress_time_s = 0.2
        self.takeoff_tilt_limit_rad = np.radians(20.0)
        self.disable_tail_at_takeoff = False
        self.takeoff_xy_lock_time_s = 3.0
        self.xy_lock_max_acc_xy = 3.0
        self.xy_lock_tilt_limit_rad = np.radians(30.0)
        self.xy_lock_kp_scale = 0.8
        self._xy_lock_initialized = False
        self._xy_lock_position = np.zeros(2)
        self._xy_lock_active = False
        self._takeoff_lock_start_time_s = None

        # 不再永久 hover_only，否则会覆盖手柄 XY 目标点
        self.hover_only = False

        # Yaw variables
        self._yaw_initialized = False
        self.initial_yaw = 0.0

        self._alpha1_cmd = 0.0
        self._alpha2_cmd = 0.0
        self._theta1_cmd = 0.0
        self._theta2_cmd = 0.0

        # Position loop
        self.Kp = np.diag([2.5, 2.5, 8.0])
        self.Dp = np.diag([1.8, 1.8, 4.0])
        self.K_pos_I = np.array([0.0, 0.0, 3.0])
        self.integral_pos_error = np.zeros(3)

        # Attitude loop
        self.KR = np.array([1.5, 1.5, 1.5])
        self.Domega = np.array([1.2, 1.2, 1.2])
        self.KI = np.array([0.0, 0.0, 0.0])
        self.integral_e_R = np.zeros(3)

        self.max_acc_xy = 20.0
        self.max_acc_z = 20.0
        self.max_climb_rate = 1.0
        # px4_position 版本全程使用 PX4 position Offboard，避免起飞前后在
        # direct_actuator/position 两套 Offboard 控制入口之间切换。
        self.use_px4_position_takeoff = True
        self.safe_hover_enabled = False
        self.safe_hover_tail_fraction = 0.16
        self.safe_hover_ramp_time_s = 3.0
        self.safe_hover_min_thrust_scale = 0.65
        self.safe_hover_max_thrust_scale = 1.25
        self.safe_hover_kp_z = 2.0
        self.safe_hover_kd_z = 1.3
        self.safe_hover_ki_z = 0.25
        self.safe_hover_integral_z = 0.0
        self.safe_hover_kR = np.array([0.55, 0.55, 0.15])
        self.safe_hover_domega = np.array([0.28, 0.28, 0.10])
        self.safe_hover_tau_limit = np.array([0.35, 0.35, 0.08])
        self.safe_hover_full_attitude_height = 0.25

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
        # LT/RT 积分得到的俯仰姿态期望。
        # 正号严格按 euler_to_rotation_matrix() 的 pitch 正方向；若实机观察方向相反，
        # 只需要在 GamepadManager 中把 target_pitch_rate 改成 (rt_expo - lt_expo)。
        self.manual_des_pitch = 0.0
        self.manual_pitch_limit_rad = np.radians(90.0)
        self._z0_initialized = False
        self._z0 = 0.0
        self._z_sp = 0.0

        # 解锁进入 Offboard 后，默认先不产生升力，只做地面倾转自检。
        # 按键盘 o 后才允许进入起飞/悬停控制。
        self.preflight_tilt_test_enabled = True
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
        self.gamepad = RobustGamepadManager(
            max_vxy=1.0,
            max_vz=0.5,
            max_yaw_rate=0.6,
            max_pitch_rate=math.radians(20.0),
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

        # Logs
        self.log_time = []
        self.log_motors = {0: [], 1: [], 2: [], 3: [], 4: []}
        self.log_servos = {0: [], 1: [], 2: [], 3: []}
        self.log_attitude = {'roll': [], 'pitch': [], 'yaw': []}
        self.log_attitude_desired = {'roll': [], 'pitch': [], 'yaw': []}
        self.log_position = {'x': [], 'y': [], 'z': []}
        self.log_position_desired = {'x': [], 'y': [], 'z': []}
        self.log_start_time = None

        self.get_logger().info(
            'Hnuter PX4 External Controller initialized: Gamepad + keyboard trajectories + fixed Offboard/Arm state machine'
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
        self.attitude_q = np.array([w, x, y, z], dtype=float)
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

    def angular_velocity_callback(self, msg):
        # PX4 FRD -> FLU
        self.angular_velocity = np.array([msg.xyz[0], -msg.xyz[1], -msg.xyz[2]], dtype=float)

    def status_callback(self, msg):
        self.armed = (int(msg.arming_state) == self.ARMING_STATE_ARMED)
        self.nav_state = int(getattr(msg, 'nav_state', -1))

    def control_mode_callback(self, msg):
        self.control_offboard_enabled = bool(getattr(msg, 'flag_control_offboard_enabled', False))

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
            self.safe_hover_integral_z = 0.0
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
                self.preflight_disarm_waiting_for_o = False
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
                self._offboard_request_sent = True
                self.get_logger().info('请求切换到 Offboard 模式...')
            return

        # 5) 已进入 Offboard 后再 Arm；position Offboard 下等键盘 o 作为起飞/解锁许可。
        if self.is_offboard() and not self.armed:
            if self.use_px4_position_takeoff and not self.takeoff_requested:
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
                self._arm_request_sent = True
                self.get_logger().info(
                    f'请求 Arm 解锁... ({self.auto_arm_attempts}/{self.max_auto_arm_attempts})'
                )

        if self.armed:
            self.was_armed_once = True

        if self.is_offboard() and self.armed and not self.takeoff_requested:
            now_s = self.px4_timestamp / 1_000_000.0
            current_time = max(0.0, now_s - self.sim_start_time_s) if self.sim_start_time_s > 0.0 else 0.0
            if self.use_px4_position_takeoff:
                self.publish_px4_trajectory_setpoint()
            else:
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

    def publish_idle_direct_actuator_setpoint(self):
        self.publish_direct_actuator_setpoint(
            motor_controls=[0.0, 0.0, 0.0, 0.0, 0.0],
            alpha1=0.0,
            alpha2=0.0,
            theta1=0.0,
            theta2=0.0
        )

    def publish_direct_actuator_setpoint(self, motor_controls, alpha1, alpha2, theta1, theta2):
        timestamp = self.timestamp_now_us()

        motor_msg = ActuatorMotors()
        motor_msg.timestamp = timestamp
        if hasattr(motor_msg, 'timestamp_sample'):
            motor_msg.timestamp_sample = timestamp
        if hasattr(motor_msg, 'reversible_flags'):
            motor_msg.reversible_flags = 0
        motor_msg.control = [float('nan')] * 12
        for index, value in enumerate(motor_controls[:12]):
            motor_msg.control[index] = float(np.clip(value, 0.0, 1.0))
        self.last_motor_cmd = np.array(motor_msg.control)
        self.actuator_motors_pub.publish(motor_msg)

        servo_msg = ActuatorServos()
        servo_msg.timestamp = timestamp
        if hasattr(servo_msg, 'timestamp_sample'):
            servo_msg.timestamp_sample = timestamp
        servo_msg.control = [float('nan')] * 8
        angle_max = np.radians(90.0)
        servo_msg.control[0] = float(np.clip(alpha2 / angle_max, -1.0, 1.0))
        servo_msg.control[1] = float(np.clip(alpha1 / angle_max, -1.0, 1.0))
        servo_msg.control[2] = float(np.clip(theta2 / angle_max, -1.0, 1.0))
        servo_msg.control[3] = float(np.clip(theta1 / angle_max, -1.0, 1.0))
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
        self.last_velocity_left = 0.0
        self.last_velocity_right = 0.0
        self.last_velocity_tail = 0.0
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
            'pitch_rate': 0.0,
            'lt': 0.0,
            'rt': 0.0,
        }

    def poll_keyboard_commands(self):
        for key in self.keyboard.get_commands():
            if key in ('o', 'O'):
                if self.preflight_disarm_waiting_for_o:
                    self.startup_blocked_after_disarm = False
                    self.preflight_disarm_waiting_for_o = False
                    self.was_armed_once = False
                    self.auto_arm_attempts = 0
                    self._last_offboard_cmd_time = 0.0
                    self._last_arm_cmd_time = 0.0
                self.takeoff_requested = True
                self.manual_pos_initialized = False
                self._takeoff_lock_start_time_s = None
                self._xy_lock_active = False
                self._z0_initialized = False
                self._z0 = 0.0
                self.integral_pos_error[:] = 0.0
                self.integral_e_R[:] = 0.0
                self.safe_hover_integral_z = 0.0
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
        self.manual_des_pitch = 0.0
        self.integral_pos_error[:] = 0.0
        self.integral_e_R[:] = 0.0
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
        self.manual_des_pitch = 0.0
        self.target_position = self.manual_des_pos.copy()
        self.target_velocity = np.zeros(3)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([0.0, 0.0, self.manual_des_yaw], dtype=float)
        self.target_attitude_rate = np.zeros(3)
        self.integral_pos_error[:] = 0.0
        self.integral_e_R[:] = 0.0
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
        self.manual_des_pitch = 0.0
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

        # 未进入 offboard 或未解锁前，目标点贴住当前点，清积分，避免一解锁就猛冲
        if (not self.is_offboard()) or (not self.armed):
            self.integral_pos_error[:] = 0.0
            self.integral_e_R[:] = 0.0
            self.safe_hover_integral_z = 0.0
            self.manual_pos_initialized = False
            self.auto_traj_mode = 'hover'
            self._z_sp = 0.0
            self._takeoff_lock_start_time_s = None
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
            return

        if not self.takeoff_requested:
            self.integral_pos_error[:] = 0.0
            self.integral_e_R[:] = 0.0
            self.safe_hover_integral_z = 0.0
            self.manual_pos_initialized = False
            self.auto_traj_mode = 'hover'
            self._z0_initialized = False
            self._z0 = 0.0
            self._z_sp = 0.0
            self._takeoff_lock_start_time_s = None
            self._xy_lock_active = False
            self.manual_des_pitch = 0.0
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
            self.manual_des_pitch = 0.0
            self._z_sp = float(self.manual_des_pos[2])
            self._xy_lock_position = self.position[:2].copy()
            self._xy_lock_initialized = True
            self._takeoff_lock_start_time_s = current_time
            self.manual_pos_initialized = True
            self.safe_hover_integral_z = 0.0

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
        pitch_rate = cmds.get('pitch_rate', 0.0)

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
        self.manual_des_pitch = float(np.clip(
            self.manual_des_pitch + pitch_rate * dt,
            -self.manual_pitch_limit_rad,
            self.manual_pitch_limit_rad
        ))

        self.target_position = self.manual_des_pos.copy()
        self.target_velocity = np.array([vx_w, vy_w, vz_w], dtype=float)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([0.0, self.manual_des_pitch, self.manual_des_yaw], dtype=float)
        self.target_attitude_rate = np.array([0.0, pitch_rate, yaw_rate], dtype=float)

    # ============================================================
    # Geometry controller and allocation
    # ============================================================
    def euler_to_rotation_matrix(self, euler):
        roll, pitch, yaw = euler
        R_x = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
        R_y = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        return R_z @ R_y @ R_x

    @staticmethod
    def vee_map(S):
        return np.array([S[2, 1], S[0, 2], S[1, 0]])

    def compute_control_wrench(self, dt):
        pos_curr = self.position.copy()
        pos_curr[2] = float(pos_curr[2] - self._z0)
        pos_error = self.target_position - pos_curr
        vel_error = self.target_velocity - self.velocity

        self.integral_pos_error += pos_error * dt
        self.integral_pos_error[:2] = np.clip(self.integral_pos_error[:2], -1.0, 1.0)
        self.integral_pos_error[2] = float(np.clip(self.integral_pos_error[2], -2.0, 2.0))

        Kp = self.Kp * float(self.xy_lock_kp_scale) if self._xy_lock_active else self.Kp
        acc_des = self.target_acceleration + Kp @ pos_error + self.Dp @ vel_error + self.K_pos_I * self.integral_pos_error

        max_acc_xy = float(self.xy_lock_max_acc_xy) if self._xy_lock_active else float(self.max_acc_xy)
        acc_des[0] = float(np.clip(acc_des[0], -max_acc_xy, max_acc_xy))
        acc_des[1] = float(np.clip(acc_des[1], -max_acc_xy, max_acc_xy))
        acc_des[2] = float(np.clip(acc_des[2], -self.max_acc_z, self.max_acc_z))
        f_c_world = self.mass * (acc_des + np.array([0, 0, self.gravity]))

        R_des = self.euler_to_rotation_matrix(self.target_attitude)
        e_R = 0.5 * self.vee_map(R_des.T @ self.R - self.R.T @ R_des)
        self.integral_e_R += e_R * dt
        self.integral_e_R = np.clip(self.integral_e_R, -1.5, 1.5)

        omega_error = self.angular_velocity - self.R.T @ R_des @ self.target_attitude_rate
        tau_c = -self.KR * e_R - self.KI * self.integral_e_R - self.Domega * omega_error
        tau_c[2] = float(np.clip(tau_c[2], -0.5, 0.5))

        f_c_body = self.R.T @ f_c_world
        return f_c_body, tau_c, e_R, f_c_world

    def inverse_nonlinear_mapping(self, W):
        l1, l2 = self.l1, self.l2
        r_x = 0.105
        r_z = -0.013

        u1 = W[0] / 2.0 - W[5] / (2.0 * l1)
        u4 = W[0] / 2.0 + W[5] / (2.0 * l1)

        Ty_parasitic = r_z * W[0] - r_x * W[2]
        Ty_comp = W[4] - Ty_parasitic
        F3 = float(Ty_comp / (r_x + l2))
        Fz_front = float(W[2] - F3)

        Tx_parasitic = -r_z * W[1]
        Tx_comp = W[3] - Tx_parasitic
        u2 = Fz_front / 2.0 + Tx_comp / (2.0 * l1)
        u5 = Fz_front / 2.0 - Tx_comp / (2.0 * l1)
        u3 = -W[1] / 2.0
        u6 = -W[1] / 2.0

        F1 = np.sqrt(u1 ** 2 + u2 ** 2 + u3 ** 2)
        F2 = np.sqrt(u4 ** 2 + u5 ** 2 + u6 ** 2)
        eps = 1e-8
        F1_safe = max(F1, eps)
        F2_safe = max(F2, eps)

        alpha1 = np.arctan2(u1, u2)
        alpha2 = np.arctan2(u4, u5)
        theta1 = np.arcsin(np.clip(u3 / F1_safe, -0.99, 0.99))
        theta2 = np.arcsin(np.clip(u6 / F2_safe, -0.99, 0.99))

        F1 = np.clip(F1, 0.0, 50.0)
        F2 = np.clip(F2, 0.0, 50.0)
        F3 = np.clip(F3, 0.0, 50.0)
        alpha1 = np.clip(alpha1, -self.alpha_limit_rad, self.alpha_limit_rad)
        alpha2 = np.clip(alpha2, -self.alpha_limit_rad, self.alpha_limit_rad)
        theta1 = np.clip(theta1, -self.theta_limit_rad, self.theta_limit_rad)
        theta2 = np.clip(theta2, -self.theta_limit_rad, self.theta_limit_rad)
        return F1, F2, F3, alpha1, alpha2, theta1, theta2

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

        # 没进入 Offboard 时不向 actuator topics 写入外部 setpoint，避免和 PX4 内部控制器冲突
        if not self.is_offboard():
            return

        if not self.takeoff_requested:
            self.control_loop_count += 1
            self.last_F1 = 0.0
            self.last_F2 = 0.0
            self.last_F3 = 0.0
            self.last_W = np.zeros(6)
            self.last_motor_cmd = np.zeros(12)
            self.last_servo_cmd = np.zeros(8)
            if self.use_px4_position_takeoff:
                self.publish_px4_trajectory_setpoint()
            elif self.armed:
                self.publish_preflight_tilt_test_setpoint(current_time, dt)
            else:
                self.publish_idle_direct_actuator_setpoint()
            now = time.time()
            if now - self._last_debug_print_time >= self.debug_print_period_s:
                self.get_logger().info(
                    f'地面自检中：Offboard={self.is_offboard()} | Armed={self.armed} | '
                    'PX4 position setpoint 贴住当前位置，按 o 后 Arm 并起飞悬停'
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
            now = time.time()
            if now - self._last_debug_print_time >= self.debug_print_period_s:
                self.get_logger().info(
                    f'PX4 位置 Offboard 起飞/悬停 dt={dt * 1000:.1f}ms | '
                    f'z={self.position[2] - self._z0:+.2f}m -> {self.target_position[2]:.2f}m'
                )
                self._last_debug_print_time = now
            return

        if self.safe_hover_enabled:
            self.control_loop_count += 1
            self.publish_safe_hover_commands(current_time, dt)
            now = time.time()
            if now - self._last_debug_print_time >= self.debug_print_period_s:
                self.get_logger().info(
                    f'安全悬停控制 dt={dt * 1000:.1f}ms | Offboard={self.is_offboard()} | '
                    f'Armed={self.armed} | z={self.position[2] - self._z0:+.2f}m | '
                    f'tail_frac={self.safe_hover_tail_fraction:.2f}'
                )
                self._last_debug_print_time = now
            return

        if not self._xy_lock_initialized:
            self._xy_lock_position = self.position[:2].copy()
            self._xy_lock_initialized = True

        takeoff_elapsed_s = (
            current_time - self._takeoff_lock_start_time_s
            if self._takeoff_lock_start_time_s is not None else float('inf')
        )
        suppress_tilts = (takeoff_elapsed_s < self.takeoff_tilt_suppress_time_s)
        xy_lock = (takeoff_elapsed_s < self.takeoff_xy_lock_time_s)
        self._xy_lock_active = bool(xy_lock)

        if xy_lock:
            self.target_position[0] = float(self._xy_lock_position[0])
            self.target_position[1] = float(self._xy_lock_position[1])
            self.target_velocity[0] = 0.0
            self.target_velocity[1] = 0.0

        f_c_body, tau_c, e_R, f_c_world = self.compute_control_wrench(dt)
        W = np.array([f_c_body[0], f_c_body[1], f_c_body[2], tau_c[0], tau_c[1], tau_c[2]])

        if suppress_tilts:
            W[0] = 0.0
            W[1] = 0.0

        F1, F2, F3, alpha1, alpha2, theta1, theta2 = self.inverse_nonlinear_mapping(W)

        if suppress_tilts:
            alpha1 = np.clip(alpha1, -self.takeoff_tilt_limit_rad, self.takeoff_tilt_limit_rad)
            alpha2 = np.clip(alpha2, -self.takeoff_tilt_limit_rad, self.takeoff_tilt_limit_rad)
            theta1 = np.clip(theta1, -self.takeoff_tilt_limit_rad, self.takeoff_tilt_limit_rad)
            theta2 = np.clip(theta2, -self.takeoff_tilt_limit_rad, self.takeoff_tilt_limit_rad)
            if self.disable_tail_at_takeoff:
                F3 = 0.0

        if self._xy_lock_active:
            alpha1 = np.clip(alpha1, -self.xy_lock_tilt_limit_rad, self.xy_lock_tilt_limit_rad)
            alpha2 = np.clip(alpha2, -self.xy_lock_tilt_limit_rad, self.xy_lock_tilt_limit_rad)
            theta1 = np.clip(theta1, -self.xy_lock_tilt_limit_rad, self.xy_lock_tilt_limit_rad)
            theta2 = np.clip(theta2, -self.xy_lock_tilt_limit_rad, self.xy_lock_tilt_limit_rad)

        self.last_F1 = F1
        self.last_F2 = F2
        self.last_F3 = F3
        self.last_W = W
        self.control_loop_count += 1
        self.publish_actuator_commands(F1, F2, F3, alpha1, alpha2, theta1, theta2, dt)

        now = time.time()
        if now - self._last_debug_print_time >= self.debug_print_period_s:
            self.get_logger().info(f'控制 dt={dt * 1000:.1f}ms | Offboard={self.is_offboard()} | Armed={self.armed}')
            self._last_debug_print_time = now

    @staticmethod
    def _slew_limit(current, target, rate_limit, dt):
        delta = target - current
        max_delta = rate_limit * dt
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target

    def publish_safe_hover_commands(self, current_time: float, dt: float):
        pos_rel_z = self.position[2] - self._z0
        z_error = float(self.target_position[2] - pos_rel_z)
        z_vel_error = float(self.target_velocity[2] - self.velocity[2])
        self.safe_hover_integral_z += z_error * dt
        self.safe_hover_integral_z = float(np.clip(self.safe_hover_integral_z, -1.0, 1.0))

        acc_z = (
            self.safe_hover_kp_z * z_error
            + self.safe_hover_kd_z * z_vel_error
            + self.safe_hover_ki_z * self.safe_hover_integral_z
        )
        acc_z = float(np.clip(acc_z, -3.0, 3.0))
        total_thrust = self.mass * (self.gravity + acc_z)

        takeoff_elapsed_s = (
            current_time - self._takeoff_lock_start_time_s
            if self._takeoff_lock_start_time_s is not None else 0.0
        )
        ramp = float(np.clip(takeoff_elapsed_s / self.safe_hover_ramp_time_s, 0.0, 1.0))
        thrust_cap = self.mass * self.gravity * (
            self.safe_hover_min_thrust_scale
            + (self.safe_hover_max_thrust_scale - self.safe_hover_min_thrust_scale) * ramp
        )
        total_thrust = float(np.clip(total_thrust, 0.0, thrust_cap))

        R_des = self.euler_to_rotation_matrix(np.array([0.0, 0.0, self.manual_des_yaw], dtype=float))
        e_R = 0.5 * self.vee_map(R_des.T @ self.R - self.R.T @ R_des)
        tau_c = -self.safe_hover_kR * e_R - self.safe_hover_domega * self.angular_velocity

        # 地面附近只保留很小的阻尼，避免错误力矩在接地约束下把机体掀翻。
        attitude_blend = float(np.clip(pos_rel_z / self.safe_hover_full_attitude_height, 0.25, 1.0))
        tau_c[:2] *= attitude_blend
        tau_c = np.clip(tau_c, -self.safe_hover_tau_limit, self.safe_hover_tau_limit)

        F3 = float(self.safe_hover_tail_fraction * total_thrust + tau_c[1] / (self.l2 + 0.105))
        F3 = float(np.clip(F3, 0.0, 0.45 * total_thrust))
        front_thrust = max(total_thrust - F3, 0.0)
        F1 = float(front_thrust / 2.0 + tau_c[0] / (2.0 * self.l1))
        F2 = float(front_thrust / 2.0 - tau_c[0] / (2.0 * self.l1))
        F1 = float(np.clip(F1, 0.0, 50.0))
        F2 = float(np.clip(F2, 0.0, 50.0))

        W = np.array([0.0, 0.0, total_thrust, tau_c[0], tau_c[1], tau_c[2]], dtype=float)

        self.last_F1 = F1
        self.last_F2 = F2
        self.last_F3 = F3
        self.last_W = W
        self.publish_actuator_commands(F1, F2, F3, 0.0, 0.0, 0.0, 0.0, dt)

    def publish_actuator_commands(self, F1, F2, F3, alpha1, alpha2, theta1, theta2, dt):
        motor_constant = 8.54858e-05
        min_velocity = 10.0
        max_velocity = 1000.0

        T_single_left = F1 / 2.0
        T_single_right = F2 / 2.0
        velocity_left = np.sqrt(max(T_single_left, 0.0) / motor_constant)
        velocity_right = np.sqrt(max(T_single_right, 0.0) / motor_constant)
        velocity_tail = np.sqrt(max(F3, 0.0) / motor_constant)

        self.last_velocity_left = velocity_left
        self.last_velocity_right = velocity_right
        self.last_velocity_tail = velocity_tail

        normalized_left = (velocity_left - min_velocity) / (max_velocity - min_velocity)
        normalized_right = (velocity_right - min_velocity) / (max_velocity - min_velocity)
        normalized_tail = (velocity_tail - min_velocity) / (max_velocity - min_velocity)

        self._alpha1_cmd = self._slew_limit(self._alpha1_cmd, alpha1, self.servo_rate_limit_rad_s, dt)
        self._alpha2_cmd = self._slew_limit(self._alpha2_cmd, alpha2, self.servo_rate_limit_rad_s, dt)
        self._theta1_cmd = self._slew_limit(self._theta1_cmd, theta1, self.servo_rate_limit_rad_s, dt)
        self._theta2_cmd = self._slew_limit(self._theta2_cmd, theta2, self.servo_rate_limit_rad_s, dt)

        motor_msg = ActuatorMotors()
        motor_msg.timestamp = self.timestamp_now_us()
        if hasattr(motor_msg, 'timestamp_sample'):
            motor_msg.timestamp_sample = motor_msg.timestamp
        if hasattr(motor_msg, 'reversible_flags'):
            motor_msg.reversible_flags = 0
        motor_msg.control = [float('nan')] * 12
        motor_msg.control[0] = float(np.clip(normalized_right, 0.0, 1.0))
        motor_msg.control[1] = float(np.clip(normalized_right, 0.0, 1.0))
        motor_msg.control[2] = float(np.clip(normalized_left, 0.0, 1.0))
        motor_msg.control[3] = float(np.clip(normalized_left, 0.0, 1.0))
        motor_msg.control[4] = float(np.clip(normalized_tail, 0.0, self.tail_control_limit))
        self.last_motor_cmd = np.array(motor_msg.control)
        self.actuator_motors_pub.publish(motor_msg)

        servo_msg = ActuatorServos()
        servo_msg.timestamp = motor_msg.timestamp
        if hasattr(servo_msg, 'timestamp_sample'):
            servo_msg.timestamp_sample = motor_msg.timestamp
        servo_msg.control = [float('nan')] * 8
        angle_max = np.radians(90.0)
        servo_msg.control[0] = float(np.clip(self._alpha2_cmd / angle_max, -1.0, 1.0))
        servo_msg.control[1] = float(np.clip(self._alpha1_cmd / angle_max, -1.0, 1.0))
        servo_msg.control[2] = float(np.clip(self._theta2_cmd / angle_max, -1.0, 1.0))
        servo_msg.control[3] = float(np.clip(self._theta1_cmd / angle_max, -1.0, 1.0))
        self.last_servo_cmd = np.array(servo_msg.control)
        self.actuator_servos_pub.publish(servo_msg)

        if self.plotting_enabled:
            self._record_log(motor_msg, servo_msg)

    def _record_log(self, motor_msg, servo_msg):
        if not self.plotting_enabled:
            return

        if self.log_start_time is None:
            self.log_start_time = time.time()
        current_t = time.time() - self.log_start_time
        self.log_time.append(current_t)
        for i in range(5):
            self.log_motors[i].append(motor_msg.control[i])
        for i in range(4):
            self.log_servos[i].append(servo_msg.control[i])

        pos_curr_rel_z = self.position[2] - self._z0 if self._z0_initialized else self.position[2]
        self.log_position['x'].append(float(self.position[0]))
        self.log_position['y'].append(float(self.position[1]))
        self.log_position['z'].append(float(pos_curr_rel_z))
        self.log_position_desired['x'].append(float(self.target_position[0]))
        self.log_position_desired['y'].append(float(self.target_position[1]))
        self.log_position_desired['z'].append(float(self.target_position[2]))

        roll = np.arctan2(self.R[2, 1], self.R[2, 2])
        pitch = np.arcsin(np.clip(-self.R[2, 0], -1.0, 1.0))
        yaw = np.arctan2(self.R[1, 0], self.R[0, 0])
        self.log_attitude['roll'].append(np.degrees(roll))
        self.log_attitude['pitch'].append(np.degrees(pitch))
        self.log_attitude['yaw'].append(np.degrees(yaw))
        self.log_attitude_desired['roll'].append(np.degrees(self.target_attitude[0]))
        self.log_attitude_desired['pitch'].append(np.degrees(self.target_attitude[1]))
        self.log_attitude_desired['yaw'].append(np.degrees(self.target_attitude[2]))

    @staticmethod
    def _unwrap_degrees(values):
        return np.degrees(np.unwrap(np.radians(values))).tolist()

    # ============================================================
    # Status/plot/shutdown
    # ============================================================
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
            f"Takeoff gate: requested={self.takeoff_requested} | preflight_done={self.preflight_tilt_test_finished} | "
            f"waiting_rearm_o={self.preflight_disarm_waiting_for_o}\n"
            f"Target ENU/Zrel: [{self.target_position[0]:6.2f}, {self.target_position[1]:6.2f}, {self.target_position[2]:6.2f}] m\n"
            f"Current ENU/Zrel: [{self.position[0]:6.2f}, {self.position[1]:6.2f}, {pos_curr_rel_z:6.2f}] m\n"
            f"Keyboard trajectory: active={self.auto_traj_mode} | pending={self.pending_auto_traj_mode}\n"
            f"Gamepad: vx_b={self._last_manual_cmd['vx_b']:+4.2f}, vy_b={self._last_manual_cmd['vy_b']:+4.2f}, "
            f"vz={self._last_manual_cmd['vz']:+4.2f}, yaw_rate={self._last_manual_cmd['yaw_rate']:+4.2f}, "
            f"LT={self._last_manual_cmd.get('lt', 0.0):4.2f}, RT={self._last_manual_cmd.get('rt', 0.0):4.2f}\n"
            f"Pitch: des={np.degrees(self.manual_des_pitch):+5.1f}° | current={current_pitch_deg:+5.1f}° | "
            f"pitch_rate={np.degrees(self._last_manual_cmd.get('pitch_rate', 0.0)):+5.1f}°/s\n"
            f"Wrench: Fx={self.last_W[0]:+5.2f}N, Fy={self.last_W[1]:+5.2f}N, Fz={self.last_W[2]:+5.2f}N\n"
            f"Thrust: F1={self.last_F1:5.2f}N | F2={self.last_F2:5.2f}N | F3={self.last_F3:5.2f}N\n"
            f"Tilt: A1={np.degrees(self._alpha1_cmd):+5.1f}° | A2={np.degrees(self._alpha2_cmd):+5.1f}° | "
            f"T1={np.degrees(self._theta1_cmd):+5.1f}° | T2={np.degrees(self._theta2_cmd):+5.1f}°\n"
            f"{'=' * 72}"
        )

    def plot_results(self):
        self.get_logger().info('绘图已关闭：不生成、不保存、不弹出任何图表。')

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
        controller.get_logger().info('接收到终止信号，退出节点。绘图已关闭。')
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
