#!/usr/bin/env python3
"""Autonomous Hnuter narrow-passage mission for PX4 Gazebo SITL.

Run after starting the narrow world and DDS agent:
  source ~/PX4-Autopilot-Hnuter/px4-venv/bin/activate
  python3 ~/px4_ws_ros2/hnuter_narrow_passage_controller.py
"""

import math
import os
import time
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleControlMode,
    VehicleLocalPosition,
    VehicleStatus,
)


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
    return raw.strip().lower() in ('1', 'true', 'yes', 'on')


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_lerp(start: float, end: float, ratio: float) -> float:
    return wrap_pi(start + wrap_pi(end - start) * ratio)


def gazebo_yaw_to_px4_ned(yaw_enu_rad: float) -> float:
    return wrap_pi(0.5 * math.pi - yaw_enu_rad)


def ned_to_gazebo_xyz(ned: np.ndarray) -> np.ndarray:
    return np.array([float(ned[1]), float(ned[0]), float(-ned[2])], dtype=float)


@dataclass
class Waypoint:
    name: str
    gazebo_x_m: float
    gazebo_y_m: float
    altitude_m: float
    yaw_enu_deg: float
    speed_mps: float = 0.9
    hold_s: float = 0.0

    @property
    def ned(self) -> np.ndarray:
        # Gazebo uses ENU: x=east, y=north, z=up. PX4 setpoints use NED:
        # x=north, y=east, z=down.
        return np.array([self.gazebo_y_m, self.gazebo_x_m, -self.altitude_m], dtype=float)

    @property
    def yaw(self) -> float:
        return gazebo_yaw_to_px4_ned(math.radians(self.yaw_enu_deg))


class HnuterNarrowPassageController(Node):
    def __init__(self) -> None:
        super().__init__('hnuter_narrow_passage_controller')

        qos_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_cmd
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_cmd
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_cmd
        )

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_position_cb, qos_in
        )
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_cb, qos_in
        )
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, qos_in
        )
        self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.command_ack_cb, qos_in
        )
        self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self.control_mode_cb, qos_in
        )

        self.cmd_do_set_mode = getattr(VehicleCommand, 'VEHICLE_CMD_DO_SET_MODE', 176)
        self.cmd_arm_disarm = getattr(VehicleCommand, 'VEHICLE_CMD_COMPONENT_ARM_DISARM', 400)
        self.cmd_nav_land = getattr(VehicleCommand, 'VEHICLE_CMD_NAV_LAND', 21)
        self.nav_offboard = getattr(VehicleStatus, 'NAVIGATION_STATE_OFFBOARD', 14)
        self.arming_armed = getattr(VehicleStatus, 'ARMING_STATE_ARMED', 2)

        self.position = np.zeros(3, dtype=float)
        self.velocity = np.zeros(3, dtype=float)
        self.heading = 0.0
        self.px4_timestamp = 0
        self.data_received = False
        self.nav_state = -1
        self.arming_state = -1
        self.armed_from_ack = False
        self.offboard_from_control_mode = False
        self.offboard_from_ack = False

        self.home_ned: Optional[np.ndarray] = None
        self.state = 'warmup'
        self.state_start = time.monotonic()
        self.last_mode_request = 0.0
        self.last_arm_request = 0.0
        self.last_log = 0.0
        self.offboard_ticks = 0
        self.segment_index = 0
        self.segment_start_time = 0.0
        self.segment_duration = 1.0
        self.segment_start = np.zeros(3, dtype=float)
        self.segment_end = np.zeros(3, dtype=float)
        self.segment_start_yaw = 0.0
        self.segment_end_yaw = 0.0
        self.hold_until = 0.0
        self.trajectory_start_time = 0.0
        self.trajectory_last_segment = -1
        self.done_time: Optional[float] = None
        self.stop_requested = False

        self.takeoff_altitude = env_float('HNUTER_NARROW_ALTITUDE_M', 1.05)
        self.speed_scale = env_float('HNUTER_NARROW_SPEED_SCALE', 1.0)
        self.yaw_enu_offset_rad = math.radians(env_float('HNUTER_NARROW_YAW_ENU_OFFSET_DEG', 0.0))
        self.use_feedforward = env_bool('HNUTER_NARROW_FEEDFORWARD', True)
        self.auto_land = env_bool('HNUTER_NARROW_AUTO_LAND', True)
        self.mission = self._build_mission()
        (
            self.trajectory_velocities,
            self.trajectory_durations,
            self.trajectory_times,
        ) = self._build_continuous_trajectory()
        self.current_sp = self.mission[0].ned.copy()
        self.current_vel = np.zeros(3, dtype=float)
        self.current_acc = np.zeros(3, dtype=float)
        self.current_yaw = self._waypoint_yaw(self.mission[0])
        self.current_yawspeed = 0.0

        self.timer = self.create_timer(0.05, self.timer_cb)
        self.get_logger().info(
            'Hnuter narrow-passage controller ready. '
            'Use PX4_GZ_WORLD=hnuter_narrow HEADLESS=1 make px4_sitl gz_hnuter, '
            'then start MicroXRCEAgent udp4 -p 8888. '
            f'yaw_enu_offset={math.degrees(self.yaw_enu_offset_rad):.1f}deg '
            f'feedforward={self.use_feedforward}'
        )

    def _build_mission(self) -> List[Waypoint]:
        h = self.takeoff_altitude
        low_gate_h = min(h - 0.20, 0.85)
        mid_gate_h = min(h, 1.05)
        raised_gate_h = max(h + 0.25, 1.30)
        return [
            Waypoint('takeoff_pad', 0.0, 0.0, h, 0.0, 0.55),
            Waypoint('gate_01_approach', 3.5, 0.0, mid_gate_h, 0.0, 0.85),
            Waypoint('gate_01_center', 6.0, 0.0, mid_gate_h, 0.0, 0.68),
            Waypoint('gate_01_clear', 7.3, 0.0, mid_gate_h, 0.0, 0.72),

            Waypoint('gate_02_turn', 8.5, 1.1, 0.95, 42.0, 0.88),
            Waypoint('gate_02_align', 9.6, 2.2, low_gate_h, 0.0, 0.65),
            Waypoint('gate_02_center', 11.0, 2.2, low_gate_h, 0.0, 0.62),
            Waypoint('gate_02_clear', 12.3, 2.2, low_gate_h, 0.0, 0.68),

            Waypoint('gate_03_turn', 13.6, 0.0, 1.10, -60.0, 0.90),
            Waypoint('gate_03_align', 15.1, -2.2, raised_gate_h, 0.0, 0.65),
            Waypoint('gate_03_center', 16.5, -2.2, raised_gate_h, 0.0, 0.62),
            Waypoint('gate_03_clear', 17.8, -2.2, raised_gate_h, 0.0, 0.68),

            Waypoint('gate_04_turn', 19.1, 0.1, 1.05, 60.0, 0.90),
            Waypoint('gate_04_align', 20.6, 2.4, low_gate_h, 0.0, 0.65),
            Waypoint('gate_04_center', 22.0, 2.4, low_gate_h, 0.0, 0.60),
            Waypoint('gate_04_clear', 23.3, 2.4, low_gate_h, 0.0, 0.68),

            Waypoint('gate_05_turn', 24.6, 0.0, 1.10, -62.0, 0.90),
            Waypoint('gate_05_align', 26.1, -2.4, raised_gate_h, 0.0, 0.65),
            Waypoint('gate_05_center', 27.5, -2.4, raised_gate_h, 0.0, 0.60),
            Waypoint('gate_05_clear', 28.8, -2.4, raised_gate_h, 0.0, 0.68),

            Waypoint('gate_06_turn', 30.1, -0.6, 1.15, 50.0, 0.88),
            Waypoint('gate_06_align', 31.6, 1.2, mid_gate_h, 0.0, 0.65),
            Waypoint('gate_06_center', 33.0, 1.2, mid_gate_h, 0.0, 0.60),
            Waypoint('gate_06_clear', 34.3, 1.2, mid_gate_h, 0.0, 0.68),
            Waypoint('exit_clear', 37.0, 0.0, h, -24.0, 0.82),
        ]

    def _waypoint_yaw(self, wp: Waypoint) -> float:
        yaw_enu = math.radians(wp.yaw_enu_deg) + self.yaw_enu_offset_rad
        return gazebo_yaw_to_px4_ned(yaw_enu)

    def _build_continuous_trajectory(self):
        count = len(self.mission)
        velocities = np.zeros((count, 3), dtype=float)

        for index in range(1, count - 1):
            previous = self.mission[index - 1]
            waypoint = self.mission[index]
            following = self.mission[index + 1]
            speed = waypoint.speed_mps * self.speed_scale
            yaw_enu = math.radians(waypoint.yaw_enu_deg)

            horizontal_span = math.hypot(
                following.gazebo_x_m - previous.gazebo_x_m,
                following.gazebo_y_m - previous.gazebo_y_m,
            )
            vertical_slope = 0.0
            if horizontal_span > 0.1:
                vertical_slope = (
                    following.altitude_m - previous.altitude_m
                ) / horizontal_span

            velocity_enu = np.array(
                [
                    speed * math.cos(yaw_enu),
                    speed * math.sin(yaw_enu),
                    float(np.clip(speed * vertical_slope, -0.25, 0.25)),
                ],
                dtype=float,
            )
            velocities[index] = np.array(
                [velocity_enu[1], velocity_enu[0], -velocity_enu[2]],
                dtype=float,
            )

        durations = np.zeros(count - 1, dtype=float)
        for index in range(count - 1):
            distance = float(np.linalg.norm(self.mission[index + 1].ned - self.mission[index].ned))
            average_speed = 0.5 * (
                self.mission[index].speed_mps + self.mission[index + 1].speed_mps
            ) * self.speed_scale
            durations[index] = max(0.9, distance / max(0.35, average_speed))

        times = np.concatenate(([0.0], np.cumsum(durations)))
        return velocities, durations, times

    def _update_yaw_from_trajectory(self) -> None:
        velocity_east = float(self.current_vel[1])
        velocity_north = float(self.current_vel[0])
        horizontal_speed_sq = velocity_east * velocity_east + velocity_north * velocity_north
        if horizontal_speed_sq < 0.01:
            return

        yaw_enu = math.atan2(velocity_north, velocity_east) + self.yaw_enu_offset_rad
        self.current_yaw = gazebo_yaw_to_px4_ned(yaw_enu)

        acceleration_east = float(self.current_acc[1])
        acceleration_north = float(self.current_acc[0])
        yaw_rate_enu = (
            velocity_east * acceleration_north
            - velocity_north * acceleration_east
        ) / horizontal_speed_sq
        self.current_yawspeed = float(np.clip(-yaw_rate_enu, -1.0, 1.0))

    def local_position_cb(self, msg: VehicleLocalPosition) -> None:
        self.px4_timestamp = int(getattr(msg, 'timestamp', 0))
        self.position = np.array([float(msg.x), float(msg.y), float(msg.z)], dtype=float)
        self.velocity = np.array([float(msg.vx), float(msg.vy), float(msg.vz)], dtype=float)
        self.heading = float(getattr(msg, 'heading', self.heading))
        self.data_received = bool(getattr(msg, 'xy_valid', True)) and bool(getattr(msg, 'z_valid', True))
        if self.data_received and self.home_ned is None:
            self.home_ned = self.position.copy()
            self.home_ned[2] = 0.0
            self.segment_start = self.home_ned + self.mission[0].ned
            self.segment_end = self.segment_start.copy()
            self.current_sp = self.mission[0].ned.copy()
            self.current_yaw = self.heading
            self.get_logger().info(
                f'Home set at NED [{self.home_ned[0]:.2f}, {self.home_ned[1]:.2f}, {self.home_ned[2]:.2f}]'
            )

    def status_cb(self, msg: VehicleStatus) -> None:
        self.nav_state = int(getattr(msg, 'nav_state', -1))
        self.arming_state = int(getattr(msg, 'arming_state', -1))

    def command_ack_cb(self, msg: VehicleCommandAck) -> None:
        command = int(msg.command)
        result = int(msg.result)
        accepted = result == getattr(VehicleCommandAck, 'VEHICLE_CMD_RESULT_ACCEPTED', 0)
        if command == self.cmd_do_set_mode:
            self.offboard_from_ack = accepted
            self.get_logger().info(f'DO_SET_MODE ack result={result}')
        elif command == self.cmd_arm_disarm:
            if accepted:
                self.armed_from_ack = True
            self.get_logger().info(f'ARM_DISARM ack result={result}')

    def control_mode_cb(self, msg: VehicleControlMode) -> None:
        self.offboard_from_control_mode = bool(getattr(msg, 'flag_control_offboard_enabled', False))

    def timestamp_us(self) -> int:
        if self.px4_timestamp > 0:
            return int(self.px4_timestamp)
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_vehicle_command(self, command: int, **params: float) -> None:
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.command = int(command)
        msg.param1 = float(params.get('param1', 0.0))
        msg.param2 = float(params.get('param2', 0.0))
        msg.param3 = float(params.get('param3', 0.0))
        msg.param4 = float(params.get('param4', 0.0))
        msg.param5 = float(params.get('param5', 0.0))
        msg.param6 = float(params.get('param6', 0.0))
        msg.param7 = float(params.get('param7', 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def publish_offboard_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        if hasattr(msg, 'thrust_and_torque'):
            msg.thrust_and_torque = False
        if hasattr(msg, 'direct_actuator'):
            msg.direct_actuator = False
        self.offboard_pub.publish(msg)

    def publish_setpoint(self) -> None:
        if self.home_ned is None:
            return
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_us()
        sp_abs = self.home_ned + self.current_sp
        msg.position = [float(sp_abs[0]), float(sp_abs[1]), float(sp_abs[2])]
        if self.use_feedforward:
            msg.velocity = [float(self.current_vel[0]), float(self.current_vel[1]), float(self.current_vel[2])]
            msg.acceleration = [float(self.current_acc[0]), float(self.current_acc[1]), float(self.current_acc[2])]
        else:
            msg.velocity = [float('nan'), float('nan'), float('nan')]
            msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float(self.current_yaw)
        msg.yawspeed = float(self.current_yawspeed)
        self.setpoint_pub.publish(msg)

    def is_offboard(self) -> bool:
        return self.offboard_from_control_mode or self.offboard_from_ack or self.nav_state == self.nav_offboard

    def is_armed(self) -> bool:
        return self.arming_state == self.arming_armed or self.armed_from_ack

    def switch_offboard_and_arm(self) -> None:
        now = time.monotonic()
        if self.offboard_ticks < 25:
            return
        if not self.is_offboard() and now - self.last_mode_request > 1.0:
            self.publish_vehicle_command(self.cmd_do_set_mode, param1=1.0, param2=6.0)
            self.last_mode_request = now
            self.get_logger().info('Requesting Offboard mode')
        if self.is_offboard() and not self.is_armed() and now - self.last_arm_request > 1.0:
            self.publish_vehicle_command(self.cmd_arm_disarm, param1=1.0)
            self.last_arm_request = now
            self.get_logger().info('Requesting Arm')

    def start_mission_segment(self, index: int, start_from_current: bool = False) -> None:
        self.segment_index = index
        now = time.monotonic()
        self.segment_start_time = now
        previous = self.mission[max(0, index - 1)]
        target = self.mission[index]
        if start_from_current:
            if self.home_ned is None:
                self.segment_start = previous.ned.copy()
            else:
                self.segment_start = self.position - self.home_ned
            self.segment_start_yaw = self.heading
        else:
            self.segment_start = previous.ned.copy()
            self.segment_start_yaw = self._waypoint_yaw(previous)
        self.segment_end = target.ned.copy()
        self.segment_end_yaw = self._waypoint_yaw(target)
        distance = float(np.linalg.norm(self.segment_end - self.segment_start))
        speed = max(0.2, target.speed_mps * self.speed_scale)
        self.segment_duration = max(2.0, distance / speed)
        self.hold_until = 0.0
        self.get_logger().info(
            f'Segment {index}/{len(self.mission) - 1}: {target.name}, '
            f'gazebo_target=[{target.gazebo_x_m:.1f}, {target.gazebo_y_m:.1f}, {target.altitude_m:.1f}], '
            f'distance={distance:.1f}m duration={self.segment_duration:.1f}s'
        )

    def update_segment_setpoint(self) -> bool:
        now = time.monotonic()
        wp = self.mission[self.segment_index]
        if self.hold_until > 0.0:
            self.current_sp = self.segment_end.copy()
            self.current_vel[:] = 0.0
            self.current_acc[:] = 0.0
            self.current_yaw = self.segment_end_yaw
            self.current_yawspeed = 0.0
            return now >= self.hold_until

        t = max(0.0, now - self.segment_start_time)
        u = min(1.0, t / self.segment_duration)
        s = 3.0 * u * u - 2.0 * u * u * u
        ds_dt = 6.0 * u * (1.0 - u) / self.segment_duration
        d2s_dt2 = 6.0 * (1.0 - 2.0 * u) / (self.segment_duration * self.segment_duration)
        delta = self.segment_end - self.segment_start

        self.current_sp = self.segment_start + delta * s
        self.current_vel = delta * ds_dt
        self.current_acc = delta * d2s_dt2
        yaw_delta = wrap_pi(self.segment_end_yaw - self.segment_start_yaw)
        self.current_yaw = wrap_pi(self.segment_start_yaw + yaw_delta * s)
        self.current_yawspeed = yaw_delta * ds_dt

        if u < 1.0:
            return False
        if wp.hold_s > 0.0:
            self.hold_until = now + wp.hold_s
            return False
        return True

    def start_continuous_mission(self) -> None:
        self.state = 'mission'
        self.state_start = time.monotonic()
        self.trajectory_start_time = self.state_start
        self.trajectory_last_segment = -1
        self.segment_index = 1
        self.current_sp = self.mission[0].ned.copy()
        self.current_vel[:] = 0.0
        self.current_acc[:] = 0.0
        self.current_yaw = self._waypoint_yaw(self.mission[0])
        self.current_yawspeed = 0.0
        self.get_logger().info(
            f'Starting continuous trajectory: duration={self.trajectory_times[-1]:.1f}s, '
            f'points={len(self.mission)}'
        )

    def update_continuous_trajectory(self) -> bool:
        elapsed = max(0.0, time.monotonic() - self.trajectory_start_time)
        if elapsed >= self.trajectory_times[-1]:
            self.segment_index = len(self.mission) - 1
            self.current_sp = self.mission[-1].ned.copy()
            self.current_vel[:] = 0.0
            self.current_acc[:] = 0.0
            self.current_yaw = self._waypoint_yaw(self.mission[-1])
            self.current_yawspeed = 0.0
            return True

        segment = int(np.searchsorted(self.trajectory_times, elapsed, side='right') - 1)
        segment = max(0, min(segment, len(self.trajectory_durations) - 1))
        self.segment_index = segment + 1
        duration = float(self.trajectory_durations[segment])
        u = float(np.clip(
            (elapsed - self.trajectory_times[segment]) / duration,
            0.0,
            1.0,
        ))

        p0 = self.mission[segment].ned
        p1 = self.mission[segment + 1].ned
        v0 = self.trajectory_velocities[segment]
        v1 = self.trajectory_velocities[segment + 1]
        u2 = u * u
        u3 = u2 * u

        h00 = 2.0 * u3 - 3.0 * u2 + 1.0
        h10 = u3 - 2.0 * u2 + u
        h01 = -2.0 * u3 + 3.0 * u2
        h11 = u3 - u2
        self.current_sp = h00 * p0 + h10 * duration * v0 + h01 * p1 + h11 * duration * v1

        dh00 = 6.0 * u2 - 6.0 * u
        dh10 = 3.0 * u2 - 4.0 * u + 1.0
        dh01 = -6.0 * u2 + 6.0 * u
        dh11 = 3.0 * u2 - 2.0 * u
        self.current_vel = (
            dh00 * p0 + dh10 * duration * v0 + dh01 * p1 + dh11 * duration * v1
        ) / duration

        d2h00 = 12.0 * u - 6.0
        d2h10 = 6.0 * u - 4.0
        d2h01 = -12.0 * u + 6.0
        d2h11 = 6.0 * u - 2.0
        self.current_acc = (
            d2h00 * p0 + d2h10 * duration * v0 + d2h01 * p1 + d2h11 * duration * v1
        ) / (duration * duration)
        self._update_yaw_from_trajectory()

        if segment != self.trajectory_last_segment:
            target = self.mission[segment + 1]
            self.trajectory_last_segment = segment
            self.get_logger().info(
                f'Curve {segment + 1}/{len(self.mission) - 1}: {target.name}, '
                f'gazebo_target=[{target.gazebo_x_m:.1f}, {target.gazebo_y_m:.1f}, '
                f'{target.altitude_m:.1f}] duration={duration:.1f}s'
            )
        return False

    def update_mission(self) -> None:
        if self.state == 'warmup':
            self.current_sp = self.mission[0].ned.copy()
            self.current_vel[:] = 0.0
            self.current_acc[:] = 0.0
            self.current_yaw = self.heading if self.data_received else 0.0
            if self.data_received and self.home_ned is not None and self.is_offboard() and self.is_armed():
                self.state = 'takeoff'
                self.state_start = time.monotonic()
                self.start_mission_segment(0, start_from_current=True)
            return

        if self.state == 'takeoff':
            if self.update_segment_setpoint():
                rel_alt = -(self.position[2] - self.home_ned[2]) if self.home_ned is not None else 0.0
                if rel_alt > self.takeoff_altitude - 0.15:
                    self.start_continuous_mission()
            return

        if self.state == 'mission':
            if self.update_continuous_trajectory():
                self.state = 'hold'
                self.state_start = time.monotonic()
                self.get_logger().info('Narrow-passage mission complete; holding at exit point')
            return

        if self.state == 'hold':
            self.current_sp = self.mission[-1].ned.copy()
            self.current_vel[:] = 0.0
            self.current_acc[:] = 0.0
            self.current_yaw = self._waypoint_yaw(self.mission[-1])
            self.current_yawspeed = 0.0
            if self.auto_land and time.monotonic() - self.state_start > 3.0:
                self.publish_vehicle_command(self.cmd_nav_land)
                self.state = 'land'
                self.state_start = time.monotonic()
                self.get_logger().info('Landing command sent')
            return

        if self.state == 'land':
            if self.done_time is None:
                self.done_time = time.monotonic() + 8.0
            elif time.monotonic() > self.done_time:
                self.get_logger().info('Mission node finished')
                self.stop_requested = True

    def log_status(self) -> None:
        now = time.monotonic()
        if now - self.last_log < 1.0:
            return
        self.last_log = now
        if self.home_ned is None:
            self.get_logger().info('Waiting for local position...')
            return
        sp_abs = self.home_ned + self.current_sp
        err = sp_abs - self.position
        rel_alt = -(self.position[2] - self.home_ned[2])
        rel_ned = self.position - self.home_ned
        world_pos = ned_to_gazebo_xyz(rel_ned)
        world_sp = ned_to_gazebo_xyz(self.current_sp)
        self.get_logger().info(
            f'state={self.state} offboard={self.is_offboard()} armed={self.is_armed()} '
            f'wp={self.segment_index}/{len(self.mission) - 1} '
            f'world_pos=[{world_pos[0]:.2f},{world_pos[1]:.2f},{rel_alt:.2f}up] '
            f'world_sp=[{world_sp[0]:.2f},{world_sp[1]:.2f},{world_sp[2]:.2f}up] '
            f'err_norm={np.linalg.norm(err):.2f} '
            f'yaw_sp={math.degrees(self.current_yaw):.1f}deg '
            f'heading={math.degrees(self.heading):.1f}deg'
        )

    def timer_cb(self) -> None:
        if self.stop_requested:
            return
        if self.state != 'land':
            self.publish_offboard_heartbeat()
            self.switch_offboard_and_arm()
        self.update_mission()
        if self.state != 'land' and not self.stop_requested:
            self.publish_setpoint()
        self.log_status()
        self.offboard_ticks += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HnuterNarrowPassageController()
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
