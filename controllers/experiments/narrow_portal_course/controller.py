#!/usr/bin/env python3
"""Direct-actuator controllers for the six-gate Hnuter narrow course."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os

import numpy as np
import rclpy

from controllers.experiments.drcda_closed_loop.controller import (
    HnuterClosedLoopDRCDAController,
)
from controllers.experiments.drcda_closed_loop.feedback import JointAngleFeedback
from controllers.simulation.hnuter_external_direct_controller_debug import (
    HnuterController as DirectController,
    env_float,
)
from controllers.simulation.hnuter_external_direct_drcda import HnuterDRCDAController


@dataclass(frozen=True)
class PortalWaypoint:
    name: str
    position: tuple[float, float, float]
    tangent_deg: float
    speed_mps: float


@dataclass(frozen=True)
class GateGeometry:
    name: str
    x_m: float
    center_y_m: float
    width_m: float
    lower_z_m: float
    upper_z_m: float


GATES = (
    GateGeometry('gate_01', 6.0, 0.0, 2.6, 0.20, 1.95),
    GateGeometry('gate_02', 11.0, 2.2, 2.4, 0.20, 1.65),
    GateGeometry('gate_03', 16.5, -2.2, 2.4, 0.45, 2.25),
    GateGeometry('gate_04', 22.0, 2.4, 2.4, 0.20, 1.60),
    GateGeometry('gate_05', 27.5, -2.4, 2.4, 0.50, 2.20),
    GateGeometry('gate_06', 33.0, 1.2, 2.3, 0.25, 1.85),
)


def cubic_hermite(
    start: np.ndarray,
    finish: np.ndarray,
    start_velocity: np.ndarray,
    finish_velocity: np.ndarray,
    elapsed_s: float,
    duration_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Evaluate one position/velocity-continuous Hermite segment."""
    duration_s = max(float(duration_s), 1e-6)
    s = float(np.clip(elapsed_s / duration_s, 0.0, 1.0))
    s2, s3 = s * s, s * s * s
    h00 = 2.0 * s3 - 3.0 * s2 + 1.0
    h10 = s3 - 2.0 * s2 + s
    h01 = -2.0 * s3 + 3.0 * s2
    h11 = s3 - s2
    position = (
        h00 * start + h10 * duration_s * start_velocity
        + h01 * finish + h11 * duration_s * finish_velocity
    )
    dh00 = 6.0 * s2 - 6.0 * s
    dh10 = 3.0 * s2 - 4.0 * s + 1.0
    dh01 = -dh00
    dh11 = 3.0 * s2 - 2.0 * s
    velocity = (
        dh00 * start + dh10 * duration_s * start_velocity
        + dh01 * finish + dh11 * duration_s * finish_velocity
    ) / duration_s
    ddh00 = 12.0 * s - 6.0
    ddh10 = 6.0 * s - 4.0
    ddh01 = -ddh00
    ddh11 = 6.0 * s - 2.0
    acceleration = (
        ddh00 * start + ddh10 * duration_s * start_velocity
        + ddh01 * finish + ddh11 * duration_s * finish_velocity
    ) / duration_s**2
    return position, velocity, acceleration


def course_waypoints(start: np.ndarray) -> list[PortalWaypoint]:
    """Return the world-aligned course used by hnuter_narrow.sdf."""
    return [
        PortalWaypoint('start', tuple(start), 0.0, 0.55),
        PortalWaypoint('gate_01_approach', (3.5, 0.0, 1.05), 0.0, 0.85),
        PortalWaypoint('gate_01_center', (6.0, 0.0, 1.05), 0.0, 0.68),
        PortalWaypoint('gate_01_clear', (7.3, 0.0, 1.05), 0.0, 0.72),
        PortalWaypoint('gate_02_turn', (8.5, 1.1, 0.95), 42.0, 0.88),
        PortalWaypoint('gate_02_align', (9.6, 2.2, 0.85), 0.0, 0.65),
        PortalWaypoint('gate_02_center', (11.0, 2.2, 0.85), 0.0, 0.62),
        PortalWaypoint('gate_02_clear', (12.3, 2.2, 0.85), 0.0, 0.68),
        PortalWaypoint('gate_03_turn', (13.6, 0.0, 1.10), -60.0, 0.90),
        PortalWaypoint('gate_03_align', (15.1, -2.2, 1.30), 0.0, 0.65),
        PortalWaypoint('gate_03_center', (16.5, -2.2, 1.30), 0.0, 0.62),
        PortalWaypoint('gate_03_clear', (17.8, -2.2, 1.30), 0.0, 0.68),
        PortalWaypoint('gate_04_turn', (19.1, 0.1, 1.05), 60.0, 0.90),
        PortalWaypoint('gate_04_align', (20.6, 2.4, 0.85), 0.0, 0.65),
        PortalWaypoint('gate_04_center', (22.0, 2.4, 0.85), 0.0, 0.60),
        PortalWaypoint('gate_04_clear', (23.3, 2.4, 0.85), 0.0, 0.68),
        PortalWaypoint('gate_05_turn', (24.6, 0.0, 1.10), -62.0, 0.90),
        PortalWaypoint('gate_05_align', (26.1, -2.4, 1.30), 0.0, 0.65),
        PortalWaypoint('gate_05_center', (27.5, -2.4, 1.30), 0.0, 0.60),
        PortalWaypoint('gate_05_clear', (28.8, -2.4, 1.30), 0.0, 0.68),
        PortalWaypoint('gate_06_turn', (30.1, -0.6, 1.15), 50.0, 0.88),
        PortalWaypoint('gate_06_align', (31.6, 1.2, 1.05), 0.0, 0.65),
        PortalWaypoint('gate_06_center', (33.0, 1.2, 1.05), 0.0, 0.60),
        PortalWaypoint('gate_06_clear', (34.3, 1.2, 1.05), 0.0, 0.68),
        PortalWaypoint('exit_clear', (37.0, 0.0, 1.05), -24.0, 0.82),
    ]


def build_course(
    waypoints: list[PortalWaypoint], speed_scale: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    positions = np.asarray([waypoint.position for waypoint in waypoints], dtype=float)
    velocities = np.zeros_like(positions)
    speed_scale = max(float(speed_scale), 0.1)
    for index in range(1, len(waypoints) - 1):
        waypoint = waypoints[index]
        heading = math.radians(waypoint.tangent_deg)
        speed = waypoint.speed_mps * speed_scale
        horizontal_span = float(np.linalg.norm(
            positions[index + 1, :2] - positions[index - 1, :2]
        ))
        vertical_slope = 0.0
        if horizontal_span > 0.1:
            vertical_slope = (
                positions[index + 1, 2] - positions[index - 1, 2]
            ) / horizontal_span
        velocities[index] = [
            speed * math.cos(heading),
            speed * math.sin(heading),
            float(np.clip(speed * vertical_slope, -0.3, 0.3)),
        ]
    durations = np.zeros(len(waypoints) - 1)
    for index in range(len(durations)):
        distance = float(np.linalg.norm(positions[index + 1] - positions[index]))
        average_speed = 0.5 * (
            waypoints[index].speed_mps + waypoints[index + 1].speed_mps
        ) * speed_scale
        durations[index] = max(0.75, distance / max(average_speed, 0.25))
    times = np.concatenate(([0.0], np.cumsum(durations)))
    return positions, velocities, durations, times


class NarrowPortalCourseMixin:
    """Replace trajectory key 2 with a fixed-heading six-gate mission."""

    def _node_name(self):
        method = getattr(self, '_portal_method', 'direct')
        return f'hnuter_narrow_portal_{method}'

    def _diagnostic_file_prefix(self):
        method = getattr(self, '_portal_method', 'direct')
        return f'experiments/narrow_portal_course/{method}/course'

    def __init__(self):
        self._portal_method = os.environ.get(
            'HNUTER_PORTAL_METHOD', 'full'
        ).strip().lower()
        self.portal_speed_scale = env_float('HNUTER_PORTAL_SPEED_SCALE', 1.0)
        self.portal_body_half_width_m = env_float(
            'HNUTER_PORTAL_BODY_HALF_WIDTH_M', 0.55
        )
        self.portal_body_half_height_m = env_float(
            'HNUTER_PORTAL_BODY_HALF_HEIGHT_M', 0.40
        )
        self._portal_elapsed_s = math.nan
        self._portal_phase = 'idle'
        self._portal_segment_index = -1
        self._portal_waypoints: list[PortalWaypoint] = []
        self._portal_positions = np.empty((0, 3))
        self._portal_velocities = np.empty((0, 3))
        self._portal_durations = np.empty(0)
        self._portal_times = np.empty(0)
        self._portal_joint_feedback = None
        super().__init__()
        existing = getattr(self, '_feedback', None)
        self._portal_joint_feedback = (
            existing if existing is not None else JointAngleFeedback()
        )

    def _start_auto_trajectory(self, mode: str, current_time: float):
        if mode != 'lissajous':
            return super()._start_auto_trajectory(mode, current_time)
        super()._start_auto_trajectory(mode, current_time)
        self.auto_traj_mode = 'narrow_portal'
        self._portal_waypoints = course_waypoints(self.auto_traj_start_pos.copy())
        (
            self._portal_positions,
            self._portal_velocities,
            self._portal_durations,
            self._portal_times,
        ) = build_course(self._portal_waypoints, self.portal_speed_scale)
        self.get_logger().info(
            '开始执行固定航向六门窄通道任务：'
            f'speed_scale={self.portal_speed_scale:.2f}, '
            f'duration={self._portal_times[-1]:.1f}s'
        )

    def _portal_reference(
        self, elapsed_s: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, str, int, bool]:
        if elapsed_s >= self._portal_times[-1]:
            return (
                self._portal_positions[-1].copy(), np.zeros(3), np.zeros(3),
                'complete', len(self._portal_durations) - 1, True,
            )
        index = int(np.searchsorted(self._portal_times, elapsed_s, side='right') - 1)
        index = int(np.clip(index, 0, len(self._portal_durations) - 1))
        local_elapsed = elapsed_s - self._portal_times[index]
        position, velocity, acceleration = cubic_hermite(
            self._portal_positions[index], self._portal_positions[index + 1],
            self._portal_velocities[index], self._portal_velocities[index + 1],
            local_elapsed, self._portal_durations[index],
        )
        phase = (
            f'{self._portal_waypoints[index].name}->'
            f'{self._portal_waypoints[index + 1].name}'
        )
        return position, velocity, acceleration, phase, index, False

    def _finish_portal_course(self):
        final = self._portal_positions[-1].copy()
        self.auto_traj_mode = 'hover'
        self.manual_des_pos = final
        self.manual_des_yaw = self.auto_traj_yaw
        self.manual_des_roll = 0.0
        self.manual_des_pitch = 0.0
        self.target_position = final.copy()
        self.target_velocity = np.zeros(3)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([0.0, 0.0, self.auto_traj_yaw])
        self.target_attitude_rate = np.zeros(3)
        self.target_R_des_ned_frd = None
        self.integral_pos_error[:] = 0.0
        self.integral_e_R[:] = 0.0
        self._attitude_error_quaternion = None
        self._portal_phase = 'post_hold'
        self.get_logger().info('固定航向六门窄通道任务完成')

    def _update_auto_trajectory(self, current_time: float):
        if self.auto_traj_mode != 'narrow_portal':
            return super()._update_auto_trajectory(current_time)
        elapsed = max(0.0, current_time - self.auto_traj_start_time)
        self._portal_elapsed_s = elapsed
        (
            position, velocity, acceleration, self._portal_phase,
            self._portal_segment_index, done,
        ) = self._portal_reference(elapsed)
        if done:
            self._finish_portal_course()
            return True
        self.manual_des_pos = position.copy()
        self.manual_des_yaw = self.auto_traj_yaw
        self.manual_des_roll = 0.0
        self.manual_des_pitch = 0.0
        self._last_manual_cmd = self._zero_manual_cmd()
        self.target_position = position
        self.target_velocity = velocity
        self.target_acceleration = acceleration
        self.target_attitude = np.array([0.0, 0.0, self.auto_traj_yaw])
        self.target_attitude_rate = np.zeros(3)
        self.target_R_des_ned_frd = None
        return True

    def _portal_clearance(self) -> tuple[str, float]:
        if not self._z0_initialized:
            return '', math.nan
        position = np.array([
            float(self.position[0]), float(self.position[1]),
            float(self.position[2] - self._z0),
        ])
        gate = min(GATES, key=lambda item: abs(position[0] - item.x_m))
        if abs(position[0] - gate.x_m) > 0.6:
            return gate.name, math.nan
        lateral = (
            0.5 * gate.width_m
            - abs(position[1] - gate.center_y_m)
            - self.portal_body_half_width_m
        )
        vertical = min(
            position[2] - gate.lower_z_m - self.portal_body_half_height_m,
            gate.upper_z_m - position[2] - self.portal_body_half_height_m,
        )
        return gate.name, float(min(lateral, vertical))

    def _diagnostic_extra_header(self):
        return super()._diagnostic_extra_header() + [
            'portal_elapsed_s', 'portal_phase', 'portal_segment_index',
            'portal_speed_scale', 'portal_nearest_gate',
            'portal_clearance_margin_m', 'portal_joint_feedback_age_s',
            'portal_joint_feedback_available',
            *[f'portal_measured_joint_{name}_rad'
              for name in ('a_l', 'b_l', 'a_r', 'b_r')],
            *[f'portal_commanded_joint_{name}_rad'
              for name in ('a_l', 'b_l', 'a_r', 'b_r')],
        ]

    def _diagnostic_extra_values(self):
        measured = None
        age = math.inf
        if self._portal_joint_feedback is not None:
            measured, age = self._portal_joint_feedback.read()
        gate, clearance = self._portal_clearance()
        commanded = np.array([
            self._alpha1_cmd, self._theta1_cmd,
            self._alpha2_cmd, self._theta2_cmd,
        ])
        return super()._diagnostic_extra_values() + [
            self._portal_elapsed_s, self._portal_phase,
            self._portal_segment_index, self.portal_speed_scale,
            gate, clearance,
            age if math.isfinite(age) else float('nan'), int(measured is not None),
            *([*measured] if measured is not None else [float('nan')] * 4),
            *commanded,
        ]


class NarrowPortalDirectController(NarrowPortalCourseMixin, DirectController):
    pass


class NarrowPortalDRCDAController(NarrowPortalCourseMixin, HnuterDRCDAController):
    pass


class NarrowPortalClosedLoopController(
    NarrowPortalCourseMixin, HnuterClosedLoopDRCDAController
):
    pass


def controller_class(method: str):
    method = method.strip().lower()
    if method == 'direct':
        return NarrowPortalDirectController
    if method == 'basic_da':
        os.environ['HNUTER_DRCDA_VARIANT'] = 'basic_da'
        return NarrowPortalDRCDAController
    if method in {'full', 'no_horizon'}:
        os.environ['HNUTER_DRCDA_VARIANT'] = method
        return NarrowPortalClosedLoopController
    raise ValueError(f'unsupported HNUTER_PORTAL_METHOD={method!r}')


def main(args=None):
    rclpy.init(args=args)
    method = os.environ.get('HNUTER_PORTAL_METHOD', 'full')
    controller = controller_class(method)()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
