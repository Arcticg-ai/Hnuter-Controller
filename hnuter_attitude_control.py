#!/usr/bin/env python3
"""ROS-independent geometric attitude-control helpers."""

from __future__ import annotations

import numpy as np


def estimator_yaw_reset_enu(delta_quaternion: np.ndarray) -> float:
    """Convert PX4's NED attitude-reset quaternion to an ENU yaw delta."""
    quaternion = np.asarray(delta_quaternion, dtype=float).reshape(-1)
    if quaternion.size != 4:
        raise ValueError('delta_quaternion must contain four values')
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-6 or not np.isfinite(norm):
        return 0.0
    w, x, y, z = quaternion / norm
    yaw_ned = np.arctan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return float(-yaw_ned)


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
