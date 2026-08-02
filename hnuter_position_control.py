#!/usr/bin/env python3
"""Coordinate helpers for anisotropic Hnuter horizontal position control."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def _xy(values: Iterable[float], name: str) -> np.ndarray:
    array = np.asarray(tuple(values), dtype=float)
    if array.shape != (2,):
        raise ValueError(f'{name} must contain two values')
    return array


def heading_rotation_ned_body_xy(rotation_ned_frd: np.ndarray) -> np.ndarray:
    """Return the yaw-only rotation mapping body XY vectors into NED XY."""
    rotation = np.asarray(rotation_ned_frd, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError('rotation_ned_frd must be a 3x3 matrix')
    yaw_ned = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    cosine = math.cos(yaw_ned)
    sine = math.sin(yaw_ned)
    return np.array([[cosine, -sine], [sine, cosine]], dtype=float)


def body_frame_horizontal_feedback_ned(
    rotation_ned_frd: np.ndarray,
    position_error_ned_xy: Iterable[float],
    velocity_error_ned_xy: Iterable[float],
    integral_error_ned_xy: Iterable[float],
    kp_body_xy: Iterable[float],
    kd_body_xy: Iterable[float],
    ki_body_xy: Iterable[float],
    position_deadband_body_xy: Iterable[float] = (0.0, 0.0),
    velocity_deadband_body_xy: Iterable[float] = (0.0, 0.0),
) -> np.ndarray:
    """Apply independent body-X/Y gains and return feedback acceleration in NED."""
    heading = heading_rotation_ned_body_xy(rotation_ned_frd)
    position_error_body = heading.T @ _xy(
        position_error_ned_xy, 'position_error_ned_xy'
    )
    velocity_error_body = heading.T @ _xy(
        velocity_error_ned_xy, 'velocity_error_ned_xy'
    )
    position_deadband = np.maximum(_xy(
        position_deadband_body_xy, 'position_deadband_body_xy'
    ), 0.0)
    velocity_deadband = np.maximum(_xy(
        velocity_deadband_body_xy, 'velocity_deadband_body_xy'
    ), 0.0)
    position_error_body = np.sign(position_error_body) * np.maximum(
        np.abs(position_error_body) - position_deadband,
        0.0,
    )
    velocity_error_body = np.sign(velocity_error_body) * np.maximum(
        np.abs(velocity_error_body) - velocity_deadband,
        0.0,
    )
    integral_error_body = heading.T @ _xy(
        integral_error_ned_xy, 'integral_error_ned_xy'
    )
    feedback_body = (
        _xy(kp_body_xy, 'kp_body_xy') * position_error_body
        + _xy(kd_body_xy, 'kd_body_xy') * velocity_error_body
        + _xy(ki_body_xy, 'ki_body_xy') * integral_error_body
    )
    return heading @ feedback_body
