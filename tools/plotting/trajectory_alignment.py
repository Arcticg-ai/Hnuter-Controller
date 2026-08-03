"""Align repeated trajectory runs into one comparison frame."""

from __future__ import annotations

import numpy as np


def _resample_xy(values: np.ndarray, sample_count: int) -> np.ndarray:
    phase = np.linspace(0.0, 1.0, values.shape[0])
    common_phase = np.linspace(0.0, 1.0, sample_count)
    return np.column_stack([
        np.interp(common_phase, phase, values[:, axis])
        for axis in range(2)
    ])


def fit_planar_rotation(
    reference_target: np.ndarray,
    moving_target: np.ndarray,
) -> tuple[np.ndarray, float]:
    """Return the proper XY rotation that aligns two target trajectories."""
    reference_target = np.asarray(reference_target, dtype=float)
    moving_target = np.asarray(moving_target, dtype=float)
    if (
        reference_target.ndim != 2
        or moving_target.ndim != 2
        or reference_target.shape[1] < 2
        or moving_target.shape[1] < 2
        or min(reference_target.shape[0], moving_target.shape[0]) < 3
    ):
        raise ValueError('target trajectories must contain at least three XY points')

    sample_count = min(reference_target.shape[0], moving_target.shape[0])
    reference_xy = _resample_xy(
        reference_target[:, :2] - reference_target[0, :2], sample_count
    )
    moving_xy = _resample_xy(
        moving_target[:, :2] - moving_target[0, :2], sample_count
    )
    left, _, right_t = np.linalg.svd(moving_xy.T @ reference_xy)
    rotation = left @ right_t
    if np.linalg.det(rotation) < 0.0:
        left[:, -1] *= -1.0
        rotation = left @ right_t

    residual = moving_xy @ rotation - reference_xy
    alignment_rmse = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))
    return rotation, alignment_rmse


def transform_points(
    values: np.ndarray,
    source_origin: np.ndarray,
    destination_origin: np.ndarray,
    rotation: np.ndarray,
) -> np.ndarray:
    """Rotate XY points about the source origin and translate to destination."""
    transformed = np.asarray(values, dtype=float).copy()
    source_origin = np.asarray(source_origin, dtype=float)
    destination_origin = np.asarray(destination_origin, dtype=float)
    transformed[:, :2] = (
        transformed[:, :2] - source_origin[:2]
    ) @ rotation + destination_origin[:2]
    if transformed.shape[1] >= 3:
        transformed[:, 2] += destination_origin[2] - source_origin[2]
    return transformed


def transform_vectors(values: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    """Rotate the XY components of vectors without applying a translation."""
    transformed = np.asarray(values, dtype=float).copy()
    transformed[:, :2] = transformed[:, :2] @ rotation
    return transformed
