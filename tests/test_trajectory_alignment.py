import unittest

import numpy as np

from tools.plotting.trajectory_alignment import (
    fit_planar_rotation,
    transform_points,
    transform_vectors,
)


class TrajectoryAlignmentTest(unittest.TestCase):
    def test_rotated_and_translated_run_returns_to_reference_frame(self):
        phase = np.linspace(0.0, 2.0 * np.pi, 101)
        reference = np.column_stack((
            np.sin(phase),
            0.7 * np.sin(2.0 * phase),
            0.2 * (1.0 - np.cos(phase)),
        ))
        angle = np.deg2rad(63.0)
        run_rotation = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)],
        ])
        moving = reference.copy()
        moving[:, :2] = moving[:, :2] @ run_rotation
        moving += np.array([4.0, -2.0, 1.5])

        rotation, residual = fit_planar_rotation(reference, moving)
        aligned = transform_points(moving, moving[0], reference[0], rotation)

        self.assertLess(residual, 1e-12)
        np.testing.assert_allclose(aligned, reference, atol=1e-12)

    def test_vector_transform_does_not_translate(self):
        rotation = np.array([[0.0, 1.0], [-1.0, 0.0]])
        vectors = np.array([[1.0, 0.0, 2.0]])
        transformed = transform_vectors(vectors, rotation)
        np.testing.assert_allclose(transformed, [[0.0, 1.0, 2.0]])


if __name__ == '__main__':
    unittest.main()
