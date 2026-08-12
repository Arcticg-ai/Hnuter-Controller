#!/usr/bin/env python3

import math
import unittest

import numpy as np

from hnuter_external_direct_controller_debug import HnuterController


class AttitudeRotationReferenceTest(unittest.TestCase):
    def setUp(self):
        self.controller = HnuterController.__new__(HnuterController)
        self.controller.attitude_segment_time_s = 15.0
        self.controller.attitude_peak_hold_s = 5.0
        self.controller.attitude_level_settle_s = 10.0
        self.controller.attitude_step_axis_rad = np.radians([80.0, 180.0, 0.0])
        self.controller.attitude_test_bidirectional = True
        self.controller.auto_traj_start_attitude = np.zeros(3)

    def reference(self, elapsed_s):
        return self.controller._attitude_reference(elapsed_s)

    def test_level_settle_is_stationary_before_next_axis(self):
        attitude, rate, rotation, done = self.reference(40.0)
        self.assertFalse(done)
        np.testing.assert_allclose(attitude, 0.0, atol=1e-12)
        np.testing.assert_allclose(rate, 0.0, atol=1e-12)
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)

    def test_positive_and_negative_pitch_follow_distinct_matrix_paths(self):
        _, positive_rate, positive_rotation, _ = self.reference(52.5)
        _, negative_rate, negative_rotation, _ = self.reference(142.5)
        self.assertGreater(positive_rate[1], 0.0)
        self.assertLess(negative_rate[1], 0.0)
        self.assertFalse(np.allclose(positive_rotation, negative_rotation))
        for rotation in (positive_rotation, negative_rotation):
            np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0, places=12)

    def test_half_turn_endpoints_are_valid_equivalent_rotations(self):
        _, positive_rate, positive_rotation, _ = self.reference(62.0)
        _, negative_rate, negative_rotation, _ = self.reference(152.0)
        np.testing.assert_allclose(positive_rate, 0.0, atol=1e-12)
        np.testing.assert_allclose(negative_rate, 0.0, atol=1e-12)
        np.testing.assert_allclose(positive_rotation, negative_rotation, atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.det(positive_rotation)), 1.0, places=12)

    def test_complete_sequence_includes_four_settle_intervals(self):
        _, _, _, before_done = self.reference(179.999)
        _, _, _, done = self.reference(180.0)
        self.assertFalse(before_done)
        self.assertTrue(done)


if __name__ == '__main__':
    unittest.main()
