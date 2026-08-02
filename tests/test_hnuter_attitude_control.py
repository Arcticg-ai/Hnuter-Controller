#!/usr/bin/env python3

import math
import unittest

import numpy as np

from hnuter_attitude_control import (
    large_tilt_yaw_scale,
    quaternion_attitude_error,
    reduced_tilt_attitude_error,
    update_attitude_axis_toggle,
)


def rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    skew = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return (
        np.eye(3)
        + math.sin(angle) * skew
        + (1.0 - math.cos(angle)) * (skew @ skew)
    )


class QuaternionAttitudeErrorTest(unittest.TestCase):
    def test_small_error_matches_rotation_vector(self):
        axis = np.array([0.3, -0.4, 0.5])
        axis /= np.linalg.norm(axis)
        angle = 1e-4
        error, _, measured_angle = quaternion_attitude_error(
            np.eye(3), rotation(axis, angle)
        )
        np.testing.assert_allclose(error, axis * angle, atol=1e-10)
        self.assertAlmostEqual(measured_angle, angle, places=10)

    def test_half_turn_error_does_not_collapse(self):
        error, quaternion, angle = quaternion_attitude_error(
            rotation(np.array([0.0, 1.0, 0.0]), math.pi),
            np.eye(3),
        )
        self.assertAlmostEqual(np.linalg.norm(error), 2.0, places=10)
        self.assertAlmostEqual(angle, math.pi, places=10)
        self.assertAlmostEqual(abs(quaternion[2]), 1.0, places=10)

    def test_previous_quaternion_keeps_half_turn_direction_continuous(self):
        desired_before = rotation(
            np.array([0.0, 1.0, 0.0]), math.pi - 1e-6
        )
        _, previous, _ = quaternion_attitude_error(
            desired_before, np.eye(3)
        )
        _, current, _ = quaternion_attitude_error(
            rotation(np.array([0.0, 1.0, 0.0]), math.pi),
            np.eye(3),
            previous,
        )
        self.assertGreater(float(np.dot(previous, current)), 0.99)


class LargeTiltYawScaleTest(unittest.TestCase):
    def test_scale_is_full_below_start_and_minimum_above_full(self):
        self.assertEqual(
            large_tilt_yaw_scale(math.radians(30.0), math.radians(45.0), math.radians(80.0), 0.1),
            1.0,
        )
        self.assertAlmostEqual(
            large_tilt_yaw_scale(math.radians(90.0), math.radians(45.0), math.radians(80.0), 0.1),
            0.1,
        )

    def test_scale_transition_is_smooth_and_symmetric(self):
        positive = large_tilt_yaw_scale(
            math.radians(62.5), math.radians(45.0), math.radians(80.0), 0.1
        )
        negative = large_tilt_yaw_scale(
            math.radians(-62.5), math.radians(45.0), math.radians(80.0), 0.1
        )
        self.assertAlmostEqual(positive, 0.55)
        self.assertAlmostEqual(negative, positive)


class ReducedTiltAttitudeErrorTest(unittest.TestCase):
    def test_body_yaw_does_not_change_tilt_error(self):
        desired = rotation(np.array([1.0, 0.0, 0.0]), math.pi / 2.0)
        current = desired @ rotation(np.array([0.0, 0.0, 1.0]), math.pi / 3.0)
        full_error, _, _ = quaternion_attitude_error(desired, current)
        error, alignment, blend = reduced_tilt_attitude_error(
            desired, current, full_error
        )
        np.testing.assert_allclose(error[:2], np.zeros(2), atol=1e-12)
        self.assertAlmostEqual(alignment, 1.0)
        self.assertEqual(blend, 0.0)
        self.assertGreater(abs(error[2]), 0.5)

    def test_half_turn_uses_full_quaternion_error(self):
        desired = rotation(np.array([0.0, 1.0, 0.0]), math.pi)
        full_error, _, _ = quaternion_attitude_error(desired, np.eye(3))
        error, alignment, blend = reduced_tilt_attitude_error(
            desired, np.eye(3), full_error
        )
        np.testing.assert_allclose(error, full_error, atol=1e-12)
        self.assertAlmostEqual(alignment, -1.0)
        self.assertEqual(blend, 1.0)


class AttitudeAxisToggleTest(unittest.TestCase):
    def test_rb_toggles_once_per_press(self):
        axis, was_pressed, toggled = update_attitude_axis_toggle(
            'roll', True, False
        )
        self.assertEqual(axis, 'pitch')
        self.assertTrue(was_pressed)
        self.assertTrue(toggled)

        axis, was_pressed, toggled = update_attitude_axis_toggle(
            axis, True, was_pressed
        )
        self.assertEqual(axis, 'pitch')
        self.assertFalse(toggled)

        axis, was_pressed, toggled = update_attitude_axis_toggle(
            axis, False, was_pressed
        )
        self.assertEqual(axis, 'pitch')
        self.assertFalse(was_pressed)
        self.assertFalse(toggled)

        axis, _, toggled = update_attitude_axis_toggle(
            axis, True, was_pressed
        )
        self.assertEqual(axis, 'roll')
        self.assertTrue(toggled)


if __name__ == '__main__':
    unittest.main()
