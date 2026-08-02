#!/usr/bin/env python3

import math
import unittest

import numpy as np

from hnuter_position_control import (
    body_frame_horizontal_feedback_ned,
    heading_rotation_ned_body_xy,
)


class HnuterPositionControlTest(unittest.TestCase):
    def test_body_feedback_uses_smooth_position_and_velocity_deadbands(self):
        feedback = body_frame_horizontal_feedback_ned(
            np.eye(3),
            [0.05, -0.02],
            [0.03, -0.08],
            [0.0, 0.0],
            [2.0, 2.0],
            [3.0, 3.0],
            [0.0, 0.0],
            [0.02, 0.02],
            [0.01, 0.01],
        )
        np.testing.assert_allclose(feedback, [0.12, -0.21], atol=1e-12)

    def test_heading_rotation_ignores_roll_and_pitch(self):
        yaw = math.radians(37.0)
        rotation = np.array([
            [math.cos(yaw), -math.sin(yaw), 0.2],
            [math.sin(yaw), math.cos(yaw), -0.1],
            [0.3, 0.4, 0.8],
        ])
        expected = np.array([
            [math.cos(yaw), -math.sin(yaw)],
            [math.sin(yaw), math.cos(yaw)],
        ])
        np.testing.assert_allclose(
            heading_rotation_ned_body_xy(rotation), expected, atol=1e-12
        )

    def test_body_axis_gains_rotate_with_heading(self):
        yaw = 0.5 * math.pi
        rotation = np.array([
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ])
        feedback = body_frame_horizontal_feedback_ned(
            rotation,
            position_error_ned_xy=[1.0, 1.0],
            velocity_error_ned_xy=[0.0, 0.0],
            integral_error_ned_xy=[0.0, 0.0],
            kp_body_xy=[4.0, 1.0],
            kd_body_xy=[0.0, 0.0],
            ki_body_xy=[0.0, 0.0],
        )
        np.testing.assert_allclose(feedback, [1.0, 4.0], atol=1e-12)

    def test_isotropic_body_gains_match_world_frame_result(self):
        yaw = math.radians(-73.0)
        rotation = np.array([
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ])
        position_error = np.array([0.7, -0.2])
        velocity_error = np.array([-0.3, 0.4])
        integral_error = np.array([0.1, 0.6])
        feedback = body_frame_horizontal_feedback_ned(
            rotation,
            position_error,
            velocity_error,
            integral_error,
            kp_body_xy=[2.0, 2.0],
            kd_body_xy=[3.0, 3.0],
            ki_body_xy=[0.5, 0.5],
        )
        expected = (
            2.0 * position_error
            + 3.0 * velocity_error
            + 0.5 * integral_error
        )
        np.testing.assert_allclose(feedback, expected, atol=1e-12)


if __name__ == '__main__':
    unittest.main()
