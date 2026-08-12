#!/usr/bin/env python3

import json
import unittest
from pathlib import Path

import numpy as np

from hnuter_external_direct_ok_hardware import HnuterOkHardwareController


class OkControllerFixture(unittest.TestCase):
    def setUp(self):
        controller = object.__new__(HnuterOkHardwareController)
        controller.velocity = np.zeros(3)
        controller.measured_acceleration_ned = np.zeros(3)
        controller.ok_position_p_ned = np.array([3.75, 3.75, 3.5])
        controller.ok_velocity_p_ned = np.array([9.05, 9.05, 4.0])
        controller.ok_velocity_i_ned = np.array([0.39, 0.39, 0.2])
        controller.ok_velocity_d_ned = np.array([0.36, 0.36, 0.4])
        controller.ok_velocity_integral_limit_ned = np.array([1.5, 1.5, 2.5])
        controller.ok_velocity_integral_ned = np.zeros(3)
        controller.integral_pos_error = np.zeros(3)
        controller.ok_velocity_xy_mps = 3.0
        controller.ok_velocity_up_mps = 1.5
        controller.ok_velocity_down_mps = 1.0
        controller.ok_acceleration_xy_mps2 = 3.0
        controller.ok_acceleration_z_mps2 = 45.6
        controller.ok_lock_acceleration_xy_mps2 = 1.0
        controller.ok_max_thrust_per_arm_n = 170.96
        controller.ok_max_tail_thrust_n = 85.48
        controller.ok_motor_hover_control = 0.5
        controller.ok_motor_thrust_exponent = 0.5
        controller.mass = 4.5
        controller.gravity = 9.81
        controller._last_control_dt_s = 0.01
        controller.land_detected = {'landed': False, 'maybe_landed': False}
        controller.allow_tail_reverse = True
        self.controller = controller


class OkCascadedPositionTest(OkControllerFixture):
    def compute(self, position_error, velocity_error=None, acceleration_ff=None):
        return self.controller._direct_position_acceleration_ned(
            np.zeros(3) if acceleration_ff is None else np.asarray(acceleration_ff),
            np.asarray(position_error, dtype=float),
            np.zeros(3) if velocity_error is None else np.asarray(velocity_error),
            False,
        )

    def test_zero_error_produces_zero_acceleration(self):
        np.testing.assert_allclose(self.compute([0.0, 0.0, 0.0]), 0.0)

    def test_large_position_error_respects_vector_xy_acceleration_limit(self):
        acceleration = self.compute([1.0, 1.0, 0.0])
        self.assertAlmostEqual(np.linalg.norm(acceleration[:2]), 3.0)
        np.testing.assert_allclose(self.controller.ok_velocity_integral_ned, 0.0)

    def test_unsaturated_velocity_error_updates_ok_integral(self):
        acceleration = self.compute([0.01, 0.0, 0.0])
        self.assertAlmostEqual(acceleration[0], 9.05 * 3.75 * 0.01)
        self.assertAlmostEqual(
            self.controller.ok_velocity_integral_ned[0],
            0.39 * 3.75 * 0.01 * 0.01,
        )

    def test_measured_acceleration_is_damping_feedback(self):
        self.controller.measured_acceleration_ned = np.array([0.5, 0.0, 0.0])
        acceleration = self.compute([0.0, 0.0, 0.0])
        self.assertAlmostEqual(acceleration[0], -0.36 * 0.5)


class OkAttitudeAndAllocationTest(OkControllerFixture):
    def test_attitude_integral_is_pitch_only_and_clamped(self):
        self.controller.integral_e_R = np.zeros(3)
        self.controller.direct_attitude_integral_limit = np.array([0.0, 1.5, 0.0])
        self.controller._update_attitude_integral(
            np.array([2.0, 2.0, 2.0]),
            np.deg2rad(170.0),
            np.array([0.0, 0.06, 0.0]),
            1.0,
        )
        np.testing.assert_allclose(self.controller.integral_e_R, [0.0, 1.5, 0.0])

    def test_front_hover_force_maps_to_logged_hover_control(self):
        hover_force_per_motor = self.controller.mass * self.controller.gravity / 4.0
        self.assertAlmostEqual(
            self.controller._front_motor_control(hover_force_per_motor, 85.48),
            0.5,
        )

    def test_tail_mapping_retains_ok_exponent_and_reverse(self):
        max_tail = self.controller.ok_max_tail_thrust_n
        self.assertAlmostEqual(self.controller._tail_motor_control(max_tail, max_tail), 1.0)
        self.assertAlmostEqual(self.controller._tail_motor_control(0.25 * max_tail, max_tail), 0.5)
        self.assertAlmostEqual(self.controller._tail_motor_control(-0.25 * max_tail, max_tail), -0.5)

    def test_current_servo_calibration_is_retained(self):
        self.controller.primary_servo_angle_max_rad = np.deg2rad(180.0)
        self.controller.secondary_servo_angle_max_rad = np.deg2rad(180.0)
        self.controller.secondary_servo_gear_ratio = 2.0
        self.controller.servo_pwm_min_us = 500
        self.controller.servo_pwm_trim_us = 1500
        self.controller.servo_pwm_max_us = 2500
        primary = self.controller._primary_joint_angle_to_normalized(np.deg2rad(180.0))
        secondary = self.controller._secondary_joint_angle_to_normalized(np.deg2rad(90.0))
        self.assertAlmostEqual(primary, 1.0)
        self.assertAlmostEqual(secondary, 1.0)
        self.assertAlmostEqual(
            self.controller._normalized_servo_to_expected_pwm_us(primary), 2500.0
        )


class OkConfigurationTest(unittest.TestCase):
    def test_config_binds_ok_source_and_current_hardware_profile(self):
        path = Path(__file__).resolve().parents[1] / 'config' / 'hnuter_direct_ok_hardware_tuning.json'
        config = json.loads(path.read_text(encoding='utf-8'))
        self.assertEqual(config['ok_source_tag'], 'hnuter-ok-144bd9fe')
        self.assertEqual(config['hardware_firmware_profile'], '3131ddd4_500_2500_gear2')
        self.assertEqual(
            [config['servo_pwm_min_us'], config['servo_pwm_trim_us'], config['servo_pwm_max_us']],
            [500, 1500, 2500],
        )
        self.assertEqual(config['direct_KR'], [18.2, 20.0, 6.0])
        self.assertEqual(config['direct_tau_limit'], [56.3, 10.0, 0.8])
        self.assertEqual(config['HNTR_MAX_ARM_T'], 170.96)
        self.assertEqual(config['HNTR_MOT_HOV'], 0.5)


if __name__ == '__main__':
    unittest.main()
