#!/usr/bin/env python3

import json
import types
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from controllers.hardware.hnuter_external_direct_drcda_hardware import (
    ANGLE_COUNT,
    DRCDAAllocator,
    DRCDAConfig,
    HnuterHardwareDRCDAController,
    HnuterHardwareController,
    HnuterWrenchModel,
)


class HardwareDRCDAMotorMappingTest(unittest.TestCase):
    def setUp(self):
        controller = object.__new__(HnuterHardwareDRCDAController)
        controller.mass = 4.8
        controller.gravity = 9.81
        controller.max_thrust_per_arm_n = 170.96
        controller.motor_hover_control = 0.5
        controller.motor_thrust_exponent = 0.5
        controller.tail_thrust_positive_n = 12.78
        controller.tail_thrust_negative_n = 6.04
        controller.tail_thrust_exponent_positive = 0.55
        controller.tail_thrust_exponent_negative = 0.68
        controller.allow_tail_reverse = True
        self.controller = controller

    def test_front_motor_mapping_round_trip_uses_hover_anchor(self):
        hover = self.controller.mass * self.controller.gravity * 0.25
        self.assertAlmostEqual(
            self.controller._front_thrust_to_control(hover), 0.5
        )
        mapped_limit = self.controller._front_reachable_thrust_max_n()
        for thrust in (0.0, 5.0, hover, 30.0, mapped_limit):
            control = self.controller._front_thrust_to_control(thrust)
            recovered = self.controller._front_control_to_thrust(control)
            self.assertAlmostEqual(recovered, thrust, places=9)

    def test_tail_bidirectional_mapping_round_trip_is_asymmetric(self):
        for thrust in (-6.04, -3.0, 0.0, 3.0, 12.78):
            control = self.controller._tail_thrust_to_control(thrust)
            recovered = self.controller._tail_control_to_thrust(control)
            self.assertAlmostEqual(recovered, thrust, places=9)

    def test_firmware_aligned_hover_model_has_no_collective_pitch_torque(self):
        model = HnuterWrenchModel(
            arm_half_span_m=0.33,
            front_x_m=0.0,
            front_z_m=0.0,
            tail_x_m=-0.72,
            reaction_torque_ratio_m=0.0,
        )
        hover = self.controller.mass * self.controller.gravity * 0.25
        state = np.array([0.0] * 4 + [hover] * 4 + [0.0])
        wrench = model.wrench(state)
        self.assertAlmostEqual(wrench[4], 0.0, places=9)
        self.assertAlmostEqual(wrench[5], 0.0, places=9)

    def test_handover_snapshot_initializes_physical_actuator_state(self):
        self.controller._hardware_handover_snapshot_valid = True
        self.controller.primary_servo_angle_max_rad = np.pi
        self.controller.secondary_servo_angle_max_rad = np.pi
        self.controller.secondary_servo_gear_ratio = 2.0
        self.controller._hardware_handover_motor_start = np.array(
            [0.50, 0.50, 0.50, 0.50, 0.25]
        )
        self.controller._hardware_handover_servo_start = np.array(
            [0.10, 0.20, 0.30, 0.40]
        )
        fallback = np.zeros(9)
        state = self.controller._captured_drcda_state(fallback)
        np.testing.assert_allclose(
            state[:4], [0.20 * np.pi, 0.20 * np.pi, 0.10 * np.pi, 0.15 * np.pi]
        )
        hover = self.controller.mass * self.controller.gravity * 0.25
        np.testing.assert_allclose(state[4:8], hover)
        self.assertAlmostEqual(
            state[8],
            self.controller.tail_thrust_positive_n
            * 0.25 ** (1.0 / self.controller.tail_thrust_exponent_positive),
        )


class HardwareDRCDAJointLimitTest(unittest.TestCase):
    def test_default_predictor_uses_current_servo_shaft_limits(self):
        config = DRCDAConfig()
        np.testing.assert_allclose(config.servo_state_limit_rad, np.pi)
        np.testing.assert_allclose(config.servo_command_limit_rad, np.pi)

    def test_allocator_respects_geared_secondary_joint_limit(self):
        config = DRCDAConfig.ideal_servos(
            prediction_dt_s=0.01,
            horizon_s=0.10,
        )
        limits = np.array([np.pi, np.pi / 2.0, np.pi, np.pi / 2.0])
        config.servo_state_limit_rad[:] = limits
        config.servo_command_limit_rad[:] = limits
        config.command_scale[:ANGLE_COUNT] = limits
        allocator = DRCDAAllocator(HnuterWrenchModel(), config)

        allocator.allocate(
            desired_wrench=np.array([0.0, 80.0, 44.1, 0.0, 0.0, 0.0]),
            dt=0.01,
            active_angle_limits=limits,
        )

        self.assertTrue(np.all(np.abs(allocator.command[:ANGLE_COUNT]) <= limits))

    def test_synchronized_handover_has_no_first_frame_wrench_derivative_spike(self):
        config = DRCDAConfig.ideal_servos(
            prediction_dt_s=0.01,
            horizon_s=0.10,
        )
        model = HnuterWrenchModel()
        allocator = DRCDAAllocator(model, config)
        hover_state = np.array([0.0] * 4 + [11.772] * 4 + [0.0])
        desired = model.wrench(hover_state)
        allocator.reset(
            angle_state=hover_state[:4], thrust_state=hover_state[4:]
        )
        allocator.synchronize_wrench_reference(desired)
        result = allocator.allocate(
            desired_wrench=desired,
            dt=0.01,
            preferred_command=hover_state,
        )
        self.assertLess(np.linalg.norm(result.jerk_reference), 1e-6)


class HardwareDRCDAConfigurationTest(unittest.TestCase):
    def test_config_uses_current_pwm_profile(self):
        path = (
            Path(__file__).resolve().parents[2]
            / 'config'
            / 'hardware'
            / 'hnuter_drcda_hardware_tuning.json'
        )
        config = json.loads(path.read_text(encoding='utf-8'))
        self.assertEqual(
            config['hardware_firmware_profile'],
            'tail_identified_20260827_500_2500_gear2',
        )
        self.assertEqual(
            [
                config['servo_pwm_min_us'],
                config['servo_pwm_trim_us'],
                config['servo_pwm_max_us'],
            ],
            [500, 1500, 2500],
        )
        self.assertEqual(config['primary_servo_angle_max_deg'], 180.0)
        self.assertEqual(config['secondary_servo_angle_max_deg'], 180.0)
        self.assertEqual(config['HNTR_S2_GEAR'], 2.0)
        self.assertEqual(config['HNTR_MAX_ARM_T'], 170.96)
        self.assertEqual(config['HNTR_TAIL_T_POS'], 12.78)
        self.assertEqual(config['HNTR_TAIL_T_NEG'], 6.04)
        self.assertTrue(config['direct_safety_attitude_check_enabled'])
        self.assertEqual(config['rc_attitude_rate_deg_s'], [20.0, 20.0])
        self.assertEqual(config['rc_attitude_angle_limit_deg'], 45.0)
        self.assertEqual(config['rc_attitude_sign'], [-1.0, -1.0])


class HardwareDRCDAHandoverGateTest(unittest.TestCase):
    def test_inactive_hardware_gate_falls_back_without_legacy_takeoff_attribute(self):
        controller = HnuterHardwareDRCDAController.__new__(
            HnuterHardwareDRCDAController
        )
        controller._drcda_ready = True
        controller._drcda_active_call = True
        controller.armed = True
        controller.takeoff_requested = True
        controller._hardware_control_active = False
        controller.drcda = types.SimpleNamespace(reset=mock.Mock())

        with mock.patch.object(
            HnuterHardwareController,
            'publish_direct_actuator_setpoint',
            return_value='fallback',
        ) as fallback:
            result = controller.publish_direct_actuator_setpoint(
                np.zeros(5), 0.0, 0.0, 0.0, 0.0
            )

        self.assertEqual(result, 'fallback')
        controller.drcda.reset.assert_called_once()
        fallback.assert_called_once()


if __name__ == '__main__':
    unittest.main()
