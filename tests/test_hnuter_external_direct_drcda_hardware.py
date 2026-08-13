#!/usr/bin/env python3

import json
import types
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from hnuter_external_direct_drcda_hardware import (
    ANGLE_COUNT,
    DRCDAAllocator,
    DRCDAConfig,
    HnuterHardwareDRCDAController,
    HnuterHardwareController,
    HnuterWrenchModel,
)


class HardwareDRCDAMotorMappingTest(unittest.TestCase):
    def test_front_motor_mapping_round_trip_uses_physical_thrust_limit(self):
        for thrust in (0.0, 5.0, 25.0, 50.0):
            control = (
                HnuterHardwareDRCDAController
                ._thrust_to_normalized_motor_control(thrust, 50.0)
            )
            recovered = HnuterHardwareDRCDAController._motor_control_to_thrust(
                control, 50.0
            )
            self.assertAlmostEqual(recovered, thrust, places=9)

    def test_tail_bidirectional_mapping_round_trip(self):
        for thrust in (-50.0, -12.5, 0.0, 12.5, 50.0):
            control = (
                HnuterHardwareDRCDAController
                ._thrust_to_normalized_bidirectional_motor_control(
                    thrust, 50.0
                )
            )
            recovered = HnuterHardwareDRCDAController._motor_control_to_thrust(
                control, 50.0, bidirectional=True
            )
            self.assertAlmostEqual(recovered, thrust, places=9)


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


class HardwareDRCDAConfigurationTest(unittest.TestCase):
    def test_config_uses_current_pwm_profile(self):
        path = (
            Path(__file__).resolve().parents[1]
            / 'config'
            / 'hnuter_drcda_hardware_tuning.json'
        )
        config = json.loads(path.read_text(encoding='utf-8'))
        self.assertEqual(
            config['hardware_firmware_profile'],
            '3131ddd4_500_2500_gear2',
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
