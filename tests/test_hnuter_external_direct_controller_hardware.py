#!/usr/bin/env python3

import types
import unittest

import numpy as np

from px4_msgs.msg import ManualControlSetpoint, RcChannels

from hnuter_external_direct_controller_hardware import (
    HnuterHardwareController,
    OffboardTaskRestartTracker,
    RCCommandManager,
)


def manager() -> RCCommandManager:
    return RCCommandManager(
        max_vxy_body_mps=[2.0, 1.0],
        max_vz=0.3,
        max_yaw_rate=0.4,
        deadzone=0.0,
        expo=0.0,
        filter_tau=0.0,
        filter_tau_body_xy_s=[0.0, 0.0],
        max_acc_body_xy_mps2=[100.0, 100.0],
        timeout_s=0.5,
    )


class RCCommandManagerTest(unittest.TestCase):
    def test_manual_control_maps_to_gamepad_reference_convention(self):
        rc = manager()
        rc.feed_manual_control(types.SimpleNamespace(
            valid=True,
            roll=0.5,
            pitch=0.25,
            yaw=0.2,
            throttle=0.4,
        ))
        command = rc.get_velocity_commands(0.1)
        self.assertEqual(rc.source, 'manual_control_setpoint')
        self.assertAlmostEqual(command['vx_b'], 0.5)
        self.assertAlmostEqual(command['vy_b'], -0.5)
        self.assertAlmostEqual(command['vz'], 0.12)
        self.assertAlmostEqual(command['yaw_rate'], -0.08)

    def test_non_rc_manual_control_source_is_rejected(self):
        rc = manager()
        rc.feed_manual_control(types.SimpleNamespace(
            valid=True,
            data_source=ManualControlSetpoint.SOURCE_MAVLINK_0,
            roll=0.5,
            pitch=0.25,
            yaw=0.2,
            throttle=0.4,
        ))

        command = rc.get_velocity_commands(0.1)

        self.assertEqual(rc.source, 'stale')
        self.assertAlmostEqual(command['vx_b'], 0.0)

    def test_rc_channels_are_used_as_fallback_and_throttle_is_recentered(self):
        rc = manager()
        function = [-1] * 30
        function[RcChannels.FUNCTION_THROTTLE] = 0
        function[RcChannels.FUNCTION_ROLL] = 1
        function[RcChannels.FUNCTION_PITCH] = 2
        function[RcChannels.FUNCTION_YAW] = 3
        channels = [0.75, -0.4, 0.3, -0.2] + [0.0] * 14
        rc.feed_rc_channels(types.SimpleNamespace(
            function=function,
            channels=channels,
            channel_count=4,
            signal_lost=False,
        ))
        command = rc.get_velocity_commands(0.1)
        self.assertEqual(rc.source, 'rc_channels')
        self.assertAlmostEqual(command['vx_b'], 0.6)
        self.assertAlmostEqual(command['vy_b'], 0.4)
        self.assertAlmostEqual(command['vz'], 0.15)
        self.assertAlmostEqual(command['yaw_rate'], 0.08)

    def test_lost_rc_channels_decay_to_zero_command(self):
        rc = manager()
        rc.filtered_cmds['vx_b'] = 1.0
        rc.feed_rc_channels(types.SimpleNamespace(
            function=[-1] * 30,
            channels=[0.0] * 18,
            channel_count=0,
            signal_lost=True,
        ))
        command = rc.get_velocity_commands(0.1)
        self.assertEqual(rc.source, 'stale')
        self.assertAlmostEqual(command['vx_b'], 0.0)
        self.assertFalse(rc.valid)


class OffboardTaskRestartTrackerTest(unittest.TestCase):
    def test_interrupted_task_restarts_after_offboard_reentry(self):
        tracker = OffboardTaskRestartTracker()
        tracker.observe(True, False, 'lissajous')
        tracker.observe(False, True, 'hover')
        self.assertEqual(tracker.consume(), 'lissajous')
        self.assertIsNone(tracker.consume())

    def test_hover_does_not_create_restart_task(self):
        tracker = OffboardTaskRestartTracker()
        tracker.observe(True, False, 'hover')
        tracker.observe(False, True, 'hover')
        self.assertIsNone(tracker.consume())


class HardwareControllerSafetyTest(unittest.TestCase):
    def test_vehicle_command_publisher_is_disabled_for_hardware(self):
        controller = types.SimpleNamespace()
        self.assertFalse(
            HnuterHardwareController._vehicle_command_publication_enabled(
                controller
            )
        )

    def test_startup_tick_only_sends_heartbeat_and_disarmed_idle(self):
        calls = []
        controller = types.SimpleNamespace(
            data_received=True,
            armed=False,
            _hardware_control_active=False,
            publish_offboard_control_mode=lambda: calls.append('heartbeat'),
            _update_hardware_control_gate=lambda: calls.append('gate'),
            publish_idle_direct_actuator_setpoint=lambda: calls.append('idle'),
        )

        HnuterHardwareController.offboard_startup_tick(controller)

        self.assertEqual(calls, ['heartbeat', 'gate', 'idle'])

    def test_arm_and_mode_helpers_do_not_need_a_vehicle_command_publisher(self):
        warnings = []
        logger = types.SimpleNamespace(warn=warnings.append)
        controller = types.SimpleNamespace(get_logger=lambda: logger)

        HnuterHardwareController.arm(controller)
        HnuterHardwareController.disarm(controller)
        HnuterHardwareController.set_offboard_mode(controller)

        self.assertEqual(len(warnings), 3)


class HardwareServoCalibrationTest(unittest.TestCase):
    def setUp(self):
        self.controller = types.SimpleNamespace(
            primary_servo_angle_max_rad=np.deg2rad(180.0),
            secondary_servo_angle_max_rad=np.deg2rad(180.0),
            secondary_servo_gear_ratio=2.0,
            servo_rate_limit_rad_s=6.0,
            servo_pwm_min_us=500,
            servo_pwm_trim_us=1500,
            servo_pwm_max_us=2500,
        )

    def test_primary_joint_uses_full_servo_shaft_range(self):
        convert = HnuterHardwareController._primary_joint_angle_to_normalized
        self.assertAlmostEqual(convert(self.controller, np.deg2rad(180.0)), 1.0)
        self.assertAlmostEqual(convert(self.controller, np.deg2rad(-90.0)), -0.5)

    def test_secondary_joint_applies_two_to_one_gear_ratio(self):
        convert = HnuterHardwareController._secondary_joint_angle_to_normalized
        self.assertAlmostEqual(convert(self.controller, np.deg2rad(90.0)), 1.0)
        self.assertAlmostEqual(convert(self.controller, np.deg2rad(45.0)), 0.5)
        self.assertAlmostEqual(convert(self.controller, np.deg2rad(-45.0)), -0.5)

    def test_secondary_joint_rate_is_reduced_by_gearing(self):
        rate = HnuterHardwareController._secondary_joint_rate_limit_rad_s(
            self.controller
        )
        self.assertAlmostEqual(rate, 3.0)

    def test_normalized_command_matches_firmware_pwm_calibration(self):
        convert = HnuterHardwareController._normalized_servo_to_expected_pwm_us
        self.assertAlmostEqual(convert(self.controller, -1.0), 500.0)
        self.assertAlmostEqual(convert(self.controller, -0.5), 1000.0)
        self.assertAlmostEqual(convert(self.controller, 0.0), 1500.0)
        self.assertAlmostEqual(convert(self.controller, 0.5), 2000.0)
        self.assertAlmostEqual(convert(self.controller, 1.0), 2500.0)

    def test_flown_e095_profile_keeps_legacy_primary_and_pwm_ranges(self):
        self.controller.primary_servo_angle_max_rad = np.deg2rad(185.0)
        self.controller.servo_pwm_min_us = 800
        self.controller.servo_pwm_max_us = 2200
        primary = HnuterHardwareController._primary_joint_angle_to_normalized
        pwm = HnuterHardwareController._normalized_servo_to_expected_pwm_us
        self.assertAlmostEqual(primary(self.controller, np.deg2rad(185.0)), 1.0)
        self.assertAlmostEqual(pwm(self.controller, -1.0), 800.0)
        self.assertAlmostEqual(pwm(self.controller, 1.0), 2200.0)


class HardwareIntegralSafetyTest(unittest.TestCase):
    def test_disabled_integral_axes_are_cleared(self):
        controller = types.SimpleNamespace(
            integral_e_R=np.array([0.6, -0.6, 0.4]),
            direct_attitude_integral_limit=np.array([0.6, 0.6, 0.4]),
            direct_attitude_integral_activation_error_rad=np.deg2rad(35.0),
        )

        previous = HnuterHardwareController._update_attitude_integral(
            controller,
            np.array([0.2, -0.1, 0.3]),
            np.deg2rad(10.0),
            np.zeros(3),
            0.1,
        )

        np.testing.assert_allclose(controller.integral_e_R, np.zeros(3))
        np.testing.assert_allclose(previous, np.zeros(3))

    def test_enabling_ki_does_not_apply_preexisting_integral(self):
        state = HnuterHardwareController._bumpless_integral_gain_change(
            np.array([0.6, -0.6, 0.4]),
            np.zeros(3),
            np.array([0.15, 0.18, 0.5]),
            np.array([0.6, 0.6, 0.4]),
        )

        np.testing.assert_allclose(state, np.zeros(3))

    def test_ki_retune_preserves_integral_torque(self):
        state = HnuterHardwareController._bumpless_integral_gain_change(
            np.array([0.2, -0.3, 0.1]),
            np.array([0.1, 0.2, 0.3]),
            np.array([0.2, 0.4, 0.6]),
            np.ones(3),
        )

        np.testing.assert_allclose(state, np.array([0.1, -0.15, 0.05]))

    def test_integral_update_that_pushes_saturation_is_rejected(self):
        controller = types.SimpleNamespace(
            integral_e_R=np.array([-0.1, 0.0, 0.0]),
        )

        rejected = HnuterHardwareController._reject_saturating_attitude_integration(
            controller,
            np.zeros(3),
            np.ones(3),
            np.array([1.2, 0.0, 0.0]),
            np.array([1.0, 1.0, 1.0]),
        )

        self.assertTrue(rejected)
        np.testing.assert_allclose(controller.integral_e_R, np.zeros(3))


if __name__ == '__main__':
    unittest.main()
