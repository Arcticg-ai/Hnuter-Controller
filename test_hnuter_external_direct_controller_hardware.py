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


if __name__ == '__main__':
    unittest.main()
