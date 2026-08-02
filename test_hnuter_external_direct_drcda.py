#!/usr/bin/env python3

import types
import unittest
from unittest.mock import patch

import numpy as np

from hnuter_external_direct_controller_debug import GamepadManager
from hnuter_external_direct_drcda import HnuterDRCDAController


class GamepadCommandShapingTest(unittest.TestCase):
    def test_gamepad_lateral_stick_uses_y_axis_shaping(self):
        fake_pygame = types.SimpleNamespace(
            init=lambda: None,
            quit=lambda: None,
            event=types.SimpleNamespace(pump=lambda: None),
            joystick=types.SimpleNamespace(
                init=lambda: None,
                get_count=lambda: 0,
            ),
        )
        fake_joystick = types.SimpleNamespace(
            get_numaxes=lambda: 6,
            get_axis=lambda axis: 1.0 if axis == 3 else -1.0,
        )
        with patch(
            'hnuter_external_direct_controller_debug.pygame',
            fake_pygame,
        ):
            gamepad = GamepadManager(
                max_vxy_body_mps=[0.5, 0.4],
                filter_tau_body_xy_s=[0.2, 0.45],
                max_acc_body_xy_mps2=[1.0, 0.55],
            )
            gamepad.joystick = fake_joystick
            gamepad.filtered_cmds['vy_b'] = 0.4
            commands = gamepad.get_velocity_commands(0.1)

        self.assertAlmostEqual(commands['raw_vy_b'], -0.4)
        self.assertAlmostEqual(commands['vy_b'], 0.345)

    def test_lateral_reversal_respects_configured_acceleration_limit(self):
        filtered = GamepadManager._filter_command(
            current=0.4,
            target=-0.4,
            dt=0.1,
            filter_tau_s=0.45,
            max_rate=0.55,
        )
        self.assertAlmostEqual(filtered, 0.345)

    def test_filter_is_backward_compatible_without_rate_limit(self):
        filtered = GamepadManager._filter_command(
            current=0.0,
            target=1.0,
            dt=0.1,
            filter_tau_s=0.2,
        )
        self.assertAlmostEqual(filtered, 1.0 / 3.0)


class HnuterDRCDAPositionLoopTest(unittest.TestCase):
    def test_body_frame_position_gains_are_applied_by_drcda_controller(self):
        controller = types.SimpleNamespace(
            direct_pos_body_frame_xy_enabled=True,
            direct_pos_Kp_body_xy=np.array([4.0, 1.0]),
            direct_pos_Kd_body_xy=np.zeros(2),
            direct_pos_Ki_body_xy=np.zeros(2),
            direct_pos_Kp_ned=np.array([9.0, 9.0, 3.0]),
            direct_pos_Kd_ned=np.array([9.0, 9.0, 2.0]),
            direct_pos_Ki_ned=np.array([9.0, 9.0, 0.5]),
            integral_pos_error=np.zeros(3),
            R_ned_frd=np.array([
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ]),
        )
        acceleration = HnuterDRCDAController._direct_position_acceleration_ned(
            controller,
            np.zeros(3),
            np.array([1.0, 1.0, 2.0]),
            np.zeros(3),
            False,
        )
        np.testing.assert_allclose(acceleration, [1.0, 4.0, 6.0], atol=1e-12)

    def test_antiwindup_uses_body_axis_integral_gain(self):
        controller = types.SimpleNamespace(
            drcda=types.SimpleNamespace(
                config=types.SimpleNamespace(
                    wrench_scale=np.ones(6),
                    antiwindup_gain=0.5,
                )
            ),
            allocator_force_x_sign=1.0,
            allocator_force_y_sign=1.0,
            mass=1.0,
            direct_pos_body_frame_xy_enabled=True,
            direct_pos_Ki_body_xy=np.array([0.5, 0.25]),
            direct_pos_Ki_ned=np.array([8.0, 9.0, 1.0]),
            direct_pos_integral_limit_ned=np.full(3, 100.0),
            integral_pos_error=np.zeros(3),
            R_ned_frd=np.array([
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ]),
        )
        HnuterDRCDAController._apply_drcda_antiwindup(
            controller,
            np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0]),
            0.1,
        )
        np.testing.assert_allclose(
            controller.integral_pos_error,
            [0.2, 0.0, 0.0],
            atol=1e-12,
        )

    def test_full_allocator_switch_transfers_estimated_state(self):
        calls = []
        paper = types.SimpleNamespace(
            state=np.arange(9, dtype=float),
        )
        reachable = types.SimpleNamespace(
            reset=lambda **kwargs: calls.append(kwargs),
        )
        controller = types.SimpleNamespace(
            _drcda_variant='full',
            auto_traj_mode='attitude',
            _drcda_paper_allocator=paper,
            _drcda_large_attitude_allocator=reachable,
            drcda=paper,
            _drcda_accumulated_dt_s=1.0,
            get_logger=lambda: types.SimpleNamespace(info=lambda message: None),
        )
        HnuterDRCDAController._select_full_allocator(controller)
        self.assertIs(controller.drcda, reachable)
        np.testing.assert_array_equal(
            calls[0]['angle_state'], np.arange(4, dtype=float)
        )
        np.testing.assert_array_equal(
            calls[0]['thrust_state'], np.arange(4, 9, dtype=float)
        )
        self.assertEqual(controller._drcda_accumulated_dt_s, 0.0)

    def test_body_axis_acceleration_limit_is_directional(self):
        controller = types.SimpleNamespace(
            direct_pos_body_frame_xy_enabled=True,
            direct_manual_max_acc_body_xy=np.array([3.0, 1.5]),
            auto_traj_mode='hover',
            R_ned_frd=np.array([
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ]),
        )
        limited = HnuterDRCDAController._limit_direct_horizontal_acceleration_ned(
            controller,
            np.array([-4.0, 4.0]),
            5.0,
        )
        np.testing.assert_allclose(limited, [-1.5, 3.0], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
