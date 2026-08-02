#!/usr/bin/env python3

import unittest

import numpy as np

from hnuter_drcda import (
    ACTUATOR_COUNT,
    ANGLE_COUNT,
    BasicDifferentialAllocator,
    DECISION_COUNT,
    DRCDAAllocator,
    DRCDAConfig,
    HnuterWrenchModel,
    PaperDifferentialAllocator,
    ServoPredictor,
    configure_allocator_variant,
)


class HnuterWrenchModelTest(unittest.TestCase):
    def test_analytic_jacobian_matches_central_difference(self):
        model = HnuterWrenchModel()
        rng = np.random.default_rng(7)
        for _ in range(12):
            q = np.concatenate((
                rng.uniform(-1.2, 1.2, 4),
                rng.uniform(1.0, 24.0, 4),
                rng.uniform(-8.0, 8.0, 1),
            ))
            self.assertLess(model.jacobian_error(q), 1e-6)


class ServoPredictorTest(unittest.TestCase):
    def test_static_calibration_round_trip(self):
        config = DRCDAConfig()
        for index in range(4):
            predictor = ServoPredictor(index, config)
            for command in (-0.7, -0.2, 0.0, 0.3, 0.8):
                target = predictor.command_to_target(command)
                self.assertAlmostEqual(
                    predictor.target_to_command(target),
                    command,
                    places=12,
                )

    def test_motion_direction_selects_negative_dynamics_for_positive_target(self):
        config = DRCDAConfig()
        config.servo_gain_positive[:] = 1.0
        config.servo_gain_negative[:] = 1.0
        config.servo_delay_positive_s[:] = 0.0
        config.servo_delay_negative_s[:] = 0.0
        config.servo_tau_positive_s[:] = 0.20
        config.servo_tau_negative_s[:] = 0.02
        config.servo_rate_positive_rad_s[:] = 1.0e6
        config.servo_rate_negative_rad_s[:] = 1.0e6
        predictor = ServoPredictor(0, config)
        predictor.reset(theta=1.0, target_angle_rad=1.0)
        predictor.enqueue(0.3)
        predictor.advance(0.01)
        expected = 1.0 + (1.0 - np.exp(-0.01 / 0.02)) * (0.3 - 1.0)
        self.assertAlmostEqual(predictor.theta, expected, places=12)

    def test_delay_direction_uses_command_change_not_absolute_sign(self):
        config = DRCDAConfig()
        config.servo_gain_positive[:] = 1.0
        config.servo_gain_negative[:] = 1.0
        config.servo_delay_positive_s[:] = 0.20
        config.servo_delay_negative_s[:] = 0.05
        predictor = ServoPredictor(0, config)
        predictor.reset(theta=1.0, target_angle_rad=1.0)
        predictor.enqueue(0.3)
        predictor.advance(0.049)
        self.assertAlmostEqual(predictor.active_target_angle_rad, 1.0)
        predictor.advance(0.002)
        self.assertAlmostEqual(predictor.active_target_angle_rad, 0.3)

    def test_midstep_activation_is_propagated_without_grid_delay(self):
        config = DRCDAConfig()
        config.servo_gain_positive[:] = 1.0
        config.servo_gain_negative[:] = 1.0
        config.servo_delay_positive_s[:] = 0.007
        config.servo_delay_negative_s[:] = 0.007
        config.servo_tau_positive_s[:] = 0.10
        config.servo_tau_negative_s[:] = 0.10
        config.servo_rate_positive_rad_s[:] = 1.0e6
        config.servo_rate_negative_rad_s[:] = 1.0e6
        predictor = ServoPredictor(0, config)
        predictor.reset()
        predictor.enqueue(1.0)
        predictor.advance(0.02)
        expected = 1.0 - np.exp(-0.013 / 0.10)
        self.assertAlmostEqual(predictor.theta, expected, places=12)


class DRCDAAllocatorTest(unittest.TestCase):
    def test_default_servo_model_has_no_dead_time_or_first_order_lag(self):
        config = DRCDAConfig()
        np.testing.assert_array_equal(config.servo_delay_positive_s, 0.0)
        np.testing.assert_array_equal(config.servo_delay_negative_s, 0.0)
        np.testing.assert_array_equal(config.servo_tau_positive_s, 0.0)
        np.testing.assert_array_equal(config.servo_tau_negative_s, 0.0)
        self.assertAlmostEqual(config.horizon_s, 0.18)

    def test_servo_is_unreachable_inside_pure_delay(self):
        config = DRCDAConfig(horizon_s=0.08)
        config.servo_delay_positive_s[:] = 0.20
        config.servo_delay_negative_s[:] = 0.20
        allocator = DRCDAAllocator(HnuterWrenchModel(), config)
        command = np.array([0.3, 0.3, -0.3, -0.3, 8.0, 8.0, 8.0, 8.0, 0.0])
        state, sensitivity = allocator.predict_terminal(command)
        np.testing.assert_allclose(state[:4], 0.0, atol=1e-12)
        np.testing.assert_allclose(np.diag(sensitivity)[:4], 0.0, atol=1e-12)
        self.assertTrue(np.all(np.diag(sensitivity)[4:] > 0.9))

    def test_prediction_sensitivity_matches_finite_difference(self):
        allocator = DRCDAAllocator(HnuterWrenchModel(), DRCDAConfig())
        command = np.array([0.25, 0.22, -0.20, -0.18, 8.0, 9.0, 10.0, 11.0, 2.0])
        _, sensitivity = allocator.predict_terminal(command)
        numerical = np.zeros((ACTUATOR_COUNT, ACTUATOR_COUNT))
        epsilon = 1e-6
        for index in range(ACTUATOR_COUNT):
            offset = np.zeros(ACTUATOR_COUNT)
            offset[index] = epsilon
            plus, _ = allocator.predict_terminal(command + offset)
            minus, _ = allocator.predict_terminal(command - offset)
            numerical[:, index] = (plus - minus) / (2.0 * epsilon)
        np.testing.assert_allclose(sensitivity, numerical, atol=2e-5, rtol=2e-4)

    def test_hover_and_coupled_wrench_converge(self):
        config = DRCDAConfig(gauss_newton_iterations=2)
        allocator = DRCDAAllocator(HnuterWrenchModel(), config)
        preferred = np.array([0.0, 0.0, 0.0, 0.0, 11.0, 11.0, 11.0, 11.0, 0.0])
        hover = np.array([0.0, 0.0, 44.145, 0.0, 0.0, 0.0])

        for _ in range(120):
            result = allocator.allocate(hover, 0.01, preferred)
        self.assertEqual(result.status, 'solved')
        self.assertLess(
            np.linalg.norm(result.wrench_residual / config.wrench_scale),
            0.005,
        )

        target = hover + np.array([8.0, -4.0, 0.0, 0.5, -0.4, 0.6])
        for _ in range(120):
            result = allocator.allocate(target, 0.01, preferred)
        self.assertEqual(result.status, 'solved')
        self.assertLess(
            np.linalg.norm(result.wrench_residual / config.wrench_scale),
            0.005,
        )

    def test_ablation_variants_change_only_the_intended_reachability_term(self):
        no_delay = configure_allocator_variant(DRCDAConfig(), 'no_delay')
        np.testing.assert_array_equal(no_delay.servo_delay_positive_s, 0.0)
        np.testing.assert_array_equal(no_delay.servo_delay_negative_s, 0.0)

        no_horizon = configure_allocator_variant(DRCDAConfig(), 'no_horizon')
        self.assertEqual(no_horizon.horizon_s, no_horizon.prediction_dt_s)
        np.testing.assert_array_equal(
            no_horizon.servo_delay_positive_s, 0.0
        )
        np.testing.assert_array_equal(
            no_horizon.servo_delay_negative_s, 0.0
        )

        no_physical_rate = configure_allocator_variant(
            DRCDAConfig(), 'no_physical_rate'
        )
        self.assertTrue(
            np.all(no_physical_rate.servo_rate_positive_rad_s == 1.0e6)
        )
        self.assertTrue(
            np.all(no_physical_rate.servo_command_rate_rad_s < 1.0e6)
        )
        self.assertEqual(no_physical_rate.motor_force_rate_cap_n_s, 1.0e6)

        no_command_slew = configure_allocator_variant(
            DRCDAConfig(), 'no_command_slew'
        )
        self.assertTrue(
            np.all(no_command_slew.servo_command_rate_rad_s == 1.0e6)
        )
        self.assertTrue(
            np.all(no_command_slew.servo_rate_positive_rad_s < 1.0e6)
        )

        no_gate = configure_allocator_variant(
            DRCDAConfig(), 'no_reachability_gate'
        )
        self.assertEqual(no_gate.reachability_gate_threshold, 0.0)

        no_multirate = configure_allocator_variant(
            DRCDAConfig(), 'no_multirate'
        )
        self.assertFalse(no_multirate.multirate_enabled)

    def test_reachability_gate_freezes_servo_inside_pure_delay(self):
        config = configure_allocator_variant(DRCDAConfig(), 'no_horizon')
        config.servo_delay_positive_s[:] = 0.20
        config.servo_delay_negative_s[:] = 0.20
        allocator = DRCDAAllocator(HnuterWrenchModel(), config)
        desired = np.array([10.0, -5.0, 44.0, 0.4, -0.3, 0.5])
        preferred = np.array([
            0.5, 0.4, -0.5, -0.4,
            10.0, 10.0, 10.0, 10.0, 4.0,
        ])
        result = allocator.allocate(desired, 0.01, preferred)
        np.testing.assert_allclose(result.command[:4], 0.0, atol=1e-12)
        np.testing.assert_array_equal(result.servo_gated, True)

    def test_default_nonuniform_grid_covers_dynamic_horizon(self):
        allocator = DRCDAAllocator(HnuterWrenchModel(), DRCDAConfig())
        steps = allocator._build_prediction_steps()
        self.assertGreater(steps.size, 4)
        self.assertAlmostEqual(float(np.sum(steps)), allocator.config.horizon_s)
        self.assertTrue(np.all(steps[:4] <= 0.01 + 1e-12))
        self.assertTrue(np.all(steps[4:] <= 0.02 + 1e-12))

    def test_multirate_trajectory_sensitivity_matches_finite_difference(self):
        config = DRCDAConfig.ideal_servos(
            horizon_s=0.12,
            motor_block_switch_s=0.06,
        )
        allocator = DRCDAAllocator(HnuterWrenchModel(), config)
        decision = np.array([
            0.10, -0.08, 0.06, -0.04,
            8.0, 9.0, 10.0, 11.0, 2.0,
            9.0, 10.0, 11.0, 12.0, 3.0,
        ])
        self.assertEqual(decision.size, DECISION_COUNT)
        limits = config.servo_state_limit_rad
        trajectory = allocator._predict_trajectory(decision, limits)
        analytical = trajectory.state_sensitivities[-1]
        numerical = np.zeros_like(analytical)
        epsilon = 1e-6
        for index in range(DECISION_COUNT):
            offset = np.zeros(DECISION_COUNT)
            offset[index] = epsilon
            plus = allocator._predict_trajectory(
                decision + offset, limits
            ).states[-1]
            minus = allocator._predict_trajectory(
                decision - offset, limits
            ).states[-1]
            numerical[:, index] = (plus - minus) / (2.0 * epsilon)
        np.testing.assert_allclose(
            analytical, numerical, atol=2e-5, rtol=2e-4
        )

        early_index = allocator._trajectory_index(trajectory, 0.04)
        early_sensitivity = trajectory.state_sensitivities[early_index]
        np.testing.assert_allclose(
            early_sensitivity[4:, 9:], 0.0, atol=1e-12
        )
        self.assertTrue(np.all(np.diag(early_sensitivity[4:, 4:9]) > 0.9))

    def test_no_multirate_keeps_late_motor_block_equal_to_fast_block(self):
        config = configure_allocator_variant(DRCDAConfig(), 'no_multirate')
        allocator = DRCDAAllocator(HnuterWrenchModel(), config)
        desired = np.array([3.0, -2.0, 44.0, 0.4, -0.3, 0.2])
        result = allocator.allocate(desired, 0.01)
        np.testing.assert_allclose(
            result.late_thrust_command,
            result.command[4:],
            atol=1e-12,
        )
        self.assertTrue(np.isfinite(result.objective))

    def test_basic_differential_allocator_tracks_hover(self):
        config = DRCDAConfig()
        allocator = BasicDifferentialAllocator(HnuterWrenchModel(), config)
        preferred = np.array([0.0, 0.0, 0.0, 0.0, 11.0, 11.0, 11.0, 11.0, 0.0])
        hover = np.array([0.0, 0.0, 44.145, 0.0, 0.0, 0.0])

        for _ in range(160):
            result = allocator.allocate(hover, 0.01, preferred)
        self.assertEqual(result.status, 'basic_da')
        self.assertLess(
            np.linalg.norm(result.wrench_residual / config.wrench_scale),
            0.01,
        )


class PaperDifferentialAllocatorTest(unittest.TestCase):
    def test_all_three_paper_stages_converge_to_hover(self):
        hover = np.array([0.0, 0.0, 44.145, 0.0, 0.0, 0.0])
        for method in PaperDifferentialAllocator.METHODS:
            config = DRCDAConfig()
            allocator = PaperDifferentialAllocator(
                HnuterWrenchModel(), config, method
            )
            for _ in range(600):
                result = allocator.allocate(hover, 0.01)
            self.assertEqual(result.status, f'paper_{method}')
            self.assertLess(
                np.linalg.norm(
                    (result.estimated_wrench - hover)
                    / config.wrench_scale
                ),
                0.005,
            )

    def test_wrench_augmentation_matches_paper_equation_7(self):
        config = DRCDAConfig(wrench_error_gain=7.0)
        allocator = PaperDifferentialAllocator(
            HnuterWrenchModel(), config, 'pda'
        )
        desired = np.array([2.0, -1.0, 44.145, 0.2, -0.1, 0.3])
        result = allocator.allocate(desired, 0.01)
        np.testing.assert_allclose(
            result.jerk_reference,
            config.wrench_error_gain
            * (desired - result.estimated_wrench),
            atol=1e-12,
        )

    def test_pda_rate_midpoint_drives_front_rotors_to_equilibrium(self):
        config = DRCDAConfig()
        allocator = PaperDifferentialAllocator(
            HnuterWrenchModel(), config, 'pda'
        )
        allocator.state[ANGLE_COUNT:ANGLE_COUNT + 4] = (
            0.5 * config.motor_trim_n[:4]
        )
        lower, upper = allocator._power_rate_bounds(
            0.01, config.servo_state_limit_rad
        )
        self.assertTrue(np.all(
            0.5 * (
                lower[ANGLE_COUNT:ANGLE_COUNT + 4]
                + upper[ANGLE_COUNT:ANGLE_COUNT + 4]
            ) > 0.0
        ))

        allocator.state[ANGLE_COUNT:ANGLE_COUNT + 4] = (
            0.5
            * (
                config.motor_trim_n[:4]
                + config.thrust_max_n[:4]
            )
        )
        lower, upper = allocator._power_rate_bounds(
            0.01, config.servo_state_limit_rad
        )
        self.assertTrue(np.all(
            0.5 * (
                lower[ANGLE_COUNT:ANGLE_COUNT + 4]
                + upper[ANGLE_COUNT:ANGLE_COUNT + 4]
            ) < 0.0
        ))


if __name__ == '__main__':
    unittest.main()
