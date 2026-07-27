#!/usr/bin/env python3

import unittest

import numpy as np

from hnuter_drcda import (
    ACTUATOR_COUNT,
    DRCDAAllocator,
    DRCDAConfig,
    HnuterWrenchModel,
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


class DRCDAAllocatorTest(unittest.TestCase):
    def test_servo_is_unreachable_inside_pure_delay(self):
        config = DRCDAConfig(horizon_s=0.08)
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


if __name__ == '__main__':
    unittest.main()
