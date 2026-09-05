import math

import numpy as np

from controllers.common.hnuter_drcda import DRCDAConfig, HnuterWrenchModel
from controllers.experiments.drcda_v2.allocator import (
    PaperNormalizedDifferentialAllocator,
    ReachabilityDRCDAAllocatorV2,
    uniform_saturate,
)


def test_uniform_saturate_preserves_direction():
    vector = np.array([2.0, -4.0, 1.0])
    saturated = uniform_saturate(vector)

    np.testing.assert_allclose(saturated, vector / 4.0)
    assert np.max(np.abs(saturated)) == 1.0


def test_paper_nda_respects_asymmetric_rate_and_position_limits():
    config = DRCDAConfig.ideal_servos()
    config.servo_command_rate_rad_s[:] = [0.5, 0.8, 0.5, 0.8]
    config.thrust_command_rate_n_s[:] = 10.0
    allocator = PaperNormalizedDifferentialAllocator(HnuterWrenchModel(), config)
    allocator.reset(
        angle_state=[math.pi - 0.002, 0.0, -math.pi + 0.002, 0.0],
        thrust_state=[10.0, 10.0, 10.0, 10.0, 0.0],
    )
    previous = allocator.command.copy()

    result = allocator.allocate([40.0, -30.0, 60.0, 3.0, -2.0, 1.0], 0.01)

    assert np.all(np.isfinite(result.command))
    assert np.all(result.command[:4] <= config.servo_state_limit_rad + 1e-12)
    assert np.all(result.command[:4] >= -config.servo_state_limit_rad - 1e-12)
    assert np.all(
        np.abs(result.command[:4] - previous[:4])
        <= config.servo_command_rate_rad_s * 0.01 + 1e-12
    )
    assert np.all(result.command[4:] <= config.thrust_max_n + 1e-12)
    assert np.all(result.command[4:] >= config.thrust_min_n - 1e-12)


def test_paper_nda_produces_finite_hover_solution():
    config = DRCDAConfig.ideal_servos(wrench_error_gain=6.0)
    allocator = PaperNormalizedDifferentialAllocator(HnuterWrenchModel(), config)
    allocator.reset(thrust_state=[8.0, 8.0, 8.0, 8.0, 0.0])

    result = allocator.allocate([0.0, 0.0, 42.0, 0.0, 0.0, 0.0], 0.01)

    assert result.status == 'paper_nda'
    assert np.all(np.isfinite(result.command))
    assert np.linalg.norm(result.wrench_residual) < 15.0


def test_paper_nda_jerk_is_eq7_error_feedback_without_feedforward():
    config = DRCDAConfig.ideal_servos(wrench_error_gain=4.0)
    allocator = PaperNormalizedDifferentialAllocator(HnuterWrenchModel(), config)
    allocator.reset(thrust_state=[5.0, 5.0, 5.0, 5.0, 0.0])
    desired = np.array([5.0, -3.0, 30.0, 1.0, -0.5, 0.2])

    result = allocator.allocate(desired, 0.01)

    np.testing.assert_allclose(
        result.jerk_reference,
        config.wrench_error_gain * (desired - result.estimated_wrench),
    )


def test_drcda_v2_update_is_bounded_and_finite():
    config = DRCDAConfig.identified_gain_no_delay(
        horizon_s=0.1,
        gauss_newton_iterations=2,
        wrench_error_gain=6.0,
    )
    allocator = ReachabilityDRCDAAllocatorV2(HnuterWrenchModel(), config)
    allocator.reset(thrust_state=[8.0, 8.0, 8.0, 8.0, 0.0])
    previous = allocator.command.copy()

    result = allocator.allocate([20.0, -15.0, 45.0, 2.0, -1.0, 0.5], 0.01)

    assert result.status in {'drcda_v2', 'drcda_v2_stalled'}
    assert np.all(np.isfinite(result.command))
    assert np.all(np.isfinite(result.predicted_wrench))
    assert np.all(
        np.abs(result.command[:4] - previous[:4])
        <= config.servo_command_rate_rad_s * 0.01 + 1e-12
    )
    assert np.all(
        np.abs(result.command[4:] - previous[4:])
        <= config.thrust_command_rate_n_s * 0.01 + 1e-12
    )
