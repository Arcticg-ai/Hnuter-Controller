import math

import pytest

from hnuter_iebc_cube_contact_experiment_closed_loop import (
    ContactForceFilter,
    SustainedForceThreshold,
    smoothstep01,
    wrap_pi,
)


@pytest.mark.parametrize(
    ('u', 'position', 'slope'),
    [(-1.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.5, 0.5, 1.5),
     (1.0, 1.0, 0.0), (2.0, 1.0, 0.0)],
)
def test_smoothstep_is_bounded_and_stops_at_endpoints(u, position, slope):
    actual_position, actual_slope, _ = smoothstep01(u)
    assert actual_position == pytest.approx(position)
    assert actual_slope == pytest.approx(slope)


def test_wrap_pi_uses_shortest_signed_angle():
    assert wrap_pi(0.0) == pytest.approx(0.0)
    assert wrap_pi(2.0 * math.pi + 0.2) == pytest.approx(0.2)
    assert wrap_pi(-2.0 * math.pi - 0.2) == pytest.approx(-0.2)


def test_contact_filter_decays_after_sample_timeout():
    force_filter = ContactForceFilter(tau_s=0.08, timeout_s=0.15)
    force_filter.feed(4.0, received_s=10.0)

    assert force_filter.update(0.08, now_s=10.05) == pytest.approx(2.0)
    stale_value = force_filter.update(0.08, now_s=10.30)

    assert stale_value == pytest.approx(1.0)
    assert math.isfinite(stale_value)


def test_contact_filter_rejects_negative_force_magnitude():
    force_filter = ContactForceFilter(tau_s=0.0)
    force_filter.feed(-3.0, received_s=1.0)
    assert force_filter.update(0.01, now_s=1.0) == 0.0


def test_force_threshold_requires_continuous_filtered_force():
    latch = SustainedForceThreshold(threshold_n=10.0, hold_s=0.04)

    assert not latch.update(9.9, now_s=1.00)
    assert not latch.update(10.0, now_s=1.02)
    assert not latch.update(9.9, now_s=1.04)
    assert not latch.update(10.1, now_s=1.06)
    assert latch.update(10.1, now_s=1.10)


def test_force_threshold_reset_and_zero_hold():
    latch = SustainedForceThreshold(threshold_n=4.0, hold_s=0.0)
    assert latch.update(4.0, now_s=2.0)
    latch.reset()
    assert latch.since_s is None
    assert not latch.update(float('nan'), now_s=2.1)


def test_time_release_does_not_depend_on_contact_force():
    from hnuter_iebc_cube_contact_experiment_closed_loop import (
        HnuterIebcClosedLoopCubeContactExperiment,
    )

    node = object.__new__(HnuterIebcClosedLoopCubeContactExperiment)
    node.release_mode = 'time'
    node.release_time_s = 85.0
    node.force_release_latch = SustainedForceThreshold(54.0, 0.04)

    assert not node._should_release(84.99, 500.0, 100.0)
    assert node._should_release(85.0, 0.0, 100.02)


def test_force_release_mode_remains_backward_compatible():
    from hnuter_iebc_cube_contact_experiment_closed_loop import (
        HnuterIebcClosedLoopCubeContactExperiment,
    )

    node = object.__new__(HnuterIebcClosedLoopCubeContactExperiment)
    node.release_mode = 'force'
    node.release_time_s = 1.0
    node.force_release_latch = SustainedForceThreshold(54.0, 0.04)

    assert not node._should_release(100.0, 53.9, 1.0)
    assert not node._should_release(100.0, 54.1, 1.02)
    assert node._should_release(100.0, 54.1, 1.07)


def test_recovery_log_columns_are_present_and_aligned():
    from pathlib import Path

    source = Path(__file__).parents[1] / 'hnuter_iebc_cube_contact_experiment_closed_loop.py'
    text = source.read_text(encoding='utf-8')

    for column in (
            'iebc_mode', 'iebc_recoverable_energy_j',
            'iebc_release_excursion_m', 'iebc_stop_distance_barrier_m',
            'iebc_reserved_stop_distance_m', 'iebc_rho',
            'iebc_release_position_m',
            'iebc_recovery_dissipation_slack_w',
            'iebc_recovery_phase',
            'iebc_recovery_reference_velocity_mps',
            'iebc_recovery_rate_infeasible',
            'iebc_recovery_terminal_position_m',
            'iebc_recovery_stop_candidate_s',
            'iebc_recovery_stop_latched',
            'release_position_change_m',
            'release_settled',
            'release_settle_time_s',
            'iebc_recovery_rebase_energy_j'):
        assert column in text
