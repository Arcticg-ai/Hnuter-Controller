import numpy as np
from pathlib import Path

from controllers.experiments.fixed_attitude_switching.controller import (
    FixedAttitudeSwitchingMixin,
    minimum_jerk,
)


def test_minimum_jerk_endpoints_are_stationary():
    start = np.array([0.0, -1.2, 1.5])
    finish = np.array([0.0, 1.2, 1.5])
    p0, v0, a0 = minimum_jerk(start, finish, 0.0, 1.0)
    p1, v1, a1 = minimum_jerk(start, finish, 1.0, 1.0)
    np.testing.assert_allclose(p0, start)
    np.testing.assert_allclose(p1, finish)
    np.testing.assert_allclose(v0, 0.0)
    np.testing.assert_allclose(v1, 0.0, atol=1e-12)
    np.testing.assert_allclose(a0, 0.0)
    np.testing.assert_allclose(a1, 0.0, atol=1e-12)


def test_switch_reference_runs_a_to_b_to_a_to_c():
    controller = object.__new__(FixedAttitudeSwitchingMixin)
    controller.switch_ingress_s = 2.5
    controller.switch_settle_s = 1.5
    controller.switch_duration_s = 1.0
    controller.switch_dwell_s = 0.5
    controller._switch_start = np.array([0.0, 0.0, 1.5])
    controller._switch_a = np.array([0.0, 0.0, 1.5])
    controller._switch_b = np.array([0.0, 1.2, 1.5])
    controller._switch_c = np.array([0.0, -1.2, 1.5])
    p, _, _, phase, done = controller._switch_reference(4.5)
    np.testing.assert_allclose(p, [0.0, 0.6, 1.5])
    assert phase == 'a_to_b'
    assert not done
    p, _, _, phase, done = controller._switch_reference(6.0)
    np.testing.assert_allclose(p, [0.0, 0.6, 1.5])
    assert phase == 'b_to_a'
    assert not done
    p, _, _, phase, done = controller._switch_reference(7.5)
    np.testing.assert_allclose(p, [0.0, -0.6, 1.5])
    assert phase == 'a_to_c'
    assert not done
    p, v, a, phase, done = controller._switch_reference(8.0)
    np.testing.assert_allclose(p, controller._switch_c)
    np.testing.assert_allclose(v, 0.0)
    np.testing.assert_allclose(a, 0.0)
    assert phase == 'complete'
    assert done


def test_gate_asset_contains_physical_collisions():
    gate = Path(__file__).parents[2] / (
        'controllers/experiments/fixed_attitude_switching/gate.sdf'
    )
    text = gate.read_text(encoding='utf-8')
    assert text.count('<collision name=') == 3
    assert 'left_wall_collision' in text
    assert 'right_wall_collision' in text
    assert 'top_bar_collision' in text
