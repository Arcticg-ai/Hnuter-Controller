import csv
from types import SimpleNamespace

import numpy as np

from controllers.common.hnuter_drcda import bounded_quadratic_step, DRCDAConfig
from controllers.simulation.hnuter_external_direct_drcda import HnuterDRCDAController
from controllers.experiments.fixed_attitude_switching.controller import FixedAttitudeSwitchingMixin
from tools.experiments.run_fixed_attitude_switching import analyze_csv


def test_box_solver_redistributes_after_saturation():
    # Coupled task x+y=2, but x cannot exceed .2. Clipping the unconstrained
    # solution leaves y near 1 rather than reallocating to y near 1.8.
    h = np.ones((2, 2)) + 1e-3 * np.eye(2)
    g = np.full(2, -2.0)
    result = bounded_quadratic_step(h, g, np.zeros(2), np.array([.2, 3.]))
    assert abs(result[0] - .2) < 1e-8
    assert abs(result[1] - 1.8/1.001) < 1e-8


def test_gazebo_rates_do_not_inherit_legacy_hardware_limits():
    config = DRCDAConfig.gazebo_joint_pid_no_delay()
    np.testing.assert_array_equal(config.servo_rate_positive_rad_s, 10.)
    np.testing.assert_array_equal(config.servo_rate_negative_rad_s, 10.)
    np.testing.assert_array_equal(config.servo_command_rate_rad_s, 50.)


def test_box_solver_satisfies_kkt_with_correlated_variables():
    rng = np.random.default_rng(41)
    for _ in range(80):
        a = rng.normal(size=(15, 9))
        h = a.T @ a + .01 * np.eye(9)
        g = rng.normal(size=9) * 8
        low = -rng.uniform(.01, .4, 9)
        high = rng.uniform(.01, .4, 9)
        x = bounded_quadratic_step(h, g, low, high)
        grad = h @ x + g
        assert np.all(x >= low-1e-10) and np.all(x <= high+1e-10)
        free = (x > low+1e-8) & (x < high-1e-8)
        np.testing.assert_allclose(grad[free], 0, atol=1e-7)
        assert np.all(grad[x <= low+1e-8] >= -1e-7)
        assert np.all(grad[x >= high-1e-8] <= 1e-7)


def test_antiwindup_reduces_demand_when_force_is_insufficient():
    controller = object.__new__(HnuterDRCDAController)
    controller.drcda = SimpleNamespace(config=SimpleNamespace(
        wrench_scale=np.ones(6), antiwindup_gain=.2))
    controller.allocator_force_x_sign = 1.
    controller.allocator_force_y_sign = -1.
    controller.R_ned_frd = np.eye(3)
    controller.mass = 4.5
    controller.direct_pos_Ki_ned = np.ones(3)
    controller.direct_pos_integral_limit_ned = np.ones(3)*5
    controller.integral_pos_error = np.array([1., 0., 0.])
    controller._apply_drcda_antiwindup(np.array([-4.5, 0., 0., 0., 0., 0.]), .01)
    assert controller.integral_pos_error[0] < 1.


def test_uninitialized_diagnostic_does_not_duplicate_subclass_columns():
    class ExtraColumns(HnuterDRCDAController):
        def _diagnostic_extra_header(self):
            return super()._diagnostic_extra_header() + ['extra_1', 'extra_2']

        def _diagnostic_extra_values(self):
            return super()._diagnostic_extra_values() + [0., 0.]
    c = object.__new__(ExtraColumns)
    c._drcda_ready = False
    assert len(c._diagnostic_extra_values()) == len(c._diagnostic_extra_header())


def test_no_preview_and_no_horizon_are_isolated_reference_ablations():
    c = object.__new__(FixedAttitudeSwitchingMixin)
    c.last_W = np.arange(6, dtype=float)
    c.switch_prediction_horizons_s = (.05, .1)
    c.switch_prediction_weights = (.5, 1.)
    c.drcda = SimpleNamespace(config=SimpleNamespace(prediction_dt_s=.01))
    c._switch_method = 'no_preview'
    refs = c._future_drcda_wrench_references()
    assert [r[0] for r in refs] == [.05, .1]
    assert sum(r[2] for r in refs) == 1.5
    for _, wrench, _ in refs:
        np.testing.assert_array_equal(wrench, c.last_W)
    c._switch_method = 'no_horizon'
    assert c._future_drcda_wrench_references()[0][0] == .01
    assert c.drcda.config.horizon_s == .01


def test_measured_nda_preserves_a_stationary_model_hover():
    from controllers.common.hnuter_drcda import HnuterWrenchModel
    from controllers.experiments.fixed_attitude_switching.audit_allocator import MeasuredNDAAllocator
    a = MeasuredNDAAllocator(HnuterWrenchModel(), DRCDAConfig.gazebo_joint_pid_no_delay())
    a.reset(thrust_state=[10.,10.,10.,10.,6.])
    desired = a.model.wrench(a.state)
    for _ in range(30):
        result = a.allocate(desired,.012)
    np.testing.assert_allclose(result.estimated_wrench, desired, atol=1e-3)


def test_evaluation_detects_vertical_failure_in_final_hold(tmp_path):
    keys = ['switch_phase', 'position_x_enu_m', 'target_x_enu_m',
            'position_y_enu_m', 'target_y_enu_m', 'position_z_rel_m',
            'target_z_rel_m', 'attitude_error_angle_deg', 'roll_deg',
            'pitch_deg', 'angular_r_frd_rps']
    path = tmp_path / 'diagnostic.csv'
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        row = dict.fromkeys(keys, 0)
        row.update(switch_phase='post_hold_c', position_z_rel_m=.2, target_z_rel_m=1.5)
        writer.writerow(row)
    thresholds = dict(position_peak_m=.75, attitude_peak_deg=8.,
                      roll_pitch_peak_deg=5., minimum_altitude_m=.5,
                      gate_clearance_m=.05, feedback_ratio=.95)
    metrics = analyze_csv(path, thresholds)
    assert metrics['position_error_peak_m'] == 1.3
    assert not metrics['passed']
