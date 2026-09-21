import math
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from controllers.common.hnuter_drcda import DRCDAConfig, HnuterWrenchModel
from controllers.experiments.drcda_closed_loop.feedback import JointAngleFeedback
from controllers.experiments.drcda_closed_loop.governor import (
    MeasuredDRCDAAllocator,
    ReachableWrenchContract,
)


def _pose(angle=0.0):
    return SimpleNamespace(orientation=SimpleNamespace(
        w=math.cos(angle / 2), x=0.0, y=math.sin(angle / 2), z=0.0
    ))


def _pose_quat(q):
    return SimpleNamespace(orientation=SimpleNamespace(
        w=q[0], x=q[1], y=q[2], z=q[3],
    ))


def _multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw-ax*bx-ay*by-az*bz,
        aw*bx+ax*bw+ay*bz-az*by,
        aw*by-ax*bz+ay*bw+az*bx,
        aw*bz+ax*by-ay*bx+az*bw,
    ])


def test_joint_feedback_extracts_primary_and_relative_secondary_angles():
    zero = {name: _pose() for name in ('base_link', 'l2', 'l1', 'r2', 'r1')}
    reference = JointAngleFeedback.relative_zero(zero)
    posed = dict(zero)
    posed['l2'] = _pose(0.3)
    posed['l1'] = _pose(0.5)
    posed['r2'] = _pose(-0.2)
    posed['r1'] = _pose(-0.1)
    np.testing.assert_allclose(
        JointAngleFeedback.angles_from_poses(posed, reference),
        [0.3, 0.2, -0.2, 0.1], atol=1e-12,
    )


def test_joint_feedback_handles_nonidentity_joint_zero_orientation():
    q0 = np.array([math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0])
    qyaw = np.array([math.cos(0.4 / 2), 0.0, math.sin(0.4 / 2), 0.0])
    zero = {name: _pose_quat(q0) for name in
            ('l2', 'l1', 'r2', 'r1')}
    zero['base_link'] = _pose()
    reference = JointAngleFeedback.relative_zero(zero)
    moved = dict(zero)
    moved['l2'] = _pose_quat(_multiply(qyaw, q0))
    moved['l1'] = _pose_quat(_multiply(qyaw, q0))
    np.testing.assert_allclose(
        JointAngleFeedback.angles_from_poses(moved, reference),
        [0.4, 0.0, 0.0, 0.0], atol=1e-12,
    )


def test_reachable_contract_has_a_rate_limited_witness_and_measured_yaw_origin():
    config = DRCDAConfig.identified_gain_no_delay(horizon_s=0.1)
    allocator = MeasuredDRCDAAllocator(HnuterWrenchModel(), config)
    allocator.reset(thrust_state=[8, 8, 8, 8, 3])
    previous = allocator.command.copy()
    raw = np.array([40, -40, 55, 2, -2, 8], dtype=float)
    result = allocator.allocate(raw, 0.01, active_angle_limits=config.servo_state_limit_rad)
    contract = ReachableWrenchContract.from_allocation(
        allocator, result, previous, 0.01, config.servo_state_limit_rad,
        0.4, 0.3264,
    )
    projected = allocator._project_command(
        contract.command, previous, 0.01, config.servo_state_limit_rad
    )
    np.testing.assert_allclose(contract.command, projected)
    np.testing.assert_allclose(contract.reachable_wrench, result.predicted_wrench)
    np.testing.assert_allclose(contract.reachable_rate,
        (result.predicted_wrench - result.estimated_wrench) / config.horizon_s)
    assert math.isclose(contract.predicted_yaw_rate_frd,
        0.4 - config.horizon_s * result.predicted_wrench[5] / 0.3264)


def test_measured_angles_override_prediction_only_when_available():
    allocator = MeasuredDRCDAAllocator(
        HnuterWrenchModel(), DRCDAConfig.identified_gain_no_delay()
    )
    allocator.reset(angle_state=[0, 0, 0, 0], thrust_state=[8, 8, 8, 8, 3])
    allocator.measured_angles = np.array([0.1, -0.2, 0.3, -0.4])
    allocator._advance_state(0.01)
    np.testing.assert_allclose(allocator.state[:4], allocator.measured_angles)


def test_reachable_contract_rejects_out_of_box_command_and_false_wrench():
    config = DRCDAConfig.identified_gain_no_delay(horizon_s=0.1)
    allocator = MeasuredDRCDAAllocator(HnuterWrenchModel(), config)
    previous = allocator.command.copy()
    result = allocator.allocate(
        [0, 0, 35, 0, 0, 0], 0.01,
        active_angle_limits=config.servo_state_limit_rad,
    )
    args = (previous, 0.01, config.servo_state_limit_rad, 0.0, 0.3264)
    bad_command = result.command.copy()
    bad_command[0] += 10.0
    with pytest.raises(ValueError, match='rate box'):
        ReachableWrenchContract.from_allocation(
            allocator, replace(result, command=bad_command), *args
        )
    bad_wrench = result.predicted_wrench.copy()
    bad_wrench[0] += 10.0
    with pytest.raises(ValueError, match='model-state witness'):
        ReachableWrenchContract.from_allocation(
            allocator, replace(result, predicted_wrench=bad_wrench), *args
        )
