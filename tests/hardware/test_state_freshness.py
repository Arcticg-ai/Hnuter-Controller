"""Offline regressions: all ROS publishers/subscriptions are mocked."""
import threading
from types import SimpleNamespace as Msg
from unittest.mock import Mock

import numpy as np
import pytest

from .test_drcda_v2_hardware import controller, activate, feed, hw


@pytest.fixture
def clock(monkeypatch):
    now = [100.]
    monkeypatch.setattr(hw.time, 'monotonic', lambda: now[0])
    return now


def messages(stamp, rate=0.):
    return {
        'position': Msg(timestamp=stamp, timestamp_sample=stamp, xy_valid=True, z_valid=True,
                        v_xy_valid=True, v_z_valid=True, x=2., y=3., z=-4., vx=0., vy=0., vz=0.),
        'rates': Msg(timestamp=stamp, timestamp_sample=stamp, xyz=[rate, 0., 0.]),
        'attitude': Msg(timestamp=stamp, timestamp_sample=stamp, q=[1., 0., 0., 0.]),
    }


def enqueue(inbox, stamp, rate=0.):
    for name, message in messages(stamp, rate).items():
        inbox.put(name, message)


def test_latest_only_bounded_inbox_during_control_work(clock):
    inbox = hw.LatestStateInbox()
    enqueue(inbox, 1_000_000)
    frozen, _, _ = inbox.snapshot()
    # The receive side runs while the caller holds a previously taken snapshot.
    done = threading.Event()

    def receive_burst():
        for stamp in range(1_010_000, 2_000_001, 10_000):
            enqueue(inbox, stamp, rate=stamp/1e6)
        done.set()

    thread = threading.Thread(target=receive_burst)
    thread.start()
    assert done.wait(2.)
    thread.join()
    latest, _, error = inbox.snapshot()
    assert not error and len(latest) == 3
    assert latest['rates'][1] == 2_000_000
    assert frozen['rates'][1] == 1_000_000


def test_duplicate_does_not_refresh_and_backwards_stamp_is_reported(clock):
    inbox = hw.LatestStateInbox()
    enqueue(inbox, 1_000_000)
    clock[0] += .1
    enqueue(inbox, 1_000_000)
    samples, offset, _ = inbox.snapshot()
    assert samples['rates'][3] == 100.
    assert 'stale source state' in inbox.reason(samples, offset, clock[0])
    inbox.put('rates', messages(900_000)['rates'])
    assert 'reset' in inbox.snapshot()[2]


def test_newly_received_but_delayed_source_is_stale(clock):
    inbox = hw.LatestStateInbox()
    enqueue(inbox, 1_000_000)
    clock[0] += .25
    enqueue(inbox, 1_010_000)
    samples, offset, _ = inbox.snapshot()
    assert samples['rates'][3] == clock[0]
    assert 'stale source state' in inbox.reason(samples, offset, clock[0])


def test_timestamp_sample_not_publication_time_drives_skew(clock):
    inbox = hw.LatestStateInbox()
    enqueue(inbox, 1_000_000)
    rate = messages(1_010_000)['rates']
    rate.timestamp_sample = 950_000
    inbox.put('rates', rate)
    samples, offset, _ = inbox.snapshot()
    assert inbox.reason(samples, offset, clock[0]) == 'state sample skew: rates/attitude'


def test_control_waits_for_matching_rate_and_consumes_only_latest(controller, clock):
    node = controller
    node.control_loop = Mock()
    enqueue(node._state_inbox, 1_000_000)
    node._state_control_tick()
    assert node.control_loop.call_count == 1
    clock[0] += .05
    new = messages(1_050_000, rate=.8)
    node._state_inbox.put('position', new['position'])
    node._state_inbox.put('attitude', new['attitude'])
    node._state_control_tick()
    assert node.control_loop.call_count == 1
    node._state_inbox.put('rates', new['rates'])
    node._state_control_tick()
    assert node.control_loop.call_count == 2
    np.testing.assert_allclose(node.angular_velocity_frd, [.8, 0., 0.])
    node._state_control_tick()
    assert node.control_loop.call_count == 2


def test_consumption_preserves_receive_time(controller, clock):
    enqueue(controller._state_inbox, 1_000_000)
    clock[0] += .04
    controller._state_control_tick()
    assert controller._received['rates'] == 100.
    assert controller._control_state_samples['rates'][3] == 100.


def test_expired_solve_result_never_publishes(controller, clock):
    activate(controller)
    controller.actuator_motors_pub.publish.reset_mock()
    controller.actuator_servos_pub.publish.reset_mock()
    clock[0] += .08  # Solver finished too late, even though callbacks were recent.
    controller._publish_normalized_direct_actuator_setpoint([.5]*5, [0.]*4)
    controller.actuator_motors_pub.publish.assert_not_called()
    controller.actuator_servos_pub.publish.assert_not_called()
    assert 'stale source state' in controller._hardware_fault
    controller._fault_position_pub.publish.assert_called()


def test_fresh_receiver_does_not_rejuvenate_inflight_snapshot(controller, clock):
    activate(controller)
    clock[0] += .08
    enqueue(controller._state_inbox, 1_080_000)
    assert 'stale source state' in controller._readiness_reason()


def test_receiver_failure_rejects_publication(controller, clock):
    activate(controller)
    controller._state_rx_failure = 'receiver stopped'
    controller.actuator_motors_pub.publish.reset_mock()
    controller._publish_normalized_direct_actuator_setpoint([.5]*5, [0.]*4)
    controller.actuator_motors_pub.publish.assert_not_called()
    assert 'receiver failed' in controller._hardware_fault


def test_subscription_is_on_receiver_with_depth_one(controller, monkeypatch):
    create = Mock()
    monkeypatch.setattr(controller._state_receiver, 'create_subscription', create)
    callback = Mock()
    controller.create_subscription(hw.VehicleAttitude, '/fmu/out/vehicle_attitude', callback, None)
    args = create.call_args.args
    assert args[3].depth == 1
    args[2](messages(1_000_000)['attitude'])
    callback.assert_not_called()
    assert controller._state_inbox.snapshot()[0]['attitude'][1] == 1_000_000
