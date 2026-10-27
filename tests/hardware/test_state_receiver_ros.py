"""Opt-in DDS/executor integration; use a private ROS_DOMAIN_ID, never flight DDS."""
import os
import threading
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('HNUTER_RUN_ISOLATED_ROS_TEST') != '1', reason='isolated ROS integration only')


def test_receive_executor_updates_while_control_callback_is_blocked(monkeypatch, tmp_path):
    assert int(os.environ['ROS_DOMAIN_ID']) >= 100, 'Use a private test DDS domain'
    from controllers.hardware import hnuter_external_direct_drcda_v2_hardware as hw
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    for key in list(os.environ):
        if key.startswith('HNUTER_'):
            monkeypatch.delenv(key)
    monkeypatch.setenv('HNUTER_LOG_DIR', str(tmp_path))
    hw.rclpy.init()
    node = None
    sender = None
    control_executor = None
    control_thread = None
    release = threading.Event()
    entered = threading.Event()
    used_stamps = []
    try:
        node = hw.HnuterHardwareDRCDAV2Controller()
        assert not node._output_enabled
        assert node._state_receiver is not node
        sender = Node('freshness_test_sender')
        qos = hw.QoSProfile(depth=1, reliability=hw.ReliabilityPolicy.BEST_EFFORT)
        publishers = {
            'position': sender.create_publisher(hw.VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', qos),
            'rates': sender.create_publisher(hw.VehicleAngularVelocity, '/fmu/out/vehicle_angular_velocity', qos),
            'attitude': sender.create_publisher(hw.VehicleAttitude, '/fmu/out/vehicle_attitude', qos),
        }

        def publish_state():
            stamp = time.monotonic_ns() // 1000
            position = hw.VehicleLocalPosition()
            position.timestamp = stamp
            position.timestamp_sample = stamp
            position.xy_valid = position.z_valid = position.v_xy_valid = position.v_z_valid = True
            attitude = hw.VehicleAttitude()
            attitude.timestamp = attitude.timestamp_sample = stamp
            attitude.q = [1., 0., 0., 0.]
            rates = hw.VehicleAngularVelocity()
            rates.timestamp = rates.timestamp_sample = stamp
            for name, message in [('position', position), ('rates', rates), ('attitude', attitude)]:
                publishers[name].publish(message)
            return stamp

        def blocked_control():
            used_stamps.append(node._control_state_samples['attitude'][1])
            if len(used_stamps) == 1:
                entered.set()
                assert release.wait(3.)

        node.control_loop = blocked_control
        node.start_state_receiver()
        control_executor = SingleThreadedExecutor()
        control_executor.add_node(node)
        control_thread = threading.Thread(target=control_executor.spin, daemon=True)
        control_thread.start()
        deadline = time.monotonic() + 3.
        while not entered.is_set() and time.monotonic() < deadline:
            publish_state()
            entered.wait(.01)
        assert entered.is_set(), 'DDS discovery/control did not start'
        frozen = used_stamps[0]
        newest = publish_state()
        deadline = time.monotonic() + 1.
        while time.monotonic() < deadline:
            samples, _, _ = node._state_inbox.snapshot()
            if all(samples.get(name, (None, 0))[1] >= newest for name in publishers):
                break
            time.sleep(.001)
        else:
            pytest.fail('Receive executor stalled with control callback')
        assert node._control_state_samples['attitude'][1] == frozen
        assert len(used_stamps) == 1
        release.set()
        deadline = time.monotonic() + 1.
        while len(used_stamps) < 2 and time.monotonic() < deadline:
            time.sleep(.001)
        assert len(used_stamps) >= 2
        assert used_stamps[1] == newest, 'Controller replayed a historical sample'
        assert not node._hardware_control_active
    finally:
        release.set()
        if control_executor is not None:
            control_executor.shutdown()
        if control_thread is not None:
            control_thread.join(2.)
        if node is not None:
            node.destroy_node()
        if sender is not None:
            sender.destroy_node()
        if hw.rclpy.ok():
            hw.rclpy.shutdown()
