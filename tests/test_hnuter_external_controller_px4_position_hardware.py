#!/usr/bin/env python3

import inspect
import types

import numpy as np

from px4_msgs.msg import RcChannels

import hnuter_external_controller_px4_position_hardware as hardware_module
from hnuter_external_controller_px4_position_hardware import (
    HnuterController,
    RCCommandManager,
)


def test_hardware_controller_has_no_vehicle_command_path():
    source = inspect.getsource(hardware_module)
    assert 'from px4_msgs.msg import VehicleCommand' not in source
    assert "'/fmu/in/vehicle_command'" not in source
    assert 'publish_vehicle_command' not in source


def test_position_controller_declares_current_firmware_profile():
    assert (
        HnuterController.HARDWARE_FIRMWARE_PROFILE
        == '3131ddd4_500_2500_gear2'
    )


def test_rc_manual_control_maps_to_velocity_references():
    rc = RCCommandManager()
    rc.deadzone = 0.0
    rc.expo = 0.0
    rc.filter_tau = 0.0
    rc.max_vxy = 2.0
    rc.max_vz = 0.5
    rc.max_yaw_rate = 1.0
    rc.pitch_sign = 1.0
    rc.roll_sign = -1.0
    rc.throttle_sign = 1.0
    rc.yaw_sign = -1.0
    rc.feed_manual_control(types.SimpleNamespace(
        valid=True,
        roll=0.5,
        pitch=0.25,
        yaw=0.2,
        throttle=0.4,
    ))

    command = rc.get_velocity_commands(0.1)

    assert rc.source == 'manual_control_setpoint'
    assert command['vx_b'] == 0.5
    assert command['vy_b'] == -1.0
    assert command['vz'] == 0.2
    assert command['yaw_rate'] == -0.2


def test_rc_channels_fallback_recenters_throttle():
    rc = RCCommandManager()
    rc.deadzone = 0.0
    rc.expo = 0.0
    rc.filter_tau = 0.0
    rc.max_vz = 0.5
    function = [-1] * 30
    function[RcChannels.FUNCTION_THROTTLE] = 0
    function[RcChannels.FUNCTION_ROLL] = 1
    function[RcChannels.FUNCTION_PITCH] = 2
    function[RcChannels.FUNCTION_YAW] = 3
    rc.feed_rc_channels(types.SimpleNamespace(
        function=function,
        channels=[0.75, 0.0, 0.0, 0.0] + [0.0] * 14,
        channel_count=4,
        signal_lost=False,
    ))

    command = rc.get_velocity_commands(0.1)

    assert rc.source == 'rc_channels'
    assert command['vz'] == 0.25


def test_zero_rc_command_does_not_create_automatic_climb():
    controller = types.SimpleNamespace(
        _z0_initialized=True,
        _z0=10.0,
        position=np.array([2.0, 3.0, 11.5]),
        _hardware_control_active=True,
        manual_pos_initialized=True,
        pending_auto_traj_mode=None,
        auto_traj_mode='hover',
        manual_enabled=True,
        rc_input=types.SimpleNamespace(
            get_velocity_commands=lambda dt: {
                'vx_b': 0.0,
                'vy_b': 0.0,
                'vz': 0.0,
                'yaw_rate': 0.0,
                'roll_rate': 0.0,
                'lt': 0.0,
                'rt': 0.0,
            }
        ),
        manual_des_pos=np.array([2.0, 3.0, 1.5]),
        manual_des_yaw=0.2,
        manual_des_roll=0.0,
        manual_roll_limit_rad=np.deg2rad(90.0),
        min_altitude=-5.0,
        max_altitude=5.0,
    )

    HnuterController.update_trajectory(controller, current_time=5.0, dt=0.1)

    assert controller.target_position[2] == 1.5
    assert controller.target_velocity[2] == 0.0


def test_trajectory_anchor_is_measured_position_at_trigger_time():
    messages = []
    controller = types.SimpleNamespace(
        position=np.array([12.0, -4.0, 8.0]),
        _z0=5.0,
        manual_des_yaw=0.3,
        auto_traj_mode='hover',
        auto_traj_start_time=0.0,
        auto_traj_yaw=0.0,
        auto_traj_start_attitude=np.zeros(3),
        auto_traj_start_pos=np.zeros(3),
        manual_des_pos=np.zeros(3),
        manual_des_roll=0.0,
        auto_traj_z=0.0,
        auto_traj_origin_xy=np.zeros(2),
        min_altitude=-5.0,
        max_altitude=5.0,
        lissajous_amp_x=1.0,
        lissajous_amp_y=0.75,
        _yaw_rotation_2d=lambda yaw: np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)],
        ]),
        get_logger=lambda: types.SimpleNamespace(info=messages.append),
    )

    HnuterController._start_auto_trajectory(
        controller, mode='rectangle', current_time=7.0
    )

    np.testing.assert_allclose(
        controller.auto_traj_start_pos,
        np.array([12.0, -4.0, 3.0]),
    )


def test_reentry_requeues_interrupted_task_from_new_session_origin():
    messages = []
    controller = types.SimpleNamespace(
        position=np.array([20.0, 6.0, 9.0]),
        px4_timestamp=8_000_000,
        initial_yaw=0.4,
        _hardware_control_active=False,
        _interrupted_task='lissajous',
        pending_auto_traj_mode=None,
        manual_pos_initialized=False,
        auto_traj_mode='hover',
        target_position=np.zeros(3),
        target_velocity=np.zeros(3),
        target_acceleration=np.zeros(3),
        target_attitude=np.zeros(3),
        target_attitude_rate=np.zeros(3),
        get_logger=lambda: types.SimpleNamespace(info=messages.append),
    )

    HnuterController._begin_hardware_control(controller)

    assert controller.pending_auto_traj_mode == 'lissajous'
    np.testing.assert_allclose(controller.manual_des_pos, [20.0, 6.0, 0.0])
    assert controller._z0 == 9.0
