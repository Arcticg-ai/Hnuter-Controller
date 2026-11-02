"""Offline mathematical and mocked-ROS tests; never publish to a flight controller."""
import copy
import json
import time
from types import SimpleNamespace as Msg

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from .test_drcda_v2_hardware import controller, activate, feed, hw


@pytest.mark.parametrize('angles', [[0.,0.,0.], [.4,-.6,1.2], [1.1,.8,-3.]])
def test_desired_body_rate_matches_rotation_derivative(controller, angles):
    rates = np.array([.3,-.2,.4])
    controller.target_attitude = np.array(angles)
    controller.target_attitude_rate = rates
    controller.target_R_des_ned_frd = None
    dt = 1e-7
    a = controller._direct_desired_attitude_ned_frd(controller.target_attitude)
    b = controller._direct_desired_attitude_ned_frd(controller.target_attitude + dt*rates)
    expected = Rotation.from_matrix(a.T@b).as_rotvec()/dt
    np.testing.assert_allclose(controller._desired_body_rate_for_control(np.zeros(3)), expected, atol=1e-7)


def test_optional_status_and_fresh_heartbeat(controller):
    activate(controller)
    controller._received.pop('status', None)
    controller.px4_timestamp = 123
    controller.offboard_startup_tick()
    assert not controller._hardware_fault
    first = controller.offboard_control_mode_pub.publish.call_args.args[0].timestamp
    time.sleep(.002)
    controller.offboard_startup_tick()
    second = controller.offboard_control_mode_pub.publish.call_args.args[0].timestamp
    assert second > first > 123


def test_manual_pitch_rate_reaches_torque_feedback(controller):
    activate(controller)
    controller.target_R_des_ned_frd = None
    controller.target_attitude_rate = np.array([0., .1, 0.])
    desired = controller._direct_desired_attitude_ned_frd(controller.target_attitude)
    controller.angular_velocity_frd = (controller.R_ned_frd.T @ desired
        @ controller._desired_body_rate_for_control(np.zeros(3)))
    controller._last_control_dt_s = .01
    controller.publish_px4_equivalent_direct_commands(.1, .01)
    np.testing.assert_allclose(controller.last_omega_error, 0., atol=1e-10)


def test_profile_migration_preserves_aircraft_calibration():
    from tools.migrate_hardware_cascade_tuning import migrate
    defaults = copy.deepcopy(hw.HARDWARE_V2_DEFAULT_CONFIG)
    source = {k:v for k,v in defaults.items() if not k.startswith('cascade_')}
    source['HNTR_PITCH_BIAS'] = .09
    source['direct_Domega'] = [3.,1.5,2.3]
    migrated = migrate(source, defaults)
    assert migrated['HNTR_PITCH_BIAS'] == .09
    assert migrated['direct_Domega'] == [3.,1.5,2.3]
    assert source.get('cascade_position_p_ned') is None
    hw.HnuterHardwareDRCDAV2Controller.validate_hardware_config(migrated)


def cascade():
    return hw.HardwareVelocityCascade(copy.deepcopy(hw.HARDWARE_V2_DEFAULT_CONFIG))


def test_cascade_matches_firmware_equations_without_saturation():
    c = cascade()
    ep = np.array([.03,-.02,.01]); v=np.array([.1,-.1,.05])
    vr=np.array([.2,.1,-.1]); aff=np.array([.1,.2,.3]); acc=np.array([.4,-.1,.2])
    expected = aff+c.kv*(vr+c.kp*ep-v)-c.kd*acc
    actual = c.step(ep,v,vr,aff,acc,.01,6.,-8.,8.)
    np.testing.assert_allclose(actual,expected)
    np.testing.assert_allclose(c.integral,c.ki*(vr+c.kp*ep-v)*.01)


def test_saturation_and_antiwindup():
    c=cascade();zero=np.zeros(3)
    for _ in range(300):
        out=c.step(np.ones(3)*10,zero,zero,np.array([0.,0.,10.]),zero,.02,3.,-8.,8.)
    assert np.linalg.norm(out[:2]) <= 3.+1e-12
    assert out[2] == 8.
    np.testing.assert_array_equal(c.integral,0.)
    c.integral[:]=[1.,0.,0.]
    c.step(np.array([-.01,0.,0.]),zero,zero,zero,zero,.02,3.,-8.,8.)
    assert c.integral[0] < 1.


def test_integrator_limit_freeze_disable_and_reset():
    c=cascade();zero=np.zeros(3)
    c.integral[:]=[2.,2.,4.]
    c.step(zero,zero,zero,zero,zero,.02,100.,-100.,100.)
    assert np.linalg.norm(c.integral[:2]) <= 1.5+1e-12
    assert c.integral[2] == 2.5
    before=c.integral.copy()
    c.step(np.ones(3)*.01,zero,zero,zero,zero,.02,100.,-100.,100.,integrate=False)
    np.testing.assert_array_equal(c.integral,before)
    c.ki[:]=0.
    c.step(zero,zero,zero,zero,zero,.02,100.,-100.,100.)
    np.testing.assert_array_equal(c.integral,0.)
    c.integral[:]=1.;c.reset();np.testing.assert_array_equal(c.integral,0.)


def test_lifecycle_resets_cascade(controller):
    controller._cascade.integral[:]=1.
    activate(controller)
    np.testing.assert_array_equal(controller._cascade.integral,0.)
    controller._cascade.integral[:]=1.
    controller.control_mode_callback(Msg(timestamp=1_050_000,flag_armed=True,flag_control_offboard_enabled=False))
    controller.offboard_startup_tick()
    np.testing.assert_array_equal(controller._cascade.integral,0.)


def test_acceleration_source_and_no_double_integrator(controller):
    feed(controller)
    controller.local_position_callback(Msg(timestamp=1_020_000,xy_valid=True,z_valid=True,
        v_xy_valid=True,v_z_valid=True,x=2.,y=3.,z=-4.,vx=.1,vy=.2,vz=.3,ax=.4,ay=.5,az=.6))
    np.testing.assert_allclose(controller._cascade_acceleration_ned,[.4,.5,.6])
    assert controller._cascade_acceleration_source == 'px4_local_position'
    np.testing.assert_array_equal(controller.direct_pos_Ki_ned,0.)


def test_small_disturbance_integral_rejects_offset_in_ideal_point_mass():
    # Algorithm sanity check, NOT Gazebo or hardware evidence.
    c=cascade();p=np.zeros(3);v=np.zeros(3);acc=np.zeros(3);dt=.005
    for _ in range(24000):
        command=c.step(-p,v,np.zeros(3),np.zeros(3),acc,dt,3.,-8.,8.)
        acc=command+np.array([.2,-.1,0.]);v+=acc*dt;p+=v*dt
    assert np.linalg.norm(p[:2]) < .002


@pytest.mark.parametrize('key', ['cascade_velocity_p_ned','cascade_speed_limits'])
def test_bad_config_rejected(key):
    cfg=copy.deepcopy(hw.HARDWARE_V2_DEFAULT_CONFIG);cfg[key]=[1.,2.]
    with pytest.raises(ValueError):hw.HnuterHardwareDRCDAV2Controller.validate_hardware_config(cfg)
