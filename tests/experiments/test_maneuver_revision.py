import numpy as np
import pytest

from controllers.experiments.maneuver_revision import phase_clock,RevisedManeuverMixin
from controllers.experiments.fixed_attitude_switching.controller import FixedAttitudeDirectController
from tests.experiments.test_maneuver_validation import controller


@pytest.mark.parametrize('period',[5.,7.])
def test_clock_endpoints_and_interior_speed(period):
    assert phase_clock(0,period,.75)==(0.,0.,0.)
    assert phase_clock(period+.75,period,.75)==(period,0.,0.)
    f,v,a=phase_clock(2.,period,.75)
    assert v==1. and a==0.
    assert f==2.-.375


@pytest.mark.parametrize('t',[.2,.74999,.75,2.,5.,5.4,5.74999])
def test_clock_derivatives(t):
    h=1e-6
    f,v,a=phase_clock(t,5.,.75)
    fm,vm,_=phase_clock(t-h,5.,.75)
    fp,vp,_=phase_clock(t+h,5.,.75)
    np.testing.assert_allclose((fp-fm)/(2*h),v,atol=1e-5)
    np.testing.assert_allclose((vp-vm)/(2*h),a,atol=1e-5)


def revised_controller():
    c=controller()
    c.__class__=type('DummyRevision',(RevisedManeuverMixin,FixedAttitudeDirectController),{})
    c.attitude_segment_time_s,c.attitude_peak_hold_s,c.attitude_level_settle_s=15.,5.,5.
    c.revision_ramp_s=.75
    return c


def test_idle_keyword_actuator_call(monkeypatch):
    from controllers.experiments.maneuver_validation import ManeuverMixin
    monkeypatch.setattr(ManeuverMixin,'publish_direct_actuator_setpoint',
                        lambda self,*args,**kwargs:kwargs['motor_controls'])
    c=revised_controller()
    c.revision_early_handover=True
    c.armed=False
    assert c.publish_direct_actuator_setpoint(motor_controls=[0.]*5,alpha1=0,
        alpha2=0,theta1=0,theta2=0)==[0.]*5


@pytest.mark.parametrize('t,phase',[(15,'roll_pos_hold'),(20,'roll_pos_return'),
    (35,'roll_pos_level'),(40,'pitch_pos_out'),(160,'complete')])
def test_restored_attitude_timing(t,phase):
    assert revised_controller()._attitude_phase(t)==phase


def test_cartesian_reference_c2_and_derivatives():
    c=revised_controller()
    for t in [4.,4.+7.75]:
        p,v,a,_,_=c._switch_reference(t)
        np.testing.assert_allclose(p,c._switch_a,atol=1e-10)
        np.testing.assert_allclose(v,0,atol=1e-10)
        np.testing.assert_allclose(a,0,atol=1e-10)
    h=1e-5
    for t in [4.2,5.7,11.3]:
        pm,vm,*_=c._switch_reference(t-h)
        pp,vp,*_=c._switch_reference(t+h)
        _,v,a,*_=c._switch_reference(t)
        np.testing.assert_allclose((pp-pm)/(2*h),v,atol=1e-7)
        np.testing.assert_allclose((vp-vm)/(2*h),a,atol=1e-6)


@pytest.mark.parametrize('geometry,enabled,expected',[
    ('1','1',1.),('1','0',.9),('0','1',.9)])
def test_gazebo_dc_gain_scope_and_stationary_prediction(monkeypatch,geometry,enabled,expected):
    from controllers.common.hnuter_drcda import DRCDAAllocator, DRCDAConfig, HnuterWrenchModel
    from controllers.experiments.maneuver_validation import ManeuverMixin

    cfg=DRCDAConfig.gazebo_joint_pid_no_delay()
    cfg.servo_gain_positive[:]=.9
    cfg.servo_gain_negative[:]=.9
    taus=(cfg.servo_tau_positive_s.copy(),cfg.servo_tau_negative_s.copy())
    rates=cfg.servo_rate_positive_rad_s.copy()
    def base_init(self):
        self.maneuver_attitude=False
        self._switch_method='full'
        self.mass,self.gravity=4.5,9.81
        self.drcda=DRCDAAllocator(HnuterWrenchModel(),cfg)
    monkeypatch.setattr(ManeuverMixin,'__init__',base_init)
    monkeypatch.setattr(RevisedManeuverMixin,'_record_revision',lambda self:None)
    monkeypatch.setenv('HNUTER_REV_SDF_GEOMETRY',geometry)
    monkeypatch.setenv('HNUTER_REV_UNIT_DC_GAIN',enabled)
    c=RevisedManeuverMixin()
    np.testing.assert_array_equal(cfg.servo_gain_positive,expected)
    np.testing.assert_array_equal(cfg.servo_gain_negative,expected)
    np.testing.assert_array_equal(cfg.servo_tau_positive_s,taus[0])
    np.testing.assert_array_equal(cfg.servo_tau_negative_s,taus[1])
    np.testing.assert_array_equal(cfg.servo_rate_positive_rad_s,rates)
    np.testing.assert_array_equal(cfg.servo_delay_positive_s,0.)
    np.testing.assert_array_equal(cfg.servo_delay_negative_s,0.)
    c.drcda.reset(angle_state=[.5,-1.2,-.5,1.2],thrust_state=[10.,10.,10.,10.,6.])
    for state,sensitivity in c.drcda._predict_horizons(
            c.drcda.command,cfg.servo_state_limit_rad,[.05,.1]):
        assert np.all(np.isfinite(sensitivity))
        if expected==1.:
            np.testing.assert_allclose(state,c.drcda.state,atol=1e-12)
        else:
            assert np.linalg.norm(state[:4]-c.drcda.state[:4])>.01
