"""No-delay matched follow-up: qualified handover and near-term balance."""
import json
import os
from pathlib import Path

import numpy as np
import rclpy

from controllers.experiments.maneuver_validation import ManeuverMixin
from controllers.experiments.fixed_attitude_switching.controller import (
    FixedAttitudeSwitchingMixin, controller_class,
)


def phase_clock(t, period, ramp):
    """C2 phase clock; retain the original interior speed, add ramp seconds."""
    if ramp <= 0:
        return float(np.clip(t, 0, period)), float(0 <= t < period), 0.
    if t <= 0:
        return 0., 0., 0.
    if t >= period+ramp:
        return period, 0., 0.
    if t < ramp:
        s=t/ramp
        return ramp*(s**3-.5*s**4), 3*s*s-2*s**3, (6*s-6*s*s)/ramp
    if t < period:
        return t-ramp/2, 1., 0.
    s=(t-period)/ramp
    return period-ramp/2+ramp*(s-s**3+.5*s**4), 1-3*s*s+2*s**3, (-6*s+6*s*s)/ramp


class RevisedManeuverMixin(ManeuverMixin):
    def __init__(self):
        super().__init__()
        self.attitude_segment_time_s = float(os.environ.get('HNUTER_REV_TILT_S','15'))
        self.attitude_peak_hold_s = 5.
        self.attitude_level_settle_s = 5.
        if self.maneuver_attitude:
            self.attitude_step_axis_rad = np.radians(
                [*map(float,self.maneuver_scenario.split('_')[1:]),0.])
        self.switch_settle_s = 5.
        self.revision_ramp_s = float(os.environ.get('HNUTER_REV_RAMP_S','0.75'))
        self.revision_early_handover = os.environ.get('HNUTER_REV_EARLY_HANDOVER','1')=='1'
        self.revision_sdf_geometry = os.environ.get('HNUTER_REV_SDF_GEOMETRY','1')=='1'
        self.revision_unit_dc_gain = (self.revision_sdf_geometry and
            os.environ.get('HNUTER_REV_UNIT_DC_GAIN','1')=='1')
        if self.revision_sdf_geometry:
            from controllers.experiments.sdf_wrench_model import SDFWrenchModel
            sdf = '/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl/Tools/simulation/gz/models/hnuter/model.sdf'
            self.revision_wrench_model = SDFWrenchModel(sdf)
            positions = self.revision_wrench_model.positions
            self.l1 = float((positions[:2,1].mean()-positions[2:4,1].mean())/2)
            self.l2 = float(-positions[4,0])
            self.allocator_front_x_m = float(positions[:4,0].mean())
            self.allocator_front_z_m = float(positions[:4,2].mean())
            if hasattr(self,'drcda'):
                self.drcda.model = self.revision_wrench_model
                if hasattr(self.drcda,'hover_front_thrust'):
                    self.drcda.hover_front_thrust = self.mass*self.gravity*self.l2/(4*(self.allocator_front_x_m+self.l2))
        near = float(os.environ.get('HNUTER_REV_NEAR_GAIN','0.25'))
        if hasattr(self,'drcda'):
            # This Gazebo position PID settles at q=u. A transient-fitted
            # K<1 otherwise predicts motion even at a measured equilibrium.
            if self.revision_unit_dc_gain:
                self.drcda.config.servo_gain_positive[:] = 1.
                self.drcda.config.servo_gain_negative[:] = 1.
            self.drcda.config.immediate_wrench_weight[:] = (
                np.array([0.,0.,2.,12.,12.,6.])*near
                if self._switch_method not in {'paper_nda','no_nearfield'} else np.zeros(6))
        self._record_revision()

    def _record_revision(self):
        from dataclasses import asdict
        path = Path(os.environ['HNUTER_LOG_DIR'])/'runtime_config.json'
        data = json.loads(path.read_text())
        data['allocator'] = asdict(self.drcda.config) if hasattr(self,'drcda') else None
        data['trajectory'] = dict(tilt_s=self.attitude_segment_time_s,
            peak_hold_s=self.attitude_peak_hold_s,level_settle_s=self.attitude_level_settle_s,
            attitude_amplitudes_deg=np.degrees(self.attitude_step_axis_rad),
            phase_ramp_s=self.revision_ramp_s,allocator_settle_s=self.switch_settle_s,
            early_handover=self.revision_early_handover,takeoff_guard_end_s=12.5)
        data['direct_output'] = dict(position=self._use_px4_position_mode(),
            allocator_setpoints=self.publish_allocator_setpoints_in_direct,
            landing_hint=self.publish_land_detector_thrust_setpoint)
        data['geometry'] = dict(sdf_articulated=self.revision_sdf_geometry,
            gazebo_unit_dc_gain=self.revision_unit_dc_gain,
            nominal_arm_half_span_m=self.l1,nominal_tail_x_m=-self.l2,
            nominal_front_x_m=getattr(self,'allocator_front_x_m',.105),
            nominal_front_z_m=getattr(self,'allocator_front_z_m',-.013))
        path.write_text(json.dumps(data,indent=2,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else x.item())+'\n')

    def _start_auto_trajectory(self,mode,current_time):
        super()._start_auto_trajectory(mode,current_time)
        if mode=='lissajous':
            elapsed=current_time-self._takeoff_lock_start_time_s
            self.switch_settle_s=max(5.,12.5-elapsed-self.switch_ingress_s)
            self._record_revision()

    def _attitude_phase(self,t):
        move,hold,settle=self.attitude_segment_time_s,self.attitude_peak_hold_s,self.attitude_level_settle_s
        cycle=2*move+hold+settle
        if t>=4*cycle: return 'complete'
        axis=('roll_pos','pitch_pos','roll_neg','pitch_neg')[int(t//cycle)]
        s=t%cycle
        phase='out' if s<move else 'hold' if s<move+hold else 'return' if s<2*move+hold else 'level'
        return f'{axis}_{phase}'

    def _switch_reference(self,elapsed):
        prelude=self.switch_ingress_s+self.switch_settle_s
        if elapsed<prelude:
            return FixedAttitudeSwitchingMixin._switch_reference(self,elapsed)
        t=elapsed-prelude
        if self.maneuver_attitude:
            phase=self._attitude_phase(t)
            return self._switch_a.copy(),np.zeros(3),np.zeros(3),phase,phase=='complete'
        f,fd,fdd=phase_clock(t,self.lissajous_period_s,self.revision_ramp_s)
        p,v,a,_=self._lissajous_reference(f)
        done=t>=self.lissajous_period_s+self.revision_ramp_s
        return p,v*fd,a*fd*fd+v*fdd,'complete' if done else 'lissajous',done

    def publish_direct_actuator_setpoint(self,*args,**kwargs):
        if (self.revision_early_handover and self.armed and self.auto_traj_mode=='fixed_switch'
                and self._switch_elapsed_s>=self.switch_ingress_s):
            self._switch_allocator_engaged=True
        return super().publish_direct_actuator_setpoint(*args,**kwargs)

    def _finish_switching(self):
        # A reference ending is not an estimator reset; retain trim integrals.
        pos,att=self.integral_pos_error.copy(),self.integral_e_R.copy()
        previous_quaternion=self._attitude_error_quaternion
        super()._finish_switching()
        self.integral_pos_error[:]=pos
        self.integral_e_R[:]=att
        self._attitude_error_quaternion=previous_quaternion


def main():
    rclpy.init()
    cls=type('RevisedManeuverController',(RevisedManeuverMixin,controller_class(os.environ['HNUTER_SWITCH_METHOD'])),{})
    controller=cls()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
