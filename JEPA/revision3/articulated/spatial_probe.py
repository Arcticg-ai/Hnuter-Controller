"""Position-entry-only bounded revision3 oracle / shadow experiment node."""
import os
import sys
import json
import math
import subprocess
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
REV2=HERE.parents[1]/'revision2'
sys.path.insert(0,str(REV2))
import sitl_position_probe as preserved
sys.path.insert(0,str(HERE.parent))
from actions import reference


class SpatialProbe(preserved.Probe):
    def __init__(self,out):
        self.spatial_request=None
        self.shadow_only=os.environ.get('JEPA_SHADOW_ONLY')=='1'
        self.spatial_started=None
        self.roll_ref=self.pitch_ref=0.
        self.phase_elapsed=0.
        self.spatial_events=[]
        self.ros_attitude=[];self.ros_velocity=[]
        self.shadow_process=None;self.shadow_done=False;self.shadow_invalid=False
        super().__init__(out)
        if self.group=='D':
            self.spatial_request=json.loads((out/'oracle_request.json').read_text())
        assert self.group in ('A','B','C','D')

    def attitude_callback(self,msg):
        self.ros_attitude.append([msg.timestamp_sample*1e-6,*msg.q])
        super().attitude_callback(msg)

    def local_position_callback(self,msg):
        self.ros_velocity.append([msg.timestamp_sample*1e-6,msg.vx,msg.vy,msg.vz])
        super().local_position_callback(msg)

    def process_sensor(self,*args):
        # Preserve safety and logging, disable the old mass-only estimator.
        group=self.group;self.group='A'
        try:super().process_sensor(*args)
        finally:self.group=group

    def publish_mass_request(self,freeze=False):
        # Fixed equal mass for every revision3 group. Never estimate/update mass.
        if freeze:self.publish_spatial(freeze=True)

    def publish_spatial(self,freeze=False):
        if self.spatial_request is None:return
        request=preserved.base.VehicleCommand();request.command=31001
        dr=self.spatial_request['delta_com'];dj=self.spatial_request['delta_inertia']
        request.param1=float(dr[0]);request.param2=float(dr[1]);request.param3=float(dj[0]);request.param4=float(dj[1]);request.param5=float(dj[2]);request.param6=54124.;request.param7=1. if freeze else 0.
        request.target_system=1;request.target_component=1;request.source_system=73;request.source_component=1;request.from_external=True;request.timestamp=self.timestamp_now_us()
        self.vehicle_command_pub.publish(request)
        self.spatial_events.append(dict(t=self.phase_elapsed,freeze=freeze,params=[*dr,*dj]))
        if self.spatial_started is None and not freeze:self.spatial_started=self.phase_elapsed

    def watchdog(self):
        super().watchdog()
        if self.phase=='CALIBRATION' and (not self.valid or self.landed or self.last_error>.3 or self.last_tilt>math.radians(20) or np.max(abs(self.motor))>.9):
            self.shadow_invalid=True
        if self.group in ('B','C') and self.phase_elapsed>=35 and self.shadow_process is None and not self.shadow_done:
            if self.shadow_invalid:
                (self.out/'online_shadow.json').write_text(json.dumps(dict(accepted=False,reason='calibration_safety_window')))
                self.shadow_done=True
            else:
                end=self.px4_timestamp*1e-6-self.phase_elapsed+34.
                np.savez_compressed(self.out/'ros_sensor_capture.npz',attitude=np.array(self.ros_attitude),velocity=np.array(self.ros_velocity),interval=np.array([end-16.,end]))
                self.shadow_log=(self.out/'shadow_worker.log').open('w')
                command=[sys.executable,str(HERE/'online_shadow.py'),str(self.out/'imu_native.csv'),str(self.out/'joints_native.csv'),str(self.out/'ros_sensor_capture.npz'),str(HERE/'nominal_calibration.json'),str(self.out/'online_shadow.json')]
                self.shadow_process=subprocess.Popen(command,stdout=self.shadow_log,stderr=subprocess.STDOUT)
        if self.shadow_process is not None and not self.shadow_done:
            if self.shadow_process.poll() is not None:
                self.shadow_done=True;self.shadow_log.close()
                result=json.loads((self.out/'online_shadow.json').read_text()) if (self.out/'online_shadow.json').exists() else {}
                result['application_disabled']=self.shadow_only
                result['decision_phase_elapsed_s']=self.phase_elapsed
                (self.out/'online_shadow.json').write_text(json.dumps(result,indent=2)+'\n')
                if result.get('accepted') and self.phase_elapsed<45 and not self.shadow_only:self.spatial_request=result['request']
            elif self.phase_elapsed>=45:
                self.shadow_process.terminate();self.shadow_process.wait(timeout=3);self.shadow_log.close();self.shadow_done=True
                (self.out/'online_shadow.json').write_text(json.dumps(dict(accepted=False,reason='calibration_compute_budget')))
        if self.phase in ('CHECK','EVALUATION','FINAL_HOVER') and self.phase_elapsed>=34 and self.armed and self.valid and not self.landed and self.last_error<.3 and self.last_tilt<math.radians(20):
            self.publish_spatial()
        if self.finished:
            (self.out/'spatial_events.json').write_text(json.dumps(self.spatial_events,indent=2)+'\n')

    def publish_px4_trajectory_setpoint(self):
        # Preserve the normal position entry. Its jerk[0:2] extension is the
        # existing Hnuter attitude reference; no actuator publisher is used.
        if self.phase not in ('EVALUATION','FINAL_HOVER') and not (self.phase=='CALIBRATION' and self.group=='C'):
            return super().publish_px4_trajectory_setpoint()
        msg=preserved.base.TrajectorySetpoint();msg.timestamp=self.timestamp_now_us()
        z=float(self._z0+self.target_position[2]) if self._z0_initialized else float(self.position[2])
        msg.position=[float(self.target_position[1]),float(self.target_position[0]),-z]
        msg.velocity=[float(self.target_velocity[1]),float(self.target_velocity[0]),float(-self.target_velocity[2])]
        msg.acceleration=[float(self.target_acceleration[1]),float(self.target_acceleration[0]),float(-self.target_acceleration[2])]
        msg.jerk=[self.roll_ref,self.pitch_ref,float('nan')]
        msg.yaw=self._yaw_enu_to_ned(self.manual_des_yaw);msg.yawspeed=float(-self.target_attitude_rate[2])
        self.trajectory_setpoint_pub.publish(msg)

    def update_trajectory(self,current_time,dt):
        elapsed=current_time-self.started if self.started is not None else 0.
        # Extend the SAME check phase by 20 seconds in all groups, giving the
        # bounded tracking bias time to decay before the shared evaluation.
        mapped=current_time-max(0.,min(20.,elapsed-34.))
        super().update_trajectory(mapped,dt)
        self.phase_elapsed=elapsed
        self.roll_ref=self.pitch_ref=0.
        if self.phase=='CALIBRATION' and self.group=='C':
            # Every axis is excited in BOTH disjoint halves; 0.75 deg each.
            t=elapsed-18.
            roll=reference(t,0,amplitude=math.radians(.75),duration=16.,frequency=.4)
            pitch=reference(t,1,amplitude=math.radians(.75),duration=16.,frequency=.5)
            yaw=reference(t,2,amplitude=math.radians(.75),duration=16.,frequency=.6)
            self.roll_ref=float(roll[0]);self.pitch_ref=float(pitch[0]);self.manual_des_yaw=float(yaw[0]);self.target_attitude_rate[2]=yaw[1]
        if self.phase=='EVALUATION':
            t=elapsed-(60. if self.phase=='EVALUATION' else 18.);duration=16./3;axis=min(2,int(t/duration));local=t-axis*duration
            angle,rate,acc=reference(local,axis,duration=duration)
            if axis==0:self.roll_ref=float(angle)
            elif axis==1:self.pitch_ref=float(angle)
            # Yaw reference derivative is supported. Roll/pitch derivatives are
            # logged but the existing firmware does not consume their feedforward.
            else:self.manual_des_yaw=float(angle);self.target_attitude_rate[2]=rate
            if self.phase=='EVALUATION':
                dx,dv,da=reference(t,0,amplitude=.06,duration=16.,frequency=.25)
                self.target_position[0]+=dx;self.target_velocity[0]+=dv;self.target_acceleration[0]+=da
        if self.rows:
            row=self.rows[-1];row['t']=elapsed
            row['x_ref']=float(self.target_position[0]);row['y_ref']=float(self.target_position[1])
            row['roll_ref']=self.roll_ref;row['pitch_ref']=self.pitch_ref
            position=np.array([*self.position[:2],self.position[2]-self._z0])
            row['error']=float(np.linalg.norm(position-self.target_position));self.last_error=row['error']


if __name__=='__main__':
    preserved.Probe=SpatialProbe
    raise SystemExit(preserved.main())
