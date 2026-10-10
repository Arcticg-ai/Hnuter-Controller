"""Automated references using the preserved ordinary PX4 position controller.

SITL-only launcher manifest required. No direct actuator publishing permitted.
"""
import csv
import json
import math
import os
from pathlib import Path
import sys
import time
from collections import deque
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent.parent))
import hnuter_external_controller_px4_position as base
from estimator import vertical_reference,MassEstimator
from px4_msgs.msg import VehicleLandDetected, ActuatorMotors
from rclpy.qos import qos_profile_sensor_data


class NoInput:
    def __init__(self, *a, **kw): pass
    def close(self): pass
    def get_commands(self): return []
    def get_velocity_commands(self, dt):
        return dict(vx_b=0.,vy_b=0.,vz=0.,yaw_rate=0.,pitch_rate=0.,lt=0.,rt=0.)


class Probe(base.HnuterController):
    def __init__(self, out):
        self.out = out
        self.phase = 'WAIT'
        self.started = None
        self.finished = False
        self.failure = None
        self.land_started = None
        self.last_pos_wall = self.last_att_wall = 0.
        self.valid = False
        self.landed = True
        self.motor = np.zeros(5)
        self.had_armed = False
        self.rows = []
        self.stable_since = None
        self.flight_origin = None
        self.land_acknowledged = False
        self.sensors=None
        self.sensor_rows=[]
        self.estimator=MassEstimator()
        self.rotation_ned=np.eye(3)
        self.group=os.environ.get('JEPA_GROUP','C_shadow')
        self.noisy=os.environ.get('JEPA_NOISE')=='noisy_delayed'
        self.rng=np.random.default_rng(int(os.environ.get('JEPA_SEED','0')))
        self.velocities=deque()
        self.attitudes=deque(maxlen=2000)
        self.accepted_at=None
        self.last_error=0.;self.last_tilt=0.
        self.saturated_since=None
        self.init_wall=time.monotonic()
        self.readiness=[]
        self.last_aligned_wall=0.
        self.last_alignment_age=None
        self.aligned_count=0
        self.ready_at=None
        self.startup_only=os.environ.get('JEPA_STARTUP_ONLY')=='1'
        self.inject_exit_at=float(os.environ.get('JEPA_INJECT_EXIT_AT','0'))
        self.recovery_rows=[]
        self.rotor_response_lost_since=None
        self.last_omega=None
        self.offboard_landing=False
        if (out/'actuator_geometry.json').exists():
            from actuator_sensors import JointSensors
            self.sensors=JointSensors(out/'actuator_geometry.json','/jepa_joint_sensors')
        super().__init__()
        assert self.use_px4_position_takeoff and not self.publish_gz_servos_direct
        self.create_subscription(VehicleLandDetected,'/fmu/out/vehicle_land_detected',self.on_land,qos_profile_sensor_data)
        self.create_subscription(ActuatorMotors,'/fmu/out/actuator_motors',self.on_motors,qos_profile_sensor_data)
        self.create_timer(.1,self.watchdog)

    def print_status(self):
        self.get_logger().info(f'PHASE={self.phase} armed={self.armed} pos={self.position} failure={self.failure}')

    def on_land(self,msg): self.landed = bool(msg.landed)
    def on_motors(self,msg): self.motor = np.array(msg.control[:5])

    def local_position_callback(self,msg):
        self.last_pos_wall = time.monotonic()
        self.valid = all([msg.xy_valid,msg.z_valid,msg.v_xy_valid,msg.v_z_valid])
        super().local_position_callback(msg)
        if self.sensors is not None:
            now=msg.timestamp*1e-6
            self.velocities.append((now,float(msg.vz),self.phase,self.valid and self.armed and not self.landed and self.last_error<.3 and self.last_tilt<math.radians(20) and np.linalg.norm(self.velocity)<1.2 and float(np.nanmax(np.abs(self.motor)))<.9))
            delay=.06 if self.noisy else .03
            while self.velocities and now-self.velocities[0][0]>=delay:
                self.process_sensor(*self.velocities.popleft())

    def process_sensor(self,t,vz,phase,eligible):
            rotations=[q for stamp,q in self.attitudes if 0<=t-stamp<.025]
            measurement=self.sensors.thrust(t,rotations[-1],self.rng if self.noisy else None) if rotations else None
            if measurement is not None:
                self.last_aligned_wall=time.monotonic();self.aligned_count+=1
                self.last_alignment_age=t-measurement['sensor_t']
                self.last_omega=np.array(measurement['omega'])
                if self.noisy:vz+=self.rng.normal(0,.01)
                self.sensor_rows.append(dict(t=t,sensor_t=measurement['sensor_t'],vz_ned=vz,
                                             thrust_up=measurement['thrust_up'],phase=phase,
                                             **{f'omega{i}':v for i,v in enumerate(measurement['omega'])},
                                             **{f'{name}_{kind}':v for name,pair in measurement['joints'].items() for kind,v in zip(('angle','velocity'),pair)}))
                if phase=='CALIBRATION' and self.group in ('B','C','C_shadow'):
                    self.estimator.push(t,vz,measurement['thrust_up'],eligible=eligible)
                    if self.estimator.accepted and self.accepted_at is None:self.accepted_at=t
            elif phase=='CALIBRATION':
                self.estimator.invalidate('missing_aligned_sensor')

    def attitude_callback(self,msg):
        self.last_att_wall = time.monotonic()
        w,x,y,z=msg.q
        self.rotation_ned=np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                                    [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                                    [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
        self.attitudes.append((msg.timestamp*1e-6,self.rotation_ned.copy()))
        super().attitude_callback(msg)

    def publish_px4_trajectory_setpoint(self):
        if os.environ.get('JEPA_ATTITUDE_REFERENCE','level')=='legacy' and self.phase!='LAND':
            return super().publish_px4_trajectory_setpoint()
        # Existing Hnuter TrajectorySetpoint extension: jerk[0:2] are roll/pitch.
        # Explicit level body reference avoids inheriting mc_pos_control tilt.
        msg=base.TrajectorySetpoint();msg.timestamp=self.timestamp_now_us()
        z=float(self._z0+self.target_position[2]) if self._z0_initialized else float(self.position[2])
        msg.position=[float(self.target_position[1]),float(self.target_position[0]),-z]
        msg.velocity=[float(self.target_velocity[1]),float(self.target_velocity[0]),float(-self.target_velocity[2])]
        msg.acceleration=[float(self.target_acceleration[1]),float(self.target_acceleration[0]),float(-self.target_acceleration[2])]
        msg.jerk=[0.,0.,float('nan')]
        msg.yaw=self._yaw_enu_to_ned(self.manual_des_yaw);msg.yawspeed=float(-self.target_attitude_rate[2])
        self.trajectory_setpoint_pub.publish(msg)

    def publish_direct_actuator_setpoint(self,*a,**kw):
        raise RuntimeError('Direct actuator control forbidden')

    def publish_actuator_commands(self,*a,**kw):
        raise RuntimeError('Direct actuator control forbidden')

    def offboard_startup_tick(self):
        if self.phase == 'LAND':
            # Stop mode-reentry requests after the explicit PX4 LAND command.
            if self.offboard_landing:self.publish_offboard_control_mode()
            return
        super().offboard_startup_tick()

    def control_loop(self):
        if self.phase != 'LAND':
            super().control_loop()
        elif self.offboard_landing and self.armed:
            # Same analytic position-only landing reference for every comparison.
            # Retain commanded descent until disarm: the original land detector
            # explicitly requires positive NED velocity even after touchdown.
            t=max(0.,self.px4_timestamp*1e-6-self.landing_start_px4)
            ramp=2.;speed=.7
            if t<ramp:
                z=self.landing_initial_z-speed*.5*(t-ramp/math.pi*math.sin(math.pi*t/ramp))
                v=-speed*.5*(1-math.cos(math.pi*t/ramp))
                a=-speed*math.pi/(2*ramp)*math.sin(math.pi*t/ramp)
            else:z=self.landing_initial_z-speed*(t-ramp/2);v=-speed;a=0.
            self.target_position=np.array([*self.flight_origin,z])
            self.target_velocity=np.array([0.,0.,v])
            self.target_acceleration=np.array([0.,0.,a])
            self.publish_px4_trajectory_setpoint()

    def land(self,reason=None):
        if self.phase == 'LAND': return
        self.failure = reason
        self.phase = 'LAND'
        self.land_started = time.monotonic()
        self.publish_mass_request(freeze=True)
        if self.valid and time.monotonic()-self.last_pos_wall<.2 and time.monotonic()-self.last_att_wall<.2:
            # Keep the proven position-only descent chain alive for recoverable limits.
            # Never resume mass updates or excitation after entering LAND.
            self.flight_origin=self.position[:2].copy()
            self.offboard_landing=True
            self.landing_start_px4=self.px4_timestamp*1e-6
            self.landing_initial_z=float(self.position[2]-self._z0)
        else:
            self.publish_vehicle_command(21)
        self.get_logger().info(f'LAND requested; reason={reason}')

    def watchdog(self):
        now = time.monotonic()
        if self.phase == 'WAIT':
            if now-self.init_wall>45:
                self.failure='prearm_stream_readiness_timeout';self.finished=True
                return
            ready=self.valid and self.data_received and now-self.last_pos_wall<.2 and now-self.last_att_wall<.2 and np.all(np.isfinite(self.velocity)) and now-self.last_aligned_wall<.2 and self.aligned_count>=25
            diag=self.sensors.snapshot() if self.sensors else {}
            self.readiness.append(dict(wall_s=now-self.init_wall,px4_s=self.px4_timestamp*1e-6,ready=bool(ready),aligned_count=self.aligned_count,alignment_age_s=self.last_alignment_age,pos_age_s=now-self.last_pos_wall,att_age_s=now-self.last_att_wall,sensors=diag))
            if ready:
                if self.stable_since is None: self.stable_since=now
                if now-self.stable_since>3.:
                    self.ready_at=now-self.init_wall
                    if self.startup_only:self.finished=True;return
                    self.takeoff_requested=True
                    self.phase='ARM_WAIT'
            else: self.stable_since=None
        if self.armed: self.had_armed=True
        if self.armed and self.phase != 'LAND':
            if now-self.last_aligned_wall>.5:
                self.land('joint_telemetry_stale')
            # Rotor motion is checked only after a nonzero final motor command.
            # This catches command-delivery loss that passive prearm sensors cannot.
            command_active=np.all(np.isfinite(self.motor)) and float(np.max(np.abs(self.motor)))>.15
            response_absent=self.last_omega is not None and float(np.max(np.abs(self.last_omega)))<50.
            if command_active and response_absent:
                if self.rotor_response_lost_since is None:self.rotor_response_lost_since=now
                if now-self.rotor_response_lost_since>.5:self.land('motor_command_without_rotor_response')
            else:self.rotor_response_lost_since=None
            if not self.valid or max(now-self.last_pos_wall,now-self.last_att_wall)>.5:
                self.land('invalid_or_stale_state')
        if self.phase == 'LAND':
            self.publish_mass_request(freeze=True)
            self.recovery_rows.append(dict(wall_s=now-self.land_started,px4_s=self.px4_timestamp*1e-6,landed=self.landed,armed=self.armed,position=self.position.tolist(),velocity=self.velocity.tolist(),offboard_landing=self.offboard_landing))
            if not self.armed and self.landed and self.had_armed:
                self.finished=True
            elif now-self.land_started>35:
                self.failure=self.failure or 'landing_timeout'
                self.finished=True
        elif self.had_armed and not self.armed:
            self.failure='unexpected_disarm'
            self.finished=True
        if self.group in ('B','C') and self.estimator.accepted and self.phase in ('CHECK','EVALUATION','FINAL_HOVER') and self.started is not None and self.px4_timestamp*1e-6-self.sim_start_time_s-self.started>=35. and self.armed and self.valid and not self.landed and self.last_error<.3 and self.last_tilt<math.radians(20) and float(np.nanmax(np.abs(self.motor)))<.9:
            self.publish_mass_request()

    def publish_mass_request(self,freeze=False):
        request=base.VehicleCommand()
        request.command=31000;request.param1=float(self.estimator.mass);request.param2=54123.
        request.param3=1. if freeze else 0.
        request.target_system=1;request.target_component=1;request.source_system=73;request.source_component=1
        request.from_external=True;request.timestamp=self.timestamp_now_us()
        self.vehicle_command_pub.publish(request)

    def update_trajectory(self,current_time,dt):
        # Use baseline takeoff/hover implementation unchanged until stable hover.
        super().update_trajectory(current_time,dt)
        if not self.armed or not self.is_offboard(): return
        if self.started is None:
            self.started=current_time
            self.flight_origin=self.position[:2].copy()
        elapsed=current_time-self.started
        if elapsed<12:
            self.phase='TAKEOFF'
            if os.environ.get('JEPA_SMOOTH_TAKEOFF')=='1':
                duration=10.;s=min(elapsed/duration,1.)
                h=self.takeoff_height
                self.target_position=np.array([*self.flight_origin,h*(10*s**3-15*s**4+6*s**5)])
                self.target_velocity=np.array([0.,0.,h/duration*(30*s**2-60*s**3+30*s**4)])
                self.target_acceleration=np.array([0.,0.,h/duration**2*(60*s-180*s**2+120*s**3)])
        else:
            if elapsed<18:
                self.phase='HOVER'
                zva=np.array([self.takeoff_height,0.,0.])
            elif elapsed<34:
                self.phase='CALIBRATION'
                zva=vertical_reference(elapsed-18,amplitude=.08 if self.group in ('C','C_shadow') else 0.)
                zva=np.array([self.takeoff_height-zva[0],-zva[1],-zva[2]])
            elif elapsed<40:
                self.phase='CHECK'
                zva=np.array([self.takeoff_height,0.,0.])
            elif elapsed<56:
                self.phase='EVALUATION'
                zva=vertical_reference(elapsed-40,amplitude=.15,frequency=.3)
                zva=np.array([self.takeoff_height-zva[0],-zva[1],-zva[2]])
            elif elapsed<61:
                self.phase='FINAL_HOVER'
                zva=np.array([self.takeoff_height,0.,0.])
            else:
                self.land()
                return
            self.target_position=np.array([*self.flight_origin,zva[0]])
            self.target_velocity=np.array([0.,0.,zva[1]])
            self.target_acceleration=np.array([0.,0.,zva[2]])
        position=np.array([*self.position[:2],self.position[2]-self._z0])
        err=position-self.target_position
        tilt=math.acos(float(np.clip(self.R[2,2],-1,1)))
        self.last_error=float(np.linalg.norm(err));self.last_tilt=tilt
        if float(np.nanmax(np.abs(self.motor)))>=.95:
            if self.saturated_since is None:self.saturated_since=elapsed
        else:self.saturated_since=None
        self.rows.append(dict(t=elapsed,px4_time_s=self.px4_timestamp*1e-6,phase=self.phase,x=position[0],y=position[1],z_up=position[2],
                             x_ref=self.target_position[0],y_ref=self.target_position[1],z_ref=self.target_position[2],
                             vz_up=self.velocity[2],vz_ref=self.target_velocity[2],az_ref=self.target_acceleration[2],
                             error=float(np.linalg.norm(err)),tilt_deg=math.degrees(tilt),landed=self.landed,
                             motor_max=float(np.nanmax(np.abs(self.motor))),valid=self.valid))
        if self.inject_exit_at and elapsed>=self.inject_exit_at:
            self.land('injected_recovery_test')
        elif tilt>math.radians(25) or np.linalg.norm(position[:2]-self.flight_origin)>1.0:
            self.land('attitude_or_horizontal_limit')
        elif elapsed>12 and (np.linalg.norm(err)>.5 or np.linalg.norm(self.velocity)>1.5 or self.landed):
            self.land('tracking_velocity_or_contact_limit')
        elif self.saturated_since is not None and elapsed-self.saturated_since>.3:
            self.land('sustained_saturation')


def main():
    out=Path(os.environ['JEPA_RUN_DIR'])
    manifest=json.loads((out/'launch.json').read_text())
    assert manifest['kind']=='owned_local_sitl' and os.environ['GZ_PARTITION']==manifest['GZ_PARTITION']
    for name in ('px4','agent'):
        os.kill(manifest['pids'][name],0)
    base.GamepadManager=base.KeyboardCommandReader=NoInput
    names=['/fmu/in/offboard_control_mode','/fmu/in/trajectory_setpoint','/fmu/in/vehicle_command',
           '/fmu/in/actuator_motors','/fmu/in/actuator_servos','/fmu/out/vehicle_local_position_v1',
           '/fmu/out/vehicle_attitude','/fmu/out/vehicle_angular_velocity','/fmu/out/vehicle_status_v1',
           '/fmu/out/vehicle_control_mode','/fmu/out/vehicle_command_ack','/fmu/out/vehicle_land_detected','/fmu/out/actuator_motors']
    args=['--ros-args']
    for name in names: args+=['-r',name+':=/'+manifest['namespace']+name]
    base.rclpy.init(args=args)
    node=Probe(out)
    started=time.monotonic()
    try:
        while not node.finished and time.monotonic()-started<150:
            base.rclpy.spin_once(node,timeout_sec=.1)
        if not node.finished: node.failure='overall_timeout'
    finally:
        if node.rows:
            with (out/'trajectory.csv').open('w') as f:
                w=csv.DictWriter(f,fieldnames=list(node.rows[0]));w.writeheader();w.writerows(node.rows)
        for name,rows in [('actuator_sensor',node.sensor_rows),('mass_windows',node.estimator.rows)]:
            if rows:
                with (out/(name+'.csv')).open('w') as f:
                    w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
        rows=[r for r in node.rows if r['phase']=='EVALUATION']
        (out/'readiness.json').write_text(json.dumps(node.readiness,indent=2)+'\n')
        (out/'recovery.json').write_text(json.dumps(node.recovery_rows,indent=2)+'\n')
        if node.sensors is not None:
            discovery=dict(topic='/jepa_joint_sensors',advertised_topics=node.sensors.node.topic_list(),topic_info=str(node.sensors.node.topic_info('/jepa_joint_sensors')))
            (out/'discovery.json').write_text(json.dumps(discovery,indent=2)+'\n')
        (out/'sensor_diagnostics.json').write_text(json.dumps(node.sensors.snapshot() if node.sensors else {},indent=2)+'\n')
        if node.finished and node.failure is None and (not node.landed or node.armed):node.failure='landing_unconfirmed'
        result=dict(status='PASS' if node.finished and node.failure is None and rows and rows[-1]['t']-rows[0]['t']>15.9 else 'FAIL',
                    failure=node.failure,ready_at_wall_s=node.ready_at,startup_only=node.startup_only,attitude_reference=os.environ.get('JEPA_ATTITUDE_REFERENCE'),phase=node.phase,samples=len(node.rows),evaluation_samples=len(rows),
                    landed=node.landed,disarmed=not node.armed,
                    sensor_samples=len(node.sensor_rows),mass_estimate_kg=node.estimator.mass if node.group in ('B','C','C_shadow') else None,
                    mass_accepted=node.estimator.accepted if node.group in ('B','C','C_shadow') else None,
                    mass_reason=node.estimator.reason if node.group in ('B','C','C_shadow') else 'disabled',
                    group=node.group,noise=os.environ.get('JEPA_NOISE'),mass_accepted_at_px4_s=node.accepted_at,
                    last_px4_timestamp_s=node.px4_timestamp*1e-6,
                    last_joint_timestamp_s=node.sensors.samples[-1][0] if node.sensors and node.sensors.samples else None,
                    position_rmse_m=float(np.sqrt(np.mean([r['error']**2 for r in rows]))) if rows else None,
                    max_position_error_m=max([r['error'] for r in rows],default=None))
        if node.startup_only and node.ready_at is not None and not node.armed:result['status']='READY_UNARMED'
        (out/'flight_result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)
        node.destroy_node();base.rclpy.shutdown()
    return 0 if result['status'] in ('PASS','READY_UNARMED') else 1


if __name__=='__main__': raise SystemExit(main())
