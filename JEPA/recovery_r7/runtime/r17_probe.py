"""PX4-only observation logger and zero-path adapter. No true joints/parameters as input."""
import json,os,time,traceback,math
from pathlib import Path
import numpy as np
# Preload the isolated snapshot fix before legacy modules prepend their paths.
import sys,hashlib
sys.path.insert(0,str(Path(__file__).resolve().parent/'fixed'))
import actuator_sensors
assert Path(actuator_sensors.__file__).resolve()==Path(__file__).resolve().parent/'fixed/actuator_sensors.py'
print('SNAPSHOT_MODULE',actuator_sensors.__file__,hashlib.sha256(Path(actuator_sensors.__file__).read_bytes()).hexdigest(),flush=True)
import probe as legacy
from px4_msgs.msg import ActuatorServos
from force_adapter import ForceAdapter
from snapshot_gate import joint_readiness
from residual_port import apply_residual,force_profile
from rclpy.qos import qos_profile_sensor_data
# Existing dependency returns numpy.bool_; normalize at the local serialization boundary.
_validate_target=legacy.validate_target
legacy.validate_target=lambda candidate: bool(_validate_target(candidate))

class LoggedPublisher:
    def __init__(self,wrapped,node):self.wrapped=wrapped;self.node=node
    def publish(self,msg):
        n=self.node
        n.command_log.write(json.dumps(dict(sequence=n.command_sequence,send_monotonic_ns=time.monotonic_ns(),
            px4_message_timestamp_us=int(msg.timestamp),phase=n.phase,elapsed=n.elapsed,
            **n.port_record,actual_acceleration_ned=list(map(float,msg.acceleration)),actual_position_ned=list(map(float,msg.position)),
            actual_velocity_ned=list(map(float,msg.velocity)),actual_jerk=list(map(float,msg.jerk)),actual_yaw=float(msg.yaw)))+'\n')
        return self.wrapped.publish(msg)
    def __getattr__(self,key):return getattr(self.wrapped,key)

class Probe(legacy.Probe):
    def __init__(self,out):
        self.reference_anchor_written=False
        self.pre_basic_result=None
        self.stability_rows=[]
        self.r12_arm=os.environ.get('R12_ARM','00')
        self.unarmed_ready_since=None
        self.unarmed_hold_s=float(os.environ.get('JEPA_UNARMED_HOLD_S','30'))
        self.health_log=(out/'health_events.jsonl').open('w',buffering=1)
        self.r6_mode=os.environ.get('R6_MODE','baseline')
        self.servo=np.zeros(4);self.servo_stamp=None;self.motor_stamp=None
        self.position_stamp=self.attitude_stamp=self.rate_stamp=None
        self.receipts={};self.command_sequence=0
        self.force_adapter=ForceAdapter();self.force_ff=np.zeros(3);self.force_last=None;self.force_valid=False
        self.command_log=(out/'commands_causal.jsonl').open('w',buffering=1)
        self.observations=(out/'deployable_observations.jsonl').open('w',buffering=1)
        self.zero_checks=0;self.zero_max=0.;self.port_record={}
        self.force_schedule=None
        super().__init__(out)
        self.trajectory_setpoint_pub=LoggedPublisher(self.trajectory_setpoint_pub,self)
        self.create_subscription(ActuatorServos,'/fmu/out/actuator_servos',self.on_servos,qos_profile_sensor_data)
    def on_control(self,msg):
        super().on_control(msg)
        now=int(msg.timestamp);dt=.004 if self.force_last is None else (now-self.force_last)*1e-6
        self.force_last=now
        enabled=self.r6_mode.endswith('_force') and 18<=self.elapsed<76 and self.valid and not self.landed
        self.force_ff,self.force_valid=self.force_adapter.step(list(msg.force_body),list(msg.torque_command),self.R,float(msg.effective_mass),dt,enabled)
    def watchdog(self):
        try:
            result=super().watchdog()
            now=time.monotonic()
            self.health_log.write(json.dumps(dict(monotonic_ns=time.monotonic_ns(),wall_ns=time.time_ns(),boottime_ns=time.clock_gettime_ns(time.CLOCK_BOOTTIME),phase=self.phase,armed=bool(self.armed),landed=bool(self.landed),px4_us=int(self.px4_timestamp),position_age_s=now-self.last_pos_wall,attitude_age_s=now-self.last_att_wall,aligned_age_s=now-self.last_aligned_wall,failure=self.failure))+'\n')
            if self.startup_only and self.failure is None and not self.armed:
                ready=self.valid and now-self.last_pos_wall<.2 and now-self.last_att_wall<.2 and now-self.last_aligned_wall<.2
                if not ready:self.unarmed_ready_since=None
                elif self.unarmed_ready_since is None:self.unarmed_ready_since=now
                if self.finished and (self.unarmed_ready_since is None or now-self.unarmed_ready_since<self.unarmed_hold_s):self.finished=False
            return result
        except Exception:
            (self.out/'watchdog_exception.txt').write_text(traceback.format_exc())
            self.request=None;self.mode='baseline';self.decision_done=True
            self.land('watchdog_exception')
    def received(self,key,msg):
        self.receipts[key]=dict(receive_monotonic_ns=time.monotonic_ns(),receive_wall_ns=time.time_ns(),
            source_publish_us=int(msg.timestamp),source_sample_us=int(getattr(msg,'timestamp_sample',msg.timestamp)),
            has_sample_timestamp=hasattr(msg,'timestamp_sample'))
    def on_servos(self,msg):
        self.received('servo',msg);self.servo=np.array(msg.control[:4]);self.servo_stamp=int(msg.timestamp)
    def on_motors(self,msg):
        self.received('motor',msg);self.motor_stamp=int(msg.timestamp);super().on_motors(msg)
    def local_position_callback(self,msg):
        self.received('position',msg);self.position_stamp=int(msg.timestamp);super().local_position_callback(msg)
    def attitude_callback(self,msg):
        self.received('attitude',msg);self.attitude_stamp=int(msg.timestamp);super().attitude_callback(msg)
    def angular_velocity_callback(self,msg):
        self.received('rate',msg);self.rate_stamp=int(msg.timestamp);super().angular_velocity_callback(msg)
    def observe(self):
        # No call to the legacy joint-sensor feature builder. It is diagnostic-only.
        t=self.position_stamp
        if t is None or any(v is None for v in [self.servo_stamp,self.motor_stamp,self.attitude_stamp,self.rate_stamp]):return
        if t*1e-6-self.last_observation<.039:return
        x=np.r_[self.position,self.velocity,self.R.ravel(),self.angular_velocity,
                self.motor,self.servo]
        if not np.isfinite(x).all():return
        self.last_observation=t*1e-6
        available_time=max(t,self.attitude_stamp,self.rate_stamp,self.motor_stamp,self.servo_stamp)*1e-6
        now_ns=time.monotonic_ns()
        record=dict(receive_cutoff_monotonic_ns=now_ns,receive_cutoff_wall_ns=time.time_ns(),
             stream_timing=self.receipts,receipt_ages_s={k:(now_ns-v['receive_monotonic_ns'])*1e-9 for k,v in self.receipts.items()},
             last_command_sequence=self.command_sequence,t=available_time,position_publish_time_s=t*1e-6,position_sample_time_s=self.receipts['position']['source_sample_us']*1e-6,elapsed=self.elapsed,phase=self.phase,position_enu=self.position.tolist(),
             velocity_enu=self.velocity.tolist(),R_flu_enu=self.R.tolist(),omega_flu=self.angular_velocity.tolist(),
             motors_normalized=self.motor.tolist(),servos_normalized=self.servo.tolist(),
             source_stamps_us=dict(position=t,attitude=self.attitude_stamp,rate=self.rate_stamp,motor=self.motor_stamp,servo=self.servo_stamp),
             ref_p_enu=self.target_position.tolist(),ref_v_enu=self.target_velocity.tolist(),ref_a_enu=self.target_acceleration.tolist())
        self.observations.write(json.dumps(record)+'\n')
        # Keep launcher quality contract, but never store truth joints in x.
        self.obs_file.write(json.dumps(dict(t=available_time,elapsed=self.elapsed,phase=self.phase,x=x.tolist()))+'\n')
    def publish_px4_trajectory_setpoint(self):
        if not self.reference_anchor_written and self.started is not None and self.elapsed>=48 and self.phase!='LAND':
            if self.pre_basic_result is None:
                recent=[r for r in self.stability_rows if 45<=r['elapsed']<48]
                self.pre_basic_result=bool(recent and recent[-1]['elapsed']-recent[0]['elapsed']>=2.9 and all(r['ready'] for r in recent))
                (self.out/'runtime_stability.json').write_text(json.dumps(dict(passed=self.pre_basic_result,rows=recent),indent=2))
            if not self.pre_basic_result:
                self.land('preintervention_stability_gate');return
            native_gate=self.out/'preintervention_servo_admission.json'
            if not native_gate.exists():
                if self.elapsed>=48.8:self.land('native_servo_gate_not_ready');return
                return self.publish_pending_nominal()
            if not json.loads(native_gate.read_text())['passed']:
                self.land('native_servo_pre_gate');return
            epoch=float(self.sim_start_time_s+self.started)
            if abs(epoch-(int(self.px4_timestamp)*1e-6-self.elapsed))>.008:raise RuntimeError('reference epoch mismatch')
            schedule=dict(start=epoch+50.,end=epoch+58.,force_y=.6,mass_kg=5.3999999800000005,reference_epoch=epoch,arm=self.r12_arm)
            target=self.out/'reference_epoch.txt';tmp=target.with_suffix('.tmp');tmp.write_text(format(epoch,'.17g')+'\n');os.replace(tmp,target)
            (self.out/'aligned_schedule.json').write_text(json.dumps(schedule,indent=2));self.force_schedule=schedule
            self.reference_anchor_written=True
        return self.publish_pending_nominal()
    def publish_pending_nominal(self):
        original=self.target_acceleration.copy();requested=np.zeros(3);fresh=True
        if self.r6_mode.endswith('_force'):
            requested=self.force_ff.copy()
            fresh=bool(self.force_valid and self.force_last is not None and abs(int(self.px4_timestamp)-self.force_last)<150000)
        if self.r12_arm[1]=='1' and self.phase=='EVALUATION' and self.force_schedule is not None:
            requested[1]=-force_profile(int(self.px4_timestamp)*1e-6,self.force_schedule)/self.force_schedule['mass_kg']
        final,applied,valid=apply_residual(original,requested,fresh)
        if not np.any(requested):
            assert np.array_equal(final.view(np.uint64),original.astype(np.float64).view(np.uint64))
            self.zero_checks+=1
        self.target_acceleration=final;self.command_sequence+=1
        self.port_record=dict(requested_ff_enu=requested.tolist(),applied_ff_enu=applied.tolist(),fresh=bool(fresh),
            residual_valid=valid,residual_applied=bool(np.any(applied)),original_acceleration_enu=original.tolist(),
            final_acceleration_enu=final.tolist(),learned_control_enabled=False)
        try:return super().publish_px4_trajectory_setpoint()
        finally:self.target_acceleration=original
    def update_trajectory(self,current_time,dt):
        legacy.preserved.base.HnuterController.update_trajectory(self,current_time,dt)
        if not self.armed or not self.is_offboard():return
        if self.started is None:
            self.started=current_time;self.flight_origin=self.position[:2].copy()
            epoch=float(self.sim_start_time_s+self.started)
            tmp=self.out/'diagnostic_epoch.tmp';tmp.write_text(json.dumps(dict(epoch=epoch,epoch_ns=int(round(epoch*1e9)))))
            os.replace(tmp,self.out/'diagnostic_epoch.json')
        elapsed=current_time-self.started;self.elapsed=elapsed
        self.roll_ref=self.pitch_ref=0.;self.target_attitude_rate=np.zeros(3)
        self.target_position=np.array([*self.flight_origin,self.takeoff_height]);self.target_velocity=np.zeros(3);self.target_acceleration=np.zeros(3)
        if elapsed<12:
            self.phase='TAKEOFF';duration=10.;s=min(elapsed/duration,1.);h=self.takeoff_height
            self.target_position[2]=h*(10*s**3-15*s**4+6*s**5)
            self.target_velocity[2]=h/duration*(30*s**2-60*s**3+30*s**4)
            self.target_acceleration[2]=h/duration**2*(60*s-180*s**2+120*s**3)
        elif elapsed<48:self.phase='CALIBRATION' # level hover; retained logger quality interval
        elif elapsed<68:self.phase='EVALUATION'
        elif elapsed<72:self.phase='FINAL_HOVER'
        else:self.land();return
        # Common small reference, driven by exactly the disturbance reference epoch.
        if 50<=elapsed<60:
            z=elapsed-50
            if z<2:s=z/2;sign=1.;offset=0.
            elif z<8:s=1.;sign=0.;offset=.02
            else:s=(z-8)/2;sign=-1.;offset=.02
            self.target_position[0]+=offset+sign*.02*(10*s**3-15*s**4+6*s**5)
            self.target_velocity[0]=sign*.01*(30*s**2-60*s**3+30*s**4)
            self.target_acceleration[0]=sign*.005*(60*s-180*s**2+120*s**3)
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

        self.rows[-1].update(roll_ref=0.,pitch_ref=0.)
        if 44<=elapsed<48:
            raw=self.sensors.raw_at(int(self.px4_timestamp)*1e-6) if self.sensors else None
            joint_detail=joint_readiness(raw,int(self.px4_timestamp))
            # R16 prospective criterion; retain old20 result separately.
            joint_detail['old_absolute20_ok']=joint_detail['ok']
            joint_ok=bool(joint_detail['reason'] in ('ready','rate') and joint_detail['max_rate_deg_s']<=math.degrees(10) and all(abs(raw[1][n][0])<=(math.radians(185) if n.endswith('2') else math.pi) for n in ['rj2','lj2','rj1','lj1']))
            joint_detail['new_physical_snapshot_ok']=joint_ok
            ready=bool(self.valid and self.last_error<=.08 and np.linalg.norm(self.velocity)<=.12 and tilt<=math.radians(5) and joint_ok)
            self.stability_rows.append(dict(elapsed=elapsed,ready=ready,error_m=self.last_error,speed_m_s=float(np.linalg.norm(self.velocity)),tilt_deg=math.degrees(tilt),joint_ok=joint_ok,joint_snapshot=joint_detail))

    def destroy_node(self):
        self.health_log.close();self.observations.close();self.command_log.close()
        (self.out/'zero_interface.json').write_text(json.dumps(dict(mode=self.r6_mode,checks=self.zero_checks,max_abs_difference=self.zero_max,learned_control_enabled=False),indent=2))
        return super().destroy_node()

if __name__=='__main__':
    preserved=legacy.preserved;init=preserved.base.rclpy.init
    def namespaced_init(*args,**kwargs):
        ns=json.loads((Path(os.environ['JEPA_RUN_DIR'])/'launch.json').read_text())['namespace']
        for name in ['hnuter_control_status','actuator_servos']:
            kwargs['args']+=['-r','/fmu/out/'+name+':=/'+ns+'/fmu/out/'+name]
        return init(*args,**kwargs)
    preserved.base.rclpy.init=namespaced_init;preserved.Probe=Probe
    raise SystemExit(preserved.main())
