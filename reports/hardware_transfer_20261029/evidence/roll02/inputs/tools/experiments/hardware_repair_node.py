"""SITL-only adapter and operator for an unchanged, frozen hardware-v2 source.

The launch runner owns the PX4/agent processes. This module never connects to
domain zero and requires a runner-written capability containing the live PX4 PID.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.experiments.hardware_repair_core import (
    JOINTS, front_control, front_force, tail_control, tail_force, phase_at, sha256, write_json,
)


def check_capability(path):
    cap = json.loads(Path(path).read_text())
    if (os.environ.get('ROS_DOMAIN_ID') != str(cap['domain']) or cap['domain'] == 0 or
            os.environ.get('GZ_PARTITION') != cap['partition'] or
            os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'):
        raise RuntimeError('SITL isolation capability/environment mismatch')
    for key in ('sitl_pid', 'agent_pid'):
        os.kill(int(cap[key]), 0)
        if os.readlink(f'/proc/{cap[key]}/ns/net') != os.readlink('/proc/self/ns/net'):
            raise RuntimeError('SITL/agent network namespace mismatch')
    from tools.experiments.hardware_repair_runner import network_evidence
    network_evidence()
    if sha256(cap['binary']) != cap['binary_sha256']:
        raise RuntimeError('Firmware changed after runner attestation')
    if sha256(cap['source']) != cap['source_sha256']:
        raise RuntimeError('Controller snapshot changed')
    return cap


def load_hardware(path):
    spec = importlib.util.spec_from_file_location('hardware_repair_frozen_hw', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class GazeboClock:
    """Use the plant clock for hardware age calculations, not host RTF drift.

    Only this imported simulation module sees the proxy. Wall-time receive and
    clock-stall checks remain in the operator; the hardware source is unchanged.
    """
    def __init__(self):
        from controllers.experiments.drcda_closed_loop.feedback import JointAngleFeedback
        from gz.transport13 import Node as GzNode
        from gz.msgs10.clock_pb2 import Clock
        self.node = GzNode()
        self.value = None
        self.received = -math.inf
        if not self.node.subscribe(Clock, '/world/default/clock', self.update):
            raise RuntimeError('Cannot subscribe to Gazebo clock')
        deadline = time.monotonic() + 5.
        while self.value is None and time.monotonic() < deadline:
            time.sleep(.01)
        if self.value is None:
            raise RuntimeError('Gazebo clock missing')

    def update(self, msg):
        value = msg.sim.sec + msg.sim.nsec * 1e-9
        if self.value is not None and value < self.value:
            raise RuntimeError('Gazebo clock reset')
        self.value = value
        self.received = time.monotonic()

    def monotonic(self):
        return self.value

    def __getattr__(self, name):
        return getattr(time, name)


def adapter_type(hw, simulation):
    """Adapt only plant calibration, input wiring and diagnostic observation."""
    from controllers.experiments.sdf_wrench_model import SDFWrenchModel

    class HardwareV2SITL(hw.HnuterHardwareDRCDAV2Controller):
        def __init__(self):
            super().__init__()
            self.drcda.model = SDFWrenchModel(simulation['sdf'])
            self.diagnostic_period_s = .005
            self._repair_observer = None
            # Write AFTER adaptation; the native runtime file precedes this change.
            write_json(Path(os.environ['HNUTER_LOG_DIR']) / 'config/sitl_adapter.json', {
                'label': 'hardware-v2 logic on a Gazebo-adapted plant, not aircraft calibration',
                'model': type(self.drcda.model).__name__, 'simulation': simulation,
                'effective_allocator': {k: v.tolist() if isinstance(v, np.ndarray) else v
                                        for k, v in vars(self.drcda.config).items()},
                'rc_boundary': 'dedicated synthetic topic -> original manual_control_callback',
                'firmware_readback_verified': False,
                'hardware_protected_methods_overridden': False,
                'hardware_monotonic_clock': 'Gazebo simulation time; separate wall-time stall watchdog',
            })

        def create_subscription(self, msg_type, topic, callback, qos_profile, **kwargs):
            if topic == '/fmu/out/manual_control_setpoint':
                topic = '/hardware_repair/manual_control'
            elif topic == '/fmu/out/rc_channels':
                topic = '/hardware_repair/unused_rc_channels'
            return super().create_subscription(msg_type, topic, callback, qos_profile, **kwargs)

        def _front_thrust_to_control(self, thrust):
            return front_control(thrust, simulation['motor_constants'][0])

        def _front_control_to_thrust(self, control):
            return front_force(control, simulation['motor_constants'][0])

        def _front_reachable_thrust_max_n(self):
            return min(.5 * self.max_thrust_per_arm_n, simulation['motor_constants'][0] * 1e6)

        def _tail_thrust_to_control(self, thrust):
            force = np.clip(thrust, -self.tail_thrust_negative_n, self.tail_thrust_positive_n)
            return tail_control(force, simulation['motor_constants'][4])

        def _tail_control_to_thrust(self, control):
            return tail_force(control, simulation['motor_constants'][4])

        def _write_diagnostic_row(self, current_time, mode):
            super()._write_diagnostic_row(current_time, mode)
            observer = getattr(self, '_repair_observer', None)
            if observer is not None:
                observer.sample('hardware_control_step')

    return HardwareV2SITL


def make_operator(hw, controller, plan, output):
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from controllers.experiments.drcda_closed_loop.feedback import JointAngleFeedback
    # JointAngleFeedback loads the installed Gazebo Python path.

    class Operator(Node):
        def __init__(self):
            super().__init__('hardware_repair_sitl_operator')
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.cmd = self.create_publisher(hw.VehicleCommand, '/fmu/in/vehicle_command', qos)
            self.manual = self.create_publisher(hw.ManualControlSetpoint, '/hardware_repair/manual_control', qos)
            self.px4_manual = self.create_publisher(hw.ManualControlSetpoint, '/fmu/in/manual_control_input', qos)
            self.feedback = JointAngleFeedback()
            from gz.transport13 import Node as GzNode
            from gz.msgs10.entity_wrench_pb2 import EntityWrench
            from gz.msgs10.entity_pb2 import Entity
            self.gz = GzNode()
            self.wrench = self.gz.advertise('/world/default/wrench/persistent', EntityWrench)
            self.clear = self.gz.advertise('/world/default/wrench/clear', Entity)
            self.force_active = False
            self.force_started = False
            self.stage = 'telemetry'
            self.started = self.stage_start = time.monotonic()
            self.last_command = -math.inf
            self.start_px4 = None
            self.ground_z = None
            self.phase = None
            self.phase_wall = 0.
            self.position_hold = None
            self.attitude_hold = None
            self.completed = False
            self.done = False
            self.error = ''
            self.last_wall = 0.
            self.last_step_px4 = 0
            self.last_health = -math.inf
            self.last_state_progress_wall = time.monotonic()
            self.last_state_progress_stamp = 0
            self.rows = (output / 'samples.jsonl').open('w', buffering=1)
            self.events = (output / 'operator_events.jsonl').open('w', buffering=1)
            self.create_subscription(hw.VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.ack, qos)
            self.timer = self.create_timer(.02, self.tick)
            controller._repair_observer = self

        def event(self, kind, **details):
            self.events.write(json.dumps(dict(wall_s=time.monotonic(), kind=kind, **details),
                                         allow_nan=False) + '\n')

        def ack(self, msg):
            self.event('command_ack', command=int(msg.command), result=int(msg.result))

        def command(self, command, **params):
            msg = hw.VehicleCommand()
            msg.timestamp = controller.timestamp_now_us()
            msg.command = command
            msg.target_system = msg.source_system = 1
            msg.target_component = 1
            msg.source_component = 191
            msg.from_external = True
            for k, value in params.items():
                setattr(msg, k, float(value))
            self.cmd.publish(msg)
            self.last_command = time.monotonic()
            self.event('operator_command', command=int(command), **{
                k: v if math.isfinite(v) else None for k, v in params.items()})

        def mode(self, name):
            self.command(hw.VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                         param1=1., param2=6. if name == 'offboard' else 3.)

        def switch(self, stage):
            self.stage = stage
            self.stage_start = time.monotonic()
            self.last_command = -math.inf
            self.event('stage', stage=stage)

        def fail(self, reason):
            if self.done:
                return
            self.error = reason
            self.done = True
            self.event('aborted', reason=reason)

        def apply_load(self, elapsed):
            from gz.msgs10.entity_wrench_pb2 import EntityWrench
            from gz.msgs10.entity_pb2 import Entity
            force = plan['synthetic_force']
            active = force['enabled'] and force['start_s'] <= elapsed < force['stop_s']
            if active and not self.force_started:
                if not self.wrench.has_connections():
                    raise RuntimeError('ApplyLinkWrench subscriber missing')
                msg = EntityWrench()
                msg.entity.name, msg.entity.type = 'hnuter_0::base_link', Entity.LINK
                for axis, v in zip(('x', 'y', 'z'), force['world_force_n']):
                    setattr(msg.wrench.force, axis, v)
                for axis, v in zip(('x', 'y', 'z'), force['body_flu_attachment_from_link_origin_m']):
                    setattr(msg.wrench.force_offset, axis, v)
                if not self.wrench.publish(msg):
                    raise RuntimeError('Synthetic force publish failed')
                self.force_started = self.force_active = True
                self.event('synthetic_load_on', force=force,
                           semantics='world force, body-fixed offset from base_link origin; not real cable identification')
            elif self.force_active and not active:
                msg = Entity()
                msg.name, msg.type = 'hnuter_0::base_link', Entity.LINK
                if not self.clear.publish(msg):
                    raise RuntimeError('Synthetic force clear failed')
                self.force_active = False
                self.event('synthetic_load_off')

        def inputs(self):
            # PX4 receives neutral pilot input while the hardware parser receives
            # the commanded synthetic sticks. Never publish forged /fmu/out data.
            msg = hw.ManualControlSetpoint()
            msg.timestamp = msg.timestamp_sample = controller.timestamp_now_us()
            msg.valid = True
            msg.data_source = hw.ManualControlSetpoint.SOURCE_RC
            for name in ('roll', 'pitch', 'yaw', 'throttle', 'aux1', 'aux2'):
                setattr(msg, name, float((self.phase or {}).get('sticks', {}).get(name, 0.)))
            self.manual.publish(msg)
            msg.data_source = hw.ManualControlSetpoint.SOURCE_MAVLINK_0
            for name in ('roll', 'pitch', 'yaw', 'throttle', 'aux1', 'aux2'):
                setattr(msg, name, 0.)
            self.px4_manual.publish(msg)
            angles, age = self.feedback.read()
            if controller.armed:
                self.feedback.stop_calibration()
            # Physical joints are observed only, matching flight command_model_only.

        def tick(self):
            if self.done:
                return
            try:
                self.step()
            except Exception as exc:
                self.fail(type(exc).__name__ + ': ' + str(exc))

        def step(self):
            c = controller
            now = time.monotonic()
            self.inputs()
            if c._state_consumed_stamp != self.last_state_progress_stamp:
                self.last_state_progress_stamp = c._state_consumed_stamp
                self.last_state_progress_wall = now
            if now - hw.time.received > .3:
                self.fail('Gazebo clock stalled in wall time')
                return
            if self.stage not in ('telemetry',) and now - self.last_state_progress_wall > .3:
                self.fail('state reception/consumption stalled in wall time: ' + c._state_wait_reason)
                return
            if now - self.last_health >= 1.:
                samples, offset, error = c._state_inbox.snapshot()
                self.event('state_health', stage=self.stage,
                           wait_reason=c._state_wait_reason,
                           inbox_error=error, receiver_error=c._state_rx_failure,
                           offset=offset if math.isfinite(offset) else None,
                           consumed=int(c._state_consumed_stamp),
                           samples={k: list(v[1:]) for k, v in samples.items()})
                self.last_health = now
            if c._hardware_fault:
                self.fail('hardware fault: ' + c._hardware_fault)
                return
            if now - self.started > plan['scenario_timeout_wall_s']:
                self.fail('wall-clock timeout')
                return
            if self.stage == 'telemetry':
                angles, _ = self.feedback.read()
                if c.data_received and angles is not None and now - self.started > 3.:
                    self.ground_z = float(c.position[2])
                    self.switch('arm')
                elif now - self.stage_start > 30.:
                    self.fail('no valid telemetry/measured joint calibration')
            elif self.stage == 'arm':
                if c.armed:
                    self.switch('takeoff')
                elif now - self.stage_start > 15.:
                    self.fail('arm not observed')
                elif now - self.last_command > 2.:
                    self.command(hw.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.)
            elif self.stage == 'takeoff':
                if c.position[2] - self.ground_z >= plan['altitude_m'] - .3:
                    self.switch('position_settle')
                elif now - self.stage_start > 40.:
                    self.fail('takeoff height not reached')
                elif c.nav_state != 17 and now - self.last_command > 5.:
                    self.command(hw.VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                                 param4=math.nan, param5=math.nan, param6=math.nan, param7=math.nan)
            elif self.stage == 'position_settle':
                if now - self.last_command > 1. and c.nav_state != 2:
                    self.mode('position')
                if c.nav_state == 2 and now - self.stage_start > 4. and np.linalg.norm(c.velocity) < .4:
                    self.switch('offboard_entry')
                elif now - self.stage_start > 20.:
                    self.fail('Position settle failed')
            elif self.stage == 'offboard_entry':
                if c.is_offboard() and c._hardware_control_active:
                    self.start_px4 = c.px4_timestamp * 1e-6
                    self.switch('scenario')
                elif now - self.stage_start > plan['mode_timeout_wall_s']:
                    self.fail('Offboard entry not observed')
                elif now - self.last_command > 1.:
                    self.mode('offboard')
            elif self.stage == 'scenario':
                if not c.armed:
                    self.fail('in-flight disarm')
                    return
                elapsed = c.px4_timestamp * 1e-6 - self.start_px4
                self.apply_load(elapsed)
                phase = phase_at(plan, elapsed)
                if phase is None:
                    self.completed = self.done = True
                    self.event('scenario_complete')
                    return
                if self.phase is None or phase['name'] != self.phase['name']:
                    self.phase = phase
                    self.phase_wall = now
                    self.position_hold = c.position.copy()
                    self.attitude_hold = c.R.copy()
                    self.event('phase', **phase)
                nav = 14 if phase['mode'] == 'offboard' else 2
                active = phase['mode'] == 'offboard'
                if c.nav_state != nav or c._hardware_control_active != active:
                    if now - self.phase_wall > plan['mode_timeout_wall_s']:
                        self.fail('mode not confirmed: ' + phase['name'])
                    elif now - self.last_command > .5:
                        self.mode(phase['mode'])
                if not c._hardware_control_active:
                    self.sample('position_observer')
                if c.position[2] - self.ground_z < .7 or np.linalg.norm(c.velocity) > 5.:
                    self.fail('SITL abort envelope reached')

        def sample(self, origin):
            if self.stage != 'scenario' or self.phase is None or self.done:
                return
            c = controller
            now = time.monotonic()
            if now <= self.last_wall:
                return
            self.last_wall = now
            target = c.target_position.copy()
            target[2] += c._z0
            desired_rotation = c._rotation_enu_flu_from_euler(c.target_attitude)
            if self.phase['mode'] == 'position':
                target = self.position_hold
                desired_rotation = self.attitude_hold
            angle = math.degrees(math.acos(float(np.clip((np.trace(desired_rotation.T @ c.R)-1.)/2., -1., 1.))))
            euler = np.degrees(c._euler_from_rotation_matrix(c.R))
            angles, age = self.feedback.read()
            inbox, offset, _ = c._state_inbox.snapshot()
            control_now = hw.time.monotonic()
            ages = [max(control_now - value[3], control_now - offset - value[2] * 1e-6, 0.)
                    for value in inbox.values()]
            motors = np.asarray(c.px4_out_motor_cmd[:5], dtype=float)
            servos = np.asarray(c.px4_out_servo_cmd[:4], dtype=float)
            data = dict(origin=origin, phase=self.phase['name'], wall_s=now,
                        px4_s=c.px4_timestamp * 1e-6, scenario_s=c.px4_timestamp * 1e-6 - self.start_px4,
                        nav_state=int(c.nav_state), armed=bool(c.armed),
                        hardware_active=bool(c._hardware_control_active), fault=c._hardware_fault,
                        session_id=int(getattr(c, '_control_session_id', 0)),
                        safety_cutoff=bool(getattr(c, 'direct_safety_cutoff', False)),
                        altitude_m=float(c.position[2] - self.ground_z),
                        position_error_m=float(np.linalg.norm(c.position - target)),
                        position_enu_m=c.position.tolist(), target_enu_m=target.tolist(),
                        velocity_enu_m_s=c.velocity.tolist(), target_velocity_enu_m_s=c.target_velocity.tolist(),
                        target_acceleration_enu_m_s2=c.target_acceleration.tolist(),
                        speed_m_s=float(np.linalg.norm(c.velocity)),
                        target_speed_m_s=float(np.linalg.norm(c.target_velocity)),
                        so3_deg=angle, roll_deg=float(euler[0]), pitch_deg=float(euler[1]),
                        yaw_deg=float(euler[2]), target_yaw_deg=float(np.degrees(c.target_attitude[2])),
                        rate_deg_s=float(np.degrees(np.linalg.norm(c.angular_velocity_frd))),
                        joint_age_s=age if math.isfinite(age) else 1e9,
                        measured_joints_rad=angles.tolist() if angles is not None else None,
                        motor_controls=motors.tolist(), servo_controls=servos.tolist(),
                        motor_abs_max=float(np.max(np.abs(motors))),
                        servo_abs_max=float(np.max(np.abs(servos))),
                        source_age_s=max(ages, default=1e9),
                        source_timestamps_us={key: int(value[1]) for key, value in inbox.items()},
                        integral_position=c.integral_pos_error.tolist(),
                        integral_attitude=c.integral_e_R.tolist(),
                        wrench_requested=c.last_W.tolist(),
                        handover_blend=float(c._hardware_handover_blend),
                        synthetic_force_applied=self.force_active,
                        synthetic_world_force_n=plan['synthetic_force']['world_force_n'] if self.force_active else [0., 0., 0.],
                        synthetic_offset_link_m=plan['synthetic_force']['body_flu_attachment_from_link_origin_m'],
                        synthetic_moment_world_about_link_nm=np.cross(
                            c.R @ np.asarray(plan['synthetic_force']['body_flu_attachment_from_link_origin_m']),
                            plan['synthetic_force']['world_force_n'] if self.force_active else [0., 0., 0.]).tolist())
            # Nulls preserve the failed sample. The analyzer rejects them.
            def clean(v):
                if isinstance(v, float) and not math.isfinite(v):
                    return None
                if isinstance(v, list):
                    return [clean(x) for x in v]
                if isinstance(v, dict):
                    return {k: clean(x) for k, x in v.items()}
                return v
            self.rows.write(json.dumps(clean(data), allow_nan=False) + '\n')

    return Operator()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capability', required=True, type=Path)
    args = parser.parse_args()
    cap = check_capability(args.capability)
    output = Path(cap['output'])
    hw = load_hardware(cap['source'])
    hw.time = GazeboClock()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init()
    node = operator = executor = None
    try:
        node = adapter_type(hw, cap['simulation'])()
        node.start_state_receiver()
        operator = make_operator(hw, node, cap['plan'], output)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(operator)
        while rclpy.ok() and not operator.done:
            executor.spin_once(timeout_sec=.02)
        write_json(output / 'completion.json', dict(completed=operator.completed, error=operator.error))
        print('HARDWARE_REPAIR_DONE', flush=True)
        # Return to PX4 ownership and request landing before runner tears down SITL.
        operator.command(hw.VehicleCommand.VEHICLE_CMD_NAV_LAND)
        deadline = time.monotonic() + 12.
        while node.armed and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
    finally:
        if executor:
            executor.shutdown()
        if operator:
            operator.rows.close()
            operator.events.close()
            operator.destroy_node()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
