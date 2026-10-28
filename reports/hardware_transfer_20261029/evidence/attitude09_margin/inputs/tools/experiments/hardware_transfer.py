"""Matched no-delay SITL diagnosis of hardware-v2 transfer, not an aircraft launcher."""
from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.experiments.hardware_repair_core import (
    SOURCE, FIRMWARE, FLIGHT, PLAN, adapted_tuning, inspect_simulation, sha256, write_json,
)
from tools.experiments import hardware_repair_runner as runner

GROUPS = ('flight', 'old_gains', 'integral', 'feedback', 'legacy_cg', 'authority',
          'authority_gains', 'yaw_unscheduled', 'full_attitude')
OLD_GAINS = dict(direct_KR=[5., 5.5, 14.], direct_Domega=[3., 3.2, 5.9],
                 direct_attitude_Ki=[.03, .03, .02], direct_pos_Kp_ned=[4., 3.5, 8.],
                 direct_pos_Kd_ned=[3.8, 3.2, 4.], direct_pos_Ki_ned=[.15, .12, 3.])
INTEGRAL = dict(direct_pos_Ki_ned=[.3, .3, .2], direct_pos_integral_limit_ned=[3., 3., 2.5],
                direct_attitude_Ki=[.3, .8, .15], direct_attitude_integral_limit=[1., 2., 1.])


def phase_clock(t, period=7., ramp=.75):
    """Same C2 clock as the archived successful simulation, without ROS imports."""
    if t <= 0:
        return 0., 0., 0.
    if t >= period + ramp:
        return period, 0., 0.
    if t < ramp:
        s = t / ramp
        return ramp*(s**3-.5*s**4), 3*s*s-2*s**3, (6*s-6*s*s)/ramp
    if t < period:
        return t-ramp/2, 1., 0.
    s = (t-period)/ramp
    return period-ramp/2+ramp*(s-s**3+.5*s**4), 1-3*s*s+2*s**3, (-6*s+6*s*s)/ramp


def task_plan(task):
    plan = json.loads(PLAN.read_text())
    duration = float(task.split('_')[1]) + .75 if task.startswith('lissajous_') else (
        40. if task == 'roll_80' else 160.)
    plan['phases'] = [dict(name=n, duration_s=d, mode='offboard', sticks={})
                      for n, d in [('hold', 8.), ('maneuver', duration), ('post_hold', 5.)]]
    plan['scenario_timeout_wall_s'] = duration * 3 + 180
    plan['limits'].update(target_speed_peak_m_s=3.2, tilt_peak_deg=185., rate_peak_deg_s=120.)
    return plan


def prepare(args):
    output = args.output
    output.mkdir(parents=True, exist_ok=False)
    inputs = output / 'inputs'
    inputs.mkdir()
    source = inputs / 'hardware.py'
    shutil.copy2(SOURCE, source)
    shutil.copy2(FLIGHT, inputs / 'actual_flight_runtime.json')
    sim = inspect_simulation(args.firmware)
    base, adaptation = adapted_tuning(source, json.loads(FLIGHT.read_text()), sim)
    manifest = dict(status='PREPARED_NOT_RUN', policy='no_delay_only',
                    controller_commit=runner.command_output(['git', 'rev-parse', 'HEAD'], ROOT),
                    firmware_commit=runner.command_output(['git', 'rev-parse', 'HEAD'], args.firmware),
                    simulation=sim, plan=task_plan(args.task), cases=[], frozen_groups={},
                    operator_module='tools.experiments.hardware_transfer',
                    experiment=dict(task=args.task, reference_start_s=8.,
                                    measured_feedback_groups=['feedback']),
                    required_firmware_sha256=None,
                    adaptation=adaptation,
                    constraints_policy='Matched actuator/torque bounds; authority groups explicitly change outer acceleration limits only; no artificial plant delay')
    for group in GROUPS:
        tuning = dict(base)
        if group == 'old_gains':
            tuning.update(OLD_GAINS)
        elif group == 'integral':
            tuning.update(INTEGRAL)
        elif group == 'legacy_cg':
            tuning.update(HNTR_CG_X=sim['parameters']['HNTR_CG_X'],
                          HNTR_CG_Z=sim['parameters']['HNTR_CG_Z'])
        elif group in ('authority', 'authority_gains'):
            tuning.update(max_acc_xy=8., attitude_test_max_acc_xy=8.)
            if group == 'authority_gains':
                tuning.update(OLD_GAINS)
        elif group in ('yaw_unscheduled', 'full_attitude'):
            tuning['direct_large_tilt_yaw_scheduling_enabled'] = False
            if group == 'full_attitude':
                tuning['direct_reduced_tilt_error_enabled'] = False
        tuning_path = inputs / (group + '.json')
        write_json(tuning_path, tuning)
        manifest['frozen_groups'][group] = dict(source=str(source), source_sha256=sha256(source),
            tuning=str(tuning_path), tuning_sha256=sha256(tuning_path))
    manifest['harness_sha256'] = {str(p.relative_to(ROOT)): sha256(p) for p in (
        Path(__file__), ROOT/'tools/experiments/hardware_repair_node.py',
        ROOT/'tools/experiments/hardware_repair_runner.py', ROOT/'tools/experiments/hardware_repair_core.py')}
    for name in manifest['harness_sha256']:
        dest = inputs / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / name, dest)
    write_json(output / 'manifest.json', manifest)
    return manifest


def trajectory_adapter(hw, sim, experiment, original_factory):
    base = original_factory(hw, sim)

    class TrajectoryAdapter(base):
        def __init__(self):
            super().__init__()
            self._transfer_origin = None
            self._transfer_feedback = experiment['group'] in experiment['measured_feedback_groups']
            self.lissajous_period_s = (float(experiment['task'].split('_')[1])
                                      if experiment['task'].startswith('lissajous_') else 7.)
            self.lissajous_amp_x, self.lissajous_amp_y, self.lissajous_amp_z = 1.2, .8, .3
            self.lissajous_a, self.lissajous_b, self.lissajous_c = 2., 3., 1.
            self.attitude_segment_time_s, self.attitude_peak_hold_s = 15., 5.
            pitch = float(experiment['task'].split('_')[2]) if experiment['task'].startswith('attitude_') else 180.
            self.attitude_step_axis_rad = np.radians([80., pitch, 0.])
            self.attitude_test_bidirectional = True

        def update_trajectory(self, current_time, dt):
            observer = self._repair_observer
            if self._transfer_feedback and observer is not None:
                angles, age = observer.feedback.read()
                if angles is not None and age < .1:
                    self.drcda.measured_joint_angles = angles.copy()
                else:
                    self.drcda.measured_joint_angles = None
            if observer is None or observer.start_px4 is None or not self._hardware_control_active:
                return super().update_trajectory(current_time, dt)
            t = self.px4_timestamp * 1e-6 - observer.start_px4 - experiment['reference_start_s']
            if t < 0:
                return super().update_trajectory(current_time, dt)
            if self._transfer_origin is None:
                self._transfer_origin = self.target_position.copy()
                self.auto_traj_start_pos = self._transfer_origin.copy()
                self.auto_traj_start_attitude = self.target_attitude.copy()
                self.auto_traj_yaw = float(self.target_attitude[2])
                self.auto_traj_z = float(self._transfer_origin[2])
                self.auto_traj_origin_xy = self._transfer_origin[:2] - self._yaw_rotation_2d(
                    self.auto_traj_yaw) @ np.array([self.lissajous_amp_x, self.lissajous_amp_y])
                observer.event('trajectory_adapter_start', experiment=experiment,
                               semantics='Reference input only; original outer loop, allocation and output unchanged')
            if experiment['task'].startswith('lissajous_'):
                self.auto_traj_mode = 'lissajous'
                f, fd, fdd = phase_clock(t, self.lissajous_period_s)
                p, v, a, _ = self._lissajous_reference(f)
                self.target_position = p
                self.target_velocity = v * fd
                self.target_acceleration = a * fd**2 + v * fdd
                self.target_attitude = self.auto_traj_start_attitude.copy()
                self.target_attitude_rate = np.zeros(3)
                self.target_R_des_ned_frd = None
            else:
                self.auto_traj_mode = 'attitude'
                # Insert the archived five-second level interval after each 35s maneuver.
                total = 40. if experiment['task'] == 'roll_80' else 160.
                if t >= total:
                    att, rate, rotation = self.auto_traj_start_attitude.copy(), np.zeros(3), None
                else:
                    native_t = int(t // 40.) * 35. + min(t % 40., 35. - 1e-8)
                    att, rate, rotation, _ = self._attitude_reference(native_t)
                self.target_position = self._transfer_origin.copy()
                self.target_velocity = np.zeros(3)
                self.target_acceleration = np.zeros(3)
                self.target_attitude, self.target_attitude_rate = att, rate
                self.target_R_des_ned_frd = rotation

    return TrajectoryAdapter


def node_main():
    from tools.experiments import hardware_repair_node as node
    path = Path(sys.argv[sys.argv.index('--capability') + 1])
    cap = node.check_capability(path)
    factory = node.adapter_type
    node.adapter_type = lambda hw, sim: trajectory_adapter(hw, sim, cap['experiment'], factory)
    node.main()


def main():
    if '--capability' in sys.argv:
        return node_main()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--firmware', type=Path, default=FIRMWARE)
    parser.add_argument('--firmware-sha256', required=True)
    parser.add_argument('--task', choices=('lissajous_7', 'lissajous_12', 'roll_80',
                                         'attitude_80_180', 'attitude_80_160'), required=True)
    parser.add_argument('--group', action='append', choices=GROUPS)
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--netns-child', action='store_true')
    args = parser.parse_args()
    if args.repeat < 1:
        parser.error('--repeat must be positive')
    args.output = args.output.resolve()
    if not args.netns_child:
        env = dict(os.environ, HARDWARE_REPAIR_PARENT_NETNS=os.readlink('/proc/self/ns/net'))
        return subprocess.run(['unshare', '--user', '--map-root-user', '--net', sys.executable,
            str(Path(__file__).resolve()), *sys.argv[1:], '--netns-child'], env=env).returncode
    subprocess.run(['ip', 'link', 'set', 'lo', 'up'], check=True)
    runner.network_evidence()
    with open('/tmp/hnuter-hardware-repair-sitl.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        manifest_path = args.output / 'manifest.json'
        manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else prepare(args)
        if manifest['experiment']['task'] != args.task:
            raise RuntimeError('Cannot change task inside a campaign')
        for path, digest in manifest['harness_sha256'].items():
            if sha256(ROOT / path) != digest:
                raise RuntimeError('Harness changed: create a new campaign')
        snapshot = args.output / 'firmware_snapshot'
        if not snapshot.exists():
            runner.freeze_firmware(args.output, args.firmware, args.firmware_sha256, manifest)
        elif manifest['required_firmware_sha256'] != args.firmware_sha256:
            raise RuntimeError('Firmware hash mismatch')
        manifest['status'] = 'RUNNING'
        write_json(manifest_path, manifest)
        for repeat in range(1, args.repeat + 1):
            for group in args.group or GROUPS:
                if any(c['group'] == group and c['repeat'] == repeat for c in manifest['cases']):
                    continue
                frozen = manifest['frozen_groups'][group]
                for key in ('source', 'tuning'):
                    if sha256(frozen[key]) != frozen[key + '_sha256']:
                        raise RuntimeError('Frozen input changed: ' + key)
                result = runner.run_case(args.output, manifest, snapshot, group, 'none', repeat)
                manifest['cases'].append(result)
                write_json(manifest_path, manifest)
        manifest['status'] = 'COMPLETED_WITH_RECORDED_VERDICTS'
        write_json(manifest_path, manifest)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
