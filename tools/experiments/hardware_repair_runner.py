#!/usr/bin/env python3
"""Freeze and compare real hardware-v2 control logic in isolated, no-delay SITL.

Default action is preparation only. Execution requires an explicitly confirmed
firmware SHA256 and enters a loopback-only Linux network namespace.
"""
from __future__ import annotations

import argparse
import copy
import fcntl
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.experiments.hardware_repair_core import (
    BASELINE, FIRMWARE, FLIGHT, GROUPS, PLAN, SOURCE, JOINTS,
    adapted_tuning, analyze_samples, inspect_simulation, sha256, validate_plan, write_json,
)
from tools.experiments.run_no_delay_maneuver_experiments import PtyProcess, drain_for, wait_for


def command_output(command, cwd=None):
    return subprocess.check_output(command, cwd=cwd, text=True).strip()


def isolated_environment(plan, partition):
    # No inherited HNUTER overrides or remote discovery/transport profiles.
    env = {k: v for k, v in os.environ.items() if not k.startswith(('HNUTER_', 'PX4_', 'GZ_', 'ROS_'))
           and k not in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                         'CYCLONEDDS_URI', 'RMW_IMPLEMENTATION')}
    env.update(ROS_DOMAIN_ID=str(plan['ros_domain_id']), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_LOCALHOST_ONLY='1', GZ_PARTITION=partition, GZ_IP='127.0.0.1',
               HEADLESS='1', PX4_SIM_MODEL='gz_hnuter', PX4_GZ_WORLD='default',
               PX4_UXRCE_DDS_PORT='8894', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
               PX4_SIM_SPEED_FACTOR='1', HNUTER_GZ_WORLD='default',
               HNUTER_HARDWARE_DRCDA_ENABLE='1', HNUTER_HARDWARE_TASKS='0')
    if plan['ros_domain_id'] != 174:
        raise ValueError('This campaign requires isolated ROS/PX4 domain 174')
    return env


def prepare(args):
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    sim = inspect_simulation(args.firmware)
    plan = validate_plan(json.loads(args.plan.read_text()))
    actual = json.loads(args.flight.read_text())
    snapshot = output / 'inputs'
    snapshot.mkdir()
    shutil.copy2(args.flight, snapshot / 'actual_flight_runtime.json')
    if args.baseline is not None:
        shutil.copy2(args.baseline, snapshot / 'baseline.py')
    else:
        reconstruct_baseline(snapshot / 'baseline.py')
    shutil.copy2(args.fixed, snapshot / 'fixed.py')
    manifest = dict(status='PREPARED_NOT_RUN', policy='no_delay_only', plan=plan,
                    simulation=sim, created_wall_ns=time.time_ns(),
                    flight_source=str(args.flight), flight_sha256=sha256(args.flight),
                    source_originals=dict(baseline=str(args.baseline), fixed=str(args.fixed)),
                    controller_commit=command_output(['git', 'rev-parse', 'HEAD'], ROOT),
                    firmware_commit=command_output(['git', 'rev-parse', 'HEAD'], args.firmware),
                    firmware_status=command_output(['git', 'status', '--short'], args.firmware),
                    frozen_groups={}, cases=[], required_firmware_sha256=None)
    for group in GROUPS:
        source = snapshot / ('baseline.py' if group == 'baseline' else 'fixed.py')
        tuning, adaptation = adapted_tuning(source, actual, sim,
                                            plan['candidate'] if group == 'tuned' else {})
        write_json(snapshot / f'{group}.json', tuning)
        write_json(snapshot / f'{group}.adaptation.json', adaptation)
        manifest['frozen_groups'][group] = dict(source=str(source), source_sha256=sha256(source),
            tuning=str(snapshot / f'{group}.json'), tuning_sha256=sha256(snapshot / f'{group}.json'))
    manifest['harness_sha256'] = {str(p.relative_to(ROOT)): sha256(p)
        for p in sorted((ROOT / 'tools/experiments').glob('hardware_repair_*.py'))}
    write_json(output / 'manifest.json', manifest)
    print(json.dumps(dict(status=manifest['status'], output=str(output),
                         actual_pitch_bias=actual['tuning']['HNTR_PITCH_BIAS'],
                         actual_Domega=actual['tuning']['direct_Domega']), indent=2))
    return manifest


def reconstruct_baseline(destination):
    archive = ROOT / 'reports/hardware_repair_20261027'
    provenance = json.loads((archive / 'baseline_provenance.json').read_text())
    source = subprocess.check_output(['git', 'show', provenance['fixed_commit'] +
        ':controllers/hardware/hnuter_external_direct_drcda_v2_hardware.py'], cwd=ROOT)
    destination.write_bytes(source)
    if sha256(destination) != provenance['fixed_sha256']:
        raise RuntimeError('Baseline reconstruction source does not match provenance')
    subprocess.run(['patch', '--batch', str(destination),
                    str(archive / 'baseline_from_bf9d06b.patch')], check=True)
    if sha256(destination) != provenance['baseline_sha256']:
        raise RuntimeError('Reconstructed baseline hash mismatch')


def freeze_firmware(output, firmware, expected_sha, manifest):
    build = firmware / 'build/px4_sitl_default'
    binary = build / 'bin/px4'
    if sha256(binary) != expected_sha:
        raise RuntimeError('Firmware binary does not match coordinated SHA256')
    target = output / 'firmware_snapshot'
    # Reuse only immutable, fully verified experiment snapshots, never hard-link
    # the live build checkout. This avoids a second binary copy for every task.
    for candidate in sorted(output.parent.glob('*/firmware_snapshot')):
        previous_path = candidate.parent / 'manifest.json'
        if candidate == target or not previous_path.exists():
            continue
        previous = json.loads(previous_path.read_text())
        hashes = previous.get('firmware_snapshot_sha256', {})
        if (previous.get('required_firmware_sha256') != expected_sha or not hashes or
                previous['simulation']['sdf_sha256'] != manifest['simulation']['sdf_sha256'] or
                previous['simulation']['airframe_sha256'] != manifest['simulation']['airframe_sha256']):
            continue
        if any(not (candidate/name).is_file() or sha256(candidate/name) != digest
               for name, digest in hashes.items()):
            continue
        shutil.copytree(candidate, target, symlinks=True, copy_function=os.link)
        manifest['required_firmware_sha256'] = expected_sha
        manifest['firmware_snapshot_sha256'] = hashes
        manifest['snapshot_storage'] = 'hardlinks to verified frozen snapshot: ' + str(candidate)
        manifest['simulation']['sdf'] = str(target / 'models/hnuter/model.sdf')
        write_json(output / 'manifest.json', manifest)
        return target
    target.mkdir(exist_ok=False)
    shutil.copytree(build / 'bin', target / 'bin', symlinks=True)
    shutil.copytree(build / 'etc', target / 'etc')
    shutil.copytree(firmware / 'Tools/simulation/gz/models/hnuter', target / 'models/hnuter')
    shutil.copytree(build / 'src/modules/simulation/gz_plugins', target / 'plugins',
                    ignore=shutil.ignore_patterns('CMakeFiles', '*.o', '*.a'))
    shutil.copy2(firmware / 'src/modules/simulation/gz_bridge/server.config', target / 'server.config')
    (target / 'worlds').mkdir()
    server = ET.parse(target / 'server.config')
    if not any(p.get('name') == 'gz::sim::systems::ApplyLinkWrench'
               for p in server.getroot().iter('plugin')):
        raise RuntimeError('Expected installed ApplyLinkWrench plugin in server config')
    shutil.copy2(firmware / 'Tools/simulation/gz/worlds/default.sdf', target / 'worlds/default.sdf')
    if sha256(binary) != expected_sha or sha256(target / 'bin/px4') != expected_sha:
        raise RuntimeError('Firmware changed during snapshot')
    if sha256(target / 'models/hnuter/model.sdf') != manifest['simulation']['sdf_sha256']:
        raise RuntimeError('SDF changed since prepare; create a new campaign')
    manifest['required_firmware_sha256'] = expected_sha
    manifest['firmware_snapshot_sha256'] = {str(p.relative_to(target)): sha256(p)
        for p in target.rglob('*') if p.is_file() and not p.is_symlink()}
    manifest['simulation']['sdf'] = str(target / 'models/hnuter/model.sdf')
    write_json(output / 'manifest.json', manifest)
    return target


def network_evidence():
    evidence = json.loads(command_output(['ip', '-json', 'address', 'show']))
    if {item['ifname'] for item in evidence} != {'lo'}:
        raise RuntimeError('Refusing simulation commands: network namespace is not loopback-only')
    if os.readlink('/proc/self/ns/net') == os.environ.get('HARDWARE_REPAIR_PARENT_NETNS'):
        raise RuntimeError('Network namespace did not change')
    return dict(network_namespace=os.readlink('/proc/self/ns/net'), interfaces=evidence,
                domain=174, agent_port=8894, client_agent_ip='127.0.0.1',
                agent_bind_note='UDP wildcard inside loopback-only namespace; no LAN interfaces',
                discovery='LOCALHOST', parent_network_namespace=os.environ.get('HARDWARE_REPAIR_PARENT_NETNS'))


def run_case(output, manifest, snapshot, group, disturbance, repeat):
    if shutil.disk_usage(output).free < 256 * 1024**2:
        raise RuntimeError('Less than 256 MiB free: archive generated data before starting another case')
    case = output / 'runs' / disturbance / f'repeat_{repeat}' / group
    case.mkdir(parents=True, exist_ok=False)
    rootfs = case / 'rootfs'
    rootfs.mkdir()
    partition = 'hardware-repair-' + uuid.uuid4().hex
    plan = copy.deepcopy(manifest['plan'])
    plan['synthetic_force']['enabled'] = disturbance == 'synthetic_up'
    env = isolated_environment(plan, partition)
    env.update(PX4_GZ_MODELS=str(snapshot / 'models'), PX4_GZ_WORLDS=str(snapshot / 'worlds'),
               GZ_SIM_RESOURCE_PATH=f'{snapshot}/models:{snapshot}/worlds',
               GZ_SIM_SYSTEM_PLUGIN_PATH=str(snapshot / 'plugins'),
               GZ_SIM_SERVER_CONFIG_PATH=str(snapshot / 'server.config'),
               PATH=str(snapshot / 'bin') + ':' + env['PATH'],
               HNUTER_LOG_DIR=str(case / 'logs'),
               HNUTER_TUNING_FILE=manifest['frozen_groups'][group]['tuning'])
    write_json(case / 'isolation.json', dict(network_evidence(), environment={
        k: v for k, v in env.items() if k.startswith(('ROS_', 'PX4_', 'GZ_', 'HNUTER_'))}))
    processes = []
    error = ''
    started = time.monotonic()
    try:
        agent = PtyProcess(['MicroXRCEAgent', 'udp4', '-p', '8894', '-v', '4'], ROOT, env, case / 'console/agent.log')
        processes.append(agent)
        drain_for(processes, 1.)
        if agent.process.poll() is not None:
            raise RuntimeError('Agent failed')
        sitl = PtyProcess([str(snapshot / 'bin/px4'), '-i', '0', '-w', str(rootfs),
                           '-s', 'etc/init.d-posix/rcS', str(snapshot / 'etc')],
                          rootfs, env, case / 'console/px4.log')
        processes.append(sitl)
        if not wait_for(processes, sitl, 'Ready for takeoff!', 80.):
            raise RuntimeError('SITL boot timeout')
        # Explicitly restart even though rcS inherits ROS_DOMAIN_ID/port.
        for line in ('uxrce_dds_client stop', 'param set UXRCE_DDS_DOM_ID 174',
                     'param set UXRCE_DDS_AG_IP 2130706433', 'param set UXRCE_DDS_PRT 8894',
                     'param set MIS_TAKEOFF_ALT 3', 'param set COM_RC_IN_MODE 1',
                     'param set COM_DISARM_PRFLT 30',
                     'uxrce_dds_client start -t udp -h 127.0.0.1 -p 8894',
                     'param show UXRCE_DDS_DOM_ID', 'param show UXRCE_DDS_AG_IP',
                     'param show UXRCE_DDS_PRT', 'uxrce_dds_client status', 'ver all'):
            sitl.send(line + '\n')
            drain_for(processes, .25)
        drain_for(processes, 2.)
        if '174' not in sitl.buffer or '8894' not in sitl.buffer or '127.0.0.1' not in sitl.buffer:
            raise RuntimeError('DDS restart/isolation evidence missing')
        # Actual socket evidence is captured from inside the same namespace.
        (case / 'sockets.txt').write_text(command_output(['ss', '-lunp']) + '\n')
        if sha256(manifest['frozen_groups'][group]['source']) != manifest['frozen_groups'][group]['source_sha256']:
            raise RuntimeError('Frozen controller source modified')
        cap = dict(output=str(case), domain=174, partition=partition, plan=plan,
                   source=manifest['frozen_groups'][group]['source'],
                   source_sha256=manifest['frozen_groups'][group]['source_sha256'],
                   binary=str(snapshot / 'bin/px4'), binary_sha256=manifest['required_firmware_sha256'],
                   sitl_pid=sitl.process.pid, agent_pid=agent.process.pid, simulation=manifest['simulation'])
        if 'experiment' in manifest:
            cap['experiment'] = dict(manifest['experiment'], group=group)
        write_json(case / 'capability.json', cap)
        node = PtyProcess([sys.executable, '-m', manifest.get(
                           'operator_module', 'tools.experiments.hardware_repair_node'),
                           '--capability', str(case / 'capability.json')],
                          ROOT, env, case / 'console/controller.log')
        processes.append(node)
        deadline = time.monotonic() + plan['scenario_timeout_wall_s'] + 20.
        while node.process.poll() is None and time.monotonic() < deadline:
            drain_for(processes, .1)
        if node.process.poll() is None:
            raise RuntimeError('Operator did not terminate in wall-clock budget')
        if node.process.returncode != 0:
            raise RuntimeError('Controller/operator process failed')
    except Exception as exc:
        error = type(exc).__name__ + ': ' + str(exc)
    finally:
        for process in reversed(processes):
            try:
                process.stop(graceful_text='shutdown\n' if process is locals().get('sitl') else None)
            finally:
                # PX4 can exit before its child Gazebo processes do.
                try:
                    os.killpg(process.process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
    path = case / 'samples.jsonl'
    rows = [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
    completion = json.loads((case / 'completion.json').read_text()) if (case / 'completion.json').exists() else {}
    verdict = analyze_samples(rows, plan, completion.get('completed', False))
    if error:
        verdict['status'] = 'INVALID'
        verdict['invalid'].append(error)
    result = dict(group=group, disturbance=disturbance, repeat=repeat,
                  elapsed_wall_s=time.monotonic()-started, operator_error=completion.get('error', ''),
                  case=str(case), **verdict)
    write_json(case / 'result.json', result)
    print(json.dumps(result, indent=2), flush=True)
    return result


def plot_campaign(output, manifest):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for disturbance in ('none', 'synthetic_up'):
        cases = [c for c in manifest['cases'] if c['disturbance'] == disturbance]
        if not cases:
            continue
        fig, axes = plt.subplots(4, 2, figsize=(13, 12), sharex=True)
        fields = [('position_error_m', 'Position error [m]'), ('so3_deg', 'SO(3) error [deg]'),
                  ('pitch_deg', 'Pitch [deg]'), ('altitude_m', 'Altitude [m]'),
                  ('target_speed_m_s', 'Target speed [m/s]'), ('rate_deg_s', 'Body rate [deg/s]'),
                  ('source_age_s', 'State age above min transport [s]'), ('joint_age_s', 'Joint feedback age [s]')]
        colors = dict(baseline='#666666', ref_fix='#1675b6', tuned='#c04444')
        for case in cases:
            path = Path(case['case']) / 'samples.jsonl'
            if not path.exists():
                continue
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            for ax, (field, label) in zip(axes.flat, fields):
                ax.plot([r['scenario_s'] for r in rows], [r.get(field) for r in rows],
                        color=colors[case['group']], alpha=.8, linewidth=.9,
                        linestyle='-' if case['repeat'] % 2 else '--',
                        label=f"{case['group']} r{case['repeat']} {case['status']}")
                ax.set_ylabel(label)
                ax.grid(alpha=.2)
        axes[0, 0].legend(fontsize=7)
        for ax in axes[-1]:
            ax.set_xlabel('PX4 scenario time [s]')
        fig.suptitle(f'Hardware-v2 SITL / {disturbance} / no injected delay / not flight evidence')
        fig.tight_layout()
        fig.savefig(output / f'closed_loop_{disturbance}.png', dpi=150)
        fig.savefig(output / f'closed_loop_{disturbance}.pdf')
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--plan', type=Path, default=PLAN)
    parser.add_argument('--baseline', type=Path, default=BASELINE)
    parser.add_argument('--fixed', type=Path, default=SOURCE)
    parser.add_argument('--flight', type=Path, default=FLIGHT)
    parser.add_argument('--firmware', type=Path, default=FIRMWARE)
    parser.add_argument('--firmware-sha256')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--netns-child', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--repeat', type=int, default=2)
    parser.add_argument('--group', choices=GROUPS, action='append')
    parser.add_argument('--disturbance', choices=('none', 'synthetic_up'), action='append')
    args = parser.parse_args()
    args.output = args.output.resolve()
    if not args.execute:
        prepare(args)
        return
    if not args.firmware_sha256 or args.repeat < 1:
        parser.error('--execute requires coordinated --firmware-sha256 and positive --repeat')
    if not args.netns_child:
        # Fail closed, not a fallback to the host network if unshare is unavailable.
        env = os.environ.copy()
        env['HARDWARE_REPAIR_PARENT_NETNS'] = os.readlink('/proc/self/ns/net')
        result = subprocess.run(['unshare', '--user', '--map-root-user', '--net', sys.executable,
                                 str(Path(__file__).resolve()), *sys.argv[1:], '--netns-child'], env=env)
        raise SystemExit(result.returncode)
    subprocess.run(['ip', 'link', 'set', 'lo', 'up'], check=True)
    network_evidence()
    with open('/tmp/hnuter-hardware-repair-sitl.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if not (args.output / 'manifest.json').exists():
            prepare(args)
        manifest = json.loads((args.output / 'manifest.json').read_text())
        snapshot = args.output / 'firmware_snapshot'
        if not snapshot.exists():
            freeze_firmware(args.output, args.firmware, args.firmware_sha256, manifest)
        elif manifest['required_firmware_sha256'] != args.firmware_sha256:
            raise RuntimeError('Cannot change firmware inside an existing campaign')
        manifest['status'] = 'RUNNING'
        write_json(args.output / 'manifest.json', manifest)
        for disturbance in args.disturbance or ('none', 'synthetic_up'):
            for repeat in range(1, args.repeat + 1):
                groups = args.group or list(GROUPS)
                for group in groups if repeat % 2 else reversed(groups):
                    if any(c['group'] == group and c['repeat'] == repeat and c['disturbance'] == disturbance
                           for c in manifest['cases']):
                        continue
                    result = run_case(args.output, manifest, snapshot, group, disturbance, repeat)
                    manifest['cases'].append(result)
                    write_json(args.output / 'manifest.json', manifest)
                    plot_campaign(args.output, manifest)
                    if result['status'] == 'INVALID':
                        manifest['status'] = 'STOPPED_INVALID'
                        write_json(args.output / 'manifest.json', manifest)
                        raise SystemExit('Run incomplete or evidence invalid; preserve the failure and inspect operator_error. This may be a control failure, not infrastructure.')
        manifest['status'] = 'COMPLETED'
        write_json(args.output / 'manifest.json', manifest)


if __name__ == '__main__':
    main()
