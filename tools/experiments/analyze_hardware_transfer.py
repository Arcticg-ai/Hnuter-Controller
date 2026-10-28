"""Summarize transfer diagnostics, keeping incomplete runs and changed constraints visible."""
import argparse
import csv
import gzip
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def open_trace(path):
    path = Path(path)
    return path.open() if path.exists() else gzip.open(str(path)+'.gz', 'rt')


def weighted_rms(t, x):
    t, x = np.asarray(t), np.asarray(x)
    return float(np.sqrt(np.trapz(x*x, t) / (t[-1]-t[0]))) if len(t) > 1 else None


def summarize(case):
    case = dict(case, outcome=('FAIL_CONTROL_ABORT' if case['operator_error'] ==
                              'SITL abort envelope reached' else case['status']))
    with open_trace(Path(case['case'])/'samples.jsonl') as handle:
        rows = [json.loads(line) for line in handle if line.strip()]
    selected = [r for r in rows if r['phase'] == 'maneuver']
    if not selected:
        return dict(case, maneuver=None), rows
    t = np.array([r['scenario_s'] for r in selected])
    force = np.array([r['target_acceleration_enu_m_s2'] for r in selected])
    acc = np.max(np.abs(force[:, :2]), axis=1)
    metrics = dict(duration_s=float(t[-1]-t[0]),
                   position_rms_m=weighted_rms(t, [r['position_error_m'] for r in selected]),
                   position_peak_m=max(r['position_error_m'] for r in selected),
                   so3_rms_deg=weighted_rms(t, [r['so3_deg'] for r in selected]),
                   so3_peak_deg=max(r['so3_deg'] for r in selected),
                   acceleration_ff_axis_peak_m_s2=float(acc.max()),
                   acceleration_ff_axis_fraction_above_3=float(np.mean(acc > 3)))
    return dict(case, maneuver=metrics), rows


def plot(campaign, cases, output):
    colors = dict(flight='#2a6fbb', old_gains='#bd5652', integral='#b68725',
                  feedback='#438c72', legacy_cg='#858585', authority='#2b9574',
                  authority_gains='#7c559c')
    colors.update(yaw_unscheduled='#168a79', full_attitude='#c45c88')
    labels = dict(flight='Flight gains / COM-consistent', old_gains='Old simulation gains',
                  integral='C2 integral candidate', feedback='Measured-joint diagnostic',
                  legacy_cg='Legacy double CG trim', authority='8 m/s2 authority diagnostic',
                  authority_gains='8 m/s2 + old gains')
    labels.update(yaw_unscheduled='No body-yaw down-scheduling', full_attitude='Full quaternion attitude')
    if campaign == 'lissajous01':
        labels['flight'] = 'Flight gains / legacy CG adapter'
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.grid': True, 'grid.alpha': .18, 'savefig.bbox': 'tight'})
    fig, axes = plt.subplots(3, 2, figsize=(11.6, 9.1), sharex=True)
    fields = [('position_error_m', 'Position error [m]'), ('so3_deg', 'SO(3) error [deg]'),
              ('roll_deg', 'Roll [deg]'), ('pitch_deg', 'Pitch [deg]'),
              ('altitude_m', 'Altitude [m]'), ('target_speed_m_s', 'Reference speed [m/s]')]
    for result, rows in cases:
        group = result['group']
        if not rows:
            continue
        t = np.array([r['scenario_s'] for r in rows]) - 8.
        for ax, (field, label) in zip(axes.flat, fields):
            ax.plot(t, [r[field] for r in rows], lw=1.2, color=colors[group],
                    ls='-' if result['repeat'] % 2 else '--',
                    label=f"{labels[group]} r{result['repeat']} [{result['outcome']}]")
            ax.set_ylabel(label)
            ax.axvline(0, lw=.7, color='#555555', ls=':')
    for ax in axes[-1]:
        ax.set_xlabel('Time since maneuver onset [s]')
    axes[0, 0].legend(fontsize=7, loc='upper left', bbox_to_anchor=(0, 1.52), ncol=2, frameon=False)
    fig.suptitle(campaign + ' | no-delay Gazebo | direct motors/servos', y=1.02, fontsize=12)
    fig.tight_layout()
    fig.savefig(output / (campaign+'.png'), dpi=180)
    fig.savefig(output / (campaign+'.pdf'))
    plt.close(fig)
    if not campaign.startswith('liss'):
        return
    fig = plt.figure(figsize=(11.6, 4.8))
    ax3 = fig.add_subplot(121, projection='3d')
    ax = fig.add_subplot(122)
    for index, (result, rows) in enumerate(cases):
        rows = [r for r in rows if r['phase'] == 'maneuver']
        if not rows:
            continue
        p = np.array([r['position_enu_m'] for r in rows])
        d = np.array([r['target_enu_m'] for r in rows])
        # Translate and yaw-align each run's task frame, not the tracking error.
        yaw = np.radians(rows[0]['target_yaw_deg'])
        r = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        origin = d[0].copy()
        p, d = p-origin, d-origin
        p[:, :2], d[:, :2] = p[:, :2] @ r, d[:, :2] @ r
        if index == 0:
            ax3.plot(*d.T, color='black', ls='--', lw=1, label='Reference')
        ax3.plot(*p.T, color=colors[result['group']], lw=1.1, label=result['group'])
        a = np.max(np.abs(np.array([v['target_acceleration_enu_m_s2'] for v in rows])[:, :2]), axis=1)
        if index == 0:
            ax.plot([v['scenario_s']-8 for v in rows], a, color='#252525', label='Reference acceleration')
    ax.axhline(3, color='#bd5652', ls='--', label='Flight outer-loop limit: 3')
    ax.axhline(8, color='#2b9574', ls=':', label='Diagnostic limit: 8')
    ax.set(xlabel='Maneuver time [s]', ylabel='Max absolute horizontal FF component [m/s2]')
    ax.legend(fontsize=8, frameon=False)
    ax3.set(xlabel='Task X [m]', ylabel='Task Y [m]', zlabel='Task Z [m]')
    ax3.legend(fontsize=7, frameon=False)
    fig.tight_layout()
    fig.savefig(output/(campaign+'_mechanism.png'), dpi=180)
    fig.savefig(output/(campaign+'_mechanism.pdf'))
    plt.close(fig)


def attitude_mechanism(cases, output):
    chosen = [(c, r) for c, r in cases if c['group'] in ('flight', 'yaw_unscheduled', 'full_attitude')]
    if not chosen:
        return
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    colors = dict(flight='#858585', yaw_unscheduled='#168a79', full_attitude='#c45c88')
    for case, rows in chosen:
        t = np.array([r['scenario_s'] for r in rows])-8.
        w = np.array([r['wrench_requested'] for r in rows])
        axes[0].plot(t, [r['so3_deg'] for r in rows], color=colors[case['group']], label=case['group'])
        axes[1].plot(t, w[:, 5], color=colors[case['group']])
        if 'yaw_authority_scale' in rows[0]:
            axes[2].plot(t, [r['yaw_authority_scale'] for r in rows], color=colors[case['group']])
        else:
            directory = Path(case['case'])/'logs/external_control/hardware/drcda_v2'
            sources = list(directory.glob('*.csv')) + list(directory.glob('*.csv.gz'))
            source = sources[0] if sources else next(Path(case['case']).glob('*.csv.gz'))
            stamps, scales = [], []
            origin = rows[0]['px4_s'] - rows[0]['scenario_s'] + 8.
            with (gzip.open(source, 'rt') if source.suffix == '.gz' else source.open()) as handle:
                for row in csv.DictReader(handle):
                    timing = json.loads(row['v2_state_timing_json'])
                    stamp = timing['source_timestamps_us']['attitude'] * 1e-6 - origin
                    if t[0] <= stamp <= t[-1]:
                        stamps.append(stamp)
                        scales.append(float(row['yaw_authority_scale']))
            axes[2].plot(stamps, scales, color=colors[case['group']])
    axes[0].set_ylabel('SO(3) error [deg]')
    axes[0].legend(frameon=False, fontsize=8)
    axes[1].set_ylabel('Requested body-Z torque [Nm]')
    axes[1].axhline(.08, color='#777777', ls=':', label='Old scaled limit')
    axes[1].axhline(-.08, color='#777777', ls=':')
    axes[2].set_ylabel('Body-yaw authority scale')
    axes[2].set_ylim(-.05, 1.1)
    axes[2].set_xlabel('Time since roll onset [s]')
    for ax in axes:
        ax.axvspan(15, 20, color='#999999', alpha=.1)
    axes[2].text(15.2, .3, '80 deg hold', fontsize=8)
    fig.tight_layout()
    fig.savefig(output/'roll_control_mechanism.png', dpi=180)
    fig.savefig(output/'roll_control_mechanism.pdf')
    plt.close(fig)


def full_attitude_tracking(campaign, cases, output, pitch_amplitude=180):
    from scipy.spatial.transform import Rotation
    fig, axes = plt.subplots(2, 2, figsize=(11.6, 6.8), sharex=True)
    for case, rows in cases:
        t = np.array([r['scenario_s'] for r in rows])-8.
        euler = np.array([[r['roll_deg'], r['pitch_deg'], r['yaw_deg']] for r in rows])
        rotations = Rotation.from_euler('xyz', euler, degrees=True).as_matrix()
        frame = Rotation.from_euler('z', rows[0]['target_yaw_deg'], degrees=True).as_matrix()
        relative = frame.T @ rotations
        angles = np.column_stack((np.arctan2(relative[:, 2, 1], relative[:, 2, 2]),
                                  np.arctan2(-relative[:, 2, 0], relative[:, 0, 0])))
        desired = np.array([r['target_attitude_enu_deg'][:2] for r in rows])
        for i, ax in enumerate(axes.flat):
            selected = (t >= 40*i) & (t < 40*(i+1))
            axis = i % 2
            actual = np.degrees(np.unwrap(angles[selected, axis]))
            ax.plot(t[selected]-40*i, desired[selected, axis], color='#333333', ls='--', label='Reference')
            ax.plot(t[selected]-40*i, actual, color='#168a79', lw=1.2, label='Measured attitude')
            ax.axvspan(15, 20, color='#999999', alpha=.1)
            ax.set_title(('+80 deg roll', f'+{pitch_amplitude:g} deg pitch',
                          '-80 deg roll', f'-{pitch_amplitude:g} deg pitch')[i])
            ax.set_ylabel('Task-axis rotation [deg]')
    axes[0, 0].legend(frameon=False, fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel('Time within maneuver segment [s]')
    fig.suptitle('Full attitude task | continuous task-axis angles, not wrapped Euler pitch')
    fig.tight_layout()
    fig.savefig(output/(campaign+'_tracking.png'), dpi=180)
    fig.savefig(output/(campaign+'_tracking.pdf'))
    plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(9, 7.6), sharex=True)
    for case, rows in cases:
        selected = [r for r in rows if 48 <= r['scenario_s'] < 88]
        if not selected or 'allocator_command' not in selected[0]:
            continue
        t = np.array([r['scenario_s'] for r in selected])-48.
        target = np.array([r['target_attitude_enu_deg'][1] for r in selected])
        command = np.degrees(np.array([r['allocator_command'][:4] for r in selected]))
        measured = np.degrees(np.array([r['measured_joints_rad'] for r in selected]))
        axes[0].plot(t, target, color='#333333')
        for index, name, color in ((0, 'Left primary', '#168a79'), (2, 'Right primary', '#bd5652')):
            axes[1].plot(t, command[:, index], color=color, label=name+' command')
            axes[1].plot(t, measured[:, index], color=color, ls='--', lw=.8, label=name+' measured')
        axes[2].plot(t, [r['position_error_m'] for r in selected], color='#2a6fbb')
    axes[1].axhline(-180, color='#555555', ls=':', label='Active primary limit')
    axes[2].axhline(.75, color='#bd5652', ls=':', label='Position criterion')
    for ax in axes:
        ax.axvspan(15, 20, color='#999999', alpha=.1)
    axes[0].set_ylabel('Desired pitch [deg]')
    axes[1].set_ylabel('Primary joint [deg]')
    axes[2].set_ylabel('Position error [m]')
    axes[2].set_xlabel('Time within positive-pitch segment [s]')
    axes[1].legend(ncol=2, fontsize=7, frameon=False)
    axes[2].legend(fontsize=8, frameon=False)
    fig.suptitle(f'{campaign} | primary-joint margin diagnostic')
    fig.tight_layout()
    fig.savefig(output/(campaign+'_joint_margin.png'), dpi=180)
    fig.savefig(output/(campaign+'_joint_margin.pdf'))
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    summary = []
    roll_cases = []
    for path in sorted(args.root.glob('*/manifest.json')):
        manifest = json.loads(path.read_text())
        cases = []
        for case in manifest['cases']:
            local = path.parent/'runs'/case['disturbance']/f"repeat_{case['repeat']}"/case['group']
            case = dict(case, case=str(local))
            sample = Path(case['case'])/'samples.jsonl'
            if sample.exists() or Path(str(sample)+'.gz').exists():
                result, rows = summarize(case)
                cases.append((result, rows))
                summary.append(dict(campaign=path.parent.name, task=manifest['experiment']['task'], **result))
            else:
                summary.append(dict(campaign=path.parent.name, maneuver=None, **case))
        if cases:
            plot(path.parent.name, cases, args.output)
            if manifest['experiment']['task'].startswith('attitude_'):
                full_attitude_tracking(path.parent.name, cases, args.output,
                                       float(manifest['experiment']['task'].split('_')[2]))
            if manifest['experiment']['task'] == 'roll_80':
                roll_cases.extend(cases)
    if roll_cases:
        plot('roll_matched_diagnostics', roll_cases, args.output)
        plot('roll_candidate_zoom', [(c, r) for c, r in roll_cases
             if c['group'] in ('yaw_unscheduled', 'full_attitude')], args.output)
        attitude_mechanism(roll_cases, args.output)
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    lines = ['| Campaign | Group | Repeat | Outcome | Recorded maneuver (s) | Position RMS / peak (m) | SO3 RMS / peak (deg) |',
             '|---|---|---:|---|---:|---:|---:|']
    for c in summary:
        m = c['maneuver']
        if m:
            lines.append(f"| {c['campaign']} | {c['group']} | {c['repeat']} | {c['outcome']} | {m['duration_s']:.2f} | "
                         f"{m['position_rms_m']:.3f} / {m['position_peak_m']:.3f} | "
                         f"{m['so3_rms_deg']:.2f} / {m['so3_peak_deg']:.2f} |")
    (args.output/'metrics.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
