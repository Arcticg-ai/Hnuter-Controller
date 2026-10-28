#!/usr/bin/env python3
"""Time-uniform DRCDA repair metrics and figures; preserves failed cases."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

COLORS = {'baseline': '#777777', 'ref_fix': '#0072B2', 'tuned': '#D55E00'}
LABELS = {'baseline': 'Original reference', 'ref_fix': 'Continuous reference',
          'tuned': 'Continuous + integral candidate'}


def uniform(rows, key, start, end):
    t = np.asarray([r['scenario_s'] for r in rows])
    v = np.asarray([r[key] for r in rows])
    grid = np.arange(max(start, t[0]), min(end, t[-1]), .02)
    if v.ndim == 1:
        return grid, np.interp(grid, t, v)
    return grid, np.column_stack([np.interp(grid, t, col) for col in v.T])


def phase_metrics(rows, phase):
    start, end = phase['start_s'], phase['end_s']
    _, ep = uniform(rows, 'position_error_m', start, end)
    _, er = uniform(rows, 'so3_deg', start, end)
    _, pos = uniform(rows, 'position_enu_m', start, end)
    _, target = uniform(rows, 'target_enu_m', start, end)
    _, vel = uniform(rows, 'target_velocity_enu_m_s', start, end)
    return dict(position_rmse_m=float(np.sqrt(np.mean(ep**2))),
                position_peak_m=float(np.max(ep)),
                position_axis_rmse_m=np.sqrt(np.mean((pos-target)**2, axis=0)).tolist(),
                so3_rmse_deg=float(np.sqrt(np.mean(er**2))), so3_peak_deg=float(er.max()),
                reference_speed_peak_m_s=float(np.linalg.norm(vel, axis=1).max()))


def firmware_handover(case, rows):
    """Use PX4's own active Hnuter target, not generic attitude_setpoint."""
    from pyulog import ULog
    paths = list(Path(case['case']).rglob('*.ulg'))
    if len(paths) != 1:
        raise ValueError('Expected one ULog per case')
    ulog = ULog(str(paths[0]))
    status = ulog.get_dataset('vehicle_status').data
    control = ulog.get_dataset('hnuter_control_status').data
    attitude = ulog.get_dataset('vehicle_attitude').data
    motors = ulog.get_dataset('actuator_motors').data
    servos = ulog.get_dataset('actuator_servos').data
    nav = status['nav_state']
    switches = np.flatnonzero((nav[1:] == 2) & (nav[:-1] == 14)) + 1
    result = []
    for index in switches:
        stamp = status['timestamp'][index]
        mask = (control['timestamp'] >= stamp) & (control['timestamp'] < stamp + 1e6)
        mask &= control['active'].astype(bool)
        if not mask.any():
            raise ValueError('No actual Hnuter control target after Position return')
        first = np.flatnonzero(mask)[0]
        j = np.searchsorted(attitude['timestamp'], control['timestamp'][first], side='right')-1
        q0 = np.array([attitude[f'q[{i}]'][j] for i in range(4)], dtype=float)
        qd = np.array([control[f'attitude_setpoint_q[{i}]'][first] for i in range(4)])
        angle = np.degrees(2*np.arccos(np.clip(abs(q0 @ qd)/(np.linalg.norm(q0)*np.linalg.norm(qd)), 0, 1)))
        sample = dict(ulog_exit_time_s=float(stamp*1e-6),
                      first_target_from_measured_so3_deg=float(angle),
                      first_second_attitude_error_peak_deg=float(np.degrees(
                          np.max(control['attitude_error_angle'][mask]))))
        for name, data, count in [('motor', motors, 5), ('servo', servos, 4)]:
            k = int(np.searchsorted(data['timestamp'], stamp))
            if k == 0 or k >= len(data['timestamp']):
                raise ValueError('Missing actuator samples around mode transition')
            sample[name+'_edge_delta_normalized'] = [float(data[f'control[{i}]'][k]-
                                                          data[f'control[{i}]'][k-1]) for i in range(count)]
            sample[name+'_edge_sample_gap_s'] = float((data['timestamp'][k]-data['timestamp'][k-1])*1e-6)
        result.append(sample)
    if len(result) != 2:
        raise ValueError(f'Expected two confirmed Position returns, found {len(result)}')
    return result


def analyze(root):
    manifest = json.loads((root/'manifest.json').read_text())
    phases, start = [], 0.
    for phase in manifest['plan']['phases']:
        phases.append(dict(phase, start_s=start, end_s=start+phase['duration_s']))
        start += phase['duration_s']
    results, traces = [], []
    for case in manifest['cases']:
        item = {k: case[k] for k in ('group', 'disturbance', 'repeat', 'status', 'invalid', 'failures')}
        item['operator_error'] = case.get('operator_error', '')
        item['tracking_statistics_eligible'] = case['status'] != 'INVALID'
        # Incomplete tracking evidence is not permission to hide an operational
        # abort. Keep the raw verdict AND the failure outcome visible.
        item['operational_outcome'] = ('FAIL' if item['operator_error'] else case['status'])
        if case['status'] == 'INVALID':
            path = Path(case['case'])/'samples.jsonl'
            if path.exists():
                rows = [json.loads(line) for line in path.read_text().splitlines()]
                if rows:
                    traces.append((case, rows))
            results.append(item)
            continue
        rows = [json.loads(line) for line in (Path(case['case'])/'samples.jsonl').read_text().splitlines()]
        item['phases'] = {p['name']: phase_metrics(rows, p) for p in phases}
        item['firmware_handover'] = firmware_handover(case, rows)
        results.append(item)
        traces.append((case, rows))
    (root/'detailed_metrics.json').write_text(json.dumps(results, indent=2, allow_nan=False)+'\n')
    plot(root, traces, phases)
    return results


def plot(root, traces, phases):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'pdf.fonttype': 42, 'axes.linewidth': .7})
    for disturbance in ('none', 'synthetic_up'):
        selected = [(c, r) for c, r in traces
                    if c['disturbance'] == disturbance and c['status'] != 'INVALID']
        if not selected:
            continue
        fig, axes = plt.subplots(3, 2, figsize=(12, 8.5), sharex=True, layout='constrained')
        for case, rows in selected:
            t = np.array([r['scenario_s'] for r in rows])
            pos = np.array([r['position_enu_m'] for r in rows])
            ref = np.array([r['target_enu_m'] for r in rows])
            errors = pos-ref
            values = [errors[:, 0], errors[:, 1], errors[:, 2],
                      [r['so3_deg'] for r in rows], [r['target_speed_m_s'] for r in rows],
                      [r['rate_deg_s'] for r in rows]]
            for ax, value in zip(axes.flat, values):
                ax.plot(t, value, color=COLORS[case['group']], linewidth=1.05,
                        alpha=.9 if case['repeat']==1 else .5,
                        linestyle='-' if case['repeat']==1 else '--',
                        label=LABELS[case['group']] if case['repeat']==1 else None)
        labels = ['East position error [m]', 'North position error [m]', 'Up position error [m]',
                  'SO(3) error [deg]', 'Reference speed [m/s]', 'Body rate norm [deg/s]']
        for ax, label in zip(axes.flat, labels):
            ax.set_ylabel(label)
            ax.grid(alpha=.18)
            for phase in phases:
                if phase['mode'] == 'position':
                    ax.axvspan(phase['start_s'], phase['end_s'], color='#009E73', alpha=.08)
            ax.set_xlim(0, phases[-1]['end_s'])
        axes[0, 0].legend(fontsize=8, loc='upper left')
        for ax in axes[-1]:
            ax.set_xlabel('Time since first Offboard entry [s]')
        fig.suptitle('DRCDA hardware-path repair | '+disturbance.replace('_', ' ')+
                     '\nNo-delay Gazebo; shaded intervals: Position; dashed: repeat 2')
        for ext in ('png', 'pdf'):
            fig.savefig(root/f'error_details_{disturbance}.{ext}', dpi=200)
        plt.close(fig)

        fig, axes = plt.subplots(3, 2, figsize=(12, 8.5), sharex='col', layout='constrained')
        for case, rows in selected:
            if case['repeat'] != 1:
                continue
            t = np.array([r['scenario_s'] for r in rows])
            values = [np.asarray([r['motor_controls'] for r in rows]),
                      np.asarray([r['servo_controls'] for r in rows])]
            for column, (start, end) in enumerate(((36.5, 38.5), (47.5, 49.5))):
                mask = (t >= start) & (t <= end)
                if not mask.any():
                    continue
                axes[0, column].plot(t[mask], np.array([r['pitch_deg'] for r in rows])[mask],
                                     color=COLORS[case['group']], label=LABELS[case['group']])
                axes[1, column].plot(t[mask], values[0][mask, 4], color=COLORS[case['group']])
                for i in range(4):
                    axes[2, column].plot(t[mask], values[1][mask, i], color=COLORS[case['group']],
                                         linestyle=['-', '--', ':', '-.'][i], linewidth=.9)
                axes[0, column].set_title('Position return '+str(column+1))
            for column in range(2):
                for ax, label in zip(axes[:, column], ['Pitch [deg]', 'Tail command [-1, 1]', 'Servo command [-1, 1]']):
                    ax.set_ylabel(label)
                    ax.grid(alpha=.18)
        axes[0, 0].legend(fontsize=7)
        for ax in axes[-1]:
            ax.set_xlabel('Time since first Offboard entry [s]')
        fig.suptitle('Actuator-command continuity | '+disturbance.replace('_', ' ')+
                     '\nCommand feedback is not measured RPM or joint response')
        for ext in ('png', 'pdf'):
            fig.savefig(root/f'handover_details_{disturbance}.{ext}', dpi=200)
        plt.close(fig)

    for case, rows in traces:
        if case['status'] != 'INVALID':
            continue
        fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True, layout='constrained')
        t = [r['scenario_s'] for r in rows]
        for ax, field, label in zip(axes, ['position_error_m', 'so3_deg', 'altitude_m'],
                                    ['Position error [m]', 'SO(3) error [deg]', 'Altitude [m]']):
            ax.plot(t, [r[field] for r in rows], color='#B33A3A', linewidth=1.)
            ax.set_ylabel(label)
            ax.grid(alpha=.18)
        axes[-1].set_xlabel('Time since first Offboard entry [s]')
        fig.suptitle('ABORTED: '+case['group']+' / '+case['disturbance']+
                     '\n'+case.get('operator_error', 'Incomplete evidence'))
        name = f"aborted_{case['disturbance']}_{case['group']}_r{case['repeat']}"
        for ext in ('png', 'pdf'):
            fig.savefig(root/f'{name}.{ext}', dpi=200)
        plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaign', type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.campaign.resolve()), indent=2))
