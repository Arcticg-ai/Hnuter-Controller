#!/usr/bin/env python3
"""Export simulation-only evidence; never copy flight logs or remote credentials."""
from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path
import shutil
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.experiments.analyze_hardware_repair import analyze, uniform

CAMPAIGNS = ('pilot01', 'pilot02', 'campaign03', 'loaded04', 'authority05',
             'projection_stress06')


def window(rows, start, end):
    if not rows or rows[0]['scenario_s'] > start+.15 or rows[-1]['scenario_s'] < end-.15:
        return None
    result = {}
    for field, prefix in [('position_error_m', 'position_m'), ('so3_deg', 'so3_deg'),
                          ('target_speed_m_s', 'reference_speed_m_s'), ('rate_deg_s', 'rate_deg_s')]:
        _, values = uniform(rows, field, start, end)
        result[prefix+'_rmse'] = float(np.sqrt(np.mean(values**2)))
        result[prefix+'_peak'] = float(np.max(np.abs(values)))
    return result


def bundle(source, destination):
    destination.mkdir(parents=True, exist_ok=True)
    audit = []
    for name in CAMPAIGNS:
        campaign = source/name
        manifest = json.loads((campaign/'manifest.json').read_text())
        target = destination/name
        target.mkdir(exist_ok=True)
        if name not in ('pilot01', 'pilot02'):
            analyze(campaign)
            shutil.copy2(campaign/'detailed_metrics.json', target/'detailed_metrics.json')
            for pattern in ('error_details_*', 'handover_details_*', 'aborted_*'):
                for path in campaign.glob(pattern):
                    if path.suffix in ('.png', '.pdf'):
                        shutil.copy2(path, target/path.name)
        provenance = {key: manifest.get(key) for key in (
            'status', 'policy', 'plan', 'controller_commit', 'firmware_commit',
            'required_firmware_sha256', 'harness_sha256')}
        provenance['sources'] = {group: {key: data[key] for key in ('source_sha256', 'tuning_sha256')}
                                 for group, data in manifest['frozen_groups'].items()}
        provenance['simulation'] = {key: value for key, value in manifest['simulation'].items()
                                     if key not in ('sdf', 'airframe')}
        (target/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
        for case in manifest['cases']:
            sample_path = Path(case['case'])/'samples.jsonl'
            rows = [json.loads(s) for s in sample_path.read_text().splitlines()] if sample_path.exists() else []
            entry = {k: case[k] for k in ('group', 'disturbance', 'repeat', 'status', 'invalid', 'failures')}
            entry.update(campaign=name, operator_error=case.get('operator_error', ''),
                         operational_outcome='FAIL' if case.get('operator_error') else case['status'],
                         pre_handover=window(rows, 0, 37), settled=window(rows, 16, 37),
                         post_handover=window(rows, 37, 65),
                         recorded_duration_s=rows[-1]['scenario_s'] if rows else 0.)
            if rows:
                entry['raw_reference_speed_peak_m_s'] = max(r['target_speed_m_s'] for r in rows)
                # Raw-rate extrema matter for spikes; 50 Hz interpolated RMSE
                # avoids bias from differing publication rates.
                fields = ('scenario_s', 'position_error_m', 'so3_deg', 'roll_deg', 'pitch_deg',
                          'yaw_deg', 'target_speed_m_s', 'rate_deg_s', 'altitude_m', 'nav_state',
                          'position_enu_m', 'target_enu_m', 'target_velocity_enu_m_s',
                          'motor_controls', 'servo_controls', 'measured_joints_rad')
                filename = f"{case['disturbance']}_{case['group']}_r{case['repeat']}.jsonl.gz"
                with gzip.open(target/filename, 'wt') as stream:
                    for row in rows:
                        stream.write(json.dumps({k: row[k] for k in fields}, allow_nan=False)+'\n')
                entry['simulation_trace'] = name+'/'+filename
            audit.append(entry)
    (destination/'summary.json').write_text(json.dumps(audit, indent=2, allow_nan=False)+'\n')
    overview(source, destination)
    return audit


def overview(source, destination):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False, 'pdf.fonttype': 42})
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
    for campaign, group, label, color in (
        ('loaded04', 'ref_fix', 'Reference repair only', '#0072B2'),
        ('authority05', 'tuned', 'C2 integral candidate', '#D55E00')):
        manifest = json.loads((source/campaign/'manifest.json').read_text())
        for case in manifest['cases']:
            if case['group'] != group or case['disturbance'] != 'synthetic_up':
                continue
            rows = [json.loads(s) for s in (Path(case['case'])/'samples.jsonl').read_text().splitlines()]
            t = np.array([r['scenario_s'] for r in rows])
            for ax, field, lo, hi in ((axes[0, 0], 'so3_deg', 0, 37),
                                      (axes[0, 1], 'position_error_m', 0, 37),
                                      (axes[1, 0], 'so3_deg', 37, 65)):
                mask = (t >= lo) & (t <= hi)
                ax.plot(t[mask], np.array([r[field] for r in rows])[mask], color=color,
                        linestyle='-' if case['repeat']==1 else '--', linewidth=1.1,
                        alpha=1. if case['repeat']==1 else .6,
                        label=label if case['repeat']==1 else None)
                ax.set_xlim(lo, hi)
    manifest = json.loads((source/'projection_stress06/manifest.json').read_text())
    for case in manifest['cases']:
        rows = [json.loads(s) for s in (Path(case['case'])/'samples.jsonl').read_text().splitlines()]
        axes[1, 1].plot([r['scenario_s'] for r in rows], [r['target_speed_m_s'] for r in rows],
                        color='#777777' if case['group']=='baseline' else '#0072B2',
                        label='Original projection' if case['group']=='baseline' else 'Continuous reference',
                        linewidth=1.1)
    axes[1, 1].set_yscale('symlog', linthresh=.2)
    for ax, title, label in zip(axes.flat,
        ['A  Loaded DRCDA, before mode changes', 'B  Loaded DRCDA, before mode changes',
         'C  Mode returns: unresolved instability', 'D  Separate lateral-stress test'],
        ['SO(3) error [deg]', 'Position error norm [m]', 'SO(3) error [deg]', 'Reference speed [m/s; symlog]']):
        ax.set_title(title, loc='left', fontsize=11)
        ax.set_ylabel(label)
        ax.set_xlabel('Time since first Offboard entry [s]')
        ax.grid(alpha=.2)
    axes[0, 0].legend(fontsize=9)
    axes[1, 1].legend(fontsize=9)
    for lo, hi in ((37, 42), (48, 53)):
        axes[1, 0].axvspan(lo, hi, color='#009E73', alpha=.08)
    fig.suptitle('Repair evidence and remaining failures | no-delay Gazebo\n'
                 'Solid/dashed: repetitions; green shading: Position; not real-flight validation', fontsize=13)
    for ext in ('png', 'pdf'):
        fig.savefig(destination/f'repair_overview.{ext}', dpi=200)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, default=ROOT/'reports/hardware_repair_20261027/evidence')
    args = parser.parse_args()
    result = bundle(args.source.resolve(), args.output.resolve())
    print(f'Exported {len(result)} attempted cases, including incomplete and failed runs.')
