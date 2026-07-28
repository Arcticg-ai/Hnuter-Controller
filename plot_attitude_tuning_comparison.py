#!/usr/bin/env python3
"""Compare large-roll attitude tuning logs without Euler-angle singularities."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import median

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt


def rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float):
    roll, pitch, yaw = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def dot(left, right):
    return sum(a * b for a, b in zip(left, right))


def column(matrix, index):
    return tuple(row[index] for row in matrix)


def relative_angle_deg(desired, current):
    trace = sum(
        desired[row][col] * current[row][col]
        for row in range(3)
        for col in range(3)
    )
    cosine = max(-1.0, min(1.0, 0.5 * (trace - 1.0)))
    return math.degrees(math.acos(cosine))


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def load_roll_samples(path: Path):
    with path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))

    samples = []
    for row in rows:
        if row.get('auto_traj_mode') != 'attitude':
            continue
        target_roll = float(row['target_roll_deg'])
        target_pitch = float(row['target_pitch_deg'])
        if abs(target_pitch) > 1.0 or abs(target_roll) < 1.0:
            continue

        desired = rotation_matrix(
            target_roll, target_pitch, float(row['target_yaw_deg'])
        )
        current = rotation_matrix(
            float(row['roll_deg']),
            float(row['pitch_deg']),
            float(row['yaw_deg']),
        )
        desired_z = column(desired, 2)
        current_z = column(current, 2)
        alignment = max(-1.0, min(1.0, dot(desired_z, current_z)))

        yaw = math.radians(float(row['target_yaw_deg']))
        current_z_yaw_frame = (
            math.cos(yaw) * current_z[0] + math.sin(yaw) * current_z[1],
            -math.sin(yaw) * current_z[0] + math.cos(yaw) * current_z[1],
            current_z[2],
        )
        samples.append({
            'source_time_s': float(row['time_s']),
            'target_roll_deg': target_roll,
            'thrust_roll_deg': math.degrees(math.atan2(
                -current_z_yaw_frame[1], current_z_yaw_frame[2]
            )),
            'thrust_axis_error_deg': math.degrees(math.acos(alignment)),
            'full_attitude_error_deg': relative_angle_deg(desired, current),
            'horizontal_error_m': math.hypot(
                float(row['position_x_enu_m']) - float(row['target_x_enu_m']),
                float(row['position_y_enu_m']) - float(row['target_y_enu_m']),
            ),
            'altitude_error_m': (
                float(row['position_z_rel_m']) - float(row['target_z_rel_m'])
            ),
        })

    if not samples:
        raise ValueError(f'no roll attitude samples found in {path}')
    intervals = [
        right['source_time_s'] - left['source_time_s']
        for left, right in zip(samples, samples[1:])
        if 0.0 < right['source_time_s'] - left['source_time_s'] < 0.2
    ]
    step = median(intervals) if intervals else 0.1
    for index, sample in enumerate(samples):
        sample['roll_sample_time_s'] = index * step
    return samples


def load_pitch_samples(path: Path):
    with path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))

    samples = []
    for row in rows:
        if row.get('auto_traj_mode') != 'attitude':
            continue
        target_roll = float(row['target_roll_deg'])
        target_pitch = float(row['target_pitch_deg'])
        if abs(target_roll) > 1.0 or abs(target_pitch) < 1.0:
            continue

        desired = rotation_matrix(
            target_roll, target_pitch, float(row['target_yaw_deg'])
        )
        current = rotation_matrix(
            float(row['roll_deg']),
            float(row['pitch_deg']),
            float(row['yaw_deg']),
        )
        alignment = max(-1.0, min(
            1.0, dot(column(desired, 2), column(current, 2))
        ))
        samples.append({
            'source_time_s': float(row['time_s']),
            'target_pitch_deg': target_pitch,
            'current_pitch_deg': float(row['continuous_test_pitch_deg']),
            'thrust_axis_error_deg': math.degrees(math.acos(alignment)),
            'horizontal_error_m': math.hypot(
                float(row['position_x_enu_m']) - float(row['target_x_enu_m']),
                float(row['position_y_enu_m']) - float(row['target_y_enu_m']),
            ),
            'altitude_error_m': (
                float(row['position_z_rel_m']) - float(row['target_z_rel_m'])
            ),
        })

    if not samples:
        raise ValueError(f'no pitch attitude samples found in {path}')
    start_time = samples[0]['source_time_s']
    for sample in samples:
        sample['roll_sample_time_s'] = sample['source_time_s'] - start_time
    return samples


def pitch_hold_metrics(samples):
    peak = max(abs(sample['target_pitch_deg']) for sample in samples)
    hold = [
        sample for sample in samples
        if abs(sample['target_pitch_deg']) >= peak - 1.0
    ]
    return {
        'target_peak_deg': peak,
        'samples': len(hold),
        'pitch_error_rmse_deg': rms([
            sample['target_pitch_deg'] - sample['current_pitch_deg']
            for sample in hold
        ]),
        'pitch_error_max_deg': max(
            abs(sample['target_pitch_deg'] - sample['current_pitch_deg'])
            for sample in hold
        ),
        'thrust_axis_error_rmse_deg': rms([
            sample['thrust_axis_error_deg'] for sample in hold
        ]),
        'thrust_axis_error_max_deg': max(
            sample['thrust_axis_error_deg'] for sample in hold
        ),
        'horizontal_error_rmse_m': rms([
            sample['horizontal_error_m'] for sample in hold
        ]),
        'horizontal_error_max_m': max(
            sample['horizontal_error_m'] for sample in hold
        ),
        'altitude_error_rmse_m': rms([
            sample['altitude_error_m'] for sample in hold
        ]),
        'altitude_error_max_abs_m': max(
            abs(sample['altitude_error_m']) for sample in hold
        ),
    }


def hold_metrics(samples):
    hold = [sample for sample in samples if abs(sample['target_roll_deg']) >= 89.0]
    if not hold:
        raise ValueError('log does not contain a +/-90 degree hold')
    return {
        'samples': len(hold),
        'thrust_axis_error_rmse_deg': rms([
            sample['thrust_axis_error_deg'] for sample in hold
        ]),
        'thrust_axis_error_max_deg': max(
            sample['thrust_axis_error_deg'] for sample in hold
        ),
        'full_attitude_error_rmse_deg': rms([
            sample['full_attitude_error_deg'] for sample in hold
        ]),
        'horizontal_error_rmse_m': rms([
            sample['horizontal_error_m'] for sample in hold
        ]),
        'horizontal_error_max_m': max(
            sample['horizontal_error_m'] for sample in hold
        ),
        'altitude_error_rmse_m': rms([
            sample['altitude_error_m'] for sample in hold
        ]),
        'altitude_error_max_abs_m': max(
            abs(sample['altitude_error_m']) for sample in hold
        ),
    }


def plot_series(axis, samples, key, label, color, linestyle='-'):
    axis.plot(
        [sample['roll_sample_time_s'] for sample in samples],
        [sample[key] for sample in samples],
        label=label,
        color=color,
        linestyle=linestyle,
        linewidth=1.6,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--tuned', type=Path, required=True)
    parser.add_argument('--pitch', type=Path)
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=Path('hnuter_logs/comparison/attitude_tuning'),
    )
    args = parser.parse_args()

    baseline = load_roll_samples(args.baseline)
    tuned = load_roll_samples(args.tuned)
    metrics = {
        'baseline_log': str(args.baseline),
        'tuned_log': str(args.tuned),
        'baseline_90deg_hold': hold_metrics(baseline),
        'tuned_90deg_hold': hold_metrics(tuned),
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(4, 1, figsize=(11, 12), constrained_layout=True)
    plot_series(axes[0], baseline, 'target_roll_deg', 'target', '#202124', '--')
    plot_series(axes[0], baseline, 'thrust_roll_deg', 'baseline', '#c43c39')
    plot_series(axes[0], tuned, 'thrust_roll_deg', 'tuned', '#16817a')
    axes[0].set_ylabel('Thrust-axis roll (deg)')

    plot_series(axes[1], baseline, 'thrust_axis_error_deg', 'baseline', '#c43c39')
    plot_series(axes[1], tuned, 'thrust_axis_error_deg', 'tuned', '#16817a')
    axes[1].set_ylabel('Thrust-axis error (deg)')

    plot_series(axes[2], baseline, 'horizontal_error_m', 'baseline', '#c43c39')
    plot_series(axes[2], tuned, 'horizontal_error_m', 'tuned', '#16817a')
    axes[2].set_ylabel('Horizontal error (m)')

    plot_series(axes[3], baseline, 'altitude_error_m', 'baseline', '#c43c39')
    plot_series(axes[3], tuned, 'altitude_error_m', 'tuned', '#16817a')
    axes[3].set_ylabel('Altitude error (m)')
    axes[3].set_xlabel('Concatenated roll-test time (s)')

    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.legend(loc='upper right')

    figure.suptitle('Large-Attitude Roll Tuning Comparison')
    plot_path = args.output_dir / 'attitude_tuning_comparison.png'
    report_path = args.output_dir / 'attitude_tuning_metrics.json'
    figure.savefig(plot_path, dpi=180)

    pitch_plot_path = None
    if args.pitch is not None:
        pitch = load_pitch_samples(args.pitch)
        metrics['pitch_log'] = str(args.pitch)
        metrics['tuned_pitch_hold'] = pitch_hold_metrics(pitch)
        pitch_figure, pitch_axes = plt.subplots(
            4, 1, figsize=(11, 12), constrained_layout=True
        )
        plot_series(
            pitch_axes[0], pitch, 'target_pitch_deg', 'target', '#202124', '--'
        )
        plot_series(
            pitch_axes[0], pitch, 'current_pitch_deg', 'current', '#16817a'
        )
        pitch_axes[0].set_ylabel('Pitch (deg)')
        plot_series(
            pitch_axes[1], pitch, 'thrust_axis_error_deg',
            'thrust-axis error', '#16817a'
        )
        pitch_axes[1].set_ylabel('Thrust-axis error (deg)')
        plot_series(
            pitch_axes[2], pitch, 'horizontal_error_m',
            'horizontal error', '#16817a'
        )
        pitch_axes[2].set_ylabel('Horizontal error (m)')
        plot_series(
            pitch_axes[3], pitch, 'altitude_error_m',
            'altitude error', '#16817a'
        )
        pitch_axes[3].set_ylabel('Altitude error (m)')
        pitch_axes[3].set_xlabel('Pitch-test time (s)')
        for axis in pitch_axes:
            axis.grid(True, alpha=0.25)
            axis.legend(loc='upper right')
        pitch_figure.suptitle('Large-Attitude Pitch Validation')
        pitch_plot_path = args.output_dir / 'large_attitude_pitch_validation.png'
        pitch_figure.savefig(pitch_plot_path, dpi=180)

    report_path.write_text(json.dumps(metrics, indent=2) + '\n')
    print(plot_path)
    if pitch_plot_path is not None:
        print(pitch_plot_path)
    print(report_path)
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
