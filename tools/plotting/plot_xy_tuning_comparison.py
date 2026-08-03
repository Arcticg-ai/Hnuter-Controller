#!/usr/bin/env python3
"""Compare lateral tracking before and after direct-controller tuning."""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

os.environ.setdefault('MPLCONFIGDIR', '/tmp/hnuter-matplotlib')

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from hnuter_log_paths import log_path, stamp
from tools.plotting.trajectory_alignment import (
    fit_planar_rotation,
    transform_points,
    transform_vectors,
)


POSITION_COLUMNS = (
    'position_x_enu_m',
    'position_y_enu_m',
    'position_z_rel_m',
)
TARGET_POSITION_COLUMNS = (
    'target_x_enu_m',
    'target_y_enu_m',
    'target_z_rel_m',
)
VELOCITY_COLUMNS = (
    'velocity_x_enu_mps',
    'velocity_y_enu_mps',
)


def _numeric(rows: list[dict[str, str]], columns: tuple[str, ...]) -> np.ndarray:
    return np.array(
        [[float(row[column]) for column in columns] for row in rows],
        dtype=float,
    )


def load_run(path: Path, hover_window_s: float) -> dict[str, dict[str, np.ndarray]]:
    with path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    lissajous_indices = [
        index for index, row in enumerate(rows)
        if row.get('auto_traj_mode') == 'lissajous'
    ]
    if len(lissajous_indices) < 10:
        raise ValueError(f'{path} does not contain a complete Lissajous segment')

    trajectory_rows = [rows[index] for index in lissajous_indices]
    hover_rows = [
        row for row in rows[lissajous_indices[-1] + 1:]
        if row.get('auto_traj_mode') == 'hover'
    ]
    if len(hover_rows) < 10:
        raise ValueError(f'{path} does not contain post-trajectory hover data')
    hover_end = float(hover_rows[-1]['time_s'])
    hover_rows = [
        row for row in hover_rows
        if float(row['time_s']) >= hover_end - hover_window_s
    ]

    return {
        'trajectory': _segment(trajectory_rows),
        'hover': _segment(hover_rows),
    }


def _segment(rows: list[dict[str, str]]) -> dict[str, np.ndarray]:
    time_s = np.array([float(row['time_s']) for row in rows], dtype=float)
    time_s -= time_s[0]
    return {
        'time_s': time_s,
        'position': _numeric(rows, POSITION_COLUMNS),
        'target_position': _numeric(rows, TARGET_POSITION_COLUMNS),
        'velocity_xy': _numeric(rows, VELOCITY_COLUMNS),
        'servo': _numeric(
            rows, tuple(f'cmd_servo_{index}' for index in range(4))
        ),
    }


def _dominant_frequency(signal: np.ndarray, time_s: np.ndarray) -> float:
    if signal.size < 10:
        return float('nan')
    dt = float(np.median(np.diff(time_s)))
    frequency = np.fft.rfftfreq(signal.size, dt)
    magnitude = np.abs(np.fft.rfft(signal - np.mean(signal)))
    valid = (frequency >= 0.05) & (frequency <= 2.0)
    if not np.any(valid):
        return float('nan')
    return float(frequency[valid][np.argmax(magnitude[valid])])


def metrics(run: dict[str, dict[str, np.ndarray]]) -> dict[str, float | int]:
    trajectory = run['trajectory']
    hover = run['hover']
    trajectory_error = trajectory['position'] - trajectory['target_position']
    hover_error = hover['position'][:, :2] - hover['target_position'][:, :2]
    servo = np.nan_to_num(hover['servo'], nan=0.0)
    hover_duration = max(float(hover['time_s'][-1]), 1e-9)
    servo_tv_per_s = np.sum(np.abs(np.diff(servo, axis=0)), axis=0) / hover_duration

    values: dict[str, float | int] = {
        'trajectory_samples': int(trajectory['time_s'].size),
        'trajectory_position_rmse_3d_m': float(
            np.sqrt(np.mean(np.sum(trajectory_error ** 2, axis=1)))
        ),
        'trajectory_position_max_3d_m': float(
            np.max(np.linalg.norm(trajectory_error, axis=1))
        ),
    }
    for index, axis in enumerate(('x', 'y', 'z')):
        values[f'trajectory_position_rmse_{axis}_m'] = float(
            np.sqrt(np.mean(trajectory_error[:, index] ** 2))
        )
    for index, axis in enumerate(('x', 'y')):
        values[f'hover_error_mean_{axis}_m'] = float(np.mean(hover_error[:, index]))
        values[f'hover_error_std_{axis}_m'] = float(np.std(hover_error[:, index]))
        values[f'hover_error_peak_to_peak_{axis}_m'] = float(
            np.ptp(hover_error[:, index])
        )
        values[f'hover_velocity_std_{axis}_mps'] = float(
            np.std(hover['velocity_xy'][:, index])
        )
        values[f'hover_dominant_frequency_{axis}_hz'] = _dominant_frequency(
            hover_error[:, index], hover['time_s']
        )
    values['hover_secondary_servo_total_variation_per_s'] = float(
        np.sum(servo_tv_per_s[2:4])
    )
    return values


def align_to_comparison_frame(
    reference: dict[str, dict[str, np.ndarray]],
    moving: dict[str, dict[str, np.ndarray]],
) -> float:
    """Put trajectory and hover data in the baseline trajectory frame."""
    reference_trajectory = reference['trajectory']
    moving_trajectory = moving['trajectory']
    rotation, alignment_rmse = fit_planar_rotation(
        reference_trajectory['target_position'],
        moving_trajectory['target_position'],
    )
    source_origin = moving_trajectory['target_position'][0]
    destination_origin = reference_trajectory['target_position'][0]
    for segment in moving.values():
        for key in ('position', 'target_position'):
            segment[key] = transform_points(
                segment[key], source_origin, destination_origin, rotation
            )
        segment['velocity_xy'] = transform_vectors(
            segment['velocity_xy'], rotation
        )
    return alignment_rmse


def improvement_percent(baseline: float, tuned: float) -> float:
    if abs(baseline) < 1e-12:
        return float('nan')
    return 100.0 * (baseline - tuned) / abs(baseline)


def plot_trajectory(
    baseline: dict[str, dict[str, np.ndarray]],
    tuned: dict[str, dict[str, np.ndarray]],
    output: Path,
) -> None:
    base = baseline['trajectory']
    new = tuned['trajectory']
    base_origin = base['target_position'][0]
    new_origin = new['target_position'][0]
    reference = base['target_position'] - base_origin
    original = base['position'] - base_origin
    optimized = new['position'] - new_origin

    figure = plt.figure(figsize=(11.2, 8.2))
    axis = figure.add_subplot(111, projection='3d')
    axis.plot(*reference.T, '--', color='#202124', linewidth=2.0, label='Reference')
    axis.plot(*original.T, color='#3666b0', linewidth=1.6, label='Before tuning')
    axis.plot(*optimized.T, color='#d45a35', linewidth=1.8, label='Damped XY tuning')
    axis.set_xlabel('East X (m)')
    axis.set_ylabel('North Y (m)')
    axis.set_zlabel('Altitude Z (m)')
    axis.set_title('3D Lissajous Tracking Before and After XY Tuning')
    axis.view_init(elev=27, azim=-55)
    axis.legend(loc='upper left')
    axis.grid(True, alpha=0.25)
    figure.tight_layout()
    figure.savefig(output, dpi=180, bbox_inches='tight')
    plt.close(figure)


def plot_lateral(
    baseline: dict[str, dict[str, np.ndarray]],
    tuned: dict[str, dict[str, np.ndarray]],
    output: Path,
) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(13.0, 8.2))
    colors = ('#3666b0', '#d45a35')
    labels = ('Before tuning', 'Damped XY tuning')
    for column, axis_name in enumerate(('Trajectory X', 'Trajectory Y')):
        for run, color, label in zip((baseline, tuned), colors, labels):
            segment = run['trajectory']
            error = segment['position'][:, column] - segment['target_position'][:, column]
            axes[0, column].plot(
                segment['time_s'], error, color=color, linewidth=1.45, label=label
            )
        axes[0, column].axhline(0.0, color='#202124', linewidth=0.8)
        axes[0, column].set_title(f'Lissajous {axis_name} tracking error')
        axes[0, column].set_ylabel('Position error (m)')
        axes[0, column].grid(True, alpha=0.25)

        for run, color, label in zip((baseline, tuned), colors, labels):
            segment = run['hover']
            error = segment['position'][:, column] - segment['target_position'][:, column]
            axes[1, column].plot(
                segment['time_s'], error, color=color, linewidth=1.45, label=label
            )
        axes[1, column].axhline(0.0, color='#202124', linewidth=0.8)
        axes[1, column].set_title(f'Post-trajectory hover {axis_name} error')
        axes[1, column].set_xlabel('Time (s)')
        axes[1, column].set_ylabel('Position error (m)')
        axes[1, column].grid(True, alpha=0.25)

    axes[0, 0].legend(loc='upper right')
    axes[1, 0].legend(loc='upper right')
    figure.suptitle('Lateral Oscillation and Offset Comparison', fontsize=15)
    figure.tight_layout()
    figure.savefig(output, dpi=180, bbox_inches='tight')
    plt.close(figure)


def write_report(
    output: Path,
    baseline_path: Path,
    tuned_path: Path,
    baseline_metrics: dict[str, float | int],
    tuned_metrics: dict[str, float | int],
    reference_alignment_rmse: float,
) -> None:
    rows = (
        ('3D trajectory RMSE (m)', 'trajectory_position_rmse_3d_m'),
        ('3D trajectory max error (m)', 'trajectory_position_max_3d_m'),
        ('Hover X error std (m)', 'hover_error_std_x_m'),
        ('Hover Y error std (m)', 'hover_error_std_y_m'),
        ('Hover X peak-to-peak (m)', 'hover_error_peak_to_peak_x_m'),
        ('Hover Y peak-to-peak (m)', 'hover_error_peak_to_peak_y_m'),
        ('Hover X velocity std (m/s)', 'hover_velocity_std_x_mps'),
        ('Hover Y velocity std (m/s)', 'hover_velocity_std_y_mps'),
        (
            'Secondary servo total variation (/s)',
            'hover_secondary_servo_total_variation_per_s',
        ),
    )
    lines = [
        '# Hnuter XY Tuning Comparison',
        '',
        f'- Before tuning: `{baseline_path}`',
        f'- Damped XY tuning: `{tuned_path}`',
        '- Frame: tuned target rigidly aligned to the before-tuning trajectory '
        'frame before evaluating X/Y components.',
        f'- Target alignment residual: `{reference_alignment_rmse:.5f} m`.',
        '',
        '| Metric | Before | Tuned | Reduction |',
        '| --- | ---: | ---: | ---: |',
    ]
    for label, key in rows:
        before = float(baseline_metrics[key])
        after = float(tuned_metrics[key])
        lines.append(
            f'| {label} | {before:.5f} | {after:.5f} | '
            f'{improvement_percent(before, after):+.2f}% |'
        )
    output.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--tuned', type=Path, required=True)
    parser.add_argument('--hover-window-s', type=float, default=30.0)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()

    output_dir = args.output_dir or log_path('comparison', 'xy_tuning')
    output_dir.mkdir(parents=True, exist_ok=True)
    run_stamp = stamp()
    baseline = load_run(args.baseline, args.hover_window_s)
    tuned = load_run(args.tuned, args.hover_window_s)
    reference_alignment_rmse = align_to_comparison_frame(baseline, tuned)
    baseline_metrics = metrics(baseline)
    tuned_metrics = metrics(tuned)

    trajectory_path = output_dir / f'xy_tuning_trajectory_{run_stamp}.png'
    lateral_path = output_dir / f'xy_tuning_lateral_{run_stamp}.png'
    metrics_path = output_dir / f'xy_tuning_metrics_{run_stamp}.json'
    report_path = output_dir / f'xy_tuning_report_{run_stamp}.md'
    plot_trajectory(baseline, tuned, trajectory_path)
    plot_lateral(baseline, tuned, lateral_path)
    comparison = {
        'baseline_log': str(args.baseline),
        'tuned_log': str(args.tuned),
        'comparison_frame': 'baseline trajectory frame',
        'tuned_reference_alignment_rmse_m': reference_alignment_rmse,
        'hover_window_s': args.hover_window_s,
        'before_tuning': baseline_metrics,
        'damped_xy_tuning': tuned_metrics,
    }
    metrics_path.write_text(
        json.dumps(comparison, indent=2, ensure_ascii=True) + '\n',
        encoding='utf-8',
    )
    write_report(
        report_path,
        args.baseline,
        args.tuned,
        baseline_metrics,
        tuned_metrics,
        reference_alignment_rmse,
    )
    print(f'trajectory_plot={trajectory_path}')
    print(f'lateral_plot={lateral_path}')
    print(f'metrics={metrics_path}')
    print(f'report={report_path}')


if __name__ == '__main__':
    main()
