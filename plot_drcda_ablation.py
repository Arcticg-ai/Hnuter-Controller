#!/usr/bin/env python3
"""Plot and summarize basic DA and DRCDA ablation flight logs."""

from __future__ import annotations

import argparse
import csv
import json
import os
from collections import Counter
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/hnuter-matplotlib')

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from hnuter_log_paths import log_path, stamp
from plot_lissajous_comparison import (
    _equal_3d_axes,
    _relative,
    load_lissajous,
    metrics,
)


DISPLAY_NAMES = {
    'basic_da': 'Basic DA',
    'full': 'Full DRCDA',
    'no_delay': 'No delay',
    'no_horizon': 'No horizon',
    'no_rate_limits': 'No rate limits',
    'original_direct': 'Original direct',
}
COLORS = {
    'basic_da': '#2864b7',
    'full': '#d3542f',
    'no_delay': '#31835c',
    'no_horizon': '#b58a18',
    'no_rate_limits': '#8c5a9e',
    'original_direct': '#687078',
}
METRIC_ROWS = (
    ('3D position RMSE (m)', 'position_rmse_3d_m'),
    ('Maximum position error (m)', 'position_max_3d_m'),
    ('3D velocity RMSE (m/s)', 'velocity_rmse_3d_mps'),
    ('Attitude error RMSE (deg)', 'attitude_rmse_norm_deg'),
    ('Servo total variation', 'servo_total_variation_normalized'),
)


def parse_run(value: str) -> tuple[str, Path]:
    if '=' not in value:
        raise argparse.ArgumentTypeError('--run must use LABEL=CSV_PATH')
    label, raw_path = value.split('=', 1)
    label = label.strip()
    if not label:
        raise argparse.ArgumentTypeError('run label cannot be empty')
    return label, Path(raw_path).expanduser()


def solver_metrics(path: Path) -> dict[str, object]:
    with path.open(newline='') as stream:
        rows = [
            row for row in csv.DictReader(stream)
            if row.get('auto_traj_mode') == 'lissajous'
        ]
    times = np.array([
        float(row['drcda_solve_ms'])
        for row in rows
        if row.get('drcda_solve_ms') not in (None, '')
    ], dtype=float)
    times = times[np.isfinite(times)]
    statuses = Counter(
        row['drcda_status']
        for row in rows
        if row.get('drcda_status')
    )
    truthy = {'1', 'true', 'yes', 'on'}
    armed_fraction = np.mean([
        row.get('armed', '').strip().lower() in truthy for row in rows
    ])
    offboard_fraction = np.mean([
        row.get('offboard', '').strip().lower() in truthy for row in rows
    ])
    ground_contact_samples = sum(
        row.get('ground_contact', '').strip().lower() in truthy for row in rows
    )
    attitude = np.abs(np.array([
        [float(row['roll_deg']), float(row['pitch_deg'])]
        for row in rows
    ]))
    max_tilt_deg = float(np.max(attitude))
    duration_s = float(rows[-1]['time_s']) - float(rows[0]['time_s'])
    flight_valid = bool(
        duration_s >= 23.5
        and armed_fraction == 1.0
        and offboard_fraction == 1.0
        and ground_contact_samples == 0
        and max_tilt_deg <= 55.0
    )
    operational = {
        'flight_valid': flight_valid,
        'armed_fraction': float(armed_fraction),
        'offboard_fraction': float(offboard_fraction),
        'ground_contact_samples': int(ground_contact_samples),
        'max_roll_pitch_deg': max_tilt_deg,
    }
    if times.size == 0:
        return {
            'solve_mean_ms': None,
            'solve_p95_ms': None,
            'solve_max_ms': None,
            'status_counts': dict(statuses),
            **operational,
        }
    return {
        'solve_mean_ms': float(np.mean(times)),
        'solve_p95_ms': float(np.percentile(times, 95.0)),
        'solve_max_ms': float(np.max(times)),
        'status_counts': dict(statuses),
        **operational,
    }


def display_name(label: str) -> str:
    return DISPLAY_NAMES.get(label, label.replace('_', ' ').title())


def color(label: str, index: int) -> str:
    fallback = ('#267d91', '#b44d78', '#5f7f3a', '#a76522')
    return COLORS.get(label, fallback[index % len(fallback)])


def plot_trajectories(
    runs: list[tuple[str, dict[str, np.ndarray]]],
    output: Path,
) -> None:
    reference = _relative(runs[0][1], 'target_position')
    figure = plt.figure(figsize=(11.8, 8.6))
    axis = figure.add_subplot(111, projection='3d')
    axis.plot(
        reference[:, 0], reference[:, 1], reference[:, 2],
        color='#202124', linestyle='--', linewidth=2.2, label='Reference',
    )
    arrays = [reference]
    for index, (label, data) in enumerate(runs):
        actual = _relative(data, 'position')
        arrays.append(actual)
        axis.plot(
            actual[:, 0], actual[:, 1], actual[:, 2],
            color=color(label, index), linewidth=1.55, label=display_name(label),
        )
    axis.scatter(
        [reference[0, 0]], [reference[0, 1]], [reference[0, 2]],
        color='#202124', marker='o', s=40,
    )
    axis.set_xlabel('East relative position (m)')
    axis.set_ylabel('North relative position (m)')
    axis.set_zlabel('Relative altitude (m)')
    axis.set_title('3D Lissajous Allocation Ablation', pad=18)
    axis.view_init(elev=27, azim=-55)
    axis.legend(loc='upper left')
    axis.grid(True, alpha=0.25)
    _equal_3d_axes(axis, arrays)
    figure.tight_layout()
    figure.savefig(output, dpi=180, bbox_inches='tight')
    plt.close(figure)


def plot_errors(
    runs: list[tuple[str, dict[str, np.ndarray]]],
    output: Path,
) -> None:
    figure, axes = plt.subplots(2, 1, figsize=(12.5, 8.2), sharex=True)
    for index, (label, data) in enumerate(runs):
        position_error = np.linalg.norm(
            data['position'] - data['target_position'], axis=1
        )
        attitude_error = np.linalg.norm(data['attitude_error_deg'], axis=1)
        axes[0].plot(
            data['time_s'], position_error,
            color=color(label, index), linewidth=1.45, label=display_name(label),
        )
        axes[1].plot(
            data['time_s'], attitude_error,
            color=color(label, index), linewidth=1.35, label=display_name(label),
        )
    axes[0].set_ylabel('3D position error (m)')
    axes[1].set_ylabel('Attitude error norm (deg)')
    axes[1].set_xlabel('Trajectory time (s)')
    for axis in axes:
        axis.grid(True, alpha=0.25)
    axes[0].legend(loc='upper right', ncol=3)
    figure.suptitle('Lissajous Tracking Error')
    figure.tight_layout()
    figure.savefig(output, dpi=180, bbox_inches='tight')
    plt.close(figure)


def plot_metric_bars(
    run_metrics: list[tuple[str, dict[str, float | int]]],
    output: Path,
) -> None:
    figure, axes = plt.subplots(1, len(METRIC_ROWS), figsize=(16.0, 5.2))
    labels = [display_name(label) for label, _ in run_metrics]
    colors = [color(label, index) for index, (label, _) in enumerate(run_metrics)]
    for axis, (title, key) in zip(axes, METRIC_ROWS):
        values = [float(item[key]) for _, item in run_metrics]
        positions = np.arange(len(values))
        bars = axis.bar(positions, values, color=colors, width=0.72)
        for bar, (_, item) in zip(bars, run_metrics):
            if not bool(item.get('flight_valid', True)):
                bar.set_hatch('//')
                bar.set_edgecolor('#202124')
        axis.set_title(title, fontsize=10)
        axis.set_xticks(positions)
        axis.set_xticklabels(labels, rotation=38, ha='right', fontsize=8)
        axis.grid(True, axis='y', alpha=0.22)
        axis.set_axisbelow(True)
        positive = [value for value in values if value > 0.0]
        if positive and max(positive) / min(positive) > 15.0:
            axis.set_yscale('log')
    figure.suptitle('Allocation Method Metrics (lower is better)')
    figure.tight_layout()
    figure.savefig(output, dpi=180, bbox_inches='tight')
    plt.close(figure)


def write_report(
    output: Path,
    paths: list[tuple[str, Path]],
    all_metrics: dict[str, dict[str, object]],
) -> None:
    labels = [label for label, _ in paths]
    lines = [
        '# Hnuter DRCDA Ablation',
        '',
        'All runs use the same identified-delay plant, controller tuning, and '
        '24 s 3D Lissajous reference.',
        '',
        '| Metric | ' + ' | '.join(display_name(label) for label in labels) + ' |',
        '| --- | ' + ' | '.join('---:' for _ in labels) + ' |',
    ]
    validity = [
        'valid' if all_metrics[label]['flight_valid'] else 'FAILED'
        for label in labels
    ]
    lines.append('| Flight validity | ' + ' | '.join(validity) + ' |')
    for title, key in METRIC_ROWS:
        values = [
            f"{float(all_metrics[label][key]):.5f}"
            for label in labels
        ]
        lines.append(f'| {title} | ' + ' | '.join(values) + ' |')
    solve_values = []
    for label in labels:
        value = all_metrics[label].get('solve_mean_ms')
        solve_values.append('n/a' if value is None else f'{float(value):.3f}')
    lines.append('| Mean solve time (ms) | ' + ' | '.join(solve_values) + ' |')
    max_tilt = [
        f"{float(all_metrics[label]['max_roll_pitch_deg']):.2f}"
        for label in labels
    ]
    lines.append('| Maximum roll/pitch (deg) | ' + ' | '.join(max_tilt) + ' |')
    lines.extend(('', '## Logs', ''))
    lines.extend(f'- {display_name(label)}: `{path}`' for label, path in paths)
    output.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--run', action='append', type=parse_run, required=True,
        help='repeat as LABEL=CSV_PATH',
    )
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    if len(args.run) < 2:
        parser.error('at least two --run arguments are required')
    labels = [label for label, _ in args.run]
    if len(labels) != len(set(labels)):
        parser.error('run labels must be unique')

    output_dir = args.output_dir or log_path('comparison', 'ablation', '.keep').parent
    output_dir.mkdir(parents=True, exist_ok=True)
    run_stamp = stamp()
    loaded = [(label, load_lissajous(path)) for label, path in args.run]
    all_metrics: dict[str, dict[str, object]] = {}
    for label, path in args.run:
        all_metrics[label] = {
            **metrics(dict(loaded)[label]),
            **solver_metrics(path),
            'log': str(path),
        }

    trajectory_path = output_dir / f'ablation_trajectory_{run_stamp}.png'
    valid_trajectory_path = output_dir / f'ablation_trajectory_valid_{run_stamp}.png'
    error_path = output_dir / f'ablation_errors_{run_stamp}.png'
    valid_error_path = output_dir / f'ablation_errors_valid_{run_stamp}.png'
    bars_path = output_dir / f'ablation_metrics_{run_stamp}.png'
    json_path = output_dir / f'ablation_metrics_{run_stamp}.json'
    report_path = output_dir / f'ablation_report_{run_stamp}.md'
    plot_trajectories(loaded, trajectory_path)
    valid_runs = [
        (label, data) for label, data in loaded
        if all_metrics[label]['flight_valid']
    ]
    if len(valid_runs) >= 2:
        plot_trajectories(valid_runs, valid_trajectory_path)
    plot_errors(loaded, error_path)
    if len(valid_runs) >= 2:
        plot_errors(valid_runs, valid_error_path)
    plot_metric_bars(
        [(label, all_metrics[label]) for label in labels],
        bars_path,
    )
    json_path.write_text(
        json.dumps(all_metrics, indent=2, ensure_ascii=True) + '\n',
        encoding='utf-8',
    )
    write_report(report_path, args.run, all_metrics)

    print(f'trajectory_plot={trajectory_path}')
    if len(valid_runs) >= 2:
        print(f'valid_trajectory_plot={valid_trajectory_path}')
    print(f'error_plot={error_path}')
    if len(valid_runs) >= 2:
        print(f'valid_error_plot={valid_error_path}')
    print(f'metric_plot={bars_path}')
    print(f'metrics={json_path}')
    print(f'report={report_path}')


if __name__ == '__main__':
    main()
