#!/usr/bin/env python3
"""Compare direct and DRCDA 3D Lissajous flight logs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path
from typing import Iterable

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

os.environ.setdefault('MPLCONFIGDIR', '/tmp/hnuter-matplotlib')

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from hnuter_log_paths import log_path, stamp


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
    'velocity_z_enu_mps',
)
TARGET_VELOCITY_COLUMNS = (
    'target_vx_enu_mps',
    'target_vy_enu_mps',
    'target_vz_enu_mps',
)


def _numeric(rows: list[dict[str, str]], columns: Iterable[str]) -> np.ndarray:
    return np.array([
        [float(row[column]) for column in columns]
        for row in rows
    ], dtype=float)


def load_lissajous(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline='') as stream:
        rows = [
            row for row in csv.DictReader(stream)
            if row.get('auto_traj_mode') == 'lissajous'
        ]
    if len(rows) < 10:
        raise ValueError(f'{path} does not contain a complete Lissajous segment')

    time_s = np.array([float(row['time_s']) for row in rows])
    time_s -= time_s[0]
    position = _numeric(rows, POSITION_COLUMNS)
    target_position = _numeric(rows, TARGET_POSITION_COLUMNS)
    velocity = _numeric(rows, VELOCITY_COLUMNS)
    target_velocity = _numeric(rows, TARGET_VELOCITY_COLUMNS)
    attitude_deg = _numeric(rows, ('roll_deg', 'pitch_deg', 'yaw_deg'))
    target_attitude_deg = _numeric(
        rows, ('target_roll_deg', 'target_pitch_deg', 'target_yaw_deg')
    )
    attitude_error_deg = attitude_deg - target_attitude_deg
    attitude_error_deg[:, 2] = (
        attitude_error_deg[:, 2] + 180.0
    ) % 360.0 - 180.0

    motor = _numeric(rows, tuple(f'cmd_motor_{index}' for index in range(5)))
    servo = _numeric(rows, tuple(f'cmd_servo_{index}' for index in range(4)))
    return {
        'time_s': time_s,
        'position': position,
        'target_position': target_position,
        'velocity': velocity,
        'target_velocity': target_velocity,
        'attitude_error_deg': attitude_error_deg,
        'motor': motor,
        'servo': servo,
    }


def metrics(data: dict[str, np.ndarray]) -> dict[str, float | int]:
    position_error = data['position'] - data['target_position']
    velocity_error = data['velocity'] - data['target_velocity']
    position_norm = np.linalg.norm(position_error, axis=1)
    velocity_norm = np.linalg.norm(velocity_error, axis=1)
    attitude_norm = np.linalg.norm(data['attitude_error_deg'], axis=1)
    motor = np.nan_to_num(data['motor'], nan=0.0)
    servo = np.nan_to_num(data['servo'], nan=0.0)
    servo_variation = np.sum(np.abs(np.diff(servo, axis=0)), axis=0)

    return {
        'samples': int(data['time_s'].size),
        'duration_s': float(data['time_s'][-1]),
        'position_rmse_x_m': float(np.sqrt(np.mean(position_error[:, 0] ** 2))),
        'position_rmse_y_m': float(np.sqrt(np.mean(position_error[:, 1] ** 2))),
        'position_rmse_z_m': float(np.sqrt(np.mean(position_error[:, 2] ** 2))),
        'position_rmse_3d_m': float(np.sqrt(np.mean(position_norm ** 2))),
        'position_mean_3d_m': float(np.mean(position_norm)),
        'position_max_3d_m': float(np.max(position_norm)),
        'velocity_rmse_3d_mps': float(np.sqrt(np.mean(velocity_norm ** 2))),
        'attitude_rmse_norm_deg': float(np.sqrt(np.mean(attitude_norm ** 2))),
        'motor_rms_normalized': float(np.sqrt(np.mean(motor ** 2))),
        'motor_peak_normalized': float(np.max(np.abs(motor))),
        'servo_total_variation_normalized': float(np.sum(servo_variation)),
        'servo_peak_normalized': float(np.max(np.abs(servo))),
    }


def _relative(data: dict[str, np.ndarray], key: str) -> np.ndarray:
    values = data[key]
    return values - data['target_position'][0]


def _equal_3d_axes(axis, arrays: Iterable[np.ndarray]) -> None:
    combined = np.vstack(tuple(arrays))
    minimum = np.min(combined, axis=0)
    maximum = np.max(combined, axis=0)
    center = 0.5 * (minimum + maximum)
    radius = max(float(np.max(maximum - minimum)) * 0.55, 0.1)
    axis.set_xlim(center[0] - radius, center[0] + radius)
    axis.set_ylim(center[1] - radius, center[1] + radius)
    axis.set_zlim(center[2] - radius, center[2] + radius)


def plot_trajectory(
    baseline: dict[str, np.ndarray],
    drcda: dict[str, np.ndarray],
    output: Path,
) -> None:
    baseline_target = _relative(baseline, 'target_position')
    baseline_actual = _relative(baseline, 'position')
    drcda_target = _relative(drcda, 'target_position')
    drcda_actual = _relative(drcda, 'position')

    figure = plt.figure(figsize=(11.5, 8.5))
    axis = figure.add_subplot(111, projection='3d')
    axis.plot(
        baseline_target[:, 0],
        baseline_target[:, 1],
        baseline_target[:, 2],
        color='#202124',
        linestyle='--',
        linewidth=2.2,
        label='Reference',
    )
    axis.plot(
        baseline_actual[:, 0],
        baseline_actual[:, 1],
        baseline_actual[:, 2],
        color='#2f6fed',
        linewidth=1.8,
        label='Original direct',
    )
    axis.plot(
        drcda_actual[:, 0],
        drcda_actual[:, 1],
        drcda_actual[:, 2],
        color='#e66a2c',
        linewidth=1.8,
        label='DRCDA',
    )
    axis.scatter(
        [baseline_target[0, 0]], [baseline_target[0, 1]], [baseline_target[0, 2]],
        color='#202124', marker='o', s=45, label='Start / finish',
    )
    axis.set_xlabel('East relative position (m)')
    axis.set_ylabel('North relative position (m)')
    axis.set_zlabel('Relative altitude (m)')
    axis.set_title('Hnuter 3D Lissajous Flight Comparison', pad=18)
    axis.view_init(elev=27, azim=-55)
    axis.legend(loc='upper left')
    axis.grid(True, alpha=0.25)
    _equal_3d_axes(
        axis,
        (baseline_target, baseline_actual, drcda_target, drcda_actual),
    )
    figure.tight_layout()
    figure.savefig(output, dpi=180, bbox_inches='tight')
    plt.close(figure)


def plot_tracking(
    baseline: dict[str, np.ndarray],
    drcda: dict[str, np.ndarray],
    output: Path,
) -> None:
    figure, axes = plt.subplots(4, 1, figsize=(12.5, 11.0), sharex=True)
    labels = ('East X', 'North Y', 'Altitude Z')
    baseline_target = _relative(baseline, 'target_position')
    baseline_actual = _relative(baseline, 'position')
    drcda_target = _relative(drcda, 'target_position')
    drcda_actual = _relative(drcda, 'position')

    for index, axis in enumerate(axes[:3]):
        axis.plot(
            baseline['time_s'], baseline_target[:, index],
            color='#202124', linestyle='--', linewidth=1.6, label='Reference',
        )
        axis.plot(
            baseline['time_s'], baseline_actual[:, index],
            color='#2f6fed', linewidth=1.4, label='Original direct',
        )
        axis.plot(
            drcda['time_s'], drcda_actual[:, index],
            color='#e66a2c', linewidth=1.4, label='DRCDA',
        )
        axis.set_ylabel(f'{labels[index]} (m)')
        axis.grid(True, alpha=0.25)

    baseline_error = np.linalg.norm(
        baseline['position'] - baseline['target_position'], axis=1
    )
    drcda_error = np.linalg.norm(
        drcda['position'] - drcda['target_position'], axis=1
    )
    axes[3].plot(
        baseline['time_s'], baseline_error,
        color='#2f6fed', linewidth=1.5, label='Original direct',
    )
    axes[3].plot(
        drcda['time_s'], drcda_error,
        color='#e66a2c', linewidth=1.5, label='DRCDA',
    )
    axes[3].set_ylabel('3D error (m)')
    axes[3].set_xlabel('Trajectory time (s)')
    axes[3].grid(True, alpha=0.25)
    axes[0].legend(loc='upper right', ncol=3)
    axes[3].legend(loc='upper right', ncol=2)
    figure.suptitle('3D Lissajous Position Tracking', fontsize=15)
    figure.tight_layout()
    figure.savefig(output, dpi=180, bbox_inches='tight')
    plt.close(figure)


def improvement_percent(baseline_value: float, drcda_value: float) -> float:
    if abs(baseline_value) < 1e-12:
        return float('nan')
    return 100.0 * (baseline_value - drcda_value) / baseline_value


def write_report(
    output: Path,
    baseline_path: Path,
    drcda_path: Path,
    baseline_metrics: dict[str, float | int],
    drcda_metrics: dict[str, float | int],
) -> None:
    tracked_metrics = (
        ('3D position RMSE (m)', 'position_rmse_3d_m'),
        ('3D position max (m)', 'position_max_3d_m'),
        ('3D velocity RMSE (m/s)', 'velocity_rmse_3d_mps'),
        ('Attitude error norm RMSE (deg)', 'attitude_rmse_norm_deg'),
        ('Servo total variation', 'servo_total_variation_normalized'),
    )
    lines = [
        '# Hnuter 3D Lissajous Comparison',
        '',
        f'- Original direct log: `{baseline_path}`',
        f'- DRCDA log: `{drcda_path}`',
        '',
        '| Metric | Original direct | DRCDA | DRCDA improvement |',
        '| --- | ---: | ---: | ---: |',
    ]
    for label, key in tracked_metrics:
        baseline_value = float(baseline_metrics[key])
        drcda_value = float(drcda_metrics[key])
        improvement = improvement_percent(baseline_value, drcda_value)
        lines.append(
            f'| {label} | {baseline_value:.5f} | {drcda_value:.5f} | '
            f'{improvement:+.2f}% |'
        )
    output.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--drcda', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()

    output_dir = args.output_dir or log_path('comparison', '.keep').parent
    output_dir.mkdir(parents=True, exist_ok=True)
    run_stamp = stamp()
    baseline = load_lissajous(args.baseline)
    drcda = load_lissajous(args.drcda)
    baseline_metrics = metrics(baseline)
    drcda_metrics = metrics(drcda)

    trajectory_path = output_dir / f'lissajous3d_trajectory_{run_stamp}.png'
    tracking_path = output_dir / f'lissajous3d_tracking_{run_stamp}.png'
    metrics_path = output_dir / f'lissajous3d_metrics_{run_stamp}.json'
    report_path = output_dir / f'lissajous3d_report_{run_stamp}.md'
    plot_trajectory(baseline, drcda, trajectory_path)
    plot_tracking(baseline, drcda, tracking_path)

    comparison = {
        'baseline_log': str(args.baseline),
        'drcda_log': str(args.drcda),
        'original_direct': baseline_metrics,
        'drcda': drcda_metrics,
    }
    metrics_path.write_text(
        json.dumps(comparison, indent=2, ensure_ascii=True) + '\n',
        encoding='utf-8',
    )
    write_report(
        report_path,
        args.baseline,
        args.drcda,
        baseline_metrics,
        drcda_metrics,
    )

    print(f'trajectory_plot={trajectory_path}')
    print(f'tracking_plot={tracking_path}')
    print(f'metrics={metrics_path}')
    print(f'report={report_path}')


if __name__ == '__main__':
    main()
