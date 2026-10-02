#!/usr/bin/env python3
"""Create figures and a compact data bundle for fixed-attitude switching."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import shutil

import matplotlib.pyplot as plt
import numpy as np


METHOD_LABELS = {
    'direct': 'Direct',
    'basic_da': 'Basic DA',
    'full': 'Full DRCDA',
    'no_horizon': 'No horizon',
}
COLORS = {
    'direct': '#4c78a8',
    'basic_da': '#e45756',
    'full': '#2a9d8f',
    'no_horizon': '#f2a541',
}


def number(value: str | None) -> float:
    try:
        parsed = float(value or '')
    except ValueError:
        return math.nan
    return parsed if math.isfinite(parsed) else math.nan


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def diagnostic_path(root: Path, duration: float, method: str) -> Path:
    case = root / 'runs' / f'T_{duration:.1f}'.replace('.', 'p') / method
    matches = list((case / 'logs').rglob('switching_*.csv'))
    if len(matches) != 1:
        raise RuntimeError(f'expected one diagnostic CSV below {case}, got {matches}')
    return matches[0]


def transition_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    return [
        row for row in rows
        if row['switch_phase'] in {'a_to_b', 'b_to_a', 'a_to_c'}
    ]


def series(rows: list[dict[str, str]], name: str) -> np.ndarray:
    return np.asarray([number(row.get(name)) for row in rows], dtype=float)


def plot_sweep(summary: list[dict[str, str]], manifest: dict, output: Path) -> None:
    methods = ('direct', 'full', 'no_horizon')
    specs = (
        ('position_error_peak_m', 'Peak horizontal error (m)', 0.75),
        ('attitude_error_peak_deg', 'Peak SO(3) error (deg)', 8.0),
        ('roll_pitch_peak_deg', 'Peak |roll/pitch| (deg)', 5.0),
    )
    fig, axes = plt.subplots(1, 3, figsize=(12.8, 3.7), constrained_layout=True)
    for axis, (key, ylabel, limit) in zip(axes, specs):
        for method in methods:
            selected = [row for row in summary if row['method'] == method]
            selected.sort(key=lambda row: number(row['switch_duration_s']))
            axis.plot(
                [number(row['switch_duration_s']) for row in selected],
                [number(row[key]) for row in selected],
                marker='o', linewidth=1.8, markersize=4.5,
                label=METHOD_LABELS[method], color=COLORS[method],
            )
        axis.axhline(limit, color='#333333', linestyle='--', linewidth=1.0,
                     label='Pass limit' if key == specs[0][0] else None)
        axis.set_xlabel('One-way switch time T (s)')
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.25)
    axes[0].legend(frameon=False, fontsize=8)
    fig.suptitle('Physical-gate fixed-attitude A-B-A-C sweep (no-delay SITL)')
    fig.savefig(output / '01_sweep_metrics.png', dpi=180)
    plt.close(fig)


def plot_tracking(root: Path, output: Path) -> None:
    methods = ('direct', 'full', 'no_horizon')
    fig, axes = plt.subplots(2, 2, figsize=(12.8, 7.2), constrained_layout=True)
    for method in methods:
        rows = transition_rows(load_rows(diagnostic_path(root, 1.5, method)))
        t = series(rows, 'switch_elapsed_s')
        t -= t[0]
        color = COLORS[method]
        axes[0, 0].plot(t, series(rows, 'position_y_enu_m'), color=color,
                        label=METHOD_LABELS[method], linewidth=1.7)
        dx = series(rows, 'position_x_enu_m') - series(rows, 'target_x_enu_m')
        dy = series(rows, 'position_y_enu_m') - series(rows, 'target_y_enu_m')
        axes[0, 1].plot(t, np.hypot(dx, dy), color=color, linewidth=1.7)
        axes[1, 0].plot(t, series(rows, 'attitude_error_angle_deg'),
                        color=color, linewidth=1.7)
        rp = np.maximum(np.abs(series(rows, 'roll_deg')),
                        np.abs(series(rows, 'pitch_deg')))
        axes[1, 1].plot(t, rp, color=color, linewidth=1.7)
        if method == 'full':
            axes[0, 0].plot(t, series(rows, 'target_y_enu_m'), color='#222222',
                            linestyle='--', linewidth=1.2, label='Reference')
    axes[0, 0].set_ylabel('Lateral position y (m)')
    axes[0, 1].set_ylabel('Horizontal error (m)')
    axes[1, 0].set_ylabel('SO(3) error (deg)')
    axes[1, 1].set_ylabel('max(|roll|, |pitch|) (deg)')
    for axis in axes.flat:
        axis.set_xlabel('Transition time (s)')
        axis.grid(True, alpha=0.25)
    axes[0, 0].legend(frameon=False, fontsize=8, ncol=2)
    axes[1, 0].axhline(8.0, color='#333333', linestyle='--', linewidth=1.0)
    axes[1, 1].axhline(5.0, color='#333333', linestyle='--', linewidth=1.0)
    fig.suptitle('Boundary case T=1.5 s')
    fig.savefig(output / '02_tracking_T1p5.png', dpi=180)
    plt.close(fig)


def plot_wrench_and_actuators(
    root: Path, output: Path, duration: float
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12.8, 7.2), constrained_layout=True)
    for column, method in enumerate(('full', 'no_horizon')):
        rows = transition_rows(load_rows(diagnostic_path(root, duration, method)))
        t = series(rows, 'switch_elapsed_s')
        t -= t[0]
        velocity_y = series(rows, 'velocity_y_enu_mps')
        actual_force_y = 4.5 * np.gradient(velocity_y, t)
        if actual_force_y.size >= 3:
            actual_force_y = np.convolve(
                actual_force_y, np.ones(3) / 3.0, mode='same'
            )
        axes[0, column].plot(t, series(rows, 'wrench_fy_body_n'),
                             label='Requested Fy', color='#222222', linewidth=1.4)
        axes[0, column].plot(t, series(rows, 'drcda_predicted_wrench_fy_n'),
                             label='Predicted reachable Fy', color=COLORS[method],
                             linewidth=1.5)
        axes[0, column].plot(
            t, actual_force_y, label='Kinematic m*a_y estimate',
            color='#8f5da2', linewidth=1.2, alpha=0.9,
        )
        for joint, joint_color in (('a_l', '#4c78a8'), ('b_l', '#e45756')):
            axes[1, column].plot(
                t, series(rows, f'switch_commanded_joint_{joint}_rad'),
                color=joint_color, linestyle='--', linewidth=1.1,
                label=f'{joint} command',
            )
            axes[1, column].plot(
                t, series(rows, f'switch_measured_joint_{joint}_rad'),
                color=joint_color, linewidth=1.5, label=f'{joint} measured',
            )
        axes[0, column].set_title(METHOD_LABELS[method])
        axes[0, column].set_ylabel('Body lateral force Fy (N)')
        axes[1, column].set_ylabel('Left joint angle (rad)')
        for axis in axes[:, column]:
            axis.set_xlabel('Transition time (s)')
            axis.grid(True, alpha=0.25)
    axes[0, 0].legend(frameon=False, fontsize=8)
    axes[1, 0].legend(frameon=False, fontsize=8, ncol=2)
    fig.suptitle(
        f'Reachable-wrench and measured-actuator response, T={duration:.1f} s'
    )
    fig.savefig(output / '03_wrench_actuator_boundary.png', dpi=180)
    plt.close(fig)


def copy_compact_data(root: Path, output: Path) -> None:
    data = output / 'data'
    data.mkdir(parents=True, exist_ok=True)
    for name in ('summary.csv', 'manifest.json'):
        shutil.copy2(root / name, data / name)
    for result in sorted((root / 'runs').rglob('result.json')):
        relative = result.relative_to(root / 'runs')
        destination = data / 'runs' / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(result, destination)
    for diagnostic in sorted((root / 'runs').rglob('switching_*.csv')):
        parts = diagnostic.relative_to(root / 'runs').parts
        destination = data / 'runs' / parts[0] / parts[1] / 'diagnostics.csv'
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(diagnostic, destination)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    summary = load_rows(args.input / 'summary.csv')
    manifest = json.loads((args.input / 'manifest.json').read_text(encoding='utf-8'))
    durations = sorted({
        number(row['switch_duration_s']) for row in summary
        if math.isfinite(number(row['switch_duration_s']))
    })
    boundary_duration = durations[0]
    plot_sweep(summary, manifest, args.output)
    plot_tracking(args.input, args.output)
    plot_wrench_and_actuators(args.input, args.output, boundary_duration)
    copy_compact_data(args.input, args.output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
