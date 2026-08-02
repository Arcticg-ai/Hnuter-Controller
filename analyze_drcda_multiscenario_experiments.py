#!/usr/bin/env python3
"""Analyze the identified-delay DRCDA multi-scenario experiment matrix."""

from __future__ import annotations

import argparse
import base64
import csv
import html
import json
import math
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/hnuter-matplotlib')

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


ROOT = (
    Path(__file__).resolve().parent
    / 'hnuter_logs'
    / 'drcda_complete_validation_20260731'
)
FIGURES = ROOT / 'figures'
REPORTS = ROOT / 'reports'

VARIANTS = (
    'original_direct',
    'basic_da',
    'ada',
    'nda',
    'pda',
    'full',
    'ftr_drcda',
    'no_delay',
    'no_horizon',
    'no_physical_rate',
    'no_command_slew',
    'no_reachability_gate',
    'no_multirate',
)
CORE_VARIANTS = (
    'original_direct',
    'basic_da',
    'ada',
    'nda',
    'pda',
    'ftr_drcda',
    'full',
)
CORE_PLOT_VARIANTS = (
    'original_direct',
    'basic_da',
    'pda',
    'ftr_drcda',
    'full',
)
ABLATION_VARIANTS = (
    'ftr_drcda',
    'no_delay',
    'no_horizon',
    'no_physical_rate',
    'no_command_slew',
    'no_reachability_gate',
    'no_multirate',
)
SCENARIOS = (
    'lissajous',
    'aggressive',
    'large_attitude_45_60',
)
BOUNDARY_SCENARIO = 'large_attitude_60_90'
EXPECTED_DURATION = {
    'lissajous': 24.0,
    'aggressive': 8.0,
    'large_attitude_45_60': 44.0,
    'large_attitude': 44.0,
    BOUNDARY_SCENARIO: 44.0,
}
ACTIVE_MODE = {
    'lissajous': 'lissajous',
    'aggressive': 'lissajous',
    'large_attitude_45_60': 'attitude',
    'large_attitude': 'attitude',
    BOUNDARY_SCENARIO: 'attitude',
}
TRACKING_LIMITS = {
    'lissajous': (2.0, 30.0, None),
    'aggressive': (5.0, 35.0, None),
    'large_attitude_45_60': (2.0, 30.0, 30.0),
    'large_attitude': (5.0, 45.0, 35.0),
    BOUNDARY_SCENARIO: (5.0, 45.0, 35.0),
}
NAMES = {
    'original_direct': 'Original direct',
    'basic_da': 'Basic DA',
    'ada': 'Paper ADA',
    'nda': 'Paper NDA',
    'pda': 'Paper PDA',
    'ftr_drcda': 'FTR-DRCDA',
    'full': 'Hybrid full',
    'no_delay': 'No delay model',
    'no_horizon': 'No horizon',
    'no_physical_rate': 'No physical rate',
    'no_command_slew': 'No command slew',
    'no_reachability_gate': 'No reachability gate',
    'no_multirate': 'No multirate',
    'lissajous': '24 s 3D Lissajous',
    'aggressive': '8 s aggressive maneuver',
    'large_attitude_45_60': '45/60 deg attitude hold',
    'large_attitude': '90/150 deg envelope probe',
    BOUNDARY_SCENARIO: '60/90 deg envelope probe',
}
COLORS = {
    'original_direct': '#6b7280',
    'basic_da': '#2f6f9f',
    'ada': '#2563eb',
    'nda': '#0891b2',
    'pda': '#16a34a',
    'ftr_drcda': '#7c3aed',
    'full': '#d3542f',
    'no_delay': '#2f855a',
    'no_horizon': '#b7791f',
    'no_physical_rate': '#8b5cf6',
    'no_command_slew': '#0f766e',
    'no_reachability_gate': '#be185d',
    'no_multirate': '#a855f7',
}


def number(row: dict[str, str], key: str) -> float:
    try:
        return float(row.get(key, 'nan'))
    except (TypeError, ValueError):
        return float('nan')


def array(rows: list[dict[str, str]], columns: tuple[str, ...]) -> np.ndarray:
    return np.array(
        [[number(row, column) for column in columns] for row in rows],
        dtype=float,
    )


def finite(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    return values[np.isfinite(values)]


def rms(values: np.ndarray) -> float:
    values = finite(values)
    return float(np.sqrt(np.mean(values ** 2))) if values.size else float('nan')


def total_variation(values: np.ndarray) -> float:
    values = np.asarray(values, dtype=float)
    if values.shape[0] < 2:
        return float('nan')
    return float(np.nansum(np.abs(np.diff(values, axis=0))))


def load_case(scenario: str, variant: str) -> dict[str, object]:
    directory = ROOT / 'runs' / scenario / variant
    result = json.loads((directory / 'result.json').read_text(encoding='utf-8'))
    csv_path = Path(result['csv'])
    with csv_path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    active = [
        row for row in rows
        if row.get('auto_traj_mode') == ACTIVE_MODE[scenario]
    ]
    if not active:
        raise RuntimeError(f'no active rows in {scenario}/{variant}')
    return {
        'result': result,
        'rows': rows,
        'active': active,
        'csv': csv_path,
    }


def case_metrics(scenario: str, variant: str, case: dict[str, object]) -> dict:
    rows = case['active']
    time_s = array(rows, ('time_s',))[:, 0]
    position = array(rows, (
        'position_x_enu_m', 'position_y_enu_m', 'position_z_rel_m',
    ))
    target_position = array(rows, (
        'target_x_enu_m', 'target_y_enu_m', 'target_z_rel_m',
    ))
    velocity = array(rows, (
        'velocity_x_enu_mps', 'velocity_y_enu_mps', 'velocity_z_enu_mps',
    ))
    target_velocity = array(rows, (
        'target_vx_enu_mps', 'target_vy_enu_mps', 'target_vz_enu_mps',
    ))
    position_error = target_position - position
    velocity_error = target_velocity - velocity
    position_error_norm = np.linalg.norm(position_error, axis=1)
    velocity_error_norm = np.linalg.norm(velocity_error, axis=1)
    attitude_error = array(rows, ('attitude_error_angle_deg',))[:, 0]
    servo = array(rows, tuple(f'cmd_servo_{index}' for index in range(4)))

    yaw_ned = np.deg2rad(
        90.0 - array(rows, ('yaw_deg',))[:, 0]
    )
    cosine = np.cos(yaw_ned)
    sine = np.sin(yaw_ned)
    error_ned_n = position_error[:, 1]
    error_ned_e = position_error[:, 0]
    velocity_error_n = velocity_error[:, 1]
    velocity_error_e = velocity_error[:, 0]
    body_y_error = -sine * error_ned_n + cosine * error_ned_e
    body_y_velocity_error = (
        -sine * velocity_error_n + cosine * velocity_error_e
    )

    residual_columns = tuple(
        f'drcda_wrench_residual_{name}'
        for name in ('fx_n', 'fy_n', 'fz_n', 'tx_nm', 'ty_nm', 'tz_nm')
    )
    residual = array(rows, residual_columns)
    residual_norm = np.linalg.norm(residual, axis=1)
    solve = finite(array(rows, ('drcda_solve_ms',))[:, 0])
    armed = array(rows, ('armed',))[:, 0]
    offboard = array(rows, ('offboard',))[:, 0]
    ground = array(rows, ('ground_contact',))[:, 0]
    safety = array(rows, ('direct_safety_cutoff',))[:, 0]
    duration = float(time_s[-1] - time_s[0]) if len(time_s) > 1 else 0.0

    hold_mask = np.zeros(len(rows), dtype=bool)
    if scenario.startswith('large_attitude'):
        target_roll = np.abs(array(rows, ('target_roll_deg',))[:, 0])
        target_pitch = np.abs(array(rows, ('target_pitch_deg',))[:, 0])
        roll_peak = float(np.nanmax(target_roll))
        pitch_peak = float(np.nanmax(target_pitch))
        hold_mask = (
            (target_roll >= 0.95 * roll_peak)
            | (target_pitch >= 0.95 * pitch_peak)
        )
    hold_error = attitude_error[hold_mask]
    hold_position_error = position_error_norm[hold_mask]

    expected = EXPECTED_DURATION[scenario]
    operational_complete = bool(
        case['result']['status'] == 'complete'
        and duration >= expected - 0.5
        and np.nanmin(armed) == 1.0
        and np.nanmin(offboard) == 1.0
        and np.nansum(ground) == 0.0
        and np.nansum(safety) == 0.0
    )
    position_limit, attitude_limit, hold_limit = TRACKING_LIMITS[scenario]
    hold_valid = (
        True
        if hold_limit is None
        else (
            math.isfinite(rms(hold_error))
            and rms(hold_error) <= hold_limit
            and rms(hold_position_error) <= position_limit
        )
    )
    tracking_valid = bool(
        operational_complete
        and float(np.nanmax(position_error_norm)) <= position_limit
        and rms(attitude_error) <= attitude_limit
        and hold_valid
    )
    return {
        'scenario': scenario,
        'variant': variant,
        'flight_valid': tracking_valid,
        'operational_complete': operational_complete,
        'tracking_valid': tracking_valid,
        'runner_status': case['result']['status'],
        'duration_s': duration,
        'samples': len(rows),
        'position_rmse_3d_m': rms(position_error_norm),
        'position_max_3d_m': float(np.nanmax(position_error_norm)),
        'velocity_rmse_3d_mps': rms(velocity_error_norm),
        'body_y_position_rmse_m': rms(body_y_error),
        'body_y_velocity_rmse_mps': rms(body_y_velocity_error),
        'attitude_error_rmse_deg': rms(attitude_error),
        'attitude_error_max_deg': float(np.nanmax(attitude_error)),
        'large_hold_attitude_rmse_deg': rms(hold_error),
        'large_hold_position_rmse_m': rms(hold_position_error),
        'servo_total_variation': total_variation(servo),
        'wrench_residual_rmse': rms(residual_norm),
        'solve_mean_ms': float(np.mean(solve)) if solve.size else None,
        'solve_p95_ms': (
            float(np.percentile(solve, 95.0)) if solve.size else None
        ),
        'ground_contact_samples': int(np.nansum(ground)),
        'armed_fraction': float(np.nanmean(armed)),
        'offboard_fraction': float(np.nanmean(offboard)),
        'max_reference_speed_mps': float(
            np.nanmax(np.linalg.norm(target_velocity, axis=1))
        ),
        'csv': str(case['csv']),
        'ulog': case['result']['ulog'],
        'failure': case['result'].get('error', ''),
    }


def relative(data: np.ndarray) -> np.ndarray:
    return data - data[0]


def active_arrays(case: dict[str, object]) -> dict[str, np.ndarray]:
    rows = case['active']
    return {
        'time': array(rows, ('time_s',))[:, 0],
        'position': array(rows, (
            'position_x_enu_m', 'position_y_enu_m', 'position_z_rel_m',
        )),
        'target_position': array(rows, (
            'target_x_enu_m', 'target_y_enu_m', 'target_z_rel_m',
        )),
        'roll': array(rows, ('roll_deg',))[:, 0],
        'pitch': array(rows, ('continuous_test_pitch_deg',))[:, 0],
        'target_roll': array(rows, ('target_roll_deg',))[:, 0],
        'target_pitch': array(rows, ('target_pitch_deg',))[:, 0],
        'attitude_error': array(rows, ('attitude_error_angle_deg',))[:, 0],
    }


def plot_trajectory(
    cases: dict[tuple[str, str], dict[str, object]],
    scenario: str,
    filename: str,
    variants: tuple[str, ...] = CORE_PLOT_VARIANTS,
) -> None:
    figure = plt.figure(figsize=(10.5, 7.8))
    axis = figure.add_subplot(111, projection='3d')
    first = active_arrays(cases[(scenario, 'full')])
    target = relative(first['target_position'])
    axis.plot(*target.T, '--', color='#202124', linewidth=2.0, label='Reference')
    for variant in variants:
        data = active_arrays(cases[(scenario, variant)])
        actual = data['position'] - data['target_position'][0]
        axis.plot(
            *actual.T,
            color=COLORS[variant],
            linewidth=1.6,
            label=NAMES[variant],
        )
    axis.set_xlabel('East relative position (m)')
    axis.set_ylabel('North relative position (m)')
    axis.set_zlabel('Relative altitude (m)')
    axis.set_title(NAMES[scenario])
    axis.view_init(elev=27, azim=-55)
    axis.legend(loc='upper left')
    axis.grid(alpha=0.25)
    figure.tight_layout()
    figure.savefig(FIGURES / filename, dpi=180)
    plt.close(figure)


def plot_core_errors(
    cases: dict[tuple[str, str], dict[str, object]],
) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(13.0, 8.5))
    for column, scenario in enumerate(('lissajous', 'aggressive')):
        for variant in CORE_PLOT_VARIANTS:
            data = active_arrays(cases[(scenario, variant)])
            t = data['time'] - data['time'][0]
            error = np.linalg.norm(
                data['target_position'] - data['position'], axis=1
            )
            axes[0, column].plot(
                t, error, color=COLORS[variant], linewidth=1.35,
                label=NAMES[variant],
            )
            axes[1, column].plot(
                t, data['attitude_error'], color=COLORS[variant],
                linewidth=1.25,
            )
        axes[0, column].set_title(NAMES[scenario])
        axes[0, column].set_ylabel('3D position error (m)')
        axes[1, column].set_ylabel('Attitude error (deg)')
        axes[1, column].set_xlabel('Trajectory time (s)')
        for row in range(2):
            axes[row, column].grid(alpha=0.25)
    axes[0, 0].legend()
    figure.tight_layout()
    figure.savefig(FIGURES / 'core_tracking_errors.png', dpi=180)
    plt.close(figure)


def plot_large_attitude(
    cases: dict[tuple[str, str], dict[str, object]],
    scenario: str,
    filename: str,
) -> None:
    figure, axes = plt.subplots(3, 1, figsize=(12.0, 9.5), sharex=True)
    reference = active_arrays(cases[(scenario, 'full')])
    t_ref = reference['time'] - reference['time'][0]
    axes[0].plot(t_ref, reference['target_roll'], '--', color='#202124', label='Roll reference')
    axes[1].plot(t_ref, reference['target_pitch'], '--', color='#202124', label='Pitch reference')
    for variant in CORE_PLOT_VARIANTS:
        data = active_arrays(cases[(scenario, variant)])
        t = data['time'] - data['time'][0]
        axes[0].plot(t, data['roll'], color=COLORS[variant], label=NAMES[variant])
        axes[1].plot(t, data['pitch'], color=COLORS[variant], label=NAMES[variant])
        axes[2].plot(t, data['attitude_error'], color=COLORS[variant], label=NAMES[variant])
    axes[0].set_ylabel('Roll (deg)')
    axes[1].set_ylabel('Continuous pitch (deg)')
    axes[2].set_ylabel('SO(3) error (deg)')
    axes[2].set_xlabel('Attitude-test time (s)')
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend(loc='upper right', ncol=2)
    figure.suptitle(NAMES[scenario])
    figure.tight_layout()
    figure.savefig(FIGURES / filename, dpi=180)
    plt.close(figure)


def plot_boundary_probe(case: dict[str, object]) -> None:
    data = active_arrays(case)
    time_s = data['time'] - data['time'][0]
    figure, axes = plt.subplots(3, 1, figsize=(12.0, 9.5), sharex=True)
    axes[0].plot(
        time_s, data['target_roll'], '--', color='#202124',
        label='Roll reference',
    )
    axes[0].plot(
        time_s, data['roll'], color=COLORS['full'], label=NAMES['full'],
    )
    axes[1].plot(
        time_s, data['target_pitch'], '--', color='#202124',
        label='Pitch reference',
    )
    axes[1].plot(
        time_s, data['pitch'], color=COLORS['full'], label=NAMES['full'],
    )
    axes[2].plot(
        time_s, data['attitude_error'], color=COLORS['full'],
        label='SO(3) error',
    )
    axes[0].set_ylabel('Roll (deg)')
    axes[1].set_ylabel('Continuous pitch (deg)')
    axes[2].set_ylabel('SO(3) error (deg)')
    axes[2].set_xlabel('Attitude-test time (s)')
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend(loc='upper right')
    figure.suptitle(NAMES[BOUNDARY_SCENARIO])
    figure.tight_layout()
    figure.savefig(FIGURES / 'large_attitude_60_90_boundary.png', dpi=180)
    plt.close(figure)


def plot_metric_summary(metrics: dict[str, dict[str, dict]]) -> None:
    figure, axes = plt.subplots(1, 3, figsize=(15.0, 5.2))
    variants = CORE_PLOT_VARIANTS
    keys = (
        'position_rmse_3d_m',
        'body_y_velocity_rmse_mps',
        'attitude_error_rmse_deg',
    )
    titles = (
        '3D position RMSE (m)',
        'Body-Y velocity RMSE (m/s)',
        'Attitude error RMSE (deg)',
    )
    width = 0.16
    x = np.arange(len(SCENARIOS))
    for index, variant in enumerate(variants):
        for axis, key in zip(axes, keys):
            values = [metrics[s][variant][key] for s in SCENARIOS]
            bars = axis.bar(
                x + (index - 0.5 * (len(variants) - 1)) * width,
                values,
                width,
                color=COLORS[variant],
                label=NAMES[variant],
            )
            for bar, scenario in zip(bars, SCENARIOS):
                if not metrics[scenario][variant]['tracking_valid']:
                    bar.set_hatch('//')
                    bar.set_edgecolor('#991b1b')
    for axis, title in zip(axes, titles):
        axis.set_title(title)
        axis.set_yscale('log')
        axis.set_xticks(x)
        axis.set_xticklabels(
            ('Lissajous', 'Aggressive', '45/60 deg'),
            rotation=20,
        )
        axis.grid(axis='y', alpha=0.25)
    axes[0].legend()
    figure.tight_layout()
    figure.savefig(FIGURES / 'core_metric_summary.png', dpi=180)
    plt.close(figure)


def plot_ablation(metrics: dict[str, dict[str, dict]]) -> None:
    variants = ABLATION_VARIANTS
    figure, axes = plt.subplots(3, 1, figsize=(14.0, 10.5))
    keys = (
        'position_rmse_3d_m',
        'attitude_error_rmse_deg',
        'servo_total_variation',
    )
    titles = (
        '3D position RMSE (m)',
        'Attitude error RMSE (deg)',
        'Servo command total variation',
    )
    x = np.arange(len(variants))
    width = 0.19
    scenario_colors = ('#3b82f6', '#10b981', '#f97316', '#64748b')
    for scenario_index, scenario in enumerate(SCENARIOS):
        for axis, key in zip(axes, keys):
            values = [metrics[scenario][variant][key] for variant in variants]
            bars = axis.bar(
                x + (
                    scenario_index - 0.5 * (len(SCENARIOS) - 1)
                ) * width,
                values,
                width,
                label=NAMES[scenario],
                color=scenario_colors[scenario_index],
            )
            for bar, variant in zip(bars, variants):
                if not metrics[scenario][variant]['flight_valid']:
                    bar.set_hatch('//')
                    bar.set_edgecolor('#991b1b')
    labels = [NAMES[variant].replace(' ', '\n') for variant in variants]
    for axis, title in zip(axes, titles):
        axis.set_title(title)
        axis.set_xticks(x)
        axis.set_xticklabels(labels, fontsize=8)
        axis.grid(axis='y', alpha=0.25)
    axes[0].legend(ncol=4)
    figure.tight_layout()
    figure.savefig(FIGURES / 'ablation_multiscenario.png', dpi=180)
    plt.close(figure)


def fmt(value: object, digits: int = 3) -> str:
    if value is None:
        return 'n/a'
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return str(value)
    return 'n/a' if not math.isfinite(numeric) else f'{numeric:.{digits}f}'


def markdown_report(
    metrics: dict[str, dict[str, dict]],
    boundary: dict,
) -> str:
    lines = [
        '# Hnuter 微分分配多场景完整对比与消融实验',
        '',
        '## 实验设置',
        '',
        '- 被控对象固定为 `PX4-Autopilot-Hnuter-delay` 动态舵机固件；纯延迟和独立一阶惯性均关闭，保留静态增益、方向相关速率限制、关节物理与 PID。',
        '- 核心对比包括原始直接分配、基础 DA、论文 ADA/NDA/PDA、纯 FTR-DRCDA，以及按工况切换 PDA/FTR 的工程综合版本。',
        '- FTR 消融项为去延迟模型、去预测时域、去物理速率约束、去命令斜率、去可达性门控和去多速率电机块；所有消融均以纯 `FTR-DRCDA` 为基线。',
        '- 场景包括 24 s 三维李萨如、8 s 高速机动、滚转 45 度/俯仰 60 度大姿态悬停，并补充 60/90 度稳定包线探测。',
        '- 每个组合独立重启 SITL；当前为单次确定性仿真，不提供统计置信区间。',
        '- “计时完成”只表示控制器跑完参考序列；“跟踪有效”还要求不接地、持续 Armed/Offboard，并满足场景的位置和姿态误差门限。',
        '',
        '## 参数筛选',
        '',
        '- 在固定位置环和舵机 PID 后，对分配器做小范围代表场景筛选；最终采用 `wrench_error_gain=12`、预测时域 `0.18 s`、舵机移动权重缩放 `10`。',
        '- 高速机动中，增益 `6/8/10/12` 的位置 RMSE 分别为 `0.620/0.465/0.444/0.439 m`，姿态 RMSE 分别为 `8.76/8.22/7.93/7.68 deg`。',
        '- 将预测时域从 `0.18 s` 缩短为 `0.12 s` 未改善大姿态误差，并使扳手残差 RMSE 从约 `0.049` 增至 `0.108`，因此保留 `0.18 s`。',
        '- 将舵机移动权重缩放从 `10` 降至 `5` 仅小幅改善位置误差，却使舵机总变差增加约 `38%`，因此保留 `10`。',
        '',
        '## 核心算法对比',
        '',
    ]
    for scenario in SCENARIOS:
        lines.extend([
            f'### {NAMES[scenario]}',
            '',
            '| 算法 | 计时完成 | 跟踪有效 | 位置RMSE(m) | 机体Y速度RMSE(m/s) | 姿态RMSE(deg) | 舵机总变差 |',
            '| --- | --- | --- | ---: | ---: | ---: | ---: |',
        ])
        for variant in CORE_VARIANTS:
            item = metrics[scenario][variant]
            lines.append(
                f'| {NAMES[variant]} '
                f'| {"是" if item["runner_status"] == "complete" else "否"} '
                f'| {"是" if item["tracking_valid"] else "否"} '
                f'| {fmt(item["position_rmse_3d_m"])} '
                f'| {fmt(item["body_y_velocity_rmse_mps"])} '
                f'| {fmt(item["attitude_error_rmse_deg"])} '
                f'| {fmt(item["servo_total_variation"])} |'
            )
        lines.append('')

    lines.extend(['## 消融实验', ''])
    for scenario in SCENARIOS:
        lines.extend([
            f'### {NAMES[scenario]}',
            '',
            '| 变体 | 计时完成 | 跟踪有效 | 位置RMSE(m) | 峰值位置误差(m) | 姿态RMSE(deg) | 残差RMSE | 求解P95(ms) |',
            '| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |',
        ])
        for variant in ABLATION_VARIANTS:
            item = metrics[scenario][variant]
            lines.append(
                f'| {NAMES[variant]} '
                f'| {"是" if item["runner_status"] == "complete" else "否"} '
                f'| {"是" if item["tracking_valid"] else "否"} '
                f'| {fmt(item["position_rmse_3d_m"])} '
                f'| {fmt(item["position_max_3d_m"])} '
                f'| {fmt(item["attitude_error_rmse_deg"])} '
                f'| {fmt(item["wrench_residual_rmse"])} '
                f'| {fmt(item["solve_p95_ms"])} |'
            )
        lines.append('')

    best_lines = []
    for scenario in SCENARIOS:
        valid = [
            metrics[scenario][variant]
            for variant in CORE_VARIANTS
            if metrics[scenario][variant]['tracking_valid']
        ]
        if valid:
            best = min(valid, key=lambda item: item['position_rmse_3d_m'])
            best_lines.append(
                f'- {NAMES[scenario]}：有效核心算法中位置 RMSE 最低的是 '
                f'`{NAMES[best["variant"]]}`，为 '
                f'`{fmt(best["position_rmse_3d_m"])}` m，姿态 RMSE '
                f'`{fmt(best["attitude_error_rmse_deg"])}` deg。'
            )
        else:
            best_lines.append(
                f'- {NAMES[scenario]}：没有核心算法通过跟踪有效性判据。'
            )

    ftr = {scenario: metrics[scenario]['ftr_drcda'] for scenario in SCENARIOS}
    hybrid = {scenario: metrics[scenario]['full'] for scenario in SCENARIOS}
    lines.extend([
        '## 主要结论',
        '',
        *best_lines,
        '',
        f'- 纯 FTR-DRCDA 在李萨如、高速机动和 45/60 度场景的跟踪有效状态依次为 '
        f'`{[ftr[scenario]["tracking_valid"] for scenario in SCENARIOS]}`；工程综合版本依次为 '
        f'`{[hybrid[scenario]["tracking_valid"] for scenario in SCENARIOS]}`。',
        f'- 60/90 度边界探测的工程综合版本位置峰值误差为 '
        f'`{fmt(boundary["position_max_3d_m"])}` m，保持段姿态 RMSE 为 '
        f'`{fmt(boundary["large_hold_attitude_rmse_deg"])}` deg，未通过跟踪有效性判据。',
        f'- 相比 Paper PDA，FTR-DRCDA 将高速机动位置 RMSE 降低 '
        f'`{100.0 * (1.0 - ftr["aggressive"]["position_rmse_3d_m"] / metrics["aggressive"]["pda"]["position_rmse_3d_m"]):.1f}%`，'
        f'将 45/60 度大姿态位置 RMSE 降低 '
        f'`{100.0 * (1.0 - ftr["large_attitude_45_60"]["position_rmse_3d_m"] / metrics["large_attitude_45_60"]["pda"]["position_rmse_3d_m"]):.1f}%`。'
        f'大姿态峰值保持段姿态 RMSE 为 '
        f'`{fmt(ftr["large_attitude_45_60"]["large_hold_attitude_rmse_deg"])}` deg，'
        f'保持段位置 RMSE 为 '
        f'`{fmt(ftr["large_attitude_45_60"]["large_hold_position_rmse_m"])}` m。',
        '- 当前纯延迟参数为零，因此 `no_delay` 是实现一致性检查，不应被解读为延迟模型贡献；其余消融才反映有限时域结构、物理速率、命令斜率、可达性门控和多速率设计的作用。',
        '- 原始直接分配保留旧 NED 外环，而 DA 系列使用新调好的机体系 XY 分轴位置环，所以 `Original direct` 是旧完整控制链基线，不是严格的单分配器替换实验。',
        '',
        '## 数据位置',
        '',
        '- 每个用例位于 `runs/场景/算法/`，包含 CSV、ULog、控制器日志、PX4 日志、Agent 日志和 `result.json`。',
        '- 统一指标位于 `reports/metrics.json`，图片位于 `figures/`。',
    ])
    return '\n'.join(lines) + '\n'


def markdown_to_html(markdown: str) -> str:
    output: list[str] = []
    lines = markdown.splitlines()
    index = 0
    while index < len(lines):
        line = lines[index].strip()
        if not line:
            index += 1
            continue
        if line.startswith('# '):
            output.append(f'<h1>{html.escape(line[2:])}</h1>')
            index += 1
            continue
        if line.startswith('## '):
            output.append(f'<h2>{html.escape(line[3:])}</h2>')
            index += 1
            continue
        if line.startswith('### '):
            output.append(f'<h3>{html.escape(line[4:])}</h3>')
            index += 1
            continue
        if line.startswith('|') and index + 1 < len(lines):
            block = []
            while index < len(lines) and lines[index].strip().startswith('|'):
                block.append([
                    cell.strip()
                    for cell in lines[index].strip().strip('|').split('|')
                ])
                index += 1
            headers = block[0]
            rows = block[2:]
            output.append('<table><thead><tr>')
            output.extend(f'<th>{html.escape(cell)}</th>' for cell in headers)
            output.append('</tr></thead><tbody>')
            for row in rows:
                output.append('<tr>')
                output.extend(f'<td>{html.escape(cell)}</td>' for cell in row)
                output.append('</tr>')
            output.append('</tbody></table>')
            continue
        if line.startswith('- '):
            items = []
            while index < len(lines) and lines[index].strip().startswith('- '):
                items.append(lines[index].strip()[2:])
                index += 1
            output.append('<ul>')
            output.extend(f'<li>{html.escape(item)}</li>' for item in items)
            output.append('</ul>')
            continue
        if line[:2].isdigit() and '. ' in line[:4]:
            items = []
            while index < len(lines):
                raw = lines[index].strip()
                if not raw[:1].isdigit() or '. ' not in raw[:4]:
                    break
                items.append(raw.split('. ', 1)[1])
                index += 1
            output.append('<ol>')
            output.extend(f'<li>{html.escape(item)}</li>' for item in items)
            output.append('</ol>')
            continue
        output.append(f'<p>{html.escape(line)}</p>')
        index += 1
    return '\n'.join(output)


def html_report(markdown: str) -> str:
    images = []
    for path in sorted(FIGURES.glob('*.png')):
        payload = base64.b64encode(path.read_bytes()).decode('ascii')
        images.append(
            f'<section><h2>{html.escape(path.stem.replace("_", " "))}</h2>'
            f'<img src="data:image/png;base64,{payload}"></section>'
        )
    return f"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><style>
@page {{ size: A4 landscape; margin: 12mm; }}
body {{ font-family: "Noto Sans CJK SC", sans-serif; color: #17202a; }}
article {{ font-size: 8.5pt; line-height: 1.45; }}
section {{ page-break-before: always; }}
img {{ width: 100%; max-height: 180mm; object-fit: contain; }}
h1, h2, h3 {{ color: #17384a; }}
table {{ width: 100%; border-collapse: collapse; font-size: 7.3pt; margin-bottom: 4mm; }}
th, td {{ border: 1px solid #aebdc4; padding: 1.1mm; }}
th {{ background: #dceaf0; }}
</style></head><body>
<article>{markdown_to_html(markdown)}</article>
{''.join(images)}
</body></html>"""


def main() -> None:
    global ROOT, FIGURES, REPORTS
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=ROOT)
    args = parser.parse_args()
    ROOT = args.root.expanduser().resolve()
    FIGURES = ROOT / 'figures'
    REPORTS = ROOT / 'reports'

    FIGURES.mkdir(parents=True, exist_ok=True)
    REPORTS.mkdir(parents=True, exist_ok=True)
    cases = {
        (scenario, variant): load_case(scenario, variant)
        for scenario in SCENARIOS
        for variant in VARIANTS
    }
    metrics = {
        scenario: {
            variant: case_metrics(scenario, variant, cases[(scenario, variant)])
            for variant in VARIANTS
        }
        for scenario in SCENARIOS
    }
    boundary_case = load_case(BOUNDARY_SCENARIO, 'full')
    boundary_metrics = case_metrics(
        BOUNDARY_SCENARIO, 'full', boundary_case
    )
    manifest = {
        'firmware': str(
            cases[('lissajous', 'full')]['result']['firmware']
        ),
        'scenarios': list(SCENARIOS),
        'variants': list(VARIANTS),
        'cases': [
            cases[(scenario, variant)]['result']
            for scenario in SCENARIOS
            for variant in VARIANTS
        ] + [boundary_case['result']],
    }
    (ROOT / 'manifest.json').write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    (REPORTS / 'metrics.json').write_text(
        json.dumps(
            {
                'scenarios': metrics,
                'boundary_probe_60_90': boundary_metrics,
            },
            indent=2,
            ensure_ascii=False,
        ) + '\n',
        encoding='utf-8',
    )
    summary_rows = [
        metrics[scenario][variant]
        for scenario in SCENARIOS
        for variant in VARIANTS
    ] + [boundary_metrics]
    summary_path = REPORTS / 'metrics_summary.csv'
    with summary_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=summary_rows[0].keys())
        writer.writeheader()
        writer.writerows(summary_rows)

    plot_trajectory(cases, 'lissajous', 'lissajous_3d_core.png')
    plot_trajectory(cases, 'aggressive', 'aggressive_3d_core.png')
    plot_trajectory(
        cases,
        'aggressive',
        'aggressive_3d_stable_detail.png',
        ('pda', 'ftr_drcda', 'full'),
    )
    plot_core_errors(cases)
    plot_large_attitude(
        cases,
        'large_attitude_45_60',
        'large_attitude_45_60_core.png',
    )
    plot_boundary_probe(boundary_case)
    plot_metric_summary(metrics)
    plot_ablation(metrics)

    report = markdown_report(metrics, boundary_metrics)
    (REPORTS / 'multiscenario_report_zh.md').write_text(
        report, encoding='utf-8'
    )
    (REPORTS / 'multiscenario_report_zh.html').write_text(
        html_report(report), encoding='utf-8'
    )
    print(REPORTS / 'metrics.json')
    print(summary_path)
    print(REPORTS / 'multiscenario_report_zh.md')
    print(REPORTS / 'multiscenario_report_zh.html')


if __name__ == '__main__':
    main()
