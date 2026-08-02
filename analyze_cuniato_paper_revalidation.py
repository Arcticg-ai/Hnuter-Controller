#!/usr/bin/env python3
"""Analyze the Cuniato 2026 allocator reconstruction on delayed SITL."""

from __future__ import annotations

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


WORKSPACE = Path(__file__).resolve().parent
OUTPUT = (
    WORKSPACE / 'hnuter_logs' / 'cuniato_paper_revalidation_20260730'
)
FIGURES = OUTPUT / 'figures'
REPORTS = OUTPUT / 'reports'

SCENARIOS = ('lissajous', 'aggressive', 'large_attitude_45_60')
METHODS = (
    'original_direct',
    'basic_da',
    'ada',
    'nda',
    'full',
    'ftr_drcda',
)
NAMES = {
    'lissajous': '24 s 3D Lissajous',
    'aggressive': '8 s aggressive maneuver',
    'large_attitude_45_60': '45/60 deg attitude hold',
    'original_direct': 'Original direct',
    'basic_da': 'Paper DA',
    'ada': 'Paper ADA',
    'nda': 'Paper NDA',
    'full': 'Final hybrid',
    'ftr_drcda': 'Previous FTR-DRCDA',
}
COLORS = {
    'original_direct': '#6b7280',
    'basic_da': '#111827',
    'ada': '#2563a6',
    'nda': '#16836a',
    'full': '#d3542f',
    'ftr_drcda': '#7c3aed',
}

V2 = WORKSPACE / 'hnuter_logs' / 'drcda_paper_revalidation_20260730_v2'
V5 = WORKSPACE / 'hnuter_logs' / 'drcda_paper_revalidation_20260730_v5'
FINAL_STAGES = (
    WORKSPACE / 'hnuter_logs'
    / 'drcda_paper_revalidation_20260730_final_stages'
)
FINAL_FULL = (
    WORKSPACE / 'hnuter_logs'
    / 'drcda_paper_revalidation_20260730_final_full'
)
BASIC_DA = (
    WORKSPACE / 'hnuter_logs'
    / 'drcda_paper_revalidation_20260730_final_basic_da'
)
HYBRID = (
    WORKSPACE / 'hnuter_logs'
    / 'drcda_paper_revalidation_20260730_hybrid'
)
PURE_PDA = (
    WORKSPACE / 'hnuter_logs'
    / 'drcda_paper_revalidation_20260730_pure_pda'
)


def source_directory(scenario: str, method: str) -> Path:
    if method == 'basic_da':
        root = BASIC_DA
        return root / 'runs' / scenario / method
    if scenario in ('lissajous', 'aggressive'):
        if method in ('ada', 'nda'):
            root = FINAL_STAGES
        elif method == 'full':
            root = FINAL_FULL
        else:
            root = V2
    else:
        root = FINAL_FULL if method == 'full' else V5
    return root / 'runs' / scenario / method


def number(row: dict[str, str], key: str) -> float:
    try:
        return float(row.get(key, 'nan'))
    except (TypeError, ValueError):
        return float('nan')


def values(
    rows: list[dict[str, str]],
    columns: tuple[str, ...],
) -> np.ndarray:
    return np.array([
        [number(row, column) for column in columns]
        for row in rows
    ])


def rms(data: np.ndarray) -> float:
    data = np.asarray(data, dtype=float)
    data = data[np.isfinite(data)]
    return (
        float(np.sqrt(np.mean(data ** 2)))
        if data.size else float('nan')
    )


def load_case(scenario: str, method: str) -> dict:
    directory = source_directory(scenario, method)
    result = json.loads(
        (directory / 'result.json').read_text(encoding='utf-8')
    )
    csv_path = Path(result['csv'])
    with csv_path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    active_mode = (
        'attitude'
        if scenario.startswith('large_attitude')
        else 'lissajous'
    )
    active = [
        row for row in rows
        if row.get('auto_traj_mode') == active_mode
    ]
    if not active:
        raise RuntimeError(f'no active rows in {scenario}/{method}')
    return {
        'directory': directory,
        'result': result,
        'rows': active,
        'csv': csv_path,
    }


def metrics(scenario: str, method: str, case: dict) -> dict:
    rows = case['rows']
    position = values(rows, (
        'position_x_enu_m', 'position_y_enu_m', 'position_z_rel_m',
    ))
    position_target = values(rows, (
        'target_x_enu_m', 'target_y_enu_m', 'target_z_rel_m',
    ))
    velocity = values(rows, (
        'velocity_x_enu_mps', 'velocity_y_enu_mps', 'velocity_z_enu_mps',
    ))
    velocity_target = values(rows, (
        'target_vx_enu_mps', 'target_vy_enu_mps', 'target_vz_enu_mps',
    ))
    position_error = np.linalg.norm(position_target - position, axis=1)
    velocity_error = np.linalg.norm(velocity_target - velocity, axis=1)
    attitude_error = values(rows, ('attitude_error_angle_deg',))[:, 0]
    servo = values(
        rows, tuple(f'cmd_servo_{index}' for index in range(4))
    )
    motor = values(
        rows, tuple(f'cmd_motor_{index}' for index in range(5))
    )
    rotor_speed = np.zeros_like(motor)
    rotor_speed[:, :4] = np.where(
        motor[:, :4] > 0.0,
        10.0 + 990.0 * motor[:, :4],
        0.0,
    )
    rotor_speed[:, 4] = 1000.0 * np.abs(motor[:, 4])
    power_proxy = np.sum(rotor_speed ** 3, axis=1)
    ground = values(rows, ('ground_contact',))[:, 0]
    armed = values(rows, ('armed',))[:, 0]
    offboard = values(rows, ('offboard',))[:, 0]
    time_s = values(rows, ('time_s',))[:, 0]
    solve = values(rows, ('drcda_solve_ms',))[:, 0]
    solve = solve[np.isfinite(solve)]

    hold = np.zeros(len(rows), dtype=bool)
    if scenario.startswith('large_attitude'):
        target_roll = np.abs(
            values(rows, ('target_roll_deg',))[:, 0]
        )
        target_pitch = np.abs(
            values(rows, ('target_pitch_deg',))[:, 0]
        )
        hold = (
            (target_roll >= 0.95 * np.max(target_roll))
            | (target_pitch >= 0.95 * np.max(target_pitch))
        )

    duration = float(time_s[-1] - time_s[0])
    position_limit = 2.0 if scenario != 'aggressive' else 5.0
    attitude_limit = 30.0 if scenario != 'aggressive' else 45.0
    valid = bool(
        case['result']['status'] == 'complete'
        and np.sum(ground) == 0.0
        and np.min(armed) == 1.0
        and np.min(offboard) == 1.0
        and np.max(position_error) <= position_limit
        and rms(attitude_error) <= attitude_limit
        and (
            not np.any(hold)
            or rms(attitude_error[hold]) <= 30.0
        )
    )
    return {
        'scenario': scenario,
        'method': method,
        'tracking_valid': valid,
        'duration_s': duration,
        'position_rmse_m': rms(position_error),
        'position_max_m': float(np.max(position_error)),
        'velocity_rmse_mps': rms(velocity_error),
        'attitude_rmse_deg': rms(attitude_error),
        'attitude_max_deg': float(np.max(attitude_error)),
        'hold_attitude_rmse_deg': rms(attitude_error[hold]),
        'hold_position_rmse_m': rms(position_error[hold]),
        'servo_total_variation': float(
            np.sum(np.abs(np.diff(servo, axis=0)))
        ),
        'motor_total_variation': float(
            np.sum(np.abs(np.diff(motor, axis=0)))
        ),
        'mean_power_proxy_1e8': float(np.mean(power_proxy) / 1e8),
        'front_rotor_speed_spread_rad_s': float(np.mean(
            np.std(rotor_speed[:, :4], axis=1)
        )),
        'max_rotor_speed_rad_s': float(np.max(rotor_speed)),
        'solve_p95_ms': (
            float(np.percentile(solve, 95.0))
            if solve.size else None
        ),
        'ground_contact_samples': int(np.sum(ground)),
        'csv': str(case['csv']),
        'ulog': case['result']['ulog'],
        'source_directory': str(case['directory']),
    }


def case_arrays(case: dict) -> dict[str, np.ndarray]:
    rows = case['rows']
    return {
        'time': values(rows, ('time_s',))[:, 0],
        'position': values(rows, (
            'position_x_enu_m',
            'position_y_enu_m',
            'position_z_rel_m',
        )),
        'position_target': values(rows, (
            'target_x_enu_m',
            'target_y_enu_m',
            'target_z_rel_m',
        )),
        'roll': values(rows, ('roll_deg',))[:, 0],
        'pitch': values(rows, ('continuous_test_pitch_deg',))[:, 0],
        'roll_target': values(rows, ('target_roll_deg',))[:, 0],
        'pitch_target': values(rows, ('target_pitch_deg',))[:, 0],
        'attitude_error': values(
            rows, ('attitude_error_angle_deg',)
        )[:, 0],
    }


def plot_tracking(cases: dict[tuple[str, str], dict]) -> None:
    figure, axes = plt.subplots(3, 2, figsize=(13.5, 11.0))
    for row, scenario in enumerate(SCENARIOS):
        for method in METHODS:
            data = case_arrays(cases[(scenario, method)])
            time_s = data['time'] - data['time'][0]
            position_error = np.linalg.norm(
                data['position_target'] - data['position'], axis=1
            )
            axes[row, 0].plot(
                time_s, position_error, color=COLORS[method],
                linewidth=1.15, label=NAMES[method],
            )
            axes[row, 1].plot(
                time_s, data['attitude_error'], color=COLORS[method],
                linewidth=1.15,
            )
        axes[row, 0].set_ylabel(
            f'{NAMES[scenario]}\nposition error (m)'
        )
        axes[row, 1].set_ylabel('SO(3) error (deg)')
        for axis in axes[row]:
            axis.grid(alpha=0.25)
    axes[-1, 0].set_xlabel('Active trajectory time (s)')
    axes[-1, 1].set_xlabel('Active trajectory time (s)')
    axes[0, 0].legend(ncol=3, fontsize=8)
    figure.tight_layout()
    figure.savefig(FIGURES / 'tracking_error_comparison.png', dpi=180)
    plt.close(figure)


def plot_summary(all_metrics: dict[str, dict[str, dict]]) -> None:
    figure, axes_grid = plt.subplots(2, 2, figsize=(13.5, 9.0))
    axes = axes_grid.ravel()
    keys = (
        'position_rmse_m',
        'attitude_rmse_deg',
        'mean_power_proxy_1e8',
        'front_rotor_speed_spread_rad_s',
    )
    titles = (
        '3D position RMSE (m)',
        'Attitude RMSE (deg)',
        'Mean rotor power proxy (1e8)',
        'Front rotor speed spread (rad/s)',
    )
    x = np.arange(len(SCENARIOS))
    width = 0.135
    for method_index, method in enumerate(METHODS):
        offset = (
            method_index - (len(METHODS) - 1) / 2.0
        ) * width
        for axis, key in zip(axes, keys):
            bars = axis.bar(
                x + offset,
                [all_metrics[s][method][key] for s in SCENARIOS],
                width,
                color=COLORS[method],
                label=NAMES[method],
            )
            for bar, scenario in zip(bars, SCENARIOS):
                if not all_metrics[scenario][method]['tracking_valid']:
                    bar.set_hatch('//')
                    bar.set_edgecolor('#991b1b')
    for axis, title in zip(axes, titles):
        axis.set_title(title)
        axis.set_xticks(x)
        axis.set_xticklabels(
            ('Lissajous', 'Aggressive', '45/60 deg'),
            rotation=18,
        )
        axis.grid(axis='y', alpha=0.25)
    axes[0].legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(FIGURES / 'paper_stage_metric_summary.png', dpi=180)
    plt.close(figure)


def plot_trajectory(
    cases: dict[tuple[str, str], dict],
    scenario: str,
    filename: str,
) -> None:
    figure = plt.figure(figsize=(10.5, 7.8))
    axis = figure.add_subplot(111, projection='3d')
    reference = case_arrays(cases[(scenario, 'full')])
    origin = reference['position_target'][0]
    axis.plot(
        *(reference['position_target'] - origin).T,
        '--', color='#202124', linewidth=2.0, label='Reference',
    )
    for method in ('original_direct', 'full', 'ftr_drcda'):
        data = case_arrays(cases[(scenario, method)])
        axis.plot(
            *(data['position'] - data['position_target'][0]).T,
            color=COLORS[method], linewidth=1.5, label=NAMES[method],
        )
    axis.set_xlabel('East relative position (m)')
    axis.set_ylabel('North relative position (m)')
    axis.set_zlabel('Relative altitude (m)')
    axis.set_title(NAMES[scenario])
    axis.legend()
    axis.view_init(elev=27, azim=-55)
    figure.tight_layout()
    figure.savefig(FIGURES / filename, dpi=180)
    plt.close(figure)


def plot_large_attitude(
    cases: dict[tuple[str, str], dict],
) -> None:
    scenario = 'large_attitude_45_60'
    figure, axes = plt.subplots(3, 1, figsize=(12.0, 9.5), sharex=True)
    reference = case_arrays(cases[(scenario, 'full')])
    time_ref = reference['time'] - reference['time'][0]
    axes[0].plot(
        time_ref, reference['roll_target'], '--',
        color='#202124', label='Roll reference',
    )
    axes[1].plot(
        time_ref, reference['pitch_target'], '--',
        color='#202124', label='Pitch reference',
    )
    for method in ('original_direct', 'full', 'ftr_drcda'):
        data = case_arrays(cases[(scenario, method)])
        time_s = data['time'] - data['time'][0]
        axes[0].plot(
            time_s, data['roll'], color=COLORS[method],
            label=NAMES[method],
        )
        axes[1].plot(
            time_s, data['pitch'], color=COLORS[method],
            label=NAMES[method],
        )
        axes[2].plot(
            time_s, data['attitude_error'], color=COLORS[method],
            label=NAMES[method],
        )
    axes[0].set_ylabel('Roll (deg)')
    axes[1].set_ylabel('Pitch (deg)')
    axes[2].set_ylabel('SO(3) error (deg)')
    axes[2].set_xlabel('Attitude-test time (s)')
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend(ncol=2, fontsize=8)
    figure.tight_layout()
    figure.savefig(FIGURES / 'large_attitude_hybrid.png', dpi=180)
    plt.close(figure)


def fmt(value: object) -> str:
    if value is None:
        return 'n/a'
    numeric = float(value)
    return 'n/a' if not math.isfinite(numeric) else f'{numeric:.3f}'


def report_markdown(
    all_metrics: dict[str, dict[str, dict]],
    pure_pda_large: dict,
) -> str:
    lines = [
        '# Cuniato 2026 论文对照修正与带延迟固件复验',
        '',
        '## 发现并修正的问题',
        '',
        '1. 旧版核心是有限时域终端扳手拟合，并不是论文的 ADA-NDA-PDA 递进实现。',
        '2. 旧版缺少论文式 (7) 的执行器反馈扳手增广、式 (8)-(12) 的非对称速率归一化，以及式 (13) 的统一比例饱和。',
        '3. 旧版所谓功率动态是在推力域按剩余裕度构造的启发式曲线；修正版先在转速域建立状态相关加速度上下界，再通过 F=k*omega^2 映射到推力变化率。',
        '4. 一阶动态反解最初直接使用 qd=q+tau*qdot，但 100 Hz 分配周期远大于 1 ms 电机时间常数。现改为精确离散反解 qd=q+dt*qdot/(1-exp(-dt/tau))。',
        '5. 论文没有描述 110-156 ms 纯延迟。最终版本在常规和高速机动使用 PDA，在自动大姿态轨迹使用有限时域可达分配，并在切换时传递执行器估计状态。',
        '6. 试验过的即时电机力矩残差补偿会显著扩大同轴电机转速离散度，破坏 PDA 的功率平衡目标，因此保留为显式开关但默认关闭。',
        '',
        '## 实验设置',
        '',
        '- 所有实验使用 `PX4-Autopilot-Hnuter-delay` identified-delay 固件。',
        '- 每个算法/场景独立重启 PX4、Gazebo、DDS Agent 和控制器。',
        '- DA、ADA、NDA、PDA 使用同一外环；Original direct 保留原始直接分配链。',
        '- 当前为单次确定性 SITL，结论不包含统计置信区间。',
        '',
        '## 结果',
        '',
    ]
    for scenario in SCENARIOS:
        lines.extend([
            f'### {NAMES[scenario]}',
            '',
            '| 方法 | 有效 | 位置RMSE(m) | 姿态RMSE(deg) | 功率代理(1e8) | 前电机转速离散(rad/s) | P95(ms) |',
            '| --- | --- | ---: | ---: | ---: | ---: | ---: |',
        ])
        for method in METHODS:
            item = all_metrics[scenario][method]
            lines.append(
                f'| {NAMES[method]} '
                f'| {"是" if item["tracking_valid"] else "否"} '
                f'| {fmt(item["position_rmse_m"])} '
                f'| {fmt(item["attitude_rmse_deg"])} '
                f'| {fmt(item["mean_power_proxy_1e8"])} '
                f'| {fmt(item["front_rotor_speed_spread_rad_s"])} '
                f'| {fmt(item["solve_p95_ms"])} |'
            )
        lines.append('')

    direct_l = all_metrics['lissajous']['original_direct']
    da_l = all_metrics['lissajous']['basic_da']
    final_l = all_metrics['lissajous']['full']
    direct_a = all_metrics['aggressive']['original_direct']
    da_a = all_metrics['aggressive']['basic_da']
    final_a = all_metrics['aggressive']['full']
    nda_a = all_metrics['aggressive']['nda']
    ftr_a = all_metrics['aggressive']['ftr_drcda']
    final_large = all_metrics['large_attitude_45_60']['full']
    lines.extend([
        '## 结论',
        '',
        f'1. 同一外环下，李萨如位置 RMSE 从论文 DA 基线的 `{fmt(da_l["position_rmse_m"])}` m 降到最终方法的 `{fmt(final_l["position_rmse_m"])}` m，姿态 RMSE 从 `{fmt(da_l["attitude_rmse_deg"])}` deg 降到 `{fmt(final_l["attitude_rmse_deg"])}` deg；原始直接控制链的位置/姿态 RMSE 为 `{fmt(direct_l["position_rmse_m"])}` m / `{fmt(direct_l["attitude_rmse_deg"])}` deg。',
        f'2. 高速机动中论文 DA 和原始直接控制链均失效，位置 RMSE 分别为 `{fmt(da_a["position_rmse_m"])}` m 和 `{fmt(direct_a["position_rmse_m"])}` m；最终方法有效，位置 RMSE `{fmt(final_a["position_rmse_m"])}` m、姿态 RMSE `{fmt(final_a["attitude_rmse_deg"])}` deg。',
        f'3. 45/60 度大姿态中最终混合方法有效，位置 RMSE `{fmt(final_large["position_rmse_m"])}` m，保持段姿态 RMSE `{fmt(final_large["hold_attitude_rmse_deg"])}` deg。',
        f'4. 高速单次试验中 NDA 的位置 RMSE `{fmt(nda_a["position_rmse_m"])}` m 优于 PDA 的 `{fmt(final_a["position_rmse_m"])}` m；PDA 的功率代理为 `{fmt(final_a["mean_power_proxy_1e8"])}`，NDA 为 `{fmt(nda_a["mean_power_proxy_1e8"])}`。这与论文“PDA 和 NDA 跟踪无显著差异，PDA 主要释放零空间”的结论一致，不能宣称 PDA 必然降低全部 RMSE。',
        f'5. 旧有限时域方法在高速姿态 RMSE `{fmt(ftr_a["attitude_rmse_deg"])}` deg，仍优于纯 PDA 的 `{fmt(final_a["attitude_rmse_deg"])}` deg；最终结构保留它处理长纯延迟和大姿态。',
        f'6. 关闭可达层的纯 PDA 大姿态位置 RMSE 为 `{fmt(pure_pda_large["position_rmse_m"])}` m、姿态 RMSE `{fmt(pure_pda_large["attitude_rmse_deg"])}` deg，并发生接地，因此大姿态调度不是可省略的调参项。',
        '',
        '## 数据',
        '',
        '- `metrics.json` 保存统一指标和每个 CSV/ULog 的绝对路径。',
        '- `manifest.json` 保存所选用例及原始结果目录。',
        '- `figures/` 保存轨迹、误差和论文递进对比图。',
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
        if line.startswith('|'):
            block = []
            while index < len(lines) and lines[index].strip().startswith('|'):
                block.append([
                    cell.strip()
                    for cell in lines[index].strip().strip('|').split('|')
                ])
                index += 1
            output.append('<table><thead><tr>')
            output.extend(
                f'<th>{html.escape(cell)}</th>' for cell in block[0]
            )
            output.append('</tr></thead><tbody>')
            for row in block[2:]:
                output.append('<tr>')
                output.extend(
                    f'<td>{html.escape(cell)}</td>' for cell in row
                )
                output.append('</tr>')
            output.append('</tbody></table>')
            continue
        if line.startswith('- '):
            items = []
            while index < len(lines) and lines[index].strip().startswith('- '):
                items.append(lines[index].strip()[2:])
                index += 1
            output.append('<ul>')
            output.extend(
                f'<li>{html.escape(item)}</li>' for item in items
            )
            output.append('</ul>')
            continue
        if line[:1].isdigit() and '. ' in line[:4]:
            items = []
            while index < len(lines):
                raw = lines[index].strip()
                if not raw[:1].isdigit() or '. ' not in raw[:4]:
                    break
                items.append(raw.split('. ', 1)[1])
                index += 1
            output.append('<ol>')
            output.extend(
                f'<li>{html.escape(item)}</li>' for item in items
            )
            output.append('</ol>')
            continue
        output.append(f'<p>{html.escape(line)}</p>')
        index += 1
    return '\n'.join(output)


def html_report(markdown: str) -> str:
    image_sections = []
    for path in sorted(FIGURES.glob('*.png')):
        payload = base64.b64encode(path.read_bytes()).decode('ascii')
        image_sections.append(
            f'<section><h2>{html.escape(path.stem)}</h2>'
            f'<img src="data:image/png;base64,{payload}"></section>'
        )
    return f"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><style>
@page {{ size: A4 landscape; margin: 12mm; }}
body {{ font-family: "Noto Sans CJK SC", sans-serif; color: #17202a; }}
article {{ font-size: 8.5pt; line-height: 1.45; }}
section {{
  page-break-before: always;
  break-inside: avoid;
  height: 172mm;
  display: flex;
  flex-direction: column;
}}
section h2 {{ margin: 0 0 3mm; }}
img {{ width: 100%; min-height: 0; flex: 1; object-fit: contain; }}
h1, h2, h3 {{ color: #17384a; }}
table {{ width: 100%; border-collapse: collapse; font-size: 7.3pt; margin-bottom: 4mm; }}
th, td {{ border: 1px solid #aebdc4; padding: 1.1mm; }}
th {{ background: #dceaf0; }}
</style></head><body>
<article>{markdown_to_html(markdown)}</article>
{''.join(image_sections)}
</body></html>"""


def main() -> None:
    FIGURES.mkdir(parents=True, exist_ok=True)
    REPORTS.mkdir(parents=True, exist_ok=True)
    cases = {
        (scenario, method): load_case(scenario, method)
        for scenario in SCENARIOS
        for method in METHODS
    }
    all_metrics = {
        scenario: {
            method: metrics(
                scenario, method, cases[(scenario, method)]
            )
            for method in METHODS
        }
        for scenario in SCENARIOS
    }
    pure_case_directory = (
        PURE_PDA / 'runs' / 'large_attitude_45_60' / 'full'
    )
    pure_result = json.loads(
        (pure_case_directory / 'result.json').read_text(encoding='utf-8')
    )
    pure_csv = Path(pure_result['csv'])
    with pure_csv.open(newline='') as stream:
        pure_rows = [
            row for row in csv.DictReader(stream)
            if row.get('auto_traj_mode') == 'attitude'
        ]
    pure_case = {
        'directory': pure_case_directory,
        'result': pure_result,
        'rows': pure_rows,
        'csv': pure_csv,
    }
    pure_pda_large = metrics(
        'large_attitude_45_60', 'full', pure_case
    )

    manifest = {
        'paper': str(
            WORKSPACE
            / 'Cuniato 等 - 2026 - Allocation for omnidirectional aerial robots incorporating power dynamics.pdf'
        ),
        'firmware': '/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-delay',
        'cases': [
            cases[(scenario, method)]['result']
            for scenario in SCENARIOS
            for method in METHODS
        ],
        'pure_pda_large_attitude_diagnostic': pure_result,
    }
    (OUTPUT / 'manifest.json').write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    (REPORTS / 'metrics.json').write_text(
        json.dumps(
            {
                'scenarios': all_metrics,
                'pure_pda_large_attitude_diagnostic': pure_pda_large,
            },
            indent=2,
            ensure_ascii=False,
        ) + '\n',
        encoding='utf-8',
    )

    plot_tracking(cases)
    plot_summary(all_metrics)
    plot_trajectory(cases, 'lissajous', 'lissajous_3d.png')
    plot_trajectory(cases, 'aggressive', 'aggressive_3d.png')
    plot_large_attitude(cases)

    markdown = report_markdown(all_metrics, pure_pda_large)
    (REPORTS / 'cuniato_revalidation_zh.md').write_text(
        markdown, encoding='utf-8'
    )
    (REPORTS / 'cuniato_revalidation_zh.html').write_text(
        html_report(markdown), encoding='utf-8'
    )
    print(REPORTS / 'metrics.json')
    print(REPORTS / 'cuniato_revalidation_zh.md')
    print(REPORTS / 'cuniato_revalidation_zh.html')


if __name__ == '__main__':
    main()
