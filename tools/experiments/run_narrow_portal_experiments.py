#!/usr/bin/env python3
"""Run fixed-heading direct-actuator methods through the six-gate world."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time

import numpy as np

SCRIPT_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

from controllers.experiments.narrow_portal_course.controller import GATES
from tools.experiments.run_fixed_attitude_switching import (
    METHODS,
    wait_for_marker_or_disarm,
)
from tools.experiments.run_no_delay_maneuver_experiments import (
    CONTROL_ROOT,
    DEFAULT_FIRMWARE,
    DEFAULT_TUNING,
    PtyProcess,
    drain_for,
    firmware_metadata,
    git_value,
    latest_ulog,
    wait_for,
)


def value(row: dict[str, str], key: str, default: float = math.nan) -> float:
    try:
        parsed = float(row.get(key, ''))
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def rms(values: list[float]) -> float:
    finite = np.asarray([entry for entry in values if math.isfinite(entry)])
    return float(np.sqrt(np.mean(finite**2))) if finite.size else math.nan


def maximum(values: list[float]) -> float:
    finite = [abs(entry) for entry in values if math.isfinite(entry)]
    return max(finite, default=math.nan)


def analyze_csv(path: Path, thresholds: dict[str, float]) -> dict[str, object]:
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    course = [row for row in rows if row.get('auto_traj_mode') == 'narrow_portal']
    position_error = [
        float(np.linalg.norm([
            value(row, 'position_x_enu_m') - value(row, 'target_x_enu_m'),
            value(row, 'position_y_enu_m') - value(row, 'target_y_enu_m'),
            value(row, 'position_z_rel_m') - value(row, 'target_z_rel_m'),
        ]))
        for row in course
    ]
    horizontal_error = [
        math.hypot(
            value(row, 'position_x_enu_m') - value(row, 'target_x_enu_m'),
            value(row, 'position_y_enu_m') - value(row, 'target_y_enu_m'),
        )
        for row in course
    ]
    gate_passages = []
    for gate in GATES:
        nearest = min(
            course,
            key=lambda row: abs(value(row, 'position_x_enu_m') - gate.x_m),
            default=None,
        )
        if nearest is None:
            gate_passages.append({'gate': gate.name, 'passed': False})
            continue
        x_error = abs(value(nearest, 'position_x_enu_m') - gate.x_m)
        y_error = value(nearest, 'position_y_enu_m') - gate.center_y_m
        clearance = value(nearest, 'portal_clearance_margin_m')
        gate_passages.append({
            'gate': gate.name,
            'passed': x_error <= thresholds['gate_plane_tolerance_m']
                      and math.isfinite(clearance) and clearance >= 0.0,
            'x_plane_error_m': x_error,
            'lateral_center_error_m': y_error,
            'altitude_m': value(nearest, 'position_z_rel_m'),
            'clearance_margin_m': clearance,
        })
    feedback_rows = [
        row for row in course
        if value(row, 'portal_joint_feedback_available', 0.0) >= 0.5
    ]
    servo_errors = []
    for row in feedback_rows:
        for name in ('a_l', 'b_l', 'a_r', 'b_r'):
            servo_errors.append(
                value(row, f'portal_measured_joint_{name}_rad')
                - value(row, f'portal_commanded_joint_{name}_rad')
            )
    metrics = {
        'row_count': len(rows),
        'course_row_count': len(course),
        'position_error_rms_m': rms(position_error),
        'position_error_peak_m': maximum(position_error),
        'horizontal_error_rms_m': rms(horizontal_error),
        'horizontal_error_peak_m': maximum(horizontal_error),
        'attitude_error_rms_deg': rms([
            value(row, 'attitude_error_angle_deg') for row in course
        ]),
        'attitude_error_peak_deg': maximum([
            value(row, 'attitude_error_angle_deg') for row in course
        ]),
        'roll_pitch_peak_deg': maximum([
            max(abs(value(row, 'roll_deg')), abs(value(row, 'pitch_deg')))
            for row in course
        ]),
        'yaw_rate_peak_rad_s': maximum([
            value(row, 'angular_r_frd_rps') for row in course
        ]),
        'minimum_altitude_m': min(
            [value(row, 'position_z_rel_m') for row in course], default=math.nan
        ),
        'minimum_clearance_m': min(
            [entry.get('clearance_margin_m', math.nan) for entry in gate_passages
             if math.isfinite(entry.get('clearance_margin_m', math.nan))],
            default=math.nan,
        ),
        'gates_passed': sum(bool(entry['passed']) for entry in gate_passages),
        'gate_passages': gate_passages,
        'joint_feedback_ratio': (
            len(feedback_rows) / len(course) if course else 0.0
        ),
        'joint_tracking_error_rms_rad': rms(servo_errors),
        'joint_tracking_error_peak_rad': maximum(servo_errors),
        'final_armed': bool(rows and rows[-1].get('armed') == '1'),
        'safety_cutoff': any(
            row.get('direct_safety_cutoff') == '1' for row in rows
        ),
    }
    failures = []
    if len(course) < 20:
        failures.append('missing course diagnostics')
    if metrics['gates_passed'] != len(GATES):
        failures.append(f'gates passed {metrics["gates_passed"]}/{len(GATES)}')
    checks = (
        ('position_error_peak_m', thresholds['position_peak_m'], 'position peak'),
        ('attitude_error_peak_deg', thresholds['attitude_peak_deg'], 'attitude peak'),
        ('roll_pitch_peak_deg', thresholds['roll_pitch_peak_deg'], 'roll/pitch peak'),
    )
    for key, limit, label in checks:
        measured = float(metrics[key])
        if not math.isfinite(measured) or measured > limit:
            failures.append(f'{label} {measured:.3f} > {limit:.3f}')
    clearance = float(metrics['minimum_clearance_m'])
    if not math.isfinite(clearance) or clearance < thresholds['clearance_m']:
        failures.append(
            f'minimum clearance {clearance:.3f} < {thresholds["clearance_m"]:.3f}'
        )
    if not metrics['final_armed']:
        failures.append('disarmed before post-hold ended')
    if metrics['safety_cutoff']:
        failures.append('direct safety cutoff')
    if metrics['joint_feedback_ratio'] < thresholds['feedback_ratio']:
        failures.append(
            f'joint feedback ratio {metrics["joint_feedback_ratio"]:.3f} < '
            f'{thresholds["feedback_ratio"]:.3f}'
        )
    metrics['passed'] = not failures
    metrics['failures'] = failures
    return metrics


def run_case(
    output: Path,
    firmware: Path,
    method: str,
    speed_scale: float,
    thresholds: dict[str, float],
    gui: bool,
    gui_warmup_s: float,
    post_hold_s: float,
) -> dict[str, object]:
    key = f'speed_{speed_scale:.2f}'.replace('.', 'p')
    case_root = output / 'runs' / key / method
    console = case_root / 'console'
    log_root = case_root / 'logs'
    case_root.mkdir(parents=True, exist_ok=True)
    tuning = case_root / 'effective_tuning.json'
    shutil.copy2(DEFAULT_TUNING, tuning)
    started_ns = time.time_ns()
    started_wall = time.monotonic()
    agent = sitl = controller = None
    status, error = 'failed', ''
    try:
        agent = PtyProcess(
            ['MicroXRCEAgent', 'udp4', '-p', '8888'],
            CONTROL_ROOT, os.environ.copy(), console / 'agent.log',
        )
        drain_for([agent], 1.0)
        if agent.process.poll() is not None:
            raise RuntimeError('Micro XRCE-DDS Agent exited during startup')
        sitl_env = os.environ.copy()
        sitl_env['PX4_GZ_WORLD'] = 'hnuter_narrow'
        if gui:
            sitl_env.pop('HEADLESS', None)
        else:
            sitl_env['HEADLESS'] = '1'
        sitl = PtyProcess(
            ['make', 'px4_sitl', 'gz_hnuter'],
            firmware, sitl_env, console / 'px4.log',
        )
        if not wait_for([agent, sitl], sitl, 'Ready for takeoff!', 70.0):
            raise RuntimeError('PX4 did not report Ready for takeoff')
        if gui and gui_warmup_s > 0.0:
            drain_for([agent, sitl], gui_warmup_s)
        env = os.environ.copy()
        env.update({
            'HNUTER_LOG_DIR': str(log_root),
            'HNUTER_TUNING_FILE': str(tuning),
            'HNUTER_PREFLIGHT_TILT_TEST': '0',
            'HNUTER_PORTAL_METHOD': method,
            'HNUTER_PORTAL_SPEED_SCALE': str(speed_scale),
            'HNUTER_GZ_WORLD': 'hnuter_narrow',
        })
        controller = PtyProcess(
            [
                sys.executable, '-m',
                'controllers.experiments.narrow_portal_course.controller',
            ],
            CONTROL_ROOT, env, console / 'controller.log',
        )
        processes = [agent, sitl, controller]
        if not wait_for(processes, controller, '地面自检中', 30.0):
            raise RuntimeError('controller did not receive PX4 telemetry')
        controller.send('o')
        if not wait_for(processes, controller, 'ARM_DISARM -> ACCEPTED', 20.0):
            raise RuntimeError('PX4 did not accept arm command')
        controller.send('2')
        reached, disarmed = wait_for_marker_or_disarm(
            processes, controller, '开始执行固定航向六门窄通道任务', 40.0,
        )
        if not reached:
            reason = 'pre-course disarm' if disarmed else 'start timeout'
            raise RuntimeError(f'narrow course did not start: {reason}')
        expected_duration = 90.0 / max(speed_scale, 0.5)
        reached, disarmed = wait_for_marker_or_disarm(
            processes, controller, '固定航向六门窄通道任务完成',
            expected_duration,
        )
        if not reached:
            reason = 'in-course disarm' if disarmed else 'finish timeout'
            raise RuntimeError(f'narrow course did not finish: {reason}')
        drain_for(processes, max(0.0, post_hold_s))
        status = 'complete'
    except Exception as exc:
        error = str(exc)
    finally:
        if controller is not None:
            controller.stop(timeout=6.0)
        if sitl is not None:
            sitl.stop(graceful_text='shutdown\n', timeout=8.0)
        if agent is not None:
            agent.stop(timeout=4.0)
        time.sleep(1.0)

    csvs = sorted(log_root.rglob('*.csv'), key=lambda item: item.stat().st_mtime_ns)
    csv_path = csvs[-1] if csvs else None
    metrics = analyze_csv(csv_path, thresholds) if csv_path else {}
    if status == 'complete' and not metrics.get('passed', False):
        status = 'failed'
        error = '; '.join(metrics.get('failures', ['diagnostic validation failed']))
    result = {
        'method': method,
        'speed_scale': speed_scale,
        'status': status,
        'error': error,
        'metrics': metrics,
        'world': 'hnuter_narrow',
        'world_has_collision': True,
        'fixed_heading': True,
        'duration_wall_s': time.monotonic() - started_wall,
        'csv': str(csv_path) if csv_path else None,
        'ulog_source': str(latest_ulog(firmware, started_ns) or ''),
        'pure_delay_model_used': False,
    }
    (case_root / 'result.json').write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + '\n', encoding='utf-8'
    )
    print(f'{key}/{method}: {status.upper()} {error}', flush=True)
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--firmware', type=Path, default=DEFAULT_FIRMWARE)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--method', action='append', choices=METHODS)
    parser.add_argument('--speed-scale', action='append', type=float)
    parser.add_argument('--position-peak-limit', type=float, default=0.75)
    parser.add_argument('--attitude-peak-limit-deg', type=float, default=8.0)
    parser.add_argument('--roll-pitch-peak-limit-deg', type=float, default=5.0)
    parser.add_argument('--minimum-clearance', type=float, default=0.05)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--gui-warmup', type=float, default=0.0)
    parser.add_argument('--post-hold', type=float, default=5.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    firmware = firmware_metadata(args.firmware)
    if firmware['dynamic_actuator_tokens_present']:
        raise RuntimeError('narrow-portal validation is no-delay-only')
    thresholds = {
        'position_peak_m': args.position_peak_limit,
        'attitude_peak_deg': args.attitude_peak_limit_deg,
        'roll_pitch_peak_deg': args.roll_pitch_peak_limit_deg,
        'clearance_m': args.minimum_clearance,
        'gate_plane_tolerance_m': 0.35,
        'feedback_ratio': 0.95,
    }
    methods = args.method or list(METHODS)
    speed_scales = args.speed_scale or [1.0, 1.3, 1.6]
    manifest = {
        'created_at_unix_s': time.time(),
        'firmware': firmware,
        'controller_commit': git_value(CONTROL_ROOT, 'rev-parse', 'HEAD'),
        'controller_worktree_dirty': bool(git_value(CONTROL_ROOT, 'status', '--short')),
        'experiment': 'fixed_heading_six_gate_narrow_portal',
        'world': 'hnuter_narrow',
        'methods': methods,
        'speed_scales': speed_scales,
        'thresholds': thresholds,
        'pure_delay_model_used': False,
        'cases': [],
    }
    for speed_scale in speed_scales:
        for method in methods:
            manifest['cases'].append(run_case(
                args.output, args.firmware, method, speed_scale, thresholds,
                args.gui, max(0.0, args.gui_warmup), max(0.0, args.post_hold),
            ))
            (args.output / 'manifest.json').write_text(
                json.dumps(manifest, indent=2, ensure_ascii=False) + '\n',
                encoding='utf-8',
            )
    return 0 if all(case['status'] == 'complete' for case in manifest['cases']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
