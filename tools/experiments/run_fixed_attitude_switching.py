#!/usr/bin/env python3
"""Sweep physical-gate A-B-A-C switching time in no-delay Hnuter SITL."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np

SCRIPT_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

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


METHODS = ('direct', 'basic_da', 'full', 'no_horizon')
DEFAULT_DURATIONS = (2.0, 1.5, 1.2, 1.0, 0.8, 0.6)
GATE_SDF = (
    CONTROL_ROOT
    / 'controllers/experiments/fixed_attitude_switching/gate.sdf'
)


def duration_key(duration: float) -> str:
    return f'T_{duration:.1f}'.replace('.', 'p')


def spawn_physical_gate() -> dict[str, object]:
    command = [
        'gz', 'service', '-s', '/world/default/create',
        '--reqtype', 'gz.msgs.EntityFactory',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000',
        '--req', f'sdf_filename: "{GATE_SDF}", name: "fixed_attitude_gate"',
    ]
    completed = subprocess.run(command, capture_output=True, text=True, timeout=8.0)
    return {
        'spawned': completed.returncode == 0 and 'data: true' in completed.stdout,
        'returncode': completed.returncode,
        'stdout': completed.stdout.strip(),
        'stderr': completed.stderr.strip(),
        'asset': str(GATE_SDF),
        'collision_enabled': True,
        'opening_width_m': 2.30,
        'opening_lower_z_m': 0.0,
        'opening_upper_z_m': 2.36,
    }


def effective_tuning(case_root: Path) -> Path:
    case_root.mkdir(parents=True, exist_ok=True)
    destination = case_root / 'effective_tuning.json'
    shutil.copy2(DEFAULT_TUNING, destination)
    return destination


def value(row: dict[str, str], key: str, default: float = math.nan) -> float:
    try:
        result = float(row.get(key, ''))
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def rms(values: list[float]) -> float:
    finite = np.asarray([entry for entry in values if math.isfinite(entry)])
    return float(np.sqrt(np.mean(finite**2))) if finite.size else math.nan


def maximum(values: list[float]) -> float:
    finite = [abs(entry) for entry in values if math.isfinite(entry)]
    return max(finite, default=math.nan)


def wait_for_marker_or_disarm(
    processes: list[PtyProcess],
    controller: PtyProcess,
    marker: str,
    timeout_s: float,
) -> tuple[bool, bool]:
    """Wait for a trajectory marker while failing fast after an in-flight disarm."""
    disarm_marker = 'PX4 已从 armed 变为 disarmed'
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if marker in controller.buffer:
            return True, False
        if disarm_marker in controller.buffer:
            return False, True
        if controller.process.poll() is not None:
            return marker in controller.buffer, disarm_marker in controller.buffer
        drain_for(processes, min(0.25, deadline - time.monotonic()))
    return marker in controller.buffer, disarm_marker in controller.buffer


def analyze_csv(path: Path, thresholds: dict[str, float]) -> dict[str, object]:
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    flight = [
        row for row in rows
        if row.get('auto_traj_mode') == 'fixed_switch'
        or row.get('switch_phase') == 'post_hold_c'
    ]
    transitions = [
        row for row in flight
        if row.get('switch_phase') in {'a_to_b', 'b_to_a', 'a_to_c'}
    ]
    position_error = [
        math.hypot(
            value(row, 'position_x_enu_m') - value(row, 'target_x_enu_m'),
            value(row, 'position_y_enu_m') - value(row, 'target_y_enu_m'),
        )
        for row in transitions
    ]
    attitude_error = [value(row, 'attitude_error_angle_deg') for row in transitions]
    roll_pitch = [
        max(abs(value(row, 'roll_deg')), abs(value(row, 'pitch_deg')))
        for row in transitions
    ]
    servo_errors = []
    feedback_rows = 0
    for row in transitions:
        if value(row, 'switch_joint_feedback_available', 0.0) < 0.5:
            continue
        feedback_rows += 1
        for name in ('a_l', 'b_l', 'a_r', 'b_r'):
            servo_errors.append(
                value(row, f'switch_measured_joint_{name}_rad')
                - value(row, f'switch_commanded_joint_{name}_rad')
            )
    residual_values = []
    for row in transitions:
        residual = [
            value(row, f'drcda_wrench_residual_{axis}')
            for axis in ('fx_n', 'fy_n', 'fz_n', 'tx_nm', 'ty_nm', 'tz_nm')
        ]
        if all(math.isfinite(entry) for entry in residual):
            residual_values.append(float(np.linalg.norm(residual)))

    prediction_metrics = {}
    row_times = np.asarray([value(row, 'time_s') for row in rows])
    axes = ('fx_n', 'fy_n', 'fz_n', 'tx_nm', 'ty_nm', 'tz_nm')
    for label in ('h1', 'h2'):
        errors = []
        objective_errors = []
        for row in transitions:
            horizon = value(row, f'drcda_prediction_{label}_s')
            source_time = value(row, 'time_s')
            predicted = np.asarray([
                value(row, f'drcda_predicted_{label}_wrench_{axis}')
                for axis in axes
            ])
            reference = np.asarray([
                value(row, f'drcda_reference_{label}_wrench_{axis}')
                for axis in axes
            ])
            if not (
                math.isfinite(horizon)
                and np.all(np.isfinite(predicted))
                and np.all(np.isfinite(reference))
            ):
                continue
            target_time = source_time + horizon
            index = int(np.argmin(np.abs(row_times - target_time)))
            if abs(row_times[index] - target_time) > 0.035:
                continue
            actual = np.asarray([
                value(rows[index], f'drcda_estimated_wrench_{axis}')
                for axis in axes
            ])
            if np.all(np.isfinite(actual)):
                errors.append(predicted - actual)
                objective_errors.append(predicted - reference)
        error_array = np.asarray(errors, dtype=float).reshape((-1, 6))
        objective_array = np.asarray(objective_errors, dtype=float).reshape((-1, 6))
        prediction_metrics.update({
            f'prediction_{label}_sample_count': len(error_array),
            f'prediction_{label}_fy_rmse_n': (
                float(np.sqrt(np.mean(error_array[:, 1] ** 2)))
                if len(error_array) else math.nan
            ),
            f'prediction_{label}_wrench_rmse': (
                float(np.sqrt(np.mean(np.sum(error_array ** 2, axis=1))))
                if len(error_array) else math.nan
            ),
            f'prediction_{label}_objective_rmse': (
                float(np.sqrt(np.mean(np.sum(objective_array ** 2, axis=1))))
                if len(objective_array) else math.nan
            ),
        })

    reversal_reductions = []
    for phase in ('a_to_b', 'b_to_a', 'a_to_c'):
        phase_rows = [row for row in transitions if row.get('switch_phase') == phase]
        if len(phase_rows) < 4:
            continue
        phase_end = max(value(row, 'switch_elapsed_s') for row in phase_rows)
        terminal = [
            row for row in phase_rows
            if phase_end - 0.12 <= value(row, 'switch_elapsed_s') <= phase_end
        ]
        before = [
            row for row in phase_rows
            if phase_end - 0.24 <= value(row, 'switch_elapsed_s') < phase_end - 0.12
        ]
        def lateral_servo_magnitude(group):
            values = [
                abs(value(row, f'switch_commanded_joint_{name}_rad'))
                for row in group for name in ('b_l', 'b_r')
            ]
            return float(np.mean(values)) if values else math.nan
        before_magnitude = lateral_servo_magnitude(before)
        terminal_magnitude = lateral_servo_magnitude(terminal)
        if math.isfinite(before_magnitude) and math.isfinite(terminal_magnitude):
            reversal_reductions.append(before_magnitude - terminal_magnitude)

    gate_rows = [
        row for row in flight
        if abs(value(row, 'position_y_enu_m')) <= 0.12
    ]
    gate_clearances = [
        min(
            1.15 - abs(value(row, 'position_x_enu_m')) - 0.82,
            value(row, 'position_z_rel_m') - 0.40,
            2.36 - value(row, 'position_z_rel_m') - 0.40,
        )
        for row in gate_rows
    ]
    target_radius_m = 0.20
    reached_b = any(
        abs(value(row, 'position_y_enu_m') - 1.2) <= target_radius_m
        for row in flight
    )
    reached_c = any(
        abs(value(row, 'position_y_enu_m') + 1.2) <= target_radius_m
        for row in flight
    )
    returned_to_gate_a = any(
        row.get('switch_phase') in {'b_to_a', 'dwell_a'}
        and abs(value(row, 'position_y_enu_m')) <= target_radius_m
        for row in flight
    )

    metrics = {
        'row_count': len(rows),
        'transition_row_count': len(transitions),
        'position_error_rms_m': rms(position_error),
        'position_error_peak_m': maximum(position_error),
        'attitude_error_rms_deg': rms(attitude_error),
        'attitude_error_peak_deg': maximum(attitude_error),
        'roll_pitch_peak_deg': maximum(roll_pitch),
        'yaw_rate_peak_rad_s': maximum([
            value(row, 'angular_r_frd_rps') for row in transitions
        ]),
        'minimum_altitude_m': min(
            [value(row, 'position_z_rel_m') for row in flight],
            default=math.nan,
        ),
        'joint_feedback_ratio': (
            feedback_rows / len(transitions) if transitions else 0.0
        ),
        'joint_tracking_error_rms_rad': rms(servo_errors),
        'joint_tracking_error_peak_rad': maximum(servo_errors),
        'reachable_wrench_residual_rms': rms(residual_values),
        'pre_reversal_servo_reduction_rad': (
            float(np.mean(reversal_reductions))
            if reversal_reductions else math.nan
        ),
        'gate_sample_count': len(gate_rows),
        'minimum_gate_clearance_m': min(gate_clearances, default=math.nan),
        'reached_b': reached_b,
        'returned_to_gate_a': returned_to_gate_a,
        'reached_c': reached_c,
        'target_acceptance_radius_m': target_radius_m,
        'final_armed': bool(rows and rows[-1].get('armed') == '1'),
        'safety_cutoff': any(
            row.get('direct_safety_cutoff') == '1' for row in rows
        ),
        **prediction_metrics,
    }
    failures = []
    if len(transitions) < 4:
        failures.append('missing transition diagnostics')
    if not reached_b:
        failures.append('did not reach target B')
    if not returned_to_gate_a:
        failures.append('did not return through gate A')
    if not reached_c:
        failures.append('did not reach target C')
    clearance = float(metrics['minimum_gate_clearance_m'])
    if not math.isfinite(clearance) or clearance < thresholds['gate_clearance_m']:
        failures.append(
            f'gate clearance {clearance:.3f} < '
            f'{thresholds["gate_clearance_m"]:.3f}'
        )
    if not metrics['final_armed']:
        failures.append('disarmed before post-hold ended')
    if metrics['safety_cutoff']:
        failures.append('direct safety cutoff')
    checks = (
        ('position_error_peak_m', thresholds['position_peak_m'], 'position peak'),
        ('attitude_error_peak_deg', thresholds['attitude_peak_deg'], 'attitude peak'),
        ('roll_pitch_peak_deg', thresholds['roll_pitch_peak_deg'], 'roll/pitch peak'),
    )
    for key, limit, label in checks:
        measured = float(metrics[key])
        if not math.isfinite(measured) or measured > limit:
            failures.append(f'{label} {measured:.3f} > {limit:.3f}')
    altitude = float(metrics['minimum_altitude_m'])
    if not math.isfinite(altitude) or altitude < thresholds['minimum_altitude_m']:
        failures.append(
            f'minimum altitude {altitude:.3f} < '
            f'{thresholds["minimum_altitude_m"]:.3f}'
        )
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
    duration: float,
    thresholds: dict[str, float],
    gui: bool = False,
    gui_warmup_s: float = 0.0,
    post_hold_s: float = 5.0,
) -> dict[str, object]:
    case_root = output / 'runs' / duration_key(duration) / method
    console = case_root / 'console'
    log_root = case_root / 'logs'
    tuning = effective_tuning(case_root)
    started_ns = time.time_ns()
    started_wall = time.monotonic()
    agent = sitl = controller = None
    status = 'failed'
    error = ''
    gate = {'spawned': False, 'error': 'not attempted'}
    try:
        agent = PtyProcess(
            ['MicroXRCEAgent', 'udp4', '-p', '8888'],
            CONTROL_ROOT, os.environ.copy(), console / 'agent.log',
        )
        drain_for([agent], 1.0)
        if agent.process.poll() is not None:
            raise RuntimeError('Micro XRCE-DDS Agent exited during startup')
        sitl_env = os.environ.copy()
        sitl_env['PX4_GZ_WORLD'] = 'default'
        if gui:
            sitl_env.pop('HEADLESS', None)
        else:
            sitl_env['HEADLESS'] = '1'
        sitl = PtyProcess(
            ['make', 'px4_sitl', 'gz_hnuter'],
            firmware, sitl_env, console / 'px4.log',
        )
        if not wait_for([agent, sitl], sitl, 'Ready for takeoff!', 60.0):
            raise RuntimeError('PX4 did not report Ready for takeoff')
        try:
            gate = spawn_physical_gate()
        except Exception as exc:
            gate = {'spawned': False, 'error': str(exc), 'asset': str(GATE_SDF)}
        if gui and gui_warmup_s > 0.0:
            drain_for([agent, sitl], gui_warmup_s)

        env = os.environ.copy()
        env.update({
            'HNUTER_LOG_DIR': str(log_root),
            'HNUTER_TUNING_FILE': str(tuning),
            'HNUTER_PREFLIGHT_TILT_TEST': '0',
            'HNUTER_SWITCH_METHOD': method,
            'HNUTER_SWITCH_DURATION_S': str(duration),
            'HNUTER_SWITCH_HALF_SPAN_M': '1.2',
            'HNUTER_SWITCH_ALTITUDE_M': '1.5',
        })
        controller = PtyProcess(
            [
                sys.executable, '-m',
                'controllers.experiments.fixed_attitude_switching.controller',
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
            processes, controller, '开始执行定姿快速侧移', 35.0,
        )
        if not reached:
            reason = 'pre-trajectory disarm' if disarmed else 'start timeout'
            raise RuntimeError(f'fixed-attitude switching did not start: {reason}')
        reached, disarmed = wait_for_marker_or_disarm(
            processes, controller, '定姿快速侧移完成', 30.0,
        )
        if not reached:
            reason = 'in-trajectory disarm' if disarmed else 'finish timeout'
            raise RuntimeError(f'fixed-attitude switching did not finish: {reason}')
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
    metrics = {}
    if csv_path is not None:
        metrics = analyze_csv(csv_path, thresholds)
        if status == 'complete' and not metrics['passed']:
            status = 'failed'
            error = '; '.join(metrics['failures'])
    elif status == 'complete':
        status, error = 'failed', 'controller diagnostic CSV is missing'
    ulog = latest_ulog(firmware, started_ns)
    result = {
        'method': method,
        'switch_duration_s': duration,
        'status': status,
        'error': error,
        'metrics': metrics,
        'gate': gate,
        'duration_wall_s': time.monotonic() - started_wall,
        'csv': str(csv_path) if csv_path else None,
        'ulog_source': str(ulog) if ulog else None,
        'tuning': str(tuning),
        'servo_model': (
            None if method == 'direct' else 'gazebo_joint_pid_no_delay'
        ),
        'pure_delay_model_used': False,
    }
    (case_root / 'result.json').write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    print(
        f'{duration_key(duration)}/{method}: {status.upper()} {error}',
        flush=True,
    )
    return result


def write_summary(output: Path, manifest: dict[str, object]) -> None:
    rows = manifest['cases']
    with (output / 'summary.csv').open('w', newline='', encoding='utf-8') as stream:
        fieldnames = [
            'method', 'switch_duration_s', 'status',
            'position_error_rms_m', 'position_error_peak_m',
            'attitude_error_rms_deg', 'attitude_error_peak_deg',
            'roll_pitch_peak_deg', 'yaw_rate_peak_rad_s',
            'minimum_altitude_m', 'joint_feedback_ratio',
            'joint_tracking_error_rms_rad',
            'reachable_wrench_residual_rms',
            'pre_reversal_servo_reduction_rad',
            'prediction_h1_fy_rmse_n', 'prediction_h2_fy_rmse_n',
            'prediction_h1_wrench_rmse', 'prediction_h2_wrench_rmse',
            'minimum_gate_clearance_m',
            'reached_b', 'returned_to_gate_a', 'reached_c', 'error',
        ]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for case in rows:
            metrics = case.get('metrics', {})
            writer.writerow({
                key: (
                    case.get(key, '')
                    if key in {'method', 'switch_duration_s', 'status', 'error'}
                    else metrics.get(key, '')
                )
                for key in fieldnames
            })
    critical = {}
    for method in METHODS:
        passed = [
            float(case['switch_duration_s'])
            for case in rows
            if case['method'] == method and case['status'] == 'complete'
        ]
        critical[method] = min(passed) if passed else None
    manifest['minimum_valid_switch_time_s'] = critical
    (output / 'manifest.json').write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--firmware', type=Path, default=DEFAULT_FIRMWARE)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--method', action='append', choices=METHODS)
    parser.add_argument('--duration', action='append', type=float)
    parser.add_argument('--position-peak-limit', type=float, default=0.75)
    parser.add_argument('--attitude-peak-limit-deg', type=float, default=8.0)
    parser.add_argument('--roll-pitch-peak-limit-deg', type=float, default=5.0)
    parser.add_argument('--minimum-altitude', type=float, default=0.5)
    parser.add_argument('--minimum-gate-clearance', type=float, default=0.05)
    parser.add_argument(
        '--gui', action='store_true',
        help='show the Gazebo client while running the selected cases',
    )
    parser.add_argument(
        '--gui-warmup', type=float, default=0.0,
        help='seconds to wait for the Gazebo GUI before starting the controller',
    )
    parser.add_argument(
        '--post-hold', type=float, default=5.0,
        help='seconds to keep the simulation open after the maneuver',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    firmware = firmware_metadata(args.firmware)
    if firmware['dynamic_actuator_tokens_present']:
        raise RuntimeError('fixed-attitude validation is no-delay-only')
    durations = args.duration or list(DEFAULT_DURATIONS)
    methods = args.method or list(METHODS)
    thresholds = {
        'position_peak_m': args.position_peak_limit,
        'attitude_peak_deg': args.attitude_peak_limit_deg,
        'roll_pitch_peak_deg': args.roll_pitch_peak_limit_deg,
        'minimum_altitude_m': args.minimum_altitude,
        'gate_clearance_m': args.minimum_gate_clearance,
        'feedback_ratio': 0.95,
    }
    manifest = {
        'created_at_unix_s': time.time(),
        'firmware': firmware,
        'controller_commit': git_value(CONTROL_ROOT, 'rev-parse', 'HEAD'),
        'controller_worktree_dirty': bool(
            git_value(CONTROL_ROOT, 'status', '--short')
        ),
        'experiment': 'physical_gate_fixed_attitude_minimum_jerk_A_B_A_C',
        'methods': methods,
        'switch_durations_s': durations,
        'half_span_m': 1.2,
        'altitude_m': 1.5,
        'thresholds': thresholds,
        'pure_delay_model_used': False,
        'gazebo_gui': args.gui,
        'cases': [],
    }
    for duration in durations:
        for method in methods:
            manifest['cases'].append(
                run_case(
                    args.output, args.firmware, method, duration, thresholds,
                    gui=args.gui,
                    gui_warmup_s=max(0.0, args.gui_warmup),
                    post_hold_s=max(0.0, args.post_hold),
                )
            )
            write_summary(args.output, manifest)
    return 0 if all(
        case['status'] == 'complete' for case in manifest['cases']
    ) else 1


if __name__ == '__main__':
    sys.exit(main())
