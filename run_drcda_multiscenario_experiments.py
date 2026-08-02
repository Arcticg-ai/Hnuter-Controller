#!/usr/bin/env python3
"""Run repeatable DRCDA comparisons on the identified-delay SITL plant."""

from __future__ import annotations

import argparse
import json
import os
import pty
import re
import shutil
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parent
DEFAULT_FIRMWARE = Path(
    '/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-delay'
)
DEFAULT_OUTPUT = (
    WORKSPACE / 'hnuter_logs' / 'drcda_paper_revalidation'
)

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


@dataclass(frozen=True)
class Scenario:
    key: str
    trigger: str
    tuning_file: str
    environment: dict[str, str]
    tuning_overrides: dict[str, object]
    start_timeout_s: float
    finish_timeout_s: float
    hold_duration_s: float = 0.0


SCENARIOS = {
    'hover': Scenario(
        key='hover',
        trigger='',
        tuning_file='config/identified_delay_damped_xy.json',
        environment={},
        tuning_overrides={},
        start_timeout_s=40.0,
        finish_timeout_s=0.0,
        hold_duration_s=60.0,
    ),
    'rectangle': Scenario(
        key='rectangle',
        trigger='1',
        tuning_file='config/identified_delay_damped_xy.json',
        environment={},
        tuning_overrides={},
        start_timeout_s=40.0,
        finish_timeout_s=40.0,
    ),
    'lissajous': Scenario(
        key='lissajous',
        trigger='2',
        tuning_file='config/identified_delay_damped_xy.json',
        environment={},
        tuning_overrides={},
        start_timeout_s=40.0,
        finish_timeout_s=40.0,
    ),
    'aggressive': Scenario(
        key='aggressive',
        trigger='2',
        tuning_file='config/identified_delay_damped_xy.json',
        environment={
            'HNUTER_LISSAJOUS_AMP_X_M': '1.2',
            'HNUTER_LISSAJOUS_AMP_Y_M': '0.8',
            'HNUTER_LISSAJOUS_AMP_Z_M': '0.30',
            'HNUTER_LISSAJOUS_PERIOD_S': '8.0',
        },
        tuning_overrides={},
        start_timeout_s=40.0,
        finish_timeout_s=25.0,
    ),
    'large_attitude': Scenario(
        key='large_attitude',
        trigger='3',
        tuning_file='config/identified_delay_large_attitude.json',
        environment={},
        tuning_overrides={},
        start_timeout_s=55.0,
        finish_timeout_s=65.0,
    ),
    'large_attitude_60_90': Scenario(
        key='large_attitude_60_90',
        trigger='3',
        tuning_file='config/identified_delay_large_attitude.json',
        environment={},
        tuning_overrides={
            'attitude_step_roll_deg': 60.0,
            'attitude_step_pitch_deg': 90.0,
        },
        start_timeout_s=55.0,
        finish_timeout_s=65.0,
    ),
    'large_attitude_45_60': Scenario(
        key='large_attitude_45_60',
        trigger='3',
        tuning_file='config/identified_delay_large_attitude.json',
        environment={},
        tuning_overrides={
            'attitude_step_roll_deg': 45.0,
            'attitude_step_pitch_deg': 60.0,
        },
        start_timeout_s=55.0,
        finish_timeout_s=65.0,
    ),
}
DEFAULT_SCENARIOS = (
    'lissajous',
    'aggressive',
    'large_attitude_45_60',
    'large_attitude',
)


class PtyProcess:
    def __init__(
        self,
        command: list[str],
        cwd: Path,
        environment: dict[str, str],
        log_path: Path,
    ) -> None:
        self.command = command
        self.log_path = log_path
        self.master_fd, slave_fd = pty.openpty()
        self.process = subprocess.Popen(
            command,
            cwd=cwd,
            env=environment,
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            start_new_session=True,
            close_fds=True,
        )
        os.close(slave_fd)
        self._text = ''
        self._condition = threading.Condition()
        self._stream = log_path.open('wb')
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self) -> None:
        while True:
            try:
                chunk = os.read(self.master_fd, 8192)
            except OSError:
                break
            if not chunk:
                break
            self._stream.write(chunk)
            self._stream.flush()
            decoded = chunk.decode('utf-8', errors='replace')
            with self._condition:
                self._text = (self._text + decoded)[-500_000:]
                self._condition.notify_all()

    def send(self, text: str) -> None:
        os.write(self.master_fd, text.encode('utf-8'))

    def wait_for(self, pattern: str, timeout_s: float) -> bool:
        expression = re.compile(pattern)
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while time.monotonic() < deadline:
                if expression.search(self._text):
                    return True
                if self.process.poll() is not None:
                    return bool(expression.search(self._text))
                self._condition.wait(timeout=min(0.5, deadline - time.monotonic()))
        return bool(expression.search(self._text))

    @property
    def text(self) -> str:
        with self._condition:
            return self._text

    def stop(self) -> None:
        _stop_process(self.process, signal.SIGINT, 8.0)
        try:
            os.close(self.master_fd)
        except OSError:
            pass
        self._reader.join(timeout=2.0)
        self._stream.close()


def _stop_process(
    process: subprocess.Popen,
    first_signal: signal.Signals,
    timeout_s: float,
) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, first_signal)
        process.wait(timeout=timeout_s)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3.0)


def _latest_csv(case_directory: Path) -> Path | None:
    paths = list((case_directory / 'logs' / 'external_control').rglob('*.csv'))
    return max(paths, key=lambda path: path.stat().st_mtime_ns) if paths else None


def _ulog_paths(firmware: Path) -> set[Path]:
    root = firmware / 'build' / 'px4_sitl_default' / 'rootfs' / 'log'
    return set(root.rglob('*.ulg')) if root.exists() else set()


def _compact_px4_log(path: Path, threshold_bytes: int = 8_000_000) -> None:
    if not path.exists() or path.stat().st_size <= threshold_bytes:
        return
    keep_bytes = 750_000
    with path.open('rb') as stream:
        start = stream.read(keep_bytes)
        stream.seek(max(path.stat().st_size - keep_bytes, 0))
        end = stream.read(keep_bytes)
    repeated_prompt = b'\x1b[2K\rpxh> '
    compact = (
        start.replace(repeated_prompt, b'')
        + b'\n\n[repeated PX4 shell prompts removed]\n\n'
        + end.replace(repeated_prompt, b'')
    )
    path.write_bytes(compact)


def _controller_command(variant: str) -> list[str]:
    script = (
        'hnuter_external_direct_controller_debug.py'
        if variant == 'original_direct'
        else 'hnuter_external_direct_drcda.py'
    )
    return ['python3', script]


def _controller_environment(
    case_directory: Path,
    scenario: Scenario,
    variant: str,
    tuning_path: Path,
) -> dict[str, str]:
    environment = os.environ.copy()
    environment.update({
        'HNUTER_LOG_DIR': str(case_directory / 'logs'),
        'HNUTER_TUNING_FILE': str(tuning_path),
        'HNUTER_DRCDA_SERVO_MODEL': 'identified',
        'HNUTER_DRCDA_VARIANT': 'full' if variant == 'original_direct' else variant,
        'HNUTER_ALLOW_REMOTE_DDS': '0',
        'PYTHONUNBUFFERED': '1',
    })
    environment.update(scenario.environment)
    return environment


def _effective_tuning(case_directory: Path, scenario: Scenario) -> Path:
    base_path = WORKSPACE / scenario.tuning_file
    if not scenario.tuning_overrides:
        return base_path
    tuning = json.loads(base_path.read_text(encoding='utf-8'))
    tuning.update(scenario.tuning_overrides)
    effective_path = case_directory / 'effective_tuning.json'
    effective_path.write_text(
        json.dumps(tuning, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    return effective_path


def run_case(
    firmware: Path,
    output_root: Path,
    variant: str,
    scenario: Scenario,
) -> dict[str, object]:
    case_directory = output_root / 'runs' / scenario.key / variant
    case_directory.mkdir(parents=True, exist_ok=True)
    result_path = case_directory / 'result.json'
    if result_path.exists():
        existing = json.loads(result_path.read_text(encoding='utf-8'))
        if existing.get('status') == 'complete':
            print(f'[skip] {scenario.key}/{variant}')
            return existing

    for stale in ('controller.log', 'px4.log'):
        path = case_directory / stale
        if path.exists():
            path.unlink()

    tuning_path = _effective_tuning(case_directory, scenario)
    before_ulogs = _ulog_paths(firmware)
    agent_log = (case_directory / 'microxrce_agent.log').open('wb')
    agent = subprocess.Popen(
        ['MicroXRCEAgent', 'udp4', '-p', '8888'],
        cwd=WORKSPACE,
        stdin=subprocess.DEVNULL,
        stdout=agent_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    time.sleep(1.0)
    if agent.poll() is not None:
        agent_log.close()
        raise RuntimeError(
            f'MicroXRCEAgent exited early with code {agent.returncode}'
        )

    px4_log = (case_directory / 'px4.log').open('wb')
    px4_environment = os.environ.copy()
    px4_environment['HEADLESS'] = '1'
    px4_environment['NO_PXH'] = '1'
    px4 = subprocess.Popen(
        ['make', 'px4_sitl', 'gz_hnuter'],
        cwd=firmware,
        env=px4_environment,
        stdin=subprocess.PIPE,
        stdout=px4_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    controller: PtyProcess | None = None
    started_at = time.time()
    status = 'failed'
    error = ''
    try:
        time.sleep(12.0)
        if px4.poll() is not None:
            raise RuntimeError(f'PX4 exited early with code {px4.returncode}')

        controller = PtyProcess(
            _controller_command(variant),
            WORKSPACE,
            _controller_environment(
                case_directory, scenario, variant, tuning_path
            ),
            case_directory / 'controller.log',
        )
        if not controller.wait_for(
            r'actuator debug controller initialized', 20.0
        ):
            raise RuntimeError('controller did not initialize')
        if not controller.wait_for(r'地面自检中|Offboard=True', 15.0):
            raise RuntimeError('controller did not receive PX4 telemetry')

        controller.send('o')
        if scenario.trigger:
            time.sleep(1.0)
            controller.send(scenario.trigger)
            if not controller.wait_for(
                r'开始执行.+轨迹', scenario.start_timeout_s
            ):
                raise RuntimeError('scenario did not start before timeout')
            if not controller.wait_for(
                r'轨迹完成', scenario.finish_timeout_s
            ):
                raise RuntimeError('scenario did not finish before timeout')
        else:
            if not controller.wait_for(
                r'Mode: Offboard=True \| Armed=True',
                scenario.start_timeout_s,
            ):
                raise RuntimeError('hover did not arm before timeout')
            time.sleep(scenario.hold_duration_s)
        time.sleep(5.0)
        status = 'complete'
    except Exception as exception:
        error = str(exception)
    finally:
        if controller is not None:
            controller.stop()
        _stop_process(px4, signal.SIGINT, 12.0)
        px4_log.close()
        _compact_px4_log(case_directory / 'px4.log')
        _stop_process(agent, signal.SIGINT, 5.0)
        agent_log.close()
        time.sleep(2.0)

    csv_path = _latest_csv(case_directory)
    new_ulogs = sorted(
        _ulog_paths(firmware) - before_ulogs,
        key=lambda path: path.stat().st_mtime_ns,
    )
    archived_ulog = None
    if new_ulogs:
        archived_ulog = case_directory / 'px4.ulg'
        shutil.copy2(new_ulogs[-1], archived_ulog)

    result = {
        'variant': variant,
        'scenario': scenario.key,
        'status': status,
        'error': error,
        'started_at_unix_s': started_at,
        'duration_wall_s': time.time() - started_at,
        'firmware': str(firmware),
        'controller': _controller_command(variant)[-1],
        'tuning_file': str(tuning_path),
        'tuning_base_file': str(WORKSPACE / scenario.tuning_file),
        'tuning_overrides': scenario.tuning_overrides,
        'environment': scenario.environment,
        'csv': str(csv_path) if csv_path else None,
        'ulog': str(archived_ulog) if archived_ulog else None,
    }
    result_path.write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    print(
        f'[{status}] {scenario.key}/{variant} '
        f'({result["duration_wall_s"]:.1f}s)'
        + (f': {error}' if error else '')
    )
    return result


def parse_names(raw: str, choices: tuple[str, ...]) -> list[str]:
    names = [item.strip() for item in raw.split(',') if item.strip()]
    unknown = sorted(set(names) - set(choices))
    if unknown:
        raise argparse.ArgumentTypeError(f'unknown names: {", ".join(unknown)}')
    return names


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--firmware', type=Path, default=DEFAULT_FIRMWARE)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--variants', default=','.join(VARIANTS))
    parser.add_argument('--scenarios', default=','.join(DEFAULT_SCENARIOS))
    args = parser.parse_args()

    firmware = args.firmware.expanduser().resolve()
    output = args.output.expanduser().resolve()
    variants = parse_names(args.variants, VARIANTS)
    scenarios = parse_names(args.scenarios, tuple(SCENARIOS))
    output.mkdir(parents=True, exist_ok=True)

    manifest = {
        'firmware': str(firmware),
        'variants': variants,
        'scenarios': scenarios,
        'cases': [],
    }
    for scenario_name in scenarios:
        for variant in variants:
            manifest['cases'].append(
                run_case(firmware, output, variant, SCENARIOS[scenario_name])
            )
            (output / 'manifest.json').write_text(
                json.dumps(manifest, indent=2, ensure_ascii=False) + '\n',
                encoding='utf-8',
            )

    failures = [
        case for case in manifest['cases'] if case['status'] != 'complete'
    ]
    if failures:
        raise SystemExit(f'{len(failures)} experiment cases failed')


if __name__ == '__main__':
    main()
