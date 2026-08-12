#!/usr/bin/env python3
"""Run repeatable no-delay Hnuter maneuver and attitude experiments."""

from __future__ import annotations

import argparse
import json
import os
import pty
import selectors
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


CONTROL_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_FIRMWARE = Path("/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter")
DEFAULT_TUNING = CONTROL_ROOT / "config/no_delay_drcda_tuning.json"
METHODS = ("original_direct", "basic_da", "full", "no_horizon", "no_rate_limits")


@dataclass(frozen=True)
class Scenario:
    key: str
    trigger: str
    start_marker: str
    finish_marker: str
    timeout_s: float
    environment: dict[str, str]
    tuning_overrides: dict[str, object]


SCENARIOS: dict[str, Scenario] = {
    "aggressive": Scenario(
        key="aggressive",
        trigger="2",
        start_marker="开始执行三维李萨如轨迹",
        finish_marker="三维李萨如轨迹完成",
        timeout_s=35.0,
        environment={
            "HNUTER_LISSAJOUS_AMP_X_M": "1.2",
            "HNUTER_LISSAJOUS_AMP_Y_M": "0.8",
            "HNUTER_LISSAJOUS_AMP_Z_M": "0.30",
            "HNUTER_LISSAJOUS_PERIOD_S": "7.0",
        },
        tuning_overrides={},
    ),
    "attitude_80_180": Scenario(
        key="attitude_80_180",
        trigger="3",
        start_marker="开始执行姿态角轨迹",
        finish_marker="姿态角轨迹完成",
        timeout_s=215.0,
        environment={},
        tuning_overrides={
            "attitude_step_roll_deg": 80.0,
            "attitude_step_pitch_deg": 180.0,
            "attitude_step_yaw_deg": 0.0,
            "attitude_segment_time_s": 15.0,
            "attitude_peak_hold_s": 5.0,
            "attitude_level_settle_s": 10.0,
            "attitude_test_bidirectional": True,
            "attitude_test_altitude_m": 3.0,
            "attitude_test_altitude_only": False,
            "attitude_test_max_acc_xy": 8.0,
            "direct_large_tilt_yaw_min_scale": 0.40,
        },
    ),
}


class PtyProcess:
    def __init__(self, command: list[str], cwd: Path, env: dict[str, str], log_path: Path):
        self.master, slave = pty.openpty()
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log = log_path.open("wb")
        self.process = subprocess.Popen(
            command,
            cwd=cwd,
            env=env,
            stdin=slave,
            stdout=slave,
            stderr=slave,
            start_new_session=True,
            close_fds=True,
        )
        os.close(slave)
        os.set_blocking(self.master, False)
        self.buffer = ""

    def send(self, value: str) -> None:
        os.write(self.master, value.encode())

    def read(self) -> str:
        try:
            data = os.read(self.master, 65536)
        except (BlockingIOError, OSError):
            return ""
        if not data:
            return ""
        self.log.write(data)
        self.log.flush()
        value = data.decode("utf-8", errors="replace")
        self.buffer = (self.buffer + value)[-50000:]
        return value

    def stop(self, graceful_text: str | None = None, timeout: float = 6.0) -> None:
        if self.process.poll() is None and graceful_text:
            self.send(graceful_text)
            drain_for([self], 2.0)
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
        try:
            self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGTERM)
            try:
                self.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=3.0)
        self.close()

    def close(self) -> None:
        self.read()
        try:
            os.close(self.master)
        except OSError:
            pass
        self.log.close()


def drain_for(processes: list[PtyProcess], duration_s: float) -> None:
    deadline = time.monotonic() + duration_s
    selector = selectors.DefaultSelector()
    for process in processes:
        selector.register(process.master, selectors.EVENT_READ, process)
    try:
        while time.monotonic() < deadline:
            for key, _ in selector.select(min(0.25, deadline - time.monotonic())):
                key.data.read()
    finally:
        selector.close()


def wait_for(processes: list[PtyProcess], target: PtyProcess, marker: str, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    selector = selectors.DefaultSelector()
    for process in processes:
        selector.register(process.master, selectors.EVENT_READ, process)
    try:
        while time.monotonic() < deadline:
            if marker in target.buffer:
                return True
            if target.process.poll() is not None:
                return marker in target.buffer
            for key, _ in selector.select(0.25):
                key.data.read()
        return marker in target.buffer
    finally:
        selector.close()


def git_value(firmware: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(firmware), *args],
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout.strip()


def firmware_metadata(firmware: Path) -> dict[str, object]:
    model = firmware / "Tools/simulation/gz/models/hnuter/model.sdf"
    model_text = model.read_text(encoding="utf-8")
    dynamic_tokens = ("servo_0_dynamic", "transport_delay", "FirstOrderActuator")
    return {
        "path": str(firmware),
        "commit": git_value(firmware, "rev-parse", "HEAD"),
        "branch": git_value(firmware, "branch", "--show-current"),
        "describe": git_value(firmware, "describe", "--always", "--tags", "--dirty"),
        "model_sdf": str(model),
        "dynamic_actuator_tokens_present": any(token in model_text for token in dynamic_tokens),
    }


def latest_ulog(firmware: Path, newer_than_ns: int) -> Path | None:
    log_root = firmware / "build/px4_sitl_default/rootfs/log"
    candidates = [
        path for path in log_root.rglob("*.ulg")
        if path.stat().st_mtime_ns >= newer_than_ns
    ]
    return max(candidates, key=lambda path: path.stat().st_mtime_ns, default=None)


def write_effective_tuning(result_root: Path, scenario: Scenario, method: str) -> Path:
    data = json.loads(DEFAULT_TUNING.read_text(encoding="utf-8"))
    data.update(scenario.tuning_overrides)
    path = result_root / "runs" / scenario.key / method / "effective_tuning.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    return path


def run_case(
    result_root: Path,
    firmware: Path,
    scenario: Scenario,
    method: str,
) -> dict[str, object]:
    case_root = result_root / "runs" / scenario.key / method
    console_root = case_root / "console"
    log_root = case_root / "logs"
    tuning_file = write_effective_tuning(result_root, scenario, method)
    started_ns = time.time_ns()
    started_wall = time.monotonic()
    controller: PtyProcess | None = None
    sitl: PtyProcess | None = None
    agent: PtyProcess | None = None
    error = ""
    status = "failed"

    sitl_env = os.environ.copy()
    sitl_env.update({"HEADLESS": "1", "PX4_GZ_WORLD": "default"})
    try:
        agent = PtyProcess(
            ["MicroXRCEAgent", "udp4", "-p", "8888"],
            CONTROL_ROOT,
            os.environ.copy(),
            console_root / "micro_xrce_agent.log",
        )
        drain_for([agent], 1.0)
        if agent.process.poll() is not None:
            raise RuntimeError("Micro XRCE-DDS Agent exited during startup")
        sitl = PtyProcess(
            ["make", "px4_sitl", "gz_hnuter"],
            firmware,
            sitl_env,
            console_root / "px4.log",
        )
        if not wait_for([agent, sitl], sitl, "Ready for takeoff!", 55.0):
            raise RuntimeError("PX4 did not report Ready for takeoff")

        controller_env = os.environ.copy()
        controller_env.update(scenario.environment)
        controller_env.update({
            "HNUTER_LOG_DIR": str(log_root),
            "HNUTER_TUNING_FILE": str(tuning_file),
            "HNUTER_PREFLIGHT_TILT_TEST": "0",
        })
        script = "hnuter_external_direct_controller_debug.py"
        if method != "original_direct":
            script = "hnuter_external_direct_drcda.py"
            controller_env["HNUTER_DRCDA_VARIANT"] = method
        controller = PtyProcess(
            [sys.executable, script],
            CONTROL_ROOT,
            controller_env,
            console_root / "controller.log",
        )
        processes = [agent, sitl, controller]
        if not wait_for(processes, controller, "地面自检中", 30.0):
            raise RuntimeError("controller did not receive PX4 telemetry")
        controller.send("o")
        if not wait_for(processes, controller, "ARM_DISARM -> ACCEPTED", 20.0):
            raise RuntimeError("PX4 did not accept arm command")
        controller.send(scenario.trigger)
        if not wait_for(processes, controller, scenario.start_marker, 35.0):
            raise RuntimeError("scenario did not start")
        if not wait_for(processes, controller, scenario.finish_marker, scenario.timeout_s):
            raise RuntimeError("scenario did not finish before timeout")
        drain_for(processes, 6.0)
        status = "complete"
    except Exception as exc:
        error = str(exc)
    finally:
        if controller is not None:
            controller.stop(timeout=6.0)
        if sitl is not None:
            sitl.stop(graceful_text="shutdown\n", timeout=8.0)
        if agent is not None:
            agent.stop(timeout=4.0)
        time.sleep(1.0)

    csvs = sorted(log_root.rglob("*.csv"), key=lambda path: path.stat().st_mtime_ns)
    csv_path = csvs[-1] if csvs else None
    ulog_source = latest_ulog(firmware, started_ns)
    ulog_path = None
    if ulog_source is not None:
        ulog_path = case_root / "px4.ulg"
        shutil.copy2(ulog_source, ulog_path)

    result = {
        "scenario": scenario.key,
        "method": method,
        "status": status,
        "error": error,
        "duration_wall_s": time.monotonic() - started_wall,
        "controller": "original_direct" if method == "original_direct" else "drcda",
        "allocator_variant": None if method == "original_direct" else method,
        "servo_model": None if method == "original_direct" else "identified_gain_no_delay",
        "tuning_file": str(tuning_file),
        "environment": scenario.environment,
        "csv": str(csv_path) if csv_path else None,
        "ulog": str(ulog_path) if ulog_path else None,
    }
    (case_root / "result.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(f"{scenario.key}/{method}: {status.upper()} {error}", flush=True)
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware", type=Path, default=DEFAULT_FIRMWARE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scenario", action="append", choices=tuple(SCENARIOS))
    parser.add_argument("--method", action="append", choices=METHODS)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    metadata = firmware_metadata(args.firmware)
    if metadata["dynamic_actuator_tokens_present"]:
        raise RuntimeError("firmware model contains delayed/dynamic actuator plugin tokens")
    scenarios = args.scenario or list(SCENARIOS)
    methods = args.method or list(METHODS)
    manifest = {
        "created_at_unix_s": time.time(),
        "firmware": metadata,
        "controller_commit": git_value(CONTROL_ROOT, "rev-parse", "HEAD"),
        "controller_worktree_dirty": bool(git_value(CONTROL_ROOT, "status", "--short")),
        "common_tuning": str(DEFAULT_TUNING),
        "scenarios": scenarios,
        "scenario_definitions": {
            name: {
                "trigger": SCENARIOS[name].trigger,
                "timeout_s": SCENARIOS[name].timeout_s,
                "environment": SCENARIOS[name].environment,
                "tuning_overrides": SCENARIOS[name].tuning_overrides,
            }
            for name in scenarios
        },
        "methods": methods,
        "servo_model": "identified_gain_no_delay",
        "cases": [],
    }
    for scenario_name in scenarios:
        for method in methods:
            manifest["cases"].append(
                run_case(
                    args.output,
                    args.firmware,
                    SCENARIOS[scenario_name],
                    method,
                )
            )
            (args.output / "manifest.json").write_text(
                json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )
    return 0 if all(case["status"] == "complete" for case in manifest["cases"]) else 1


if __name__ == "__main__":
    sys.exit(main())
