#!/usr/bin/env python3
"""Robust gamepad input for the Hnuter external controllers.

SDL does not enumerate some Linux joystick devices in headless or remote
desktop sessions.  This module reads the stable Linux joystick API directly
and reconnects after USB hotplug.  Axis mapping remains configurable through
environment variables so the flight controller never depends on SDL order.
"""

import errno
import glob
import math
import os
import struct
import time
from typing import Dict, Optional

import numpy as np


_JS_EVENT_BUTTON = 0x01
_JS_EVENT_AXIS = 0x02
_JS_EVENT_INIT = 0x80
_JS_EVENT_FORMAT = "IhBB"
_JS_EVENT_SIZE = struct.calcsize(_JS_EVENT_FORMAT)


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, str(default)))
    except ValueError:
        return default


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except ValueError:
        return default


class GamepadManager:
    """Read velocity commands from a Linux joystick with safe reconnect."""

    def __init__(self,
                 max_vxy: float = 1.0,
                 max_vz: float = 0.5,
                 max_yaw_rate: float = 0.6,
                 max_pitch_rate: float = math.radians(20.0),
                 deadzone: float = 0.10,
                 expo: float = 0.40,
                 filter_tau: float = 0.20,
                 lt_axis: int = 2,
                 rt_axis: int = 5,
                 trigger_mode: str = "minus_one_to_one",
                 logger=None):
        self.logger = logger
        self.max_vxy = float(max_vxy)
        self.max_vz = float(max_vz)
        self.max_yaw_rate = float(max_yaw_rate)
        self.max_pitch_rate = float(max_pitch_rate)
        self.deadzone = float(deadzone)
        self.expo = float(expo)
        self.filter_tau = float(filter_tau)
        self.trigger_mode = os.environ.get("HNUTER_TRIGGER_MODE", trigger_mode)

        self.axis_yaw = _env_int("HNUTER_AXIS_YAW", 0)
        self.axis_throttle = _env_int("HNUTER_AXIS_THROTTLE", 1)
        self.axis_roll = _env_int("HNUTER_AXIS_ROLL", 3)
        self.axis_pitch = _env_int("HNUTER_AXIS_PITCH", 4)
        self.axis_lt = _env_int("HNUTER_AXIS_LT", lt_axis)
        self.axis_rt = _env_int("HNUTER_AXIS_RT", rt_axis)
        self.axis_sign_yaw = _env_float("HNUTER_AXIS_SIGN_YAW", 1.0)
        self.axis_sign_throttle = _env_float("HNUTER_AXIS_SIGN_THROTTLE", 1.0)
        self.axis_sign_roll = _env_float("HNUTER_AXIS_SIGN_ROLL", 1.0)
        self.axis_sign_pitch = _env_float("HNUTER_AXIS_SIGN_PITCH", 1.0)

        self.device_preference = os.environ.get("HNUTER_JOYSTICK_DEVICE", "/dev/input/js0")
        self.fd: Optional[int] = None
        self.device_path = ""
        self.axes = np.zeros(16, dtype=float)
        # Xbox triggers are released at -1.  Init events replace these values.
        self.axes[self.axis_lt] = -1.0
        self.axes[self.axis_rt] = -1.0
        self.last_connect_attempt = 0.0
        self.last_input_time = 0.0
        self.last_status_time = 0.0
        self.filtered_cmds = self._zero_commands()
        self._connect(force=True)

    @staticmethod
    def _zero_commands() -> Dict[str, float]:
        return {
            "vx_b": 0.0,
            "vy_b": 0.0,
            "vz": 0.0,
            "yaw_rate": 0.0,
            "pitch_rate": 0.0,
            "lt": 0.0,
            "rt": 0.0,
        }

    def _log_info(self, message: str) -> None:
        if self.logger:
            self.logger.info(message)
        else:
            print(message)

    def _log_warn(self, message: str) -> None:
        if self.logger:
            self.logger.warn(message)
        else:
            print(message)

    def _candidate_devices(self):
        paths = [self.device_preference]
        paths.extend(sorted(glob.glob("/dev/input/js*")))
        return list(dict.fromkeys(paths))

    def _connect(self, force: bool = False) -> None:
        now = time.monotonic()
        if self.fd is not None or (not force and now - self.last_connect_attempt < 1.0):
            return
        self.last_connect_attempt = now
        for path in self._candidate_devices():
            try:
                self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                self.device_path = path
                self.axes[:] = 0.0
                self.axes[self.axis_lt] = -1.0
                self.axes[self.axis_rt] = -1.0
                self._log_info(
                    f"Gamepad connected through Linux joystick API: {path}; "
                    f"axes yaw/throttle/roll/pitch/LT/RT="
                    f"{self.axis_yaw}/{self.axis_throttle}/{self.axis_roll}/"
                    f"{self.axis_pitch}/{self.axis_lt}/{self.axis_rt}"
                )
                return
            except OSError as exc:
                if exc.errno not in (errno.ENOENT, errno.EACCES, errno.ENODEV):
                    self._log_warn(f"Unable to open gamepad {path}: {exc}")
        if force:
            self._log_warn(
                "No readable /dev/input/js* gamepad. Commands remain neutral; "
                "check HNUTER_JOYSTICK_DEVICE and device ACL permissions."
            )

    def _disconnect(self, reason: str) -> None:
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None
        self.device_path = ""
        self._log_warn(f"Gamepad disconnected ({reason}); commands are returning to neutral.")

    def close(self) -> None:
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None

    def _poll_events(self) -> None:
        self._connect()
        if self.fd is None:
            return
        while True:
            try:
                payload = os.read(self.fd, _JS_EVENT_SIZE)
                if not payload:
                    self._disconnect("end of device stream")
                    return
                if len(payload) != _JS_EVENT_SIZE:
                    continue
                _, value, event_type, number = struct.unpack(_JS_EVENT_FORMAT, payload)
                event_type &= ~_JS_EVENT_INIT
                if event_type == _JS_EVENT_AXIS and number < len(self.axes):
                    self.axes[number] = float(np.clip(value / 32767.0, -1.0, 1.0))
                    self.last_input_time = time.monotonic()
                elif event_type == _JS_EVENT_BUTTON:
                    self.last_input_time = time.monotonic()
            except BlockingIOError:
                return
            except OSError as exc:
                if exc.errno in (errno.ENODEV, errno.EIO, errno.EBADF):
                    self._disconnect(str(exc))
                    return
                raise

    def _axis(self, index: int, sign: float = 1.0) -> float:
        if index < 0 or index >= len(self.axes):
            return 0.0
        return float(np.clip(sign * self.axes[index], -1.0, 1.0))

    def _apply_deadzone(self, value: float) -> float:
        value = float(value)
        if abs(value) <= self.deadzone:
            return 0.0
        return math.copysign((abs(value) - self.deadzone) / max(1.0 - self.deadzone, 1e-6), value)

    def _apply_expo(self, value: float) -> float:
        return self.expo * value ** 3 + (1.0 - self.expo) * value

    def _trigger_to_unit(self, raw: float) -> float:
        if self.trigger_mode == "zero_to_one":
            value = raw
        elif self.trigger_mode == "one_to_minus_one":
            value = 0.5 * (1.0 - raw)
        else:
            value = 0.5 * (raw + 1.0)
        value = float(np.clip(value, 0.0, 1.0))
        if value <= self.deadzone:
            return 0.0
        value = (value - self.deadzone) / max(1.0 - self.deadzone, 1e-6)
        return float(np.clip(self._apply_expo(value), 0.0, 1.0))

    def get_velocity_commands(self, dt: float) -> Dict[str, float]:
        self._poll_events()
        connected = self.fd is not None

        raw_yaw = self._axis(self.axis_yaw, self.axis_sign_yaw) if connected else 0.0
        raw_throttle = self._axis(self.axis_throttle, self.axis_sign_throttle) if connected else 0.0
        raw_roll = self._axis(self.axis_roll, self.axis_sign_roll) if connected else 0.0
        raw_pitch = self._axis(self.axis_pitch, self.axis_sign_pitch) if connected else 0.0
        raw_lt = self._axis(self.axis_lt) if connected else -1.0
        raw_rt = self._axis(self.axis_rt) if connected else -1.0

        yaw = self._apply_expo(self._apply_deadzone(raw_yaw))
        throttle = self._apply_expo(self._apply_deadzone(raw_throttle))
        roll = self._apply_expo(self._apply_deadzone(raw_roll))
        pitch = self._apply_expo(self._apply_deadzone(raw_pitch))
        lt = self._trigger_to_unit(raw_lt)
        rt = self._trigger_to_unit(raw_rt)
        targets = {
            "vx_b": -pitch * self.max_vxy,
            "vy_b": -roll * self.max_vxy,
            "vz": -throttle * self.max_vz,
            "yaw_rate": -yaw * self.max_yaw_rate,
            "pitch_rate": (lt - rt) * self.max_pitch_rate,
            "lt": lt,
            "rt": rt,
        }

        dt = float(np.clip(dt, 0.0, 0.2))
        alpha = dt / (self.filter_tau + dt) if self.filter_tau > 1e-3 else 1.0
        for key in ("vx_b", "vy_b", "vz", "yaw_rate", "pitch_rate"):
            self.filtered_cmds[key] += alpha * (targets[key] - self.filtered_cmds[key])
        self.filtered_cmds["lt"] = targets["lt"]
        self.filtered_cmds["rt"] = targets["rt"]
        return self.filtered_cmds.copy()

    @property
    def connected(self) -> bool:
        return self.fd is not None

