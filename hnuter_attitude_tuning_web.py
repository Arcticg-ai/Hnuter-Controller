#!/usr/bin/env python3
"""LAN web dashboard for Hnuter real-aircraft tuning.

The service has no web-framework dependency. DDS telemetry is streamed to the
browser with Server-Sent Events, while parameter reads/writes use MAVLink.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import mimetypes
import os
import signal
import site
import struct
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Iterable, Optional, Tuple
from urllib.parse import parse_qs, urlparse

# Keep DDS discovery on the companion computer unless explicitly overridden.
if os.environ.get('HNUTER_ALLOW_REMOTE_DDS', '0') != '1':
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_STATIC_PEERS', None)

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleLocalPositionSetpoint
from px4_msgs.msg import VehicleTorqueSetpoint

try:
    from pymavlink import mavutil
except Exception:  # noqa: BLE001
    user_site = site.getusersitepackages()
    if user_site and user_site not in sys.path:
        sys.path.append(user_site)
    try:
        from pymavlink import mavutil
    except Exception:  # noqa: BLE001
        mavutil = None


def param(minimum: float, maximum: float, step: float, default: float) -> dict:
    return {'min': minimum, 'max': maximum, 'step': step, 'default': default}


PARAM_GROUPS = {
    'Pitch attitude': {
        'HNTR_PITCH_BIAS': param(-1.0, 1.0, 0.001, 0.0),
        'HNTR_ATT_KR_P': param(0.0, 20.0, 0.1, 1.5),
        'HNTR_ATT_D_P': param(0.0, 20.0, 0.1, 1.2),
        'HNTR_ATT_I_P': param(0.0, 20.0, 0.01, 0.0),
        'HNTR_ATT_ILIM_P': param(0.0, 50.0, 0.1, 3.0),
        'HNTR_TAU_P': param(0.0, 100.0, 0.1, 0.9),
    },
    'Roll and yaw': {
        'HNTR_ATT_KR_R': param(0.0, 20.0, 0.1, 1.5),
        'HNTR_ATT_D_R': param(0.0, 20.0, 0.1, 1.2),
        'HNTR_TAU_R': param(0.0, 100.0, 0.1, 0.9),
        'HNTR_ATT_KR_Y': param(0.0, 20.0, 0.1, 1.5),
        'HNTR_ATT_D_Y': param(0.0, 20.0, 0.1, 1.2),
        'HNTR_TAU_Y': param(0.0, 100.0, 0.1, 1.8),
    },
    'Position XY': {
        'HNTR_POS_P_XY': param(0.0, 10.0, 0.05, 0.6),
        'HNTR_VEL_P_XY': param(0.0, 30.0, 0.05, 1.5),
        'HNTR_VEL_I_XY': param(0.0, 10.0, 0.01, 0.10),
        'HNTR_VEL_D_XY': param(0.0, 10.0, 0.01, 0.10),
        'HNTR_VEL_ILIM_XY': param(0.0, 30.0, 0.1, 1.5),
        'HNTR_VEL_XY': param(0.0, 30.0, 0.1, 3.0),
        'HNTR_ACC_XY': param(0.1, 100.0, 0.5, 5.0),
        'HNTR_TILT_MAX': param(0.0, 185.0, 1.0, 185.0),
    },
    'Position Z': {
        'HNTR_MOT_HOV': param(0.05, 0.95, 0.005, 0.40),
        'HNTR_MOT_EXPO': param(0.2, 1.5, 0.01, 0.50),
        'HNTR_POS_P_Z': param(0.0, 10.0, 0.05, 1.0),
        'HNTR_VEL_P_Z': param(0.0, 30.0, 0.05, 2.5),
        'HNTR_VEL_I_Z': param(0.0, 10.0, 0.01, 0.40),
        'HNTR_VEL_D_Z': param(0.0, 10.0, 0.01, 0.20),
        'HNTR_VEL_ILIM_Z': param(0.0, 30.0, 0.1, 2.5),
        'HNTR_VEL_UP': param(0.0, 20.0, 0.1, 1.5),
        'HNTR_VEL_DN': param(0.0, 20.0, 0.1, 1.0),
        'HNTR_ACC_Z': param(0.1, 100.0, 0.5, 8.0),
    },
    'Stabilized Z': {
        'HNTR_STAB_Z_P': param(0.0, 30.0, 0.1, 3.0),
        'HNTR_STAB_Z_D': param(0.0, 30.0, 0.1, 2.0),
        'HNTR_STAB_Z_I': param(0.0, 10.0, 0.01, 0.5),
        'HNTR_STAB_ACC_Z': param(0.1, 100.0, 0.5, 8.0),
        'HNTR_STAB_Z_VEL': param(0.0, 10.0, 0.1, 0.8),
        'HNTR_STAB_THR_DB': param(0.0, 0.8, 0.01, 0.15),
    },
    'Takeoff lock': {
        'HNTR_TO_SUP_T': param(0.0, 10.0, 0.1, 1.0),
        'HNTR_TO_LOCK_T': param(0.0, 20.0, 0.1, 3.0),
        'HNTR_TO_TILT': param(0.0, 185.0, 1.0, 20.0),
        'HNTR_LOCK_TILT': param(0.0, 185.0, 1.0, 30.0),
        'HNTR_LOCK_ACC': param(0.1, 100.0, 0.5, 3.0),
        'HNTR_LOCK_KP': param(0.0, 1.0, 0.01, 0.8),
    },
    'Allocator': {
        'HNTR_CTRL_MODE': param(0.0, 1.0, 1.0, 0.0),
        'HNTR_ROLL_SIGN': param(-1.0, 1.0, 2.0, 1.0),
        'HNTR_TAIL_SIGN': param(-1.0, 1.0, 2.0, 1.0),
        'HNTR_TAIL_COMP': param(0.0, 1.0, 0.01, 0.0),
    },
    'Vehicle model': {
        'HNTR_MASS': param(0.1, 50.0, 0.1, 4.5),
        'HNTR_MAX_ARM_T': param(1.0, 500.0, 1.0, 170.96),
        'HNTR_MAX_TAIL_T': param(1.0, 500.0, 1.0, 85.48),
        'HNTR_L1': param(0.01, 5.0, 0.01, 0.33),
        'HNTR_L2': param(0.01, 5.0, 0.01, 0.664),
    },
}

PARAM_CONFIG = {
    name: cfg
    for group in PARAM_GROUPS.values()
    for name, cfg in group.items()
}

INTEGER_PARAMS = {'HNTR_CTRL_MODE'}


def offboard_param(
    json_key: str,
    index: Optional[int],
    minimum: float,
    maximum: float,
    step: float,
    default: float,
) -> dict:
    config = param(minimum, maximum, step, default)
    config.update({'json_key': json_key, 'index': index})
    return config


OFFBOARD_PARAM_GROUPS = {
    'Attitude roll / pitch': {
        'HNTR_PITCH_BIAS': offboard_param('HNTR_PITCH_BIAS', None, -0.3, 0.3, 0.001, 0.09),
        'direct_KR.roll': offboard_param('direct_KR', 0, 0.0, 20.0, 0.05, 4.0),
        'direct_KR.pitch': offboard_param('direct_KR', 1, 0.0, 20.0, 0.05, 6.0),
        'direct_Domega.roll': offboard_param('direct_Domega', 0, 0.0, 20.0, 0.05, 2.2),
        'direct_Domega.pitch': offboard_param('direct_Domega', 1, 0.0, 20.0, 0.05, 3.2),
        'direct_attitude_Ki.roll': offboard_param('direct_attitude_Ki', 0, 0.0, 3.0, 0.01, 0.3),
        'direct_attitude_Ki.pitch': offboard_param('direct_attitude_Ki', 1, 0.0, 3.0, 0.01, 0.3),
        'direct_attitude_integral_limit.roll': offboard_param(
            'direct_attitude_integral_limit', 0, 0.0, 5.0, 0.05, 1.2
        ),
        'direct_attitude_integral_limit.pitch': offboard_param(
            'direct_attitude_integral_limit', 1, 0.0, 5.0, 0.05, 1.2
        ),
        'direct_tau_limit.roll': offboard_param('direct_tau_limit', 0, 0.05, 20.0, 0.05, 2.0),
        'direct_tau_limit.pitch': offboard_param('direct_tau_limit', 1, 0.05, 20.0, 0.05, 2.5),
    },
    'Attitude yaw': {
        'direct_KR.yaw': offboard_param('direct_KR', 2, 0.0, 20.0, 0.05, 5.5),
        'direct_Domega.yaw': offboard_param('direct_Domega', 2, 0.0, 20.0, 0.05, 2.6),
        'direct_attitude_Ki.yaw': offboard_param('direct_attitude_Ki', 2, 0.0, 3.0, 0.01, 0.6),
        'direct_attitude_integral_limit.yaw': offboard_param(
            'direct_attitude_integral_limit', 2, 0.0, 5.0, 0.05, 0.6
        ),
        'direct_tau_limit.yaw': offboard_param('direct_tau_limit', 2, 0.05, 20.0, 0.05, 1.2),
    },
    'Position XY': {
        'direct_pos_Kp.north': offboard_param('direct_pos_Kp_ned', 0, 0.0, 15.0, 0.05, 6.0),
        'direct_pos_Kp.east': offboard_param('direct_pos_Kp_ned', 1, 0.0, 15.0, 0.05, 6.0),
        'direct_pos_Kd.north': offboard_param('direct_pos_Kd_ned', 0, 0.0, 15.0, 0.05, 3.5),
        'direct_pos_Kd.east': offboard_param('direct_pos_Kd_ned', 1, 0.0, 15.0, 0.05, 3.5),
        'direct_pos_Ki.north': offboard_param('direct_pos_Ki_ned', 0, 0.0, 10.0, 0.01, 0.0),
        'direct_pos_Ki.east': offboard_param('direct_pos_Ki_ned', 1, 0.0, 10.0, 0.01, 0.0),
        'direct_pos_integral_limit.north': offboard_param(
            'direct_pos_integral_limit_ned', 0, 0.0, 10.0, 0.05, 1.0
        ),
        'direct_pos_integral_limit.east': offboard_param(
            'direct_pos_integral_limit_ned', 1, 0.0, 10.0, 0.05, 1.0
        ),
        'max_acc_xy': offboard_param('max_acc_xy', None, 0.1, 30.0, 0.1, 3.0),
    },
    'Position Z': {
        'direct_pos_Kp.down': offboard_param('direct_pos_Kp_ned', 2, 0.0, 20.0, 0.05, 8.0),
        'direct_pos_Kd.down': offboard_param('direct_pos_Kd_ned', 2, 0.0, 20.0, 0.05, 4.0),
        'direct_pos_Ki.down': offboard_param('direct_pos_Ki_ned', 2, 0.0, 10.0, 0.01, 3.0),
        'direct_pos_integral_limit.down': offboard_param(
            'direct_pos_integral_limit_ned', 2, 0.0, 10.0, 0.05, 2.0
        ),
        'max_acc_z': offboard_param('max_acc_z', None, 0.1, 30.0, 0.1, 20.0),
    },
    'RC response': {
        'gamepad_filter_tau.forward': offboard_param(
            'gamepad_filter_tau_body_xy_s', 0, 0.0, 2.0, 0.01, 0.10
        ),
        'gamepad_filter_tau.right': offboard_param(
            'gamepad_filter_tau_body_xy_s', 1, 0.0, 2.0, 0.01, 0.10
        ),
        'RC attitude / yaw filter (s)': offboard_param(
            'gamepad_filter_tau_s', None, 0.0, 2.0, 0.01, 0.10
        ),
        'gamepad_max_acc.forward': offboard_param(
            'gamepad_max_acc_body_xy_mps2', 0, 0.05, 5.0, 0.05, 3.0
        ),
        'gamepad_max_acc.right': offboard_param(
            'gamepad_max_acc_body_xy_mps2', 1, 0.05, 5.0, 0.05, 3.0
        ),
        'gamepad_max_speed.forward': offboard_param(
            'gamepad_max_vxy_body_mps', 0, 0.05, 3.0, 0.05, 1.2
        ),
        'gamepad_max_speed.right': offboard_param(
            'gamepad_max_vxy_body_mps', 1, 0.05, 3.0, 0.05, 1.2
        ),
        'manual_max_position_lead_xy': offboard_param(
            'manual_max_position_lead_xy', None, 0.05, 2.0, 0.05, 0.75
        ),
        'gamepad_deadzone': offboard_param('gamepad_deadzone', None, 0.0, 0.4, 0.01, 0.1),
        'gamepad_expo': offboard_param('gamepad_expo', None, 0.0, 1.0, 0.05, 0.4),
        'RC15 AUX1 roll rate (deg/s)': offboard_param(
            'rc_attitude_rate_deg_s', 0, 0.0, 90.0, 1.0, 20.0
        ),
        'RC16 AUX2 pitch rate (deg/s)': offboard_param(
            'rc_attitude_rate_deg_s', 1, 0.0, 90.0, 1.0, 20.0
        ),
        'RC attitude angle limit (deg)': offboard_param(
            'rc_attitude_angle_limit_deg', None, 0.0, 90.0, 1.0, 45.0
        ),
        'RC15 AUX1 roll sign': offboard_param(
            'rc_attitude_sign', 0, -1.0, 1.0, 2.0, -1.0
        ),
        'RC16 AUX2 pitch sign': offboard_param(
            'rc_attitude_sign', 1, -1.0, 1.0, 2.0, -1.0
        ),
    },
}

OFFBOARD_PARAM_CONFIG = {
    name: cfg
    for group in OFFBOARD_PARAM_GROUPS.values()
    for name, cfg in group.items()
}


def public_param_groups(groups: dict) -> dict:
    public = {}
    for group_name, group in groups.items():
        public[group_name] = {
            name: {
                key: cfg[key]
                for key in ('min', 'max', 'step', 'default')
            }
            for name, cfg in group.items()
        }
    return public


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_euler(q: Iterable[float]) -> Tuple[float, float, float]:
    w, x, y, z = [float(value) for value in q]
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def finite(value: float) -> Optional[float]:
    number = float(value)
    return number if math.isfinite(number) else None


class MavlinkParamClient:
    def __init__(self, endpoint: str, source_system: int = 250, source_component: int = 190):
        self.endpoint = endpoint
        self.endpoints = self._expand_endpoints(endpoint)
        self.source_system = source_system
        self.source_component = source_component
        self.master = None
        self.lock = threading.Lock()
        self.enabled = mavutil is not None and endpoint.lower() != 'none'
        self.connected_endpoint = None

    @staticmethod
    def _expand_endpoints(endpoint: str) -> list[str]:
        if endpoint.lower() != 'auto':
            return [endpoint]
        return [
            'udpin:0.0.0.0:14550',
            'udp:127.0.0.1:14540',
            'udpin:0.0.0.0:14540',
            'udp:127.0.0.1:14550',
        ]

    @property
    def connected(self) -> bool:
        return self.master is not None

    def connect(self, timeout: float = 8.0) -> bool:
        self.close()
        if not self.enabled:
            print('[MAVLink] disabled or pymavlink unavailable; web UI is read-only.')
            return False
        per_endpoint_timeout = max(1.0, timeout / max(len(self.endpoints), 1))
        for endpoint in self.endpoints:
            try:
                print(f'[MAVLink] connecting {endpoint} ...')
                master = mavutil.mavlink_connection(
                    endpoint,
                    source_system=self.source_system,
                    source_component=self.source_component,
                    autoreconnect=True,
                    robust_parsing=True,
                )
                heartbeat = master.wait_heartbeat(timeout=per_endpoint_timeout)
                if heartbeat is None:
                    master.close()
                    continue
                self.master = master
                self.connected_endpoint = endpoint
                print(
                    f'[MAVLink] heartbeat system={master.target_system} '
                    f'component={master.target_component} via {endpoint}'
                )
                return True
            except Exception as exc:  # noqa: BLE001
                print(f'[MAVLink] connection failed on {endpoint}: {exc}')
        print('[MAVLink] no heartbeat; parameter operations are unavailable.')
        return False

    def close(self) -> None:
        if self.master is not None:
            try:
                self.master.close()
            except Exception:  # noqa: BLE001
                pass
        self.master = None
        self.connected_endpoint = None

    @staticmethod
    def _param_name(msg) -> str:
        param_id = msg.param_id
        if isinstance(param_id, bytes):
            param_id = param_id.decode('utf-8', errors='ignore')
        return str(param_id).strip('\x00')

    @staticmethod
    def _decode_param_value(msg) -> float:
        if msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            packed = struct.pack('<f', float(msg.param_value))
            return float(struct.unpack('<i', packed)[0])
        return float(msg.param_value)

    @staticmethod
    def _wire_value(name: str, value: float) -> tuple[float, int]:
        if name in INTEGER_PARAMS:
            packed = struct.pack('<i', int(round(value)))
            return struct.unpack('<f', packed)[0], mavutil.mavlink.MAV_PARAM_TYPE_INT32
        return float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32

    def _drain_param_values(self, limit: int = 1024) -> None:
        for _ in range(limit):
            if self.master.recv_match(type='PARAM_VALUE', blocking=False) is None:
                break

    def request_param(self, name: str, timeout: float = 1.0) -> Optional[float]:
        if self.master is None:
            return None
        with self.lock:
            try:
                self.master.mav.param_request_read_send(
                    self.master.target_system,
                    self.master.target_component,
                    name.encode('utf-8'),
                    -1,
                )
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.1)
                    if msg is not None and self._param_name(msg) == name:
                        return self._decode_param_value(msg)
            except Exception as exc:  # noqa: BLE001
                print(f'[PARAM_GET] failed {name}: {exc}')
        return None

    def set_and_confirm(self, name: str, value: float, timeout: float = 1.5) -> tuple[Optional[float], Optional[float]]:
        if self.master is None:
            return None, None
        with self.lock:
            try:
                wire_value, param_type = self._wire_value(name, value)
                expected = float(round(value)) if name in INTEGER_PARAMS else float(value)
                tolerance = 0.0 if name in INTEGER_PARAMS else max(1e-5, abs(expected) * 1e-6)
                last_observed = None

                for attempt in range(1, 4):
                    self._drain_param_values()
                    self.master.mav.param_set_send(
                        self.master.target_system,
                        self.master.target_component,
                        name.encode('utf-8'),
                        wire_value,
                        param_type,
                    )
                    deadline = time.monotonic() + timeout
                    while time.monotonic() < deadline:
                        msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.1)
                        if msg is None or self._param_name(msg) != name:
                            continue
                        last_observed = self._decode_param_value(msg)
                        if abs(last_observed - expected) <= tolerance:
                            print(
                                f'[PARAM_SET] {name} requested={expected:.6g} '
                                f'confirmed={last_observed:.6g} attempt={attempt}'
                            )
                            return last_observed, last_observed
                        print(
                            f'[PARAM_SET] stale/mismatched confirmation {name} '
                            f'requested={expected:.6g} observed={last_observed:.6g} attempt={attempt}'
                        )
                return None, last_observed
            except Exception as exc:  # noqa: BLE001
                print(f'[PARAM_SET] failed {name}: {exc}')
        return None, None

    def save(self) -> bool:
        if self.master is None:
            return False
        with self.lock:
            try:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
                    0,
                    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                )
                print('[PARAM_SAVE] requested')
                return True
            except Exception as exc:  # noqa: BLE001
                print(f'[PARAM_SAVE] failed: {exc}')
                return False


class OffboardTuningStore:
    """Validated, atomic access to the controller's hot-reloaded JSON file."""

    def __init__(self, path: Path):
        self.path = path.expanduser().resolve()
        self.status_path = self.path.with_suffix('.applied.json')
        self.lock = threading.Lock()
        self.history: list[dict] = []

    @property
    def available(self) -> bool:
        return self.path.is_file()

    def _read_unlocked(self) -> dict:
        with self.path.open('r', encoding='utf-8') as stream:
            data = json.load(stream)
        if not isinstance(data, dict):
            raise ValueError('Offboard tuning file must contain a JSON object')
        return data

    @staticmethod
    def _configured_value(data: dict, cfg: dict) -> Optional[float]:
        value = data.get(cfg['json_key'])
        index = cfg['index']
        if index is not None:
            if not isinstance(value, list) or index >= len(value):
                return None
            value = value[index]
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None

    def group_values(self, group_name: str) -> dict:
        group = OFFBOARD_PARAM_GROUPS.get(group_name)
        if group is None:
            raise KeyError('unknown Offboard parameter group')
        with self.lock:
            data = self._read_unlocked()
            return {
                name: self._configured_value(data, cfg)
                for name, cfg in group.items()
            }

    def _atomic_write_unlocked(self, data: dict) -> int:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(
            f'.{self.path.name}.{os.getpid()}.{time.time_ns()}.tmp'
        )
        try:
            with temporary.open('x', encoding='utf-8') as stream:
                json.dump(data, stream, indent=2, sort_keys=True)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        finally:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass
        return self.path.stat().st_mtime_ns

    def set_value(self, name: str, value: float) -> tuple[float, int, int]:
        cfg = OFFBOARD_PARAM_CONFIG.get(name)
        if cfg is None:
            raise KeyError('unknown Offboard parameter')
        if not math.isfinite(value) or value < cfg['min'] or value > cfg['max']:
            raise ValueError(f"value must be in [{cfg['min']}, {cfg['max']}]")
        with self.lock:
            data = self._read_unlocked()
            previous = json.loads(json.dumps(data))
            index = cfg['index']
            if index is None:
                data[cfg['json_key']] = float(value)
            else:
                array = data.get(cfg['json_key'])
                if not isinstance(array, list) or index >= len(array):
                    raise ValueError(
                        f"{cfg['json_key']} must be an array with at least {index + 1} values"
                    )
                array[index] = float(value)
            revision = time.time_ns()
            data['_web_revision'] = revision
            mtime_ns = self._atomic_write_unlocked(data)
            confirmed = self._configured_value(data, cfg)
            if confirmed is None:
                raise RuntimeError('failed to read back Offboard tuning value')
            self.history.append(previous)
            if len(self.history) > 50:
                self.history.pop(0)
            return confirmed, revision, mtime_ns

    def revert(self) -> tuple[int, int]:
        with self.lock:
            if not self.history:
                raise LookupError('no Offboard change to undo since the web server started')
            data = self.history.pop()
            revision = time.time_ns()
            data['_web_revision'] = revision
            mtime_ns = self._atomic_write_unlocked(data)
            return revision, mtime_ns

    def wait_applied(self, revision: int, timeout: float = 1.5) -> tuple[bool, Optional[dict]]:
        deadline = time.monotonic() + max(timeout, 0.0)
        last_status = None
        while time.monotonic() <= deadline:
            try:
                with self.status_path.open('r', encoding='utf-8') as stream:
                    status = json.load(stream)
                if isinstance(status, dict):
                    last_status = status
                    if status.get('revision') == revision and status.get('ok') is True:
                        return True, status
            except (OSError, ValueError, json.JSONDecodeError):
                pass
            time.sleep(0.05)
        return False, last_status

    def status(self) -> dict:
        try:
            with self.status_path.open('r', encoding='utf-8') as stream:
                status = json.load(stream)
            return status if isinstance(status, dict) else {}
        except (OSError, ValueError, json.JSONDecodeError):
            return {}


class HnuterTelemetry(Node):
    TOPICS = ('attitude', 'setpoint', 'position', 'position_setpoint', 'torque', 'motors', 'mode')

    def __init__(self):
        super().__init__('hnuter_attitude_tuning_web')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.started = time.monotonic()
        self.lock = threading.Lock()
        self.attitude = (math.nan, math.nan, math.nan)
        self.setpoint = (math.nan, math.nan, math.nan)
        self.position = (math.nan, math.nan, math.nan)
        self.position_setpoint = (math.nan, math.nan, math.nan)
        self.torque = (math.nan, math.nan, math.nan)
        self.motors = [math.nan] * 12
        self.mode = {'armed': False, 'posctl': False, 'offboard': False}
        self.topic_time = {name: None for name in self.TOPICS}

        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.on_attitude, qos)
        self.create_subscription(
            VehicleAttitudeSetpoint,
            '/fmu/out/vehicle_attitude_setpoint_v1',
            self.on_setpoint,
            qos,
        )
        self.create_subscription(
            VehicleTorqueSetpoint,
            '/fmu/out/vehicle_torque_setpoint',
            self.on_torque,
            qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.on_position,
            qos,
        )
        self.create_subscription(
            VehicleLocalPositionSetpoint,
            '/fmu/out/vehicle_local_position_setpoint',
            self.on_position_setpoint,
            qos,
        )
        self.create_subscription(ActuatorMotors, '/fmu/out/actuator_motors', self.on_motors, qos)
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.on_mode, qos)

    def on_attitude(self, msg: VehicleAttitude) -> None:
        with self.lock:
            self.attitude = quat_to_euler(msg.q)
            self.topic_time['attitude'] = time.monotonic()

    def on_setpoint(self, msg: VehicleAttitudeSetpoint) -> None:
        with self.lock:
            self.setpoint = quat_to_euler(msg.q_d)
            self.topic_time['setpoint'] = time.monotonic()

    def on_torque(self, msg: VehicleTorqueSetpoint) -> None:
        with self.lock:
            self.torque = tuple(float(value) for value in msg.xyz)
            self.topic_time['torque'] = time.monotonic()

    def on_position(self, msg: VehicleLocalPosition) -> None:
        with self.lock:
            self.position = (float(msg.x), float(msg.y), float(msg.z))
            self.topic_time['position'] = time.monotonic()

    def on_position_setpoint(self, msg: VehicleLocalPositionSetpoint) -> None:
        with self.lock:
            self.position_setpoint = (float(msg.x), float(msg.y), float(msg.z))
            self.topic_time['position_setpoint'] = time.monotonic()

    def on_motors(self, msg: ActuatorMotors) -> None:
        with self.lock:
            self.motors = [float(value) for value in msg.control]
            self.topic_time['motors'] = time.monotonic()

    def on_mode(self, msg: VehicleControlMode) -> None:
        with self.lock:
            self.mode = {
                'armed': bool(msg.flag_armed),
                'posctl': bool(msg.flag_control_position_enabled),
                'offboard': bool(msg.flag_control_offboard_enabled),
            }
            self.topic_time['mode'] = time.monotonic()

    def snapshot(self) -> dict:
        now = time.monotonic()
        with self.lock:
            attitude = tuple(self.attitude)
            setpoint = tuple(self.setpoint)
            position = tuple(self.position)
            position_setpoint = tuple(self.position_setpoint)
            torque = tuple(self.torque)
            motors = tuple(self.motors[:5])
            mode = dict(self.mode)
            topic_time = dict(self.topic_time)
        scale = 180.0 / math.pi
        attitude_deg = [finite(value * scale) for value in attitude]
        setpoint_deg = [finite(value * scale) for value in setpoint]
        errors = [
            finite(wrap_pi(sp - actual) * scale)
            if math.isfinite(sp) and math.isfinite(actual) else None
            for actual, sp in zip(attitude, setpoint)
        ]
        position_values = [finite(value) for value in position]
        position_setpoint_values = [finite(value) for value in position_setpoint]
        position_errors = [
            finite(sp - actual)
            if math.isfinite(sp) and math.isfinite(actual) else None
            for actual, sp in zip(position, position_setpoint)
        ]
        ages = {
            name: finite(now - stamp) if stamp is not None else None
            for name, stamp in topic_time.items()
        }
        return {
            't': round(now - self.started, 4),
            'attitude': attitude_deg,
            'setpoint': setpoint_deg,
            'error': errors,
            'position': position_values,
            'position_setpoint': position_setpoint_values,
            'position_error': position_errors,
            'torque': [finite(value) for value in torque],
            'motors': [finite(value) for value in motors],
            'mode': mode,
            'age': ages,
        }


class CsvRecorder:
    FIELDS = [
        't', 'roll_deg', 'pitch_deg', 'yaw_deg',
        'roll_sp_deg', 'pitch_sp_deg', 'yaw_sp_deg',
        'roll_err_deg', 'pitch_err_deg', 'yaw_err_deg',
        'north_m', 'east_m', 'down_m',
        'north_sp_m', 'east_sp_m', 'down_sp_m',
        'north_err_m', 'east_err_m', 'down_err_m',
        'tx', 'ty', 'tz', 'motor1', 'motor2', 'motor3', 'motor4', 'motor5',
        'armed', 'posctl', 'offboard',
    ]

    def __init__(self, telemetry: HnuterTelemetry, path: Path, rate_hz: float, stop_event: threading.Event):
        self.telemetry = telemetry
        self.path = path
        self.period = 1.0 / max(rate_hz, 0.1)
        self.stop_event = stop_event
        self.thread = threading.Thread(target=self.run, name='csv-recorder', daemon=True)

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.thread.start()

    def run(self) -> None:
        with self.path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=self.FIELDS)
            writer.writeheader()
            next_sample = time.monotonic()
            while not self.stop_event.is_set():
                sample = self.telemetry.snapshot()
                row = {
                    't': sample['t'],
                    'roll_deg': sample['attitude'][0],
                    'pitch_deg': sample['attitude'][1],
                    'yaw_deg': sample['attitude'][2],
                    'roll_sp_deg': sample['setpoint'][0],
                    'pitch_sp_deg': sample['setpoint'][1],
                    'yaw_sp_deg': sample['setpoint'][2],
                    'roll_err_deg': sample['error'][0],
                    'pitch_err_deg': sample['error'][1],
                    'yaw_err_deg': sample['error'][2],
                    'north_m': sample['position'][0],
                    'east_m': sample['position'][1],
                    'down_m': sample['position'][2],
                    'north_sp_m': sample['position_setpoint'][0],
                    'east_sp_m': sample['position_setpoint'][1],
                    'down_sp_m': sample['position_setpoint'][2],
                    'north_err_m': sample['position_error'][0],
                    'east_err_m': sample['position_error'][1],
                    'down_err_m': sample['position_error'][2],
                    'tx': sample['torque'][0],
                    'ty': sample['torque'][1],
                    'tz': sample['torque'][2],
                    'motor1': sample['motors'][0],
                    'motor2': sample['motors'][1],
                    'motor3': sample['motors'][2],
                    'motor4': sample['motors'][3],
                    'motor5': sample['motors'][4],
                    **sample['mode'],
                }
                writer.writerow(row)
                next_sample += self.period
                delay = next_sample - time.monotonic()
                if delay < -self.period:
                    next_sample = time.monotonic()
                    delay = 0.0
                self.stop_event.wait(max(delay, 0.0))
            stream.flush()


class TuningHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address,
        handler,
        telemetry: HnuterTelemetry,
        mavlink: MavlinkParamClient,
        offboard_tuning: OffboardTuningStore,
        static_dir: Path,
        stream_hz: float,
        token: str,
        stop_event: threading.Event,
    ):
        super().__init__(address, handler)
        self.telemetry = telemetry
        self.mavlink = mavlink
        self.offboard_tuning = offboard_tuning
        self.static_dir = static_dir
        self.stream_period = 1.0 / max(stream_hz, 1.0)
        self.token = token
        self.stop_event = stop_event


class TuningRequestHandler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server_version = 'HnuterTuning/1.0'

    @property
    def tuning_server(self) -> TuningHttpServer:
        return self.server  # type: ignore[return-value]

    def log_message(self, fmt: str, *args) -> None:
        if self.path.startswith('/api/events'):
            return
        print(f'[HTTP] {self.address_string()} {fmt % args}')

    def _authorized(self, parsed=None) -> bool:
        token = self.tuning_server.token
        if not token:
            return True
        parsed = parsed or urlparse(self.path)
        query_token = parse_qs(parsed.query).get('token', [''])[0]
        return self.headers.get('X-Hnuter-Token', '') == token or query_token == token

    def _send_json(self, payload: dict, status: int = HTTPStatus.OK) -> None:
        data = json.dumps(payload, separators=(',', ':'), allow_nan=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def _error(self, message: str, status: int = HTTPStatus.BAD_REQUEST) -> None:
        self._send_json({'ok': False, 'error': message}, status)

    def _read_json(self) -> Optional[dict]:
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if length <= 0 or length > 65536:
                return None
            return json.loads(self.rfile.read(length).decode('utf-8'))
        except (ValueError, json.JSONDecodeError, UnicodeDecodeError):
            return None

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path.startswith('/api/') and not self._authorized(parsed):
            self._error('unauthorized', HTTPStatus.UNAUTHORIZED)
            return
        if parsed.path == '/api/config':
            self._send_json({
                'ok': True,
                'sources': {
                    'firmware': {
                        'label': 'PX4 firmware (MAVLink)',
                        'description': 'Read and write HNTR_* parameters in PX4 firmware.',
                        'groups': public_param_groups(PARAM_GROUPS),
                        'available': self.tuning_server.mavlink.connected,
                    },
                    'offboard': {
                        'label': 'Offboard controller (live JSON)',
                        'description': 'Atomically update the Hardware controller JSON; reload is normally confirmed within 0.5 s.',
                        'groups': public_param_groups(OFFBOARD_PARAM_GROUPS),
                        'available': self.tuning_server.offboard_tuning.available,
                        'path': str(self.tuning_server.offboard_tuning.path),
                    },
                },
                'stream_hz': round(1.0 / self.tuning_server.stream_period, 2),
                'mavlink': self.tuning_server.mavlink.connected,
                'endpoint': self.tuning_server.mavlink.connected_endpoint,
            })
        elif parsed.path == '/api/state':
            self._send_json({
                'ok': True,
                'telemetry': self.tuning_server.telemetry.snapshot(),
                'mavlink': self.tuning_server.mavlink.connected,
                'endpoint': self.tuning_server.mavlink.connected_endpoint,
            })
        elif parsed.path == '/api/params':
            self._get_params(parsed)
        elif parsed.path == '/api/events':
            self._stream_events()
        else:
            self._serve_static(parsed.path)

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if not self._authorized(parsed):
            self._error('unauthorized', HTTPStatus.UNAUTHORIZED)
            return
        body = self._read_json()
        if body is None:
            self._error('invalid JSON body')
            return
        if parsed.path == '/api/params/set':
            self._set_param(body)
        elif parsed.path == '/api/params/save':
            if str(body.get('source', 'firmware')) != 'firmware':
                self._error('Offboard values are persisted on every Apply; use Undo to revert')
                return
            saved = self.tuning_server.mavlink.save()
            if saved:
                self._send_json({'ok': True})
            else:
                self._error('MAVLink is not connected', HTTPStatus.SERVICE_UNAVAILABLE)
        elif parsed.path == '/api/params/revert':
            self._revert_offboard(body)
        elif parsed.path == '/api/mavlink/reconnect':
            connected = self.tuning_server.mavlink.connect(timeout=5.0)
            self._send_json({
                'ok': connected,
                'endpoint': self.tuning_server.mavlink.connected_endpoint,
            }, HTTPStatus.OK if connected else HTTPStatus.SERVICE_UNAVAILABLE)
        else:
            self._error('not found', HTTPStatus.NOT_FOUND)

    def _get_params(self, parsed) -> None:
        query = parse_qs(parsed.query)
        source = query.get('source', ['firmware'])[0]
        group_name = query.get('group', [''])[0]
        if source == 'offboard':
            try:
                values = self.tuning_server.offboard_tuning.group_values(group_name)
            except KeyError as exc:
                self._error(str(exc))
                return
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                self._error(
                    f'cannot read Offboard tuning file: {exc}',
                    HTTPStatus.SERVICE_UNAVAILABLE,
                )
                return
            missing = [name for name, value in values.items() if value is None]
            self._send_json({
                'ok': not missing,
                'source': source,
                'values': values,
                'missing': missing,
                'applied_status': self.tuning_server.offboard_tuning.status(),
            })
            return
        if source != 'firmware':
            self._error('unknown parameter source')
            return
        group = PARAM_GROUPS.get(group_name)
        if group is None:
            self._error('unknown parameter group')
            return
        if not self.tuning_server.mavlink.connected:
            self._error('MAVLink is not connected', HTTPStatus.SERVICE_UNAVAILABLE)
            return
        values = {}
        for name in group:
            value = self.tuning_server.mavlink.request_param(name, timeout=0.8)
            if value is None:
                value = self.tuning_server.mavlink.request_param(name, timeout=1.0)
            values[name] = value
        missing = [name for name, value in values.items() if value is None]
        self._send_json({
            'ok': not missing,
            'source': source,
            'values': values,
            'missing': missing,
        })

    def _set_param(self, body: dict) -> None:
        source = str(body.get('source', 'firmware'))
        name = str(body.get('name', ''))
        cfg = (
            OFFBOARD_PARAM_CONFIG.get(name)
            if source == 'offboard'
            else PARAM_CONFIG.get(name)
        )
        if source not in ('firmware', 'offboard'):
            self._error('unknown parameter source')
            return
        if cfg is None:
            self._error('unknown parameter')
            return
        try:
            value = float(body['value'])
        except (KeyError, TypeError, ValueError):
            self._error('invalid parameter value')
            return
        if not math.isfinite(value) or value < cfg['min'] or value > cfg['max']:
            self._error(f"value must be in [{cfg['min']}, {cfg['max']}]")
            return
        if source == 'offboard':
            mode = self.tuning_server.telemetry.snapshot()['mode']
            if mode.get('armed') and not bool(body.get('armed_confirmed')):
                self._error(
                    'vehicle is armed; explicit confirmation is required',
                    HTTPStatus.CONFLICT,
                )
                return
            try:
                confirmed, revision, _mtime_ns = (
                    self.tuning_server.offboard_tuning.set_value(name, value)
                )
            except KeyError as exc:
                self._error(str(exc))
                return
            except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
                self._error(f'cannot update Offboard tuning file: {exc}')
                return
            applied, status = self.tuning_server.offboard_tuning.wait_applied(revision)
            self._send_json({
                'ok': True,
                'source': source,
                'name': name,
                'requested': value,
                'confirmed': confirmed,
                'persisted': True,
                'applied': applied,
                'applied_status': status,
                'revert_available': True,
            })
            return
        confirmed, observed = self.tuning_server.mavlink.set_and_confirm(name, value)
        if confirmed is None:
            detail = '' if observed is None else f'; PX4 still reports {observed:.6g}'
            self._error(
                f'PX4 did not confirm requested value {value:.6g}{detail}',
                HTTPStatus.GATEWAY_TIMEOUT,
            )
            return
        self._send_json({
            'ok': True,
            'source': source,
            'name': name,
            'requested': value,
            'confirmed': confirmed,
        })

    def _revert_offboard(self, body: dict) -> None:
        if str(body.get('source', 'offboard')) != 'offboard':
            self._error('Undo is only available for Offboard tuning')
            return
        mode = self.tuning_server.telemetry.snapshot()['mode']
        if mode.get('armed') and not bool(body.get('armed_confirmed')):
            self._error(
                'vehicle is armed; explicit confirmation is required',
                HTTPStatus.CONFLICT,
            )
            return
        try:
            revision, _mtime_ns = self.tuning_server.offboard_tuning.revert()
        except LookupError as exc:
            self._error(str(exc), HTTPStatus.CONFLICT)
            return
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self._error(f'cannot undo Offboard tuning change: {exc}')
            return
        applied, status = self.tuning_server.offboard_tuning.wait_applied(revision)
        self._send_json({
            'ok': True,
            'source': 'offboard',
            'persisted': True,
            'applied': applied,
            'applied_status': status,
            'revert_available': bool(self.tuning_server.offboard_tuning.history),
        })

    def _stream_events(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.send_header('X-Accel-Buffering', 'no')
        self.end_headers()
        try:
            while not self.tuning_server.stop_event.is_set():
                payload = {
                    'telemetry': self.tuning_server.telemetry.snapshot(),
                    'mavlink': self.tuning_server.mavlink.connected,
                    'endpoint': self.tuning_server.mavlink.connected_endpoint,
                }
                message = f"event: telemetry\ndata: {json.dumps(payload, separators=(',', ':'), allow_nan=False)}\n\n"
                self.wfile.write(message.encode('utf-8'))
                self.wfile.flush()
                self.tuning_server.stop_event.wait(self.tuning_server.stream_period)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def _serve_static(self, request_path: str) -> None:
        relative = 'index.html' if request_path in ('', '/') else request_path.lstrip('/')
        if relative not in ('index.html', 'app.js', 'style.css'):
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        path = self.tuning_server.static_dir / relative
        try:
            data = path.read_bytes()
        except OSError:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content_type = mimetypes.guess_type(path.name)[0] or 'application/octet-stream'
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', f'{content_type}; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)


def spin_ros(node: HnuterTelemetry, stop_event: threading.Event) -> None:
    while rclpy.ok() and not stop_event.is_set():
        try:
            rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:  # noqa: BLE001
            if stop_event.is_set() or not rclpy.ok():
                break
            raise


def local_addresses(port: int) -> list[str]:
    import socket
    addresses = {'127.0.0.1'}
    try:
        for item in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            addresses.add(item[4][0])
    except OSError:
        pass
    return [f'http://{address}:{port}' for address in sorted(addresses)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Hnuter LAN web tuning dashboard')
    parser.add_argument('--host', default='0.0.0.0', help='HTTP bind address')
    parser.add_argument('--port', type=int, default=8765, help='HTTP port')
    parser.add_argument('--stream-hz', type=float, default=15.0, help='browser telemetry rate')
    parser.add_argument('--csv-rate-hz', type=float, default=25.0, help='CSV logging rate')
    parser.add_argument('--csv', type=Path, default=None, help='CSV output path')
    parser.add_argument('--mavlink', default=os.environ.get('HNUTER_MAVLINK', 'auto'))
    parser.add_argument(
        '--offboard-tuning',
        type=Path,
        default=Path(__file__).resolve().parent / 'config' / 'hnuter_direct_hardware_tuning.json',
        help='Hardware Offboard live-tuning JSON path',
    )
    parser.add_argument('--token', default=os.environ.get('HNUTER_WEB_TOKEN', ''),
                        help='optional LAN access token')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    static_dir = Path(__file__).resolve().parent / 'hnuter_tuning_web'
    if not (static_dir / 'index.html').is_file():
        print(f'Web assets not found: {static_dir}', file=sys.stderr)
        return 2

    csv_path = args.csv
    if csv_path is None:
        stamp = time.strftime('%Y%m%d_%H%M%S')
        csv_path = Path.home() / 'px4_ws_ros2' / 'hnuter_saved_plots' / f'hnuter_web_tuning_{stamp}.csv'

    stop_event = threading.Event()
    rclpy.init()
    telemetry = HnuterTelemetry()
    mavlink = MavlinkParamClient(args.mavlink)
    mavlink.connect(timeout=5.0)
    offboard_tuning = OffboardTuningStore(args.offboard_tuning)
    ros_thread = threading.Thread(target=spin_ros, args=(telemetry, stop_event), name='ros-spin', daemon=True)
    ros_thread.start()
    recorder = CsvRecorder(telemetry, csv_path, args.csv_rate_hz, stop_event)
    recorder.start()

    server = TuningHttpServer(
        (args.host, args.port),
        TuningRequestHandler,
        telemetry,
        mavlink,
        offboard_tuning,
        static_dir,
        args.stream_hz,
        args.token,
        stop_event,
    )

    def request_shutdown(_signum=None, _frame=None) -> None:
        if stop_event.is_set():
            return
        stop_event.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    print(f'[CSV] {csv_path}')
    print(f'[OFFBOARD] live tuning: {offboard_tuning.path}')
    print('[WEB] Hnuter tuning dashboard:')
    for address in local_addresses(args.port):
        suffix = f'?token={args.token}' if args.token else ''
        print(f'  {address}{suffix}')
    if not args.token:
        print('[WEB] No access token configured; use only on a trusted LAN.')

    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        stop_event.set()
        server.server_close()
        ros_thread.join(timeout=1.0)
        recorder.thread.join(timeout=1.0)
        mavlink.close()
        telemetry.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
