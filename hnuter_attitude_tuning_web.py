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
from px4_msgs.msg import DebugVect
from px4_msgs.msg import VehicleAngularVelocity
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



def split_prefixes(raw: str) -> list[str]:
    prefixes = []
    for item in raw.split(','):
        item = item.strip()
        if not item:
            continue
        if item.lower() in ('*', 'all'):
            return []
        prefixes.append(item)
    return prefixes or ['HNTR_']


def param_group_name(name: str, prefixes: list[str]) -> str:
    upper = name.upper()
    if upper.startswith('HNTR_'):
        if ('ATT_' in upper or upper.startswith('HNTR_TAU_') or upper.startswith('HNTR_RC_')):
            if upper.endswith('_R') or '_ROLL' in upper:
                return 'Attitude Roll'
            if upper.endswith('_P') or 'PITCH' in upper or 'TAIL' in upper:
                return 'Attitude Pitch'
            if upper.endswith('_Y') or 'YAW' in upper:
                return 'Attitude Yaw'
            return 'Attitude General'
        if upper.startswith(('HNTR_POS_', 'HNTR_VEL_', 'HNTR_ACC_')):
            if upper.endswith('_Z') or upper.endswith('_UP') or upper.endswith('_DN'):
                return 'Position Z'
            if upper.endswith('_XY') or upper.endswith('_X') or upper.endswith('_Y'):
                return 'Position XY'
            return 'Position General'
        if upper.startswith('HNTR_STAB_'):
            return 'Stabilized Height'
        if upper.startswith(('HNTR_TO_', 'HNTR_LOCK_', 'HNTR_LND_')):
            return 'Takeoff And Landing'
        if upper.startswith(('HNTR_T1_', 'HNTR_T2_', 'HNTR_SYNC_', 'HNTR_TILT')):
            return 'Tilt Dynamics'
        if upper.startswith(('HNTR_MASS', 'HNTR_MAX_', 'HNTR_L1', 'HNTR_L2', 'HNTR_MOT_')):
            return 'Vehicle Model'
        if upper.startswith('HNTR_CTRL_') or 'SIGN' in upper or 'COMP' in upper:
            return 'Allocator'

    for prefix in prefixes:
        if name.startswith(prefix):
            suffix = name[len(prefix):].strip('_')
            token = suffix.split('_', 1)[0] if suffix else prefix.rstrip('_')
            return f'{prefix}{token}' if token else prefix.rstrip('_')
    return name.split('_', 1)[0] if '_' in name else 'Other'


def _dynamic_param_limits(name: str, default: float, is_integer: bool) -> tuple[float, float, float]:
    upper = name.upper()
    abs_default = abs(default) if math.isfinite(default) else 0.0

    if is_integer:
        if 'SIGN' in upper or upper.endswith('_REV') or '_REV_' in upper:
            return -1.0, 1.0, 1.0
        if upper.endswith('_EN') or upper.endswith('_ENABLE') or upper.endswith('_MODE'):
            return 0.0, 10.0, 1.0
        return 0.0, max(100.0, abs_default * 5.0), 1.0

    signed_tokens = ('BIAS', 'TRIM', 'OFFSET', '_OFF', 'ERR', 'SIGN', 'CENTER')
    signed = any(token in upper for token in signed_tokens)

    if any(token in upper for token in ('ANGLE', 'MAXA', 'MINA', 'TILT')):
        return (-185.0 if signed else 0.0), 185.0, 0.1
    if any(token in upper for token in ('PITCH_BIAS', 'TAIL_BIAS')):
        return -1.0, 1.0, 0.001
    if any(token in upper for token in ('TAU', 'TC', 'TIME', 'DELAY')):
        return 0.0, max(5.0, abs_default * 5.0), 0.001
    if any(token in upper for token in ('LIM', 'LIMIT', 'MAX', 'MIN', 'MASS', 'GAIN', '_P_', '_I_', '_D_', 'THR', 'VEL', 'ACC', 'RATE')):
        return 0.0, max(10.0, abs_default * 5.0), 0.001

    limit = min(max(10.0, max(abs_default, 1.0) * 5.0), 100000.0)
    return (-limit if signed else 0.0), limit, 0.001


def build_dynamic_param_config(name: str, value: float, param_type: int, is_integer: bool) -> dict:
    default = 0.0 if value is None or not math.isfinite(float(value)) else float(value)
    low, high, step = _dynamic_param_limits(name, default, is_integer)
    return {
        'min': low,
        'max': high,
        'step': step,
        'default': default,
        'type': 'integer' if is_integer else 'float',
        'dynamic': True,
        'param_type': int(param_type),
    }

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
        self.catalog = {}
        self.catalog_time = 0.0
        self.catalog_param_count = 0

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
        self.catalog = {}
        self.catalog_time = 0.0
        self.catalog_param_count = 0

    @staticmethod
    def _param_name(msg) -> str:
        param_id = msg.param_id
        if isinstance(param_id, bytes):
            param_id = param_id.decode('utf-8', errors='ignore')
        return str(param_id).strip('\x00')

    @staticmethod
    def _integer_param_types() -> set[int]:
        if mavutil is None:
            return set()
        names = (
            'MAV_PARAM_TYPE_UINT8', 'MAV_PARAM_TYPE_INT8',
            'MAV_PARAM_TYPE_UINT16', 'MAV_PARAM_TYPE_INT16',
            'MAV_PARAM_TYPE_UINT32', 'MAV_PARAM_TYPE_INT32',
            'MAV_PARAM_TYPE_UINT64', 'MAV_PARAM_TYPE_INT64',
        )
        return {int(getattr(mavutil.mavlink, name)) for name in names if hasattr(mavutil.mavlink, name)}

    @classmethod
    def _is_integer_type(cls, param_type: int) -> bool:
        return int(param_type) in cls._integer_param_types()

    @classmethod
    def _decode_param_value(cls, msg) -> float:
        if cls._is_integer_type(int(msg.param_type)):
            packed = struct.pack('<f', float(msg.param_value))
            return float(struct.unpack('<i', packed)[0])
        return float(msg.param_value)

    def _wire_value(self, name: str, value: float) -> tuple[float, int]:
        meta = self.catalog.get(name, {})
        param_type = int(meta.get('param_type', mavutil.mavlink.MAV_PARAM_TYPE_REAL32))
        if self._is_integer_type(param_type):
            packed = struct.pack('<i', int(round(value)))
            return struct.unpack('<f', packed)[0], mavutil.mavlink.MAV_PARAM_TYPE_INT32
        return float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32

    def _drain_param_values(self, limit: int = 1024) -> None:
        for _ in range(limit):
            if self.master.recv_match(type='PARAM_VALUE', blocking=False) is None:
                break

    def request_all_params(self, timeout: float = 12.0, idle_timeout: float = 1.2) -> dict:
        if self.master is None:
            return {}
        with self.lock:
            try:
                self._drain_param_values(limit=4096)
                self.master.mav.param_request_list_send(
                    self.master.target_system,
                    self.master.target_component,
                )
                params = {}
                expected_count = None
                deadline = time.monotonic() + max(timeout, 1.0)
                idle_deadline = time.monotonic() + max(idle_timeout, 0.2)
                while time.monotonic() < deadline and time.monotonic() < idle_deadline:
                    msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.2)
                    if msg is None:
                        continue
                    name = self._param_name(msg)
                    if not name:
                        continue
                    value = self._decode_param_value(msg)
                    param_type = int(msg.param_type)
                    index = int(getattr(msg, 'param_index', -1))
                    count = int(getattr(msg, 'param_count', 0))
                    if count > 0:
                        expected_count = max(expected_count or 0, count)
                    params[name] = {
                        'value': value,
                        'param_type': param_type,
                        'index': index,
                        'count': count,
                        'is_integer': self._is_integer_type(param_type),
                    }
                    idle_deadline = time.monotonic() + max(idle_timeout, 0.2)
                    if expected_count is not None and len(params) >= expected_count:
                        break
                if params:
                    merged = dict(self.catalog)
                    merged.update(params)
                    self.catalog = merged
                self.catalog_time = time.monotonic()
                self.catalog_param_count = expected_count or max(len(self.catalog), len(params))
                print(
                    f'[PARAM_LIST] discovered {len(params)}/{self.catalog_param_count or "?"} parameters; '
                    f'catalog has {len(self.catalog)}'
                )
                return dict(self.catalog)
            except Exception as exc:  # noqa: BLE001
                print(f'[PARAM_LIST] failed: {exc}')
                return dict(self.catalog)

    def dynamic_groups(self, prefixes: list[str], force: bool = False) -> dict:
        if self.master is None:
            return {}
        if force or not self.catalog or time.monotonic() - self.catalog_time > 30.0:
            self.request_all_params()
        groups = {}
        for name, meta in sorted(self.catalog.items()):
            if prefixes and not any(name.startswith(prefix) for prefix in prefixes):
                continue
            cfg = build_dynamic_param_config(
                name,
                float(meta.get('value', 0.0)),
                int(meta.get('param_type', 0)),
                bool(meta.get('is_integer', False)),
            )
            group_name = param_group_name(name, prefixes)
            groups.setdefault(group_name, {})[name] = cfg
        if groups:
            all_group = {}
            for group in groups.values():
                all_group.update(group)
            title = 'All discovered' if not prefixes else 'All ' + ','.join(prefixes)
            return {title: dict(sorted(all_group.items())), **dict(sorted(groups.items()))}
        return {}

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
                expected = float(round(value)) if self._is_integer_type(param_type) else float(value)
                tolerance = 0.0 if self._is_integer_type(param_type) else max(1e-5, abs(expected) * 1e-6)
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
                            if name in self.catalog:
                                self.catalog[name]['value'] = last_observed
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


class HnuterTelemetry(Node):
    TOPICS = (
        'attitude', 'setpoint', 'angular_velocity',
        'position', 'position_setpoint', 'velocity', 'velocity_setpoint',
        'torque', 'motors', 'mode',
    )

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
        self.standard_setpoint = (math.nan, math.nan, math.nan)
        self.hnuter_setpoint = (math.nan, math.nan, math.nan)
        self.hnuter_setpoint_time = None
        self.position = (math.nan, math.nan, math.nan)
        self.position_setpoint = (math.nan, math.nan, math.nan)
        self.velocity = (math.nan, math.nan, math.nan)
        self.velocity_setpoint = (math.nan, math.nan, math.nan)
        self.angular_velocity = (math.nan, math.nan, math.nan)
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
        self.create_subscription(DebugVect, '/fmu/out/debug_vect', self.on_debug_vect, qos)
        self.create_subscription(
            VehicleAngularVelocity,
            '/fmu/out/vehicle_angular_velocity',
            self.on_angular_velocity,
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

    @staticmethod
    def _debug_name(raw) -> str:
        if isinstance(raw, str):
            return raw.strip('\x00')
        try:
            return ''.join(chr(int(value)) for value in raw if int(value) != 0)
        except Exception:  # noqa: BLE001
            return str(raw).strip('\x00')

    def on_setpoint(self, msg: VehicleAttitudeSetpoint) -> None:
        with self.lock:
            self.standard_setpoint = quat_to_euler(msg.q_d)
            self.topic_time['setpoint'] = time.monotonic()

    def on_debug_vect(self, msg: DebugVect) -> None:
        if self._debug_name(msg.name) != 'hntr_att':
            return
        with self.lock:
            self.hnuter_setpoint = (math.radians(float(msg.x)), math.radians(float(msg.y)), math.radians(float(msg.z)))
            self.hnuter_setpoint_time = time.monotonic()
            self.topic_time['setpoint'] = self.hnuter_setpoint_time

    def on_angular_velocity(self, msg: VehicleAngularVelocity) -> None:
        with self.lock:
            self.angular_velocity = tuple(float(value) for value in msg.xyz)
            self.topic_time['angular_velocity'] = time.monotonic()

    def on_torque(self, msg: VehicleTorqueSetpoint) -> None:
        with self.lock:
            self.torque = tuple(float(value) for value in msg.xyz)
            self.topic_time['torque'] = time.monotonic()

    def on_position(self, msg: VehicleLocalPosition) -> None:
        with self.lock:
            self.position = (float(msg.x), float(msg.y), float(msg.z))
            self.velocity = (float(msg.vx), float(msg.vy), float(msg.vz))
            self.topic_time['position'] = time.monotonic()
            self.topic_time['velocity'] = time.monotonic()

    def on_position_setpoint(self, msg: VehicleLocalPositionSetpoint) -> None:
        with self.lock:
            self.position_setpoint = (float(msg.x), float(msg.y), float(msg.z))
            self.velocity_setpoint = (float(msg.vx), float(msg.vy), float(msg.vz))
            self.topic_time['position_setpoint'] = time.monotonic()
            self.topic_time['velocity_setpoint'] = time.monotonic()

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
            now_locked = time.monotonic()
            hnuter_setpoint_fresh = self.hnuter_setpoint_time is not None and (now_locked - self.hnuter_setpoint_time) < 0.5
            setpoint = tuple(self.hnuter_setpoint if hnuter_setpoint_fresh else self.standard_setpoint)
            setpoint_source = 'hnuter' if hnuter_setpoint_fresh else 'px4'
            position = tuple(self.position)
            position_setpoint = tuple(self.position_setpoint)
            velocity = tuple(self.velocity)
            velocity_setpoint = tuple(self.velocity_setpoint)
            angular_velocity = tuple(self.angular_velocity)
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
        angular_velocity_deg = [finite(value * scale) for value in angular_velocity]
        position_values = [finite(value) for value in position]
        position_setpoint_values = [finite(value) for value in position_setpoint]
        velocity_values = [finite(value) for value in velocity]
        velocity_setpoint_values = [finite(value) for value in velocity_setpoint]
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
            'setpoint_source': setpoint_source,
            'error': errors,
            'angular_velocity': angular_velocity_deg,
            'position': position_values,
            'position_setpoint': position_setpoint_values,
            'velocity': velocity_values,
            'velocity_setpoint': velocity_setpoint_values,
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
        'roll_rate_deg_s', 'pitch_rate_deg_s', 'yaw_rate_deg_s',
        'north_m', 'east_m', 'down_m',
        'north_sp_m', 'east_sp_m', 'down_sp_m',
        'north_vel_m_s', 'east_vel_m_s', 'down_vel_m_s',
        'north_vel_sp_m_s', 'east_vel_sp_m_s', 'down_vel_sp_m_s',
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
                    'roll_rate_deg_s': sample['angular_velocity'][0],
                    'pitch_rate_deg_s': sample['angular_velocity'][1],
                    'yaw_rate_deg_s': sample['angular_velocity'][2],
                    'north_m': sample['position'][0],
                    'east_m': sample['position'][1],
                    'down_m': sample['position'][2],
                    'north_sp_m': sample['position_setpoint'][0],
                    'east_sp_m': sample['position_setpoint'][1],
                    'down_sp_m': sample['position_setpoint'][2],
                    'north_vel_m_s': sample['velocity'][0],
                    'east_vel_m_s': sample['velocity'][1],
                    'down_vel_m_s': sample['velocity'][2],
                    'north_vel_sp_m_s': sample['velocity_setpoint'][0],
                    'east_vel_sp_m_s': sample['velocity_setpoint'][1],
                    'down_vel_sp_m_s': sample['velocity_setpoint'][2],
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
        static_dir: Path,
        stream_hz: float,
        token: str,
        stop_event: threading.Event,
        param_prefixes: list[str],
    ):
        super().__init__(address, handler)
        self.telemetry = telemetry
        self.mavlink = mavlink
        self.static_dir = static_dir
        self.stream_period = 1.0 / max(stream_hz, 1.0)
        self.token = token
        self.stop_event = stop_event
        self.param_prefixes = param_prefixes
        self.param_groups_cache = {}


class TuningRequestHandler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server_version = 'HnuterTuning/1.1-dynamic-params'

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
            force = parse_qs(parsed.query).get('refresh', ['0'])[0] in ('1', 'true', 'yes')
            groups = self.tuning_server.mavlink.dynamic_groups(self.tuning_server.param_prefixes, force=force)
            if groups:
                self.tuning_server.param_groups_cache = groups
            self._send_json({
                'ok': True,
                'groups': groups,
                'dynamic_params': True,
                'param_prefixes': self.tuning_server.param_prefixes,
                'param_count': len(self.tuning_server.mavlink.catalog),
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
            saved = self.tuning_server.mavlink.save()
            if saved:
                self._send_json({'ok': True})
            else:
                self._error('MAVLink is not connected', HTTPStatus.SERVICE_UNAVAILABLE)
        elif parsed.path == '/api/mavlink/reconnect':
            connected = self.tuning_server.mavlink.connect(timeout=5.0)
            self._send_json({
                'ok': connected,
                'endpoint': self.tuning_server.mavlink.connected_endpoint,
            }, HTTPStatus.OK if connected else HTTPStatus.SERVICE_UNAVAILABLE)
        else:
            self._error('not found', HTTPStatus.NOT_FOUND)

    def _get_params(self, parsed) -> None:
        group_name = parse_qs(parsed.query).get('group', [''])[0]
        if not self.tuning_server.mavlink.connected:
            self._error('MAVLink is not connected', HTTPStatus.SERVICE_UNAVAILABLE)
            return
        groups = self.tuning_server.param_groups_cache or self.tuning_server.mavlink.dynamic_groups(self.tuning_server.param_prefixes)
        group = groups.get(group_name)
        if group is None:
            groups = self.tuning_server.mavlink.dynamic_groups(self.tuning_server.param_prefixes, force=True)
            if groups:
                self.tuning_server.param_groups_cache = groups
            group = groups.get(group_name) or self.tuning_server.param_groups_cache.get(group_name)
        if group is None:
            available = ', '.join(sorted(groups.keys() or self.tuning_server.param_groups_cache.keys()))
            suffix = f'; available: {available}' if available else ''
            self._error(f'unknown parameter group: {group_name}{suffix}')
            return
        query = parse_qs(parsed.query)
        force_refresh = query.get('refresh', ['0'])[0] in ('1', 'true', 'yes')
        values = {}
        missing = []

        # The dynamic parameter catalog already comes from PX4 PARAM_VALUE replies.
        # Use it as the fast path; per-parameter reads are slow and can make the
        # browser look stuck on large groups if one reply is missed.
        if force_refresh:
            self.tuning_server.mavlink.request_all_params(timeout=8.0, idle_timeout=0.8)

        for name, cfg in group.items():
            meta = self.tuning_server.mavlink.catalog.get(name)
            value = meta.get('value') if meta is not None else None

            if value is None:
                value = self.tuning_server.mavlink.request_param(name, timeout=0.25)

            if value is None and isinstance(cfg, dict):
                # Last-resort fallback to the value discovered when the group was
                # built. This prevents a single partial MAVLink catalog refresh
                # from making already-known parameters disappear from the UI.
                value = cfg.get('default')

            if value is None:
                missing.append(name)

            values[name] = value

        self._send_json({
            'ok': not missing,
            'values': values,
            'missing': missing,
            'source': 'catalog-refresh' if force_refresh else 'catalog',
        })

    def _set_param(self, body: dict) -> None:
        name = str(body.get('name', ''))
        if not self.tuning_server.mavlink.connected:
            self._error('MAVLink is not connected', HTTPStatus.SERVICE_UNAVAILABLE)
            return
        groups = self.tuning_server.mavlink.dynamic_groups(self.tuning_server.param_prefixes)
        cfg = None
        for group in groups.values():
            if name in group:
                cfg = group[name]
                break
        if cfg is None:
            groups = self.tuning_server.mavlink.dynamic_groups(self.tuning_server.param_prefixes, force=True)
            if groups:
                self.tuning_server.param_groups_cache = groups
            for group in groups.values():
                if name in group:
                    cfg = group[name]
                    break
        if cfg is None:
            for group in self.tuning_server.param_groups_cache.values():
                if name in group:
                    cfg = group[name]
                    break
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
        confirmed, observed = self.tuning_server.mavlink.set_and_confirm(name, value)
        if confirmed is None:
            detail = '' if observed is None else f'; PX4 still reports {observed:.6g}'
            self._error(
                f'PX4 did not confirm requested value {value:.6g}{detail}',
                HTTPStatus.GATEWAY_TIMEOUT,
            )
            return
        self._send_json({'ok': True, 'name': name, 'requested': value, 'confirmed': confirmed})

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
        '--param-prefix',
        default=os.environ.get('HNUTER_PARAM_PREFIX', 'HNTR_'),
        help="comma-separated parameter prefixes to discover; use 'all' for every PX4 parameter",
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
    ros_thread = threading.Thread(target=spin_ros, args=(telemetry, stop_event), name='ros-spin', daemon=True)
    ros_thread.start()
    recorder = CsvRecorder(telemetry, csv_path, args.csv_rate_hz, stop_event)
    recorder.start()

    server = TuningHttpServer(
        (args.host, args.port),
        TuningRequestHandler,
        telemetry,
        mavlink,
        static_dir,
        args.stream_hz,
        args.token,
        stop_event,
        split_prefixes(args.param_prefix),
    )

    def request_shutdown(_signum=None, _frame=None) -> None:
        if stop_event.is_set():
            return
        stop_event.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    print(f'[CSV] {csv_path}')
    print('[WEB] Hnuter tuning dashboard:')
    for address in local_addresses(args.port):
        suffix = f'?token={args.token}' if args.token else ''
        print(f'  {address}{suffix}')
    print(f"[DDS] discovery range: {os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SYSTEM_DEFAULT')}")
    print('[DDS] If the page shows DDS 0/10 with a remote Agent, restart with HNUTER_ALLOW_REMOTE_DDS=1.')
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
