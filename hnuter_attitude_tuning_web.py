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
        'HNTR_XY_P': param(0.0, 20.0, 0.1, 2.5),
        'HNTR_XY_D': param(0.0, 30.0, 0.1, 1.8),
        'HNTR_XY_I': param(0.0, 10.0, 0.01, 0.0),
        'HNTR_ACC_XY': param(0.1, 100.0, 0.5, 20.0),
        'HNTR_TILT_MAX': param(0.0, 185.0, 1.0, 185.0),
    },
    'Position Z': {
        'HNTR_HOV_THR': param(0.05, 0.95, 0.01, 0.65),
        'HNTR_Z_P': param(0.0, 30.0, 0.1, 8.0),
        'HNTR_Z_D': param(0.0, 30.0, 0.1, 4.0),
        'HNTR_Z_I': param(0.0, 20.0, 0.1, 3.0),
        'HNTR_ACC_Z': param(0.1, 100.0, 0.5, 65.0),
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
                        return float(msg.param_value)
            except Exception as exc:  # noqa: BLE001
                print(f'[PARAM_GET] failed {name}: {exc}')
        return None

    def set_and_confirm(self, name: str, value: float, timeout: float = 1.5) -> Optional[float]:
        if self.master is None:
            return None
        with self.lock:
            try:
                self.master.mav.param_set_send(
                    self.master.target_system,
                    self.master.target_component,
                    name.encode('utf-8'),
                    float(value),
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                )
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.1)
                    if msg is not None and self._param_name(msg) == name:
                        confirmed = float(msg.param_value)
                        print(f'[PARAM_SET] {name} requested={value:.6g} confirmed={confirmed:.6g}')
                        return confirmed
            except Exception as exc:  # noqa: BLE001
                print(f'[PARAM_SET] failed {name}: {exc}')
        return None

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
        static_dir: Path,
        stream_hz: float,
        token: str,
        stop_event: threading.Event,
    ):
        super().__init__(address, handler)
        self.telemetry = telemetry
        self.mavlink = mavlink
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
                'groups': PARAM_GROUPS,
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
        self._send_json({'ok': not missing, 'values': values, 'missing': missing})

    def _set_param(self, body: dict) -> None:
        name = str(body.get('name', ''))
        cfg = PARAM_CONFIG.get(name)
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
        confirmed = self.tuning_server.mavlink.set_and_confirm(name, value)
        if confirmed is None:
            self._error('PX4 did not confirm the parameter', HTTPStatus.GATEWAY_TIMEOUT)
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
