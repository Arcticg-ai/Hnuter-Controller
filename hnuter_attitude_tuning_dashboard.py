#!/usr/bin/env python3
"""
Hnuter attitude tuning dashboard.

Live DDS plots:
- roll/pitch/yaw actual vs setpoint and tracking error
- vehicle torque setpoint Tx/Ty/Tz
- Motor1-5 normalized allocator commands, highlighting Motor5

Online tuning covers the complete Hnuter controller/allocator parameter set in
paged groups. Physical/model parameters are included but should only be changed
when the vehicle model is known to be wrong.

Parameter writes use MAVLink PARAM_SET via pymavlink. State monitoring uses ROS2/DDS.
Run inside the px4 venv/workspace, for example:

    source ~/PX4-Autopilot-Hnuter/px4-venv/bin/activate
    cd ~/px4_ws_ros2
    python3 hnuter_attitude_tuning_dashboard.py --mavlink udp:127.0.0.1:14550

Keyboard/terminal commands while running:
    set HNTR_PITCH_BIAS 0.084
    get HNTR_PITCH_BIAS
    save
    snapshot
    q
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import queue
import site
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

# Keep ROS2 discovery local by default so a real PX4 on LAN does not leak in.
if os.environ.get('HNUTER_ALLOW_REMOTE_DDS', '0') != '1':
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_STATIC_PEERS', None)

import numpy as np

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-hnuter')

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleControlMode
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


@dataclass
class Sample:
    t: float
    roll: float = math.nan
    pitch: float = math.nan
    yaw: float = math.nan
    roll_sp: float = math.nan
    pitch_sp: float = math.nan
    yaw_sp: float = math.nan
    tx: float = math.nan
    ty: float = math.nan
    tz: float = math.nan
    m1: float = math.nan
    m2: float = math.nan
    m3: float = math.nan
    m4: float = math.nan
    m5: float = math.nan
    armed: int = 0
    posctl: int = 0
    offboard: int = 0


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_euler(q: Iterable[float]) -> Tuple[float, float, float]:
    w, x, y, z = [float(v) for v in q]
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def finite_or_nan(value: float) -> float:
    value = float(value)
    return value if math.isfinite(value) else math.nan


def unwrap_finite(values: np.ndarray) -> np.ndarray:
    """Unwrap finite angles without letting an initial missing sample poison the trace."""
    result = np.array(values, dtype=float, copy=True)
    finite = np.isfinite(result)
    if finite.any():
        result[finite] = np.unwrap(result[finite])
    return result


class MavlinkParamClient:
    def __init__(self, endpoint: str, source_system: int = 250, source_component: int = 190):
        self.endpoint = endpoint
        self.endpoints = self._expand_endpoints(endpoint)
        self.source_system = source_system
        self.source_component = source_component
        self.master = None
        self.lock = threading.Lock()
        self.enabled = mavutil is not None and endpoint.lower() != 'none'

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

    def connect(self, timeout: float = 8.0) -> bool:
        if not self.enabled:
            print('[MAVLink] disabled or pymavlink unavailable; sliders will be local-only.')
            return False

        per_endpoint_timeout = max(1.0, timeout / max(len(self.endpoints), 1))
        for endpoint in self.endpoints:
            try:
                print(f'[MAVLink] connecting {endpoint} ...')
                self.master = mavutil.mavlink_connection(
                    endpoint,
                    source_system=self.source_system,
                    source_component=self.source_component,
                    autoreconnect=True,
                    robust_parsing=True,
                )
                hb = self.master.wait_heartbeat(timeout=per_endpoint_timeout)
                if hb is None:
                    print(f'[MAVLink] heartbeat timeout on {endpoint}')
                    self.master.close()
                    self.master = None
                    continue
                print(f'[MAVLink] heartbeat from system={self.master.target_system} component={self.master.target_component} via {endpoint}')
                return True
            except Exception as exc:  # noqa: BLE001
                print(f'[MAVLink] connection failed on {endpoint}: {exc}')
                try:
                    if self.master is not None:
                        self.master.close()
                except Exception:
                    pass
                self.master = None
        print('[MAVLink] no heartbeat; sliders will still move but PARAM_SET will fail until endpoint is fixed.')
        return False

    def set_param(self, name: str, value: float) -> bool:
        if self.master is None:
            print(f'[MAVLink] no connection, cannot set {name}={value}')
            return False

        try:
            with self.lock:
                self.master.mav.param_set_send(
                    self.master.target_system,
                    self.master.target_component,
                    name.encode('utf-8'),
                    float(value),
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                )
            print(f'[PARAM_SET] {name}={float(value):.6g}')
            return True
        except Exception as exc:  # noqa: BLE001
            print(f'[PARAM_SET] failed {name}: {exc}')
            return False

    def request_param(self, name: str, timeout: float = 1.0) -> Optional[float]:
        if self.master is None:
            return None
        try:
            with self.lock:
                self.master.mav.param_request_read_send(
                    self.master.target_system,
                    self.master.target_component,
                    name.encode('utf-8'),
                    -1,
                )
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.1)
                    if msg is None:
                        continue
                    param_id = msg.param_id
                    if isinstance(param_id, bytes):
                        param_id = param_id.decode('utf-8', errors='ignore')
                    param_id = str(param_id).strip('\x00')
                    if param_id == name:
                        return float(msg.param_value)
        except Exception as exc:  # noqa: BLE001
            print(f'[PARAM_GET] failed {name}: {exc}')
        return None

    def save(self) -> bool:
        if self.master is None:
            print('[MAVLink] no connection, cannot save parameters')
            return False
        try:
            with self.lock:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
                    0,
                    1.0,  # write current params to storage
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                )
            print('[PARAM_SAVE] requested')
            return True
        except Exception as exc:  # noqa: BLE001
            print(f'[PARAM_SAVE] failed: {exc}')
            return False


class HnuterAttitudeMonitor(Node):
    def __init__(self, history_s: float, csv_path: Path, csv_rate_hz: float):
        super().__init__('hnuter_attitude_tuning_dashboard')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.start_wall = time.monotonic()
        self.history_s = float(history_s)
        self.samples: deque[Sample] = deque(maxlen=max(1000, int(history_s * 80)))
        self.lock = threading.Lock()
        self.last_attitude = (math.nan, math.nan, math.nan)
        self.last_attitude_sp = (math.nan, math.nan, math.nan)
        self.last_torque = (math.nan, math.nan, math.nan)
        self.last_outputs = [math.nan] * 16
        self.last_mode = {'armed': 0, 'posctl': 0, 'offboard': 0}
        self.last_csv_t = 0.0
        self.csv_period = 1.0 / max(float(csv_rate_hz), 1e-3)
        self.csv_path = csv_path
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open('w', newline='', encoding='utf-8')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=[
            't', 'roll_deg', 'pitch_deg', 'yaw_deg',
            'roll_sp_deg', 'pitch_sp_deg', 'yaw_sp_deg',
            'roll_err_deg', 'pitch_err_deg', 'yaw_err_deg',
            'tx', 'ty', 'tz', 'main1', 'main2', 'main3', 'main4', 'main5',
            'armed', 'posctl', 'offboard',
        ])
        self.csv_writer.writeheader()

        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.on_attitude, qos)
        # VehicleAttitudeSetpoint is versioned by the PX4 DDS bridge, which
        # exposes publication topics with the generated _v1 suffix.
        self.create_subscription(VehicleAttitudeSetpoint, '/fmu/out/vehicle_attitude_setpoint_v1', self.on_attitude_sp, qos)
        self.create_subscription(VehicleTorqueSetpoint, '/fmu/out/vehicle_torque_setpoint', self.on_torque, qos)
        self.create_subscription(ActuatorMotors, '/fmu/out/actuator_motors', self.on_outputs, qos)
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.on_mode, qos)

    def now_s(self) -> float:
        return time.monotonic() - self.start_wall

    def on_attitude(self, msg: VehicleAttitude) -> None:
        self.last_attitude = quat_to_euler(msg.q)
        self.add_sample()

    def on_attitude_sp(self, msg: VehicleAttitudeSetpoint) -> None:
        self.last_attitude_sp = quat_to_euler(msg.q_d)

    def on_torque(self, msg: VehicleTorqueSetpoint) -> None:
        self.last_torque = tuple(finite_or_nan(v) for v in msg.xyz)

    def on_outputs(self, msg: ActuatorMotors) -> None:
        self.last_outputs = [finite_or_nan(v) for v in msg.control]

    def on_mode(self, msg: VehicleControlMode) -> None:
        self.last_mode = {
            'armed': int(bool(msg.flag_armed)),
            'posctl': int(bool(msg.flag_control_position_enabled)),
            'offboard': int(bool(msg.flag_control_offboard_enabled)),
        }

    def add_sample(self) -> None:
        t = self.now_s()
        roll, pitch, yaw = self.last_attitude
        roll_sp, pitch_sp, yaw_sp = self.last_attitude_sp
        tx, ty, tz = self.last_torque
        outputs = self.last_outputs
        sample = Sample(
            t=t,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            roll_sp=roll_sp,
            pitch_sp=pitch_sp,
            yaw_sp=yaw_sp,
            tx=tx,
            ty=ty,
            tz=tz,
            m1=outputs[0] if len(outputs) > 0 else math.nan,
            m2=outputs[1] if len(outputs) > 1 else math.nan,
            m3=outputs[2] if len(outputs) > 2 else math.nan,
            m4=outputs[3] if len(outputs) > 3 else math.nan,
            m5=outputs[4] if len(outputs) > 4 else math.nan,
            armed=self.last_mode['armed'],
            posctl=self.last_mode['posctl'],
            offboard=self.last_mode['offboard'],
        )
        with self.lock:
            self.samples.append(sample)
        if t - self.last_csv_t >= self.csv_period:
            self.write_csv_sample(sample)
            self.last_csv_t = t

    def write_csv_sample(self, s: Sample) -> None:
        roll_err = wrap_pi(s.roll_sp - s.roll) if math.isfinite(s.roll_sp) and math.isfinite(s.roll) else math.nan
        pitch_err = wrap_pi(s.pitch_sp - s.pitch) if math.isfinite(s.pitch_sp) and math.isfinite(s.pitch) else math.nan
        yaw_err = wrap_pi(s.yaw_sp - s.yaw) if math.isfinite(s.yaw_sp) and math.isfinite(s.yaw) else math.nan
        self.csv_writer.writerow({
            't': f'{s.t:.4f}',
            'roll_deg': f'{math.degrees(s.roll):.4f}',
            'pitch_deg': f'{math.degrees(s.pitch):.4f}',
            'yaw_deg': f'{math.degrees(s.yaw):.4f}',
            'roll_sp_deg': f'{math.degrees(s.roll_sp):.4f}',
            'pitch_sp_deg': f'{math.degrees(s.pitch_sp):.4f}',
            'yaw_sp_deg': f'{math.degrees(s.yaw_sp):.4f}',
            'roll_err_deg': f'{math.degrees(roll_err):.4f}',
            'pitch_err_deg': f'{math.degrees(pitch_err):.4f}',
            'yaw_err_deg': f'{math.degrees(yaw_err):.4f}',
            'tx': f'{s.tx:.6f}', 'ty': f'{s.ty:.6f}', 'tz': f'{s.tz:.6f}',
            'main1': f'{s.m1:.2f}', 'main2': f'{s.m2:.2f}', 'main3': f'{s.m3:.2f}',
            'main4': f'{s.m4:.2f}', 'main5': f'{s.m5:.2f}',
            'armed': s.armed, 'posctl': s.posctl, 'offboard': s.offboard,
        })

    def snapshot(self) -> list[Sample]:
        with self.lock:
            return list(self.samples)

    def close(self) -> None:
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass


class Dashboard:
    def __init__(self, node: HnuterAttitudeMonitor, mav: MavlinkParamClient, update_hz: float):
        self.node = node
        self.mav = mav
        self.update_period = 1.0 / max(update_hz, 1e-3)
        self.last_update = 0.0
        self.stop_requested = False
        self.command_queue: queue.Queue[str] = queue.Queue()
        self.param_values: Dict[str, float] = {k: float(v['default']) for k, v in PARAM_CONFIG.items()}
        self.group_names = list(PARAM_GROUPS)
        self.group_index = 0
        self.slider_names: list[Optional[str]] = []
        self.slider_programmatic = False
        self._setup_figure()
        self._start_command_thread()

    def _setup_figure(self) -> None:
        self.fig = plt.figure(figsize=(15, 9))
        grid = self.fig.add_gridspec(
            4, 3,
            height_ratios=[1.4, 1.4, 1.0, 1.0],
            left=0.10, right=0.95, bottom=0.31, top=0.92,
            hspace=0.45, wspace=0.28,
        )
        self.ax_att = self.fig.add_subplot(grid[0, :2])
        self.ax_err = self.fig.add_subplot(grid[1, :2], sharex=self.ax_att)
        self.ax_torque = self.fig.add_subplot(grid[2, :2], sharex=self.ax_att)
        self.ax_pwm = self.fig.add_subplot(grid[3, :2], sharex=self.ax_att)
        self.ax_text = self.fig.add_subplot(grid[0:4, 2])
        self.ax_text.axis('off')
        self.fig.suptitle('Hnuter Attitude Tuning Dashboard')

        self.lines = {}
        for key, color, style in [
            ('roll', 'tab:red', '-'), ('roll_sp', 'tab:red', '--'),
            ('pitch', 'tab:blue', '-'), ('pitch_sp', 'tab:blue', '--'),
            ('yaw', 'tab:green', '-'), ('yaw_sp', 'tab:green', '--'),
        ]:
            self.lines[key], = self.ax_att.plot([], [], style, color=color, lw=1.3, label=key)
        self.ax_att.set_ylabel('attitude [deg]')
        self.ax_att.grid(True)
        self.ax_att.legend(loc='upper left', ncol=3, fontsize=8)

        for key, color in [('roll_err', 'tab:red'), ('pitch_err', 'tab:blue'), ('yaw_err', 'tab:green')]:
            self.lines[key], = self.ax_err.plot([], [], '-', color=color, lw=1.2, label=key)
        self.ax_err.axhline(0.0, color='k', lw=0.6, alpha=0.4)
        self.ax_err.set_ylabel('error [deg]')
        self.ax_err.grid(True)
        self.ax_err.legend(loc='upper left', ncol=3, fontsize=8)

        for key, color in [('tx', 'tab:red'), ('ty', 'tab:blue'), ('tz', 'tab:green')]:
            self.lines[key], = self.ax_torque.plot([], [], '-', color=color, lw=1.2, label=key)
        self.ax_torque.axhline(0.0, color='k', lw=0.6, alpha=0.4)
        self.ax_torque.set_ylabel('torque sp')
        self.ax_torque.grid(True)
        self.ax_torque.legend(loc='upper left', ncol=3, fontsize=8)

        for key, color, width in [('m1', '0.55', 0.9), ('m2', '0.65', 0.9), ('m3', '0.45', 0.9), ('m4', '0.35', 0.9), ('m5', 'tab:purple', 1.8)]:
            self.lines[key], = self.ax_pwm.plot([], [], '-', color=color, lw=width, label=key.upper())
        self.ax_pwm.axhline(0.0, color='tab:purple', lw=0.8, ls='--', alpha=0.5)
        self.ax_pwm.set_ylabel('motor cmd [-1, 1]')
        self.ax_pwm.set_xlabel('time [s]')
        self.ax_pwm.grid(True)
        self.ax_pwm.legend(loc='upper left', ncol=5, fontsize=8)

        self.status_text = self.ax_text.text(0.02, 0.98, 'Waiting for DDS data...', va='top', ha='left', family='monospace', fontsize=9)

        self.sliders: list[Slider] = []
        slider_area = self.fig.add_gridspec(6, 1, left=0.15, right=0.92, bottom=0.025, top=0.19, hspace=0.5)
        initial_group = PARAM_GROUPS[self.group_names[0]]
        initial_items = list(initial_group.items())
        for idx in range(6):
            name, cfg = initial_items[idx]
            ax = self.fig.add_subplot(slider_area[idx, 0])
            slider = Slider(
                ax=ax,
                label=name,
                valmin=cfg['min'],
                valmax=cfg['max'],
                valinit=cfg['default'],
                valstep=cfg['step'],
            )
            slider.on_changed(lambda value, slot=idx: self.on_slider(slot, float(value)))
            self.sliders.append(slider)
            self.slider_names.append(name)

        self.group_text = self.fig.text(0.15, 0.225, '', fontsize=10, weight='bold')
        previous_ax = self.fig.add_axes([0.36, 0.208, 0.08, 0.035])
        self.previous_button = Button(previous_ax, '< group')
        self.previous_button.on_clicked(lambda _event: self.change_group(-1))

        next_ax = self.fig.add_axes([0.45, 0.208, 0.08, 0.035])
        self.next_button = Button(next_ax, 'group >')
        self.next_button.on_clicked(lambda _event: self.change_group(1))

        save_ax = self.fig.add_axes([0.80, 0.208, 0.10, 0.035])
        self.save_button = Button(save_ax, 'param save')
        self.save_button.on_clicked(lambda _event: self.mav.save())

        refresh_ax = self.fig.add_axes([0.68, 0.208, 0.10, 0.035])
        self.refresh_button = Button(refresh_ax, 'read params')
        self.refresh_button.on_clicked(lambda _event: self.read_params_into_sliders())

        self.fig.canvas.mpl_connect('close_event', lambda _event: self.request_stop())
        self.show_group(0, read_values=False)

    def _start_command_thread(self) -> None:
        thread = threading.Thread(target=self._command_worker, daemon=True)
        thread.start()

    def _command_worker(self) -> None:
        print('\nCommands: set NAME VALUE | get NAME | save | snapshot | q\n')
        while not self.stop_requested:
            try:
                line = sys.stdin.readline()
            except Exception:
                return
            if not line:
                time.sleep(0.1)
                continue
            self.command_queue.put(line.strip())

    def on_slider(self, slot: int, value: float) -> None:
        name = self.slider_names[slot]
        if name is None:
            return
        self.param_values[name] = float(value)
        if self.slider_programmatic:
            return
        self.mav.set_param(name, value)

    def show_group(self, index: int, read_values: bool = True) -> None:
        self.group_index = index % len(self.group_names)
        group_name = self.group_names[self.group_index]
        items = list(PARAM_GROUPS[group_name].items())
        self.slider_programmatic = True
        try:
            for slot, slider in enumerate(self.sliders):
                if slot >= len(items):
                    self.slider_names[slot] = None
                    slider.ax.set_visible(False)
                    continue
                name, cfg = items[slot]
                self.slider_names[slot] = name
                slider.ax.set_visible(True)
                slider.label.set_text(name)
                slider.valmin = cfg['min']
                slider.valmax = cfg['max']
                slider.valstep = cfg['step']
                slider.ax.set_xlim(cfg['min'], cfg['max'])
                slider.set_val(self.param_values[name])
            self.group_text.set_text(
                f'Parameters: {group_name}  ({self.group_index + 1}/{len(self.group_names)})'
            )
        finally:
            self.slider_programmatic = False
        self.fig.canvas.draw_idle()
        if read_values:
            self.read_params_into_sliders()

    def change_group(self, direction: int) -> None:
        self.show_group(self.group_index + direction)

    def read_params_into_sliders(self) -> None:
        self.slider_programmatic = True
        try:
            for name, slider in zip(self.slider_names, self.sliders):
                if name is None:
                    continue
                value = self.mav.request_param(name)
                if value is None:
                    continue
                self.param_values[name] = value
                slider.set_val(value)
                print(f'[PARAM_GET] {name}={value:.6g}')
        finally:
            self.slider_programmatic = False

    def process_commands(self) -> None:
        while True:
            try:
                line = self.command_queue.get_nowait()
            except queue.Empty:
                break
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()
            if cmd in ('q', 'quit', 'exit'):
                self.request_stop()
            elif cmd == 'save':
                self.mav.save()
            elif cmd == 'snapshot':
                self.print_snapshot()
            elif cmd == 'get' and len(parts) == 2:
                value = self.mav.request_param(parts[1])
                print(f'{parts[1]}={value}')
            elif cmd == 'set' and len(parts) == 3:
                name = parts[1]
                try:
                    value = float(parts[2])
                except ValueError:
                    print(f'Invalid value: {parts[2]}')
                    continue
                if name in self.slider_names:
                    slot = self.slider_names.index(name)
                    self.slider_programmatic = True
                    self.sliders[slot].set_val(value)
                    self.slider_programmatic = False
                    self.param_values[name] = value
                self.mav.set_param(name, value)
            else:
                print('Unknown command. Use: set NAME VALUE | get NAME | save | snapshot | q')

    def print_snapshot(self) -> None:
        samples = self.node.snapshot()
        if not samples:
            print('No samples yet')
            return
        recent = samples[-min(len(samples), 200):]
        pitch_err = [math.degrees(wrap_pi(s.pitch_sp - s.pitch)) for s in recent if math.isfinite(s.pitch_sp) and math.isfinite(s.pitch)]
        m5 = [s.m5 for s in recent if math.isfinite(s.m5)]
        ty = [s.ty for s in recent if math.isfinite(s.ty)]
        print('[SNAPSHOT] last samples:', len(recent))
        if pitch_err:
            print(f'  pitch_err mean={np.mean(pitch_err):+.2f}deg rms={math.sqrt(np.mean(np.square(pitch_err))):.2f}deg max={np.max(np.abs(pitch_err)):.2f}deg')
        if m5:
            print(f'  M5 mean={np.mean(m5):+.3f} range=[{np.min(m5):+.3f},{np.max(m5):+.3f}]')
        if ty:
            print(f'  Ty mean={np.mean(ty):+.4f} range=[{np.min(ty):+.4f},{np.max(ty):+.4f}]')

    def request_stop(self) -> None:
        self.stop_requested = True

    def update_plot(self) -> None:
        samples = self.node.snapshot()
        if not samples:
            return
        now = samples[-1].t
        samples = [s for s in samples if now - s.t <= self.node.history_s]
        if not samples:
            return
        t = np.array([s.t for s in samples], dtype=float)
        x = t - t[-1]
        deg = 180.0 / math.pi

        data = {
            'roll': np.array([s.roll for s in samples]) * deg,
            'pitch': np.array([s.pitch for s in samples]) * deg,
            'yaw': unwrap_finite(np.array([s.yaw for s in samples])) * deg,
            'roll_sp': np.array([s.roll_sp for s in samples]) * deg,
            'pitch_sp': np.array([s.pitch_sp for s in samples]) * deg,
            'yaw_sp': unwrap_finite(np.array([s.yaw_sp for s in samples])) * deg,
            'tx': np.array([s.tx for s in samples]),
            'ty': np.array([s.ty for s in samples]),
            'tz': np.array([s.tz for s in samples]),
            'm1': np.array([s.m1 for s in samples]),
            'm2': np.array([s.m2 for s in samples]),
            'm3': np.array([s.m3 for s in samples]),
            'm4': np.array([s.m4 for s in samples]),
            'm5': np.array([s.m5 for s in samples]),
        }
        data['roll_err'] = np.array([wrap_pi(s.roll_sp - s.roll) for s in samples]) * deg
        data['pitch_err'] = np.array([wrap_pi(s.pitch_sp - s.pitch) for s in samples]) * deg
        data['yaw_err'] = np.array([wrap_pi(s.yaw_sp - s.yaw) for s in samples]) * deg

        for key, line in self.lines.items():
            line.set_data(x, data[key])

        self.ax_att.set_xlim(-self.node.history_s, 0.0)
        for ax, keys, margin in [
            (self.ax_att, ['roll', 'pitch', 'yaw', 'roll_sp', 'pitch_sp', 'yaw_sp'], 5.0),
            (self.ax_err, ['roll_err', 'pitch_err', 'yaw_err'], 2.0),
            (self.ax_torque, ['tx', 'ty', 'tz'], 0.02),
            (self.ax_pwm, ['m1', 'm2', 'm3', 'm4', 'm5'], 0.05),
        ]:
            valid_arrays = [data[k][np.isfinite(data[k])] for k in keys if np.isfinite(data[k]).any()]
            vals = np.concatenate(valid_arrays) if valid_arrays else np.array([])
            if vals.size:
                lo = float(np.nanmin(vals)) - margin
                hi = float(np.nanmax(vals)) + margin
                if abs(hi - lo) < 1e-6:
                    lo -= 1.0
                    hi += 1.0
                ax.set_ylim(lo, hi)

        last = samples[-1]
        pitch_err = data['pitch_err'][-1]
        roll_err = data['roll_err'][-1]
        yaw_err = data['yaw_err'][-1]
        recent = samples[-min(200, len(samples)):]
        recent_pitch_err = [wrap_pi(s.pitch_sp - s.pitch) * deg for s in recent if math.isfinite(s.pitch_sp) and math.isfinite(s.pitch)]
        pitch_rms = math.sqrt(float(np.mean(np.square(recent_pitch_err)))) if recent_pitch_err else math.nan
        active_names = PARAM_GROUPS[self.group_names[self.group_index]]
        param_lines = '\n'.join(f'{k:17s} {self.param_values[k]:8.4g}' for k in active_names)
        self.status_text.set_text(
            f't={last.t:8.1f}s armed={last.armed} posctl={last.posctl} offboard={last.offboard}\n'
            f'roll  actual/sp/err {math.degrees(last.roll):+7.2f} {math.degrees(last.roll_sp):+7.2f} {roll_err:+7.2f} deg\n'
            f'pitch actual/sp/err {math.degrees(last.pitch):+7.2f} {math.degrees(last.pitch_sp):+7.2f} {pitch_err:+7.2f} deg\n'
            f'yaw   actual/sp/err {math.degrees(last.yaw):+7.2f} {math.degrees(last.yaw_sp):+7.2f} {yaw_err:+7.2f} deg\n'
            f'pitch err RMS recent: {pitch_rms:6.2f} deg\n'
            f'Tx/Ty/Tz: {last.tx:+.4f} {last.ty:+.4f} {last.tz:+.4f}\n'
            f'Motor1-5: {last.m1:+.3f} {last.m2:+.3f} {last.m3:+.3f} {last.m4:+.3f} {last.m5:+.3f}\n\n'
            f'{param_lines}\n\n'
            'Terminal: set NAME VALUE | save | snapshot | q'
        )
        self.fig.canvas.draw_idle()

    def run(self) -> None:
        plt.ion()
        self.fig.show()
        self.read_params_into_sliders()
        while not self.stop_requested and plt.fignum_exists(self.fig.number):
            self.process_commands()
            now = time.monotonic()
            if now - self.last_update >= self.update_period:
                self.update_plot()
                self.last_update = now
            plt.pause(0.01)
        self.request_stop()


def spin_ros(node: HnuterAttitudeMonitor, dashboard: Dashboard) -> None:
    while rclpy.ok() and not dashboard.stop_requested:
        try:
            rclpy.spin_once(node, timeout_sec=0.02)
        except Exception as exc:
            # During Ctrl-C/timeout shutdown the rcl context can disappear while
            # the spin thread is inside the wait set. Treat that as normal exit.
            if dashboard.stop_requested or not rclpy.ok():
                break
            raise exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Hnuter attitude tuning dashboard')
    parser.add_argument('--history', type=float, default=30.0, help='plot history window [s]')
    parser.add_argument('--update-hz', type=float, default=10.0, help='plot refresh rate [Hz]')
    parser.add_argument('--csv-rate-hz', type=float, default=25.0, help='CSV logging rate [Hz]')
    parser.add_argument('--csv', type=Path, default=None, help='CSV log path')
    parser.add_argument('--mavlink', default=os.environ.get('HNUTER_MAVLINK', 'auto'),
                        help='pymavlink endpoint, auto, or none to disable')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    csv_path = args.csv
    if csv_path is None:
        stamp = time.strftime('%Y%m%d_%H%M%S')
        csv_path = Path.home() / 'px4_ws_ros2' / 'hnuter_saved_plots' / f'hnuter_attitude_tuning_{stamp}.csv'

    rclpy.init()
    node = HnuterAttitudeMonitor(history_s=args.history, csv_path=csv_path, csv_rate_hz=args.csv_rate_hz)
    mav = MavlinkParamClient(args.mavlink)
    mav.connect(timeout=4.0)
    dashboard = Dashboard(node=node, mav=mav, update_hz=args.update_hz)
    ros_thread = threading.Thread(target=spin_ros, args=(node, dashboard), daemon=True)
    ros_thread.start()
    print(f'[CSV] logging to {csv_path}')
    try:
        dashboard.run()
    except KeyboardInterrupt:
        dashboard.request_stop()
    finally:
        dashboard.request_stop()
        ros_thread.join(timeout=1.0)
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
