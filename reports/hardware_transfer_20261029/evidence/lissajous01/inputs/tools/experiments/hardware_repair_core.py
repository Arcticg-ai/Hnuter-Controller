"""Pure, offline-testable policy for the hardware-v2 SITL repair experiment."""
from __future__ import annotations

import ast
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'controllers/hardware/hnuter_external_direct_drcda_v2_hardware.py'
FIRMWARE = Path('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl')
BASELINE = None  # Reconstructed from the archived commit and verified patch.
FLIGHT = ROOT / 'config/experiments/hardware_repair_flight_tuning.json'
PLAN = ROOT / 'config/experiments/hardware_repair_no_delay.json'
JOINTS = ('lj2', 'lj1', 'rj2', 'rj1')
GROUPS = ('baseline', 'ref_fix', 'tuned')
PROTECTED_METHODS = ('update_trajectory', '_advance_manual_position_reference',
                     '_update_attitude_integral', '_state_control_tick',
                     '_begin_hardware_control', '_end_hardware_control',
                     '_publish_normalized_direct_actuator_setpoint')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def literal_defaults(source):
    tree = ast.parse(Path(source).read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == 'HARDWARE_V2_DEFAULT_CONFIG'
                for t in node.targets):
            return ast.literal_eval(node.value)
    raise ValueError('No standalone hardware-v2 default configuration')


def airframe_defaults(path):
    result = {}
    for line in Path(path).read_text().splitlines():
        match = re.match(r'\s*param set(?:-default)?\s+(\w+)\s+([-+\d.eE]+)(?:\s|$)', line)
        if match:
            result[match[1]] = float(match[2])
    return result


def inspect_simulation(firmware):
    """Read the actual plant, not a hardware calibration or an old report."""
    firmware = Path(firmware)
    sdf = firmware / 'Tools/simulation/gz/models/hnuter/model.sdf'
    airframe = firmware / 'ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter'
    text = sdf.read_text()
    for token in ('transport_delay', 'FirstOrderActuator', 'servo_0_dynamic'):
        if token in text:
            raise ValueError('Forbidden artificial-delay model: ' + token)
    model = ET.fromstring(text).find('model')
    p = airframe_defaults(airframe)
    motors = sorted((e for e in model.findall('plugin') if e.find('motorNumber') is not None),
                    key=lambda e: int(e.findtext('motorNumber')))
    if [int(e.findtext('motorNumber')) for e in motors] != list(range(5)):
        raise ValueError('Expected exactly five motors')
    constants = [float(e.findtext('motorConstant')) for e in motors]
    if not np.allclose(constants[:4], constants[0]):
        raise ValueError('Per-front-motor calibration changed; revise adapter')
    for i in range(1, 6):
        if p[f'SIM_GZ_EC_MAX{i}'] != 1000 or p[f'SIM_GZ_EC_MIN{i}'] != 10:
            raise ValueError('Gazebo ESC range changed; revise adapter')
    if any(p[f'SIM_GZ_SV_MAXA{i}'] != (185 if i < 3 else 180) or
           p[f'SIM_GZ_SV_MINA{i}'] != (-185 if i < 3 else -180) for i in range(1, 5)):
        raise ValueError('Gazebo servo mapping changed')
    if p['HNTR_S2_GEAR'] != 1 or p['CA_R_REV'] != 16:
        raise ValueError('Expected direct-drive secondary joints and reversible tail')
    if not bool(int(motors[4].findtext('bidirectionalMotor', '0'))):
        raise ValueError('Tail plant is not bidirectional')
    mass = sum(float(e.findtext('inertial/mass')) for e in model.findall('link'))
    return dict(sdf=str(sdf), sdf_sha256=sha256(sdf), airframe=str(airframe),
                airframe_sha256=sha256(airframe), mass_kg=mass,
                motor_constants=constants, motor_max_velocity_rad_s=1000.,
                front_min_velocity_rad_s=10.,
                plant_motor_tau_up_s=[float(e.findtext('timeConstantUp')) for e in motors],
                plant_motor_tau_down_s=[float(e.findtext('timeConstantDown')) for e in motors],
                servo_normalized_limits_deg=[185., 185., 180., 180.],
                secondary_gear=1., parameters=p, pure_delay_model_used=False,
                note='Native Gazebo motor lags and joint PID retained; no injected pure delay.')


def adapted_tuning(source, flight, simulation, overrides=None):
    defaults = literal_defaults(source)
    actual = flight.get('tuning', flight)
    missing = set(defaults) - set(actual)
    # New schema keys use their fixed-source defaults, never replace known flight values.
    data = dict(defaults)
    data.update(actual)
    changes = dict(overrides or {})
    allowed = {'direct_pos_Ki_ned', 'direct_pos_integral_limit_ned',
               'direct_attitude_Ki', 'direct_attitude_integral_limit', 'direct_Domega'}
    if set(changes) - allowed:
        raise ValueError('Candidate may only change declared integral/damping fields')
    data.update(changes)
    p = simulation['parameters']
    adaptation = dict(HNTR_MASS=simulation['mass_kg'], HNTR_S2_GEAR=1.,
                      primary_servo_angle_max_deg=185., secondary_servo_angle_max_deg=180.,
                      HNTR_S1_RATE=p['HNTR_S1_RATE'], HNTR_S2_RATE=p['HNTR_S2_RATE'],
                      HNTR_L2=p['HNTR_L2'], HNTR_CG_X=p['HNTR_CG_X'],
                      HNTR_CG_Z=p['HNTR_CG_Z'],
                      HNTR_TAIL_T_POS=simulation['motor_constants'][4] * 1e6,
                      HNTR_TAIL_T_NEG=6.02,
                      hardware_servo_tau_positive_s=[0.] * 4,
                      hardware_servo_tau_negative_s=[0.] * 4)
    data.update(adaptation)
    if any('delay' in k.lower() and np.any(np.asarray(v, dtype=float) != 0)
           for k, v in data.items()):
        raise ValueError('No artificial delay is allowed')
    return data, dict(flight_values=actual, new_schema_defaults={k: defaults[k] for k in missing},
                      candidate_overrides=changes, simulation_overrides=adaptation,
                      retained_flight_pitch_bias=actual['HNTR_PITCH_BIAS'],
                      motor_mapping='SDF k*omega^2; front omega=10+990*u; tail omega=1000*u',
                      geometry='SDFWrenchModel articulated rotor geometry about assembly COM',
                      limitations=['Not the aircraft plant or its calibrated motor/PWM model',
                                   'No modeled cable or identified attachment stiffness',
                                   'Synthetic RC enters hardware parser, not a physical RC receiver'])


def front_force(control, constant):
    return float(constant * (10. + 990. * np.clip(control, 0., 1.)) ** 2)


def front_control(force, constant):
    return float(np.clip((math.sqrt(max(force, 0.) / constant) - 10.) / 990., 0., 1.))


def tail_force(control, constant):
    return float(np.clip(control, -1., 1.) * abs(np.clip(control, -1., 1.)) * constant * 1e6)


def tail_control(force, constant):
    return float(np.clip(math.copysign(math.sqrt(abs(force) / (constant * 1e6)), force), -1., 1.))


def phase_at(plan, elapsed):
    start = 0.
    for phase in plan['phases']:
        end = start + phase['duration_s']
        if elapsed < end:
            return dict(phase, start_s=start, end_s=end)
        start = end
    return None


def validate_plan(plan):
    if plan['policy'] != 'no_delay_only' or plan['groups'] != list(GROUPS):
        raise ValueError('Only matched hardware DRCDA baseline/ref_fix/tuned is supported')
    names = [p['name'] for p in plan['phases']]
    if len(names) != len(set(names)):
        raise ValueError('Duplicate phase')
    for p in plan['phases']:
        if not math.isfinite(p['duration_s']) or p['duration_s'] < 1:
            raise ValueError('Invalid phase duration')
        if p['mode'] not in ('offboard', 'position'):
            raise ValueError('Invalid flight mode')
        if any(not math.isfinite(v) or abs(v) > 1 for v in p.get('sticks', {}).values()):
            raise ValueError('Invalid stick command')
    if names != ['hold', 'yaw_out', 'yaw_back', 'yaw_settle', 'lateral_out',
                 'lateral_back', 'lateral_settle', 'position_exit', 'offboard_reentry',
                 'position_exit_2', 'offboard_reentry_2', 'final_hold']:
        raise ValueError('Required scenario coverage changed')
    force = plan['synthetic_force']
    if force['enabled']:
        raise ValueError('Synthetic wrench is not enabled in the first no-force campaign')
    return plan


def analyze_samples(rows, plan, completed):
    """Fail closed for missing modes, stale/nonfinite state, or truncated evidence."""
    invalid, failures = [], []
    if not rows:
        return dict(status='INVALID', invalid=['no samples'], failures=[], metrics={})
    required = ('wall_s', 'px4_s', 'altitude_m', 'position_error_m', 'so3_deg',
                'roll_deg', 'pitch_deg', 'speed_m_s', 'target_speed_m_s',
                'joint_age_s', 'source_age_s', 'rate_deg_s', 'motor_abs_max', 'servo_abs_max')
    if any(any(k not in r or r[k] is None or not math.isfinite(float(r[k])) for k in required) for r in rows):
        return dict(status='INVALID', invalid=['missing/nonfinite required measurements'],
                    failures=[], metrics={})
    wall = np.array([r['wall_s'] for r in rows])
    sim = np.array([r['px4_s'] for r in rows])
    if len(rows) < 2 or np.any(np.diff(wall) <= 0) or np.any(np.diff(sim) < 0):
        invalid.append('nonmonotonic timestamps or too few samples')
    gap = float(np.max(np.diff(wall))) if len(rows) > 1 else 1e9
    if gap > plan['limits']['max_sample_gap_s']:
        invalid.append('observer sample gap exceeds limit')
    by_phase = {}
    for phase in plan['phases']:
        selected = [r for r in rows if r['phase'] == phase['name']]
        if not selected:
            invalid.append('missing phase: ' + phase['name'])
            continue
        # Transition samples are retained; require eventual observed mode, not an ACK alone.
        nav = 14 if phase['mode'] == 'offboard' else 2
        confirmed = [r for r in selected if r['nav_state'] == nav and
                     bool(r['hardware_active']) == (nav == 14)]
        if not confirmed:
            failures.append('mode not observed: ' + phase['name'])
        span = selected[-1]['px4_s'] - selected[0]['px4_s']
        if span < phase['duration_s'] - .5:
            invalid.append('truncated phase: ' + phase['name'])
        ep = np.array([r['position_error_m'] for r in selected])
        by_phase[phase['name']] = dict(samples=len(selected), observed_mode_samples=len(confirmed),
                                      span_s=span, position_rms_m=float(np.sqrt(np.mean(ep ** 2))),
                                      position_peak_m=float(ep.max()),
                                      so3_peak_deg=max(r['so3_deg'] for r in selected),
                                      target_speed_peak_m_s=max(r['target_speed_m_s'] for r in selected))
    if not completed:
        failures.append('scenario not completed')
    if any(not r['armed'] for r in rows):
        failures.append('in-flight disarm')
    if any(r.get('fault') for r in rows):
        failures.append('hardware fault latched')
    if any(r.get('safety_cutoff', False) for r in rows):
        failures.append('safety cutoff')
    for name, field, bound in (
        ('position error', 'position_error_m', 'position_peak_m'),
        ('attitude error', 'so3_deg', 'so3_peak_deg'),
        ('body angular rate', 'rate_deg_s', 'rate_peak_deg_s'),
        ('target velocity spike', 'target_speed_m_s', 'target_speed_peak_m_s'),
    ):
        if max(r[field] for r in rows) > plan['limits'][bound]:
            failures.append(name + ' exceeds bound')
    if min(r['altitude_m'] for r in rows) < plan['limits']['altitude_min_m']:
        failures.append('altitude below bound')
    if max(max(abs(r['roll_deg']), abs(r['pitch_deg'])) for r in rows) > plan['limits']['tilt_peak_deg']:
        failures.append('roll/pitch outside envelope')
    if any(r['motor_abs_max'] > 1.000001 or r['servo_abs_max'] > 1.000001 for r in rows):
        failures.append('normalized physical output constraint violated')
    if any(r['source_age_s'] > .3 for r in rows):
        invalid.append('stale source state')
    ratio = sum(r['joint_age_s'] <= .12 for r in rows) / len(rows)
    if ratio < .95:
        invalid.append('measured joint coverage below 95%')
    if plan['synthetic_force']['enabled']:
        loaded = [r for r in rows if r.get('synthetic_force_applied')]
        if not loaded:
            invalid.append('no synthetic load application evidence')
        elif loaded[-1]['scenario_s'] - loaded[0]['scenario_s'] < (
                plan['synthetic_force']['stop_s'] - plan['synthetic_force']['start_s'] - .5):
            invalid.append('synthetic load coverage truncated')
    metrics = dict(phases=by_phase, measured_joint_ratio=ratio, sample_gap_peak_s=gap,
                   measured_realtime_factor=float((sim[-1] - sim[0]) / (wall[-1] - wall[0]))
                   if wall[-1] > wall[0] else 0.,
                   target_speed_peak_m_s=max(r['target_speed_m_s'] for r in rows),
                   altitude_min_m=min(r['altitude_m'] for r in rows),
                   source_age_peak_s=max(r['source_age_s'] for r in rows))
    return dict(status='INVALID' if invalid else 'FAIL' if failures else 'PASS',
                invalid=invalid, failures=failures, metrics=metrics)
