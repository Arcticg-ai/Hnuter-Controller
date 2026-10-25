"""Matched allocation-only maneuvers using the existing no-delay SITL harness."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.experiments.run_fixed_attitude_switching import effective_tuning, wait_for_marker_or_disarm
from tools.experiments.run_no_delay_maneuver_experiments import (
    DEFAULT_FIRMWARE, PtyProcess, drain_for, firmware_metadata, latest_ulog, wait_for,
)

METHODS = ['direct', 'paper_nda', 'full', 'no_preview', 'no_horizon']
SCENARIOS = ['lissajous_7', 'lissajous_5', 'attitude_60_60', 'attitude_80_120']


def analyze(csv_path, finished, scenario):
    df = pd.read_csv(csv_path)
    active = ~df.switch_phase.isin(['idle', 'ingress', 'settle_a'])
    df = df[active].copy()
    if df.empty:
        return {'passed': False, 'failures': ['no evaluated maneuver samples']}
    positions = df[['position_x_enu_m', 'position_y_enu_m', 'position_z_rel_m']].values
    targets = df[['target_x_enu_m', 'target_y_enu_m', 'target_z_rel_m']].values
    ep = np.linalg.norm(positions-targets, axis=1)
    eo = df.attitude_error_angle_deg.to_numpy()
    finite = np.isfinite(ep).all() and np.isfinite(eo).all()
    holds = df.switch_phase.str.endswith('_hold')
    r = dict(position_rms_m=float(np.sqrt(np.mean(ep**2))), position_peak_m=float(np.max(ep)),
             so3_rms_deg=float(np.sqrt(np.mean(eo**2))), so3_peak_deg=float(np.max(eo)),
             yaw_rate_peak_rad_s=float(df.angular_r_frd_rps.abs().max()),
             minimum_altitude_m=float(df.position_z_rel_m.min()),
             feedback_ratio=float(df.switch_joint_feedback_available.mean()),
             remained_armed=bool((df.armed==1).all()),
             safety_cutoff=bool((df.direct_safety_cutoff!=0).any()),
             finished=finished, evaluated_duration_s=float(df.time_s.iloc[-1]-df.time_s.iloc[0]),
             hold_so3_rms_deg=float(np.sqrt(np.mean(eo[holds]**2))) if holds.any() else None,
             hold_position_rms_m=float(np.sqrt(np.mean(ep[holds]**2))) if holds.any() else None)
    failures = []
    if not finite: failures.append('nonfinite tracking data')
    if not finished: failures.append('trajectory not completed')
    if r['position_peak_m'] > .75: failures.append('position peak > .75 m')
    if r['so3_peak_deg'] > 12.: failures.append('SO3 peak > 12 deg')
    if scenario.startswith('attitude') and (not holds.any() or r['hold_so3_rms_deg'] > 5.):
        failures.append('no peak hold or hold SO3 RMS > 5 deg')
    if r['minimum_altitude_m'] < .5: failures.append('altitude < .5 m')
    if r['feedback_ratio'] < .95: failures.append('joint feedback coverage < 95%')
    if not r['remained_armed']: failures.append('disarmed during evaluation')
    if r['safety_cutoff']: failures.append('safety cutoff')
    r.update(passed=not failures, failures=failures)
    return r


def run_case(output, scenario, method, repeat, gui=False, controller_python=sys.executable,
             controller_module='controllers.experiments.maneuver_validation', task_timeout=None):
    root = output/scenario/f'repeat_{repeat}'/method
    root.mkdir(parents=True, exist_ok=False)
    tuning = effective_tuning(root)
    started = time.time_ns()
    agent = sitl = controller = None
    finished, error = False, ''
    try:
        agent = PtyProcess(['MicroXRCEAgent', 'udp4', '-p', '8888'], ROOT,
                           os.environ.copy(), root/'console/agent.log')
        drain_for([agent], 1.)
        if agent.process.poll() is not None:
            raise RuntimeError('DDS agent startup failed')
        env = os.environ.copy()
        env['PX4_GZ_WORLD'] = 'default'
        if gui: env.pop('HEADLESS', None)
        else: env['HEADLESS'] = '1'
        sitl = PtyProcess(['make', 'px4_sitl', 'gz_hnuter'], DEFAULT_FIRMWARE,
                          env, root/'console/px4.log')
        if not wait_for([agent, sitl], sitl, 'Ready for takeoff!', 60.):
            raise RuntimeError('PX4 startup timed out')
        env.update(HNUTER_LOG_DIR=str(root/'logs'), HNUTER_TUNING_FILE=str(tuning),
                   HNUTER_SWITCH_METHOD=method, HNUTER_MANEUVER_SCENARIO=scenario,
                   HNUTER_SWITCH_ALTITUDE_M='3.0', HNUTER_PREFLIGHT_TILT_TEST='0')
        controller = PtyProcess([controller_python, '-m', controller_module],
                                ROOT, env, root/'console/controller.log')
        processes = [agent, sitl, controller]
        if not wait_for(processes, controller, '地面自检中', 30.):
            raise RuntimeError('controller telemetry startup failed')
        controller.send('o')
        if not wait_for(processes, controller, 'ARM_DISARM -> ACCEPTED', 20.):
            raise RuntimeError('arm not accepted')
        controller.send('2')
        ok, _ = wait_for_marker_or_disarm(processes, controller, 'MANEUVER_START', 40.)
        if not ok: raise RuntimeError('maneuver start failed')
        finished, _ = wait_for_marker_or_disarm(processes, controller, 'MANEUVER_DONE',
                                               task_timeout or (105. if scenario.startswith('attitude') else 30.))
        if not finished: raise RuntimeError('maneuver finish failed')
        drain_for(processes, 5.)
    except Exception as exc:
        error = str(exc)
    finally:
        if controller: controller.stop(timeout=6.)
        if sitl: sitl.stop(graceful_text='shutdown\n', timeout=8.)
        if agent: agent.stop(timeout=4.)
        time.sleep(1.)
    csvs = sorted(root.rglob('switching_*.csv'))
    metrics = analyze(csvs[-1], finished, scenario) if csvs else {'passed': False, 'failures': ['no CSV']}
    ulog = latest_ulog(DEFAULT_FIRMWARE, started)
    result = dict(scenario=scenario, method=method, repeat=repeat, error=error,
                  metrics=metrics, csv=str(csvs[-1]) if csvs else None,
                  ulog_source=str(ulog) if ulog else None,
                  tuning_sha256=hashlib.sha256(tuning.read_bytes()).hexdigest(),
                  source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in sorted((ROOT/'controllers').rglob('*.py'))},
                  pure_delay_model_used=False)
    result['constraints'] = dict(servo_command_rate_rad_s=50., servo_physical_rate_rad_s=10.,
                                 front_force_command_rate_n_s=2500., tail_force_command_rate_n_s=100.,
                                 front_force_max_n=25., angle_limits_deg=[185.,180.,185.,180.])
    (root/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(f'{scenario}/{repeat}/{method}: {"PASS" if metrics["passed"] else "FAIL"} '
          f'{error} {metrics["failures"]}', flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenario', action='append', choices=SCENARIOS)
    parser.add_argument('--method', action='append', choices=METHODS)
    parser.add_argument('--repeat', type=int, default=2)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--controller-python', default=sys.executable)
    args = parser.parse_args()
    metadata = firmware_metadata(DEFAULT_FIRMWARE)
    if metadata['dynamic_actuator_tokens_present']:
        raise SystemExit('Refusing delayed firmware model')
    os.environ.update(OPENBLAS_NUM_THREADS='1', HNUTER_NDA_GAIN='12', HNUTER_NDA_NULL_GAIN='2',
                      HNUTER_NDA_POSTURE_GAIN='2', HNUTER_NDA_INVERSION='sampled',
                      HNUTER_SWITCH_COMMON_PRELUDE='1', HNUTER_SWITCH_COMMON_AW_GAIN='0',
                      HNUTER_DRCDA_REACTION_RATIO_M='0.016')
    os.environ.pop('HNUTER_SWITCH_JOINT_MODEL', None)
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = dict(firmware=metadata, methods=args.method or METHODS,
                    scenarios=args.scenario or SCENARIOS, repeats=args.repeat,
                    controller_python=args.controller_python,
                    environment={k:v for k,v in os.environ.items() if k.startswith('HNUTER_')}, cases=[])
    for scenario in manifest['scenarios']:
        for repeat in range(1, args.repeat+1):
            methods = manifest['methods'] if repeat%2 else list(reversed(manifest['methods']))
            for method in methods:
                manifest['cases'].append(run_case(args.output, scenario, method, repeat, args.gui, args.controller_python))
                (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
                if not manifest['cases'][-1]['csv']:
                    raise SystemExit('Infrastructure failure: no controller CSV; stopping batch')


if __name__ == '__main__':
    main()
