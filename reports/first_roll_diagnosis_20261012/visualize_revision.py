"""Interactive desktop session for the existing no-delay Full maneuver."""
import json
import argparse
import csv
import ctypes
import os
from pathlib import Path
import select
import re
import signal
import sys
import subprocess
import time

ROOT = Path('/home/hnuter/px4_ws_ros2_consolidated')
sys.path.insert(0, str(ROOT))
from tools.experiments.run_fixed_attitude_switching import effective_tuning
from tools.experiments.run_no_delay_maneuver_experiments import (
    DEFAULT_FIRMWARE, PtyProcess, drain_for, firmware_metadata, wait_for,
)


def show_gazebo():
    """Place the observed Gazebo client on the primary X11 display."""
    try:
        tree = subprocess.check_output(['xwininfo','-root','-tree'],text=True)
        match = re.search(r'(0x[0-9a-f]+) "Gazebo Sim": \("gz-sim-gui"',tree)
        if not match:
            return
        x = ctypes.CDLL('libX11.so.6')
        x.XOpenDisplay.restype = ctypes.c_void_p
        x.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x.XMoveResizeWindow.argtypes = [ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_int,ctypes.c_uint,ctypes.c_uint]
        x.XRaiseWindow.argtypes = [ctypes.c_void_p,ctypes.c_ulong]
        x.XFlush.argtypes = [ctypes.c_void_p]
        x.XCloseDisplay.argtypes = [ctypes.c_void_p]
        display = x.XOpenDisplay(None)
        if display:
            window = int(match.group(1),16)
            x.XMoveResizeWindow(display,window,100,80,1400,1000)
            x.XRaiseWindow(display,window)
            x.XFlush(display)
            x.XCloseDisplay(display)
    except (OSError,subprocess.SubprocessError) as exc:
        print(f'Window positioning unavailable: {exc}',flush=True)


def live_sample(output):
    paths=list((output/'logs').rglob('switching_*.csv'))
    if not paths:
        return None
    with paths[0].open('rb') as stream:
        columns=next(csv.reader([stream.readline().decode()]))
        start=stream.tell()
        stream.seek(0,2)
        size=stream.tell()
        stream.seek(max(start,size-32768))
        lines=stream.read().decode(errors='replace').split('\n')
    for line in reversed(lines[:-1]):
        values=next(csv.reader([line]))
        if len(values)==len(columns):
            return dict(zip(columns,values))
    return None


def run_session(output, scenario, advance_after=None, controller_module='controllers.experiments.maneuver_revision'):
    metadata = firmware_metadata(DEFAULT_FIRMWARE)
    if metadata['dynamic_actuator_tokens_present']:
        raise RuntimeError('Delayed firmware is forbidden')
    output.mkdir(parents=True, exist_ok=False)
    os.environ['HNUTER_SWITCH_COMMON_AW_GAIN'] = '0'
    tuning = effective_tuning(output)
    env = os.environ.copy()
    env.pop('HEADLESS', None)
    env.pop('PX4_SIM_SPEED_FACTOR', None)
    env.pop('HNUTER_SWITCH_JOINT_MODEL', None)
    env.update(OPENBLAS_NUM_THREADS='1', CCACHE_DISABLE='1',
        PX4_GZ_WORLD='default', PX4_GZ_FOLLOW='1', HNUTER_CONTROL_MODE='direct',
        HNUTER_DIRECT_PUBLISH_ALLOCATOR_SETPOINTS='0',
        HNUTER_DIRECT_PUBLISH_THRUST_SETPOINT='1',
        HNUTER_NDA_GAIN='12', HNUTER_NDA_NULL_GAIN='2', HNUTER_NDA_POSTURE_GAIN='2',
        HNUTER_NDA_INVERSION='sampled', HNUTER_SWITCH_COMMON_PRELUDE='1',
        HNUTER_SWITCH_COMMON_AW_GAIN='0', HNUTER_DRCDA_REACTION_RATIO_M='0.016',
        HNUTER_REV_RAMP_S='0.75', HNUTER_REV_NEAR_GAIN='0.25', HNUTER_REV_TILT_S='15',
        HNUTER_REV_EARLY_HANDOVER='1', HNUTER_REV_SDF_GEOMETRY='1',
        HNUTER_LOG_DIR=str(output/'logs'), HNUTER_TUNING_FILE=str(tuning),
        HNUTER_SWITCH_METHOD='full', HNUTER_MANEUVER_SCENARIO=scenario,
        HNUTER_SWITCH_ALTITUDE_M='3.0', HNUTER_PREFLIGHT_TILT_TEST='0')
    (output/'session.json').write_text(json.dumps(dict(firmware=metadata,controller_module=controller_module,
        environment={k:v for k,v in env.items() if k.startswith(('HNUTER_', 'PX4_'))},
        purpose='interactive visualization, not independent formal repetition'),indent=2)+'\n')
    agent = sitl = controller = None
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    try:
        agent = PtyProcess(['MicroXRCEAgent','udp4','-p','8888'], ROOT, env, output/'console/agent.log')
        drain_for([agent],1.)
        if agent.process.poll() is not None:
            raise RuntimeError('DDS agent exited')
        sitl = PtyProcess(['make','px4_sitl','gz_hnuter'], DEFAULT_FIRMWARE, env, output/'console/px4.log')
        if not wait_for([agent,sitl],sitl,'Ready for takeoff!',90.):
            raise RuntimeError('PX4 did not become ready')
        show_gazebo()
        print(f'Gazebo ready. Starting Full DRCDA / {scenario} in 10 seconds.',flush=True)
        drain_for([agent,sitl],10.)
        controller = PtyProcess(['/home/hnuter/px4_ws_ros2/px4-venv/bin/python',
            '-m',controller_module],ROOT,env,output/'console/controller.log')
        processes = [agent,sitl,controller]
        if not wait_for(processes,controller,'地面自检中',30.):
            raise RuntimeError('Controller telemetry unavailable')
        controller.send('o')
        if not wait_for(processes,controller,'ARM_DISARM -> ACCEPTED',20.):
            raise RuntimeError('Simulation arming rejected')
        controller.send('2')
        print('Flight requested. After completion: r + Enter = replay; q + Enter = stop.\n'
              'Only the simulation is armed. Session closes automatically after 10 minutes.',flush=True)
        if advance_after is not None:
            print(f'Session ends {advance_after} seconds after completion; next queued task follows. Enter n to advance now.',flush=True)
        end = time.monotonic()+600
        running = True
        next_status=0.
        while time.monotonic()<end:
            if time.monotonic()>=next_status:
                next_status=time.monotonic()+1.
                sample=live_sample(output)
                if sample:
                    phase=sample['switch_phase']
                    error=sum((float(sample[p])-float(sample[r]))**2 for p,r in zip(
                        ['position_x_enu_m','position_y_enu_m','position_z_rel_m'],
                        ['target_x_enu_m','target_y_enu_m','target_z_rel_m']))**.5
                    line=(f"phase={phase:20s} target R/P={float(sample['target_roll_deg']):+6.1f}/"
                          f"{float(sample['target_pitch_deg']):+6.1f} deg | "
                          f"position error={error:6.2f} m | SO3={float(sample['attitude_error_angle_deg']):6.1f} deg")
                    print(line,flush=True)
                    with (output/'live_phases.log').open('a') as status:
                        status.write(f"csv_time={sample['time_s']} "+line+'\n')
            for p in processes:
                text = p.read()
                if p is controller and text:
                    heights = re.findall(r'Current ENU/Zrel:\s*\[[^,]+,[^,]+,\s*([-+\d.]+)',text)
                    if any(abs(float(h))>12 for h in heights):
                        raise RuntimeError('Visualization stopped: height outside 12 m guard')
                    positions = re.findall(r'Current ENU/Zrel:\s*\[\s*([-+\d.]+),\s*([-+\d.]+)',text)
                    if any(float(px)**2+float(py)**2>30**2 for px,py in positions):
                        raise RuntimeError('Visualization stopped: horizontal position outside 30 m guard')
                    if 'MANEUVER_START' in text:
                        print('MANEUVER_START',flush=True)
                    if 'MANEUVER_DONE' in text:
                        running = False
                        print('MANEUVER_DONE: hovering. Enter r to replay, q to close.',flush=True)
                        if advance_after is not None:
                            end = min(end,time.monotonic()+advance_after)
                if p.process.poll() is not None:
                    raise RuntimeError('A simulation process exited')
            if select.select([sys.stdin],[],[],.05)[0]:
                line = sys.stdin.readline()
                if not line or line.strip().lower()=='q':
                    return False
                if line.strip().lower()=='n' and advance_after is not None:
                    return True
                if line.strip().lower()=='r' and not running:
                    running=True
                    end=time.monotonic()+600
                    controller.send('2')
                    print('Replay requested.',flush=True)
    except KeyboardInterrupt:
        print('Stopping simulation.',flush=True)
        return False
    except RuntimeError as exc:
        print(f'{scenario}: {exc}',flush=True)
        (output/'failure.txt').write_text(str(exc)+'\n')
    finally:
        for child,timeout,shutdown in [(controller,6.,None),(sitl,8.,'shutdown\n'),(agent,4.,None)]:
            if child is None:
                continue
            try:
                child.stop(graceful_text=shutdown,timeout=timeout)
            except (OSError,subprocess.SubprocessError) as exc:
                print(f'Cleanup warning: {exc}',flush=True)
            finally:
                # The make/px4 parent can exit before its Gazebo children.
                try:
                    os.killpg(child.process.pid,signal.SIGTERM)
                except ProcessLookupError:
                    pass
        (output/'stopped.txt').write_text('Session stopped; child processes cleaned up.\n')
    return True


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    parser.add_argument('--scenario',action='append',choices=['lissajous_7','lissajous_5','attitude_80_180'])
    parser.add_argument('--controller-module',default='controllers.experiments.maneuver_revision')
    parser.add_argument('--close-after-done',type=float)
    args=parser.parse_args()
    scenarios=args.scenario or ['lissajous_7']
    for i,scenario in enumerate(scenarios):
        output=args.output/scenario if len(scenarios)>1 else args.output
        after=15 if i<len(scenarios)-1 else args.close_after_done
        if not run_session(output,scenario,after,args.controller_module):
            break


if __name__=='__main__':
    main()
