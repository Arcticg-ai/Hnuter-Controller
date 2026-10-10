"""Bounded diagnostic launcher; frozen revision2 position node, read-only truth recorder.

Derived from revision2/run_sitl.py at 03c9b1ff. No new controller or estimator
input: truth is written to a separate CSV for offline balance checks only.
"""
import argparse
import sys
import datetime
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import time
import uuid
import hashlib
import xml.etree.ElementTree as ET
from zoneinfo import ZoneInfo

ROOT=Path(__file__).resolve().parent
REV2=ROOT.parents[1]/'revision2'
sys.path.insert(0,str(REV2))
PX4=Path('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl')
BUILD=PX4/'build/px4_sitl_default'


def main():
    p=argparse.ArgumentParser();p.add_argument('--label',default='nominal')
    p.add_argument('--loaded',action='store_true');p.add_argument('--instrument',action='store_true')
    p.add_argument('--speed',type=float,default=1.)
    p.add_argument('--smooth-takeoff',action='store_true')
    p.add_argument('--group',choices=['A','B','C','D','C_shadow'],default='A')
    p.add_argument('--noise',choices=['native','zero','low_noise','noisy_delayed'],default='native')
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--instance',type=int,default=0)
    p.add_argument('--attitude-reference',choices=['legacy','level'],default='level')
    p.add_argument('--discovery',choices=['shared','isolated'],default='shared')
    p.add_argument('--startup-only',action='store_true')
    p.add_argument('--omit-joint-telemetry',action='store_true')
    p.add_argument('--inject-exit-at',type=float,default=0.)
    p.add_argument('--source-model',type=Path)
    p.add_argument('--physics-step',type=float,default=.004)
    p.add_argument('--spatial-case',choices=['offset_y','symmetric_inertia'])
    p.add_argument('--shadow-only',action='store_true')
    args=p.parse_args()
    assert not args.shadow_only or (args.spatial_case and args.group in ('B','C')), 'shadow-only is restricted to spatial B/C'
    if args.spatial_case and args.group in ('B','C') and not args.shadow_only:
        gate=json.loads((ROOT/'results/consumer_gate.json').read_text())
        assert gate.get('oracle_consumer_pass'), 'Oracle consumer gate must pass before estimator groups'
    # No reuse or attach to an existing instance. PX4 itself enforces its lock.
    if not 0<=args.instance<=4:p.error('instance must be 0..4')
    port=28888+args.instance
    for port in (port,):
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s: s.bind(('0.0.0.0',port))
    stamp=datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S')
    out=ROOT/'runs'/('sitl_'+args.label+'_'+stamp);out.mkdir(parents=True)
    rootfs=out/'rootfs';rootfs.mkdir()
    shutil.copytree(BUILD/'etc',rootfs/'etc')
    # Networking-only change to private startup copy: no MAVLink/remote GCS sockets.
    (rootfs/'etc/init.d-posix/px4-rc.mavlink').write_text('#!/bin/sh\n# Disabled for isolated JEPA DDS-only SITL.\n')
    rc=rootfs/'etc/init.d-posix/rcS'
    rc.write_text(rc.read_text().replace('param set MAV_SYS_ID $((px4_instance+1))','param set MAV_SYS_ID 1 # isolated per-domain JEPA instance'))
    shutil.copy(BUILD/'rootfs/gz_env.sh',rootfs/'gz_env.sh')
    model_name='hnuter'
    if args.instrument or args.loaded:
        from actuator_sensors import export_geometry
        source=PX4/'Tools/simulation/gz/models/hnuter/model.sdf'
        if args.loaded:source=ROOT.parents[1]/'generated/native_noise/hnuter_mass_cal_payload/model.sdf'
        if args.source_model:source=args.source_model.resolve()
        if args.spatial_case:source=ROOT.parent/'generated'/args.spatial_case/'model.sdf'
        tree=ET.parse(source);model=tree.getroot().find('model')
        if args.noise in ('zero','low_noise'):
            for noise in model.findall('.//sensor//noise'):
                for child in noise:
                    if child.tag in ('mean','stddev','bias_mean','bias_stddev','dynamic_bias_stddev'):
                        # Exact-zero gyro/accel/baro/mag trip unchanged-value validators.
                        child.text=str(float(child.text)*.01) if args.noise=='low_noise' and child.tag=='stddev' else '0'
        if args.spatial_case:
            imu_rate=float(model.find("link/sensor[@type='imu']/update_rate").text)
            assert abs(imu_rate*args.physics_step-1.)<1e-9, 'IMU must sample every physics step'
        model_name='hnuter_jepa';model.set('name',model_name)
        for uri in model.findall('.//mesh/uri'):
            if not uri.text.startswith(('/', 'model://', 'file://')):
                uri.text=str(source.parent/uri.text)
        if not args.omit_joint_telemetry:
            plugin=ET.SubElement(model,'plugin',filename='gz-sim-joint-state-publisher-system',name='gz::sim::systems::JointStatePublisher')
            ET.SubElement(plugin,'topic').text='/jepa_joint_sensors'
        plugin=ET.SubElement(model,'plugin',filename=str(ROOT/'build/libJepaTruthRecorder.so'),name='jepa::TruthRecorder')
        ET.SubElement(plugin,'output').text=str(out/'truth_diagnostic.csv')
        models=out/'models';folder=models/model_name;folder.mkdir(parents=True)
        tree.write(folder/'model.sdf',encoding='utf-8',xml_declaration=True)
        export_geometry(PX4/'Tools/simulation/gz/models/hnuter/model.sdf',out/'actuator_geometry.json')
        with (rootfs/'gz_env.sh').open('a') as f:
            f.write(f'\nexport PX4_GZ_MODELS={models}\nexport GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:{models}\n')
    assert args.instrument, 'Diagnostic requires independent joint and truth instrumentation'
    if args.physics_step != .004:
        assert .0005<=args.physics_step<=.004
        world=ET.parse(PX4/'Tools/simulation/gz/worlds/default.sdf')
        world.getroot().find('world/physics/max_step_size').text=str(args.physics_step)
        worlds=out/'worlds';worlds.mkdir()
        world.write(worlds/'default.sdf')
        with (rootfs/'gz_env.sh').open('a') as f:f.write(f'\nexport PX4_GZ_WORLDS={worlds}\n')
    profile=out/'dds_loopback.xml'
    profile.write_text('''<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
<transport_descriptors><transport_descriptor><transport_id>jepa_udp</transport_id><type>UDPv4</type>
<interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList></transport_descriptor></transport_descriptors>
<participant profile_name="jepa_local" is_default_profile="true"><rtps>
<userTransports><transport_id>jepa_udp</transport_id></userTransports><useBuiltinTransports>false</useBuiltinTransports>
</rtps></participant></profiles>''')
    token='jepa_'+uuid.uuid4().hex[:12]
    env={k:v for k,v in os.environ.items() if not k.startswith(('PX4_','ROS_','GZ_','HNUTER_','RMW_'))}
    env.update(HEADLESS='1',PX4_SIM_MODEL='gz_'+model_name,PX4_SYS_AUTOSTART='4051',PX4_GZ_WORLD='default',
               GZ_IP='127.0.0.1',GZ_PARTITION=token,ROS_DOMAIN_ID=str(73+args.instance),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               PX4_UXRCE_DDS_PORT=str(port),PX4_UXRCE_DDS_NS=token,
               PX4_PARAM_UXRCE_DDS_SYNCT='0',
               FASTDDS_DEFAULT_PROFILES_FILE=str(profile),FASTRTPS_DEFAULT_PROFILES_FILE=str(profile),
               RMW_IMPLEMENTATION='rmw_fastrtps_cpp',ROS_LOG_DIR=str(out/'ros_logs'),JEPA_RUN_DIR=str(out),
               PYTHONUNBUFFERED='1',SDL_VIDEODRIVER='dummy',SDL_AUDIODRIVER='dummy')
    if args.discovery=='isolated':
        env['GZ_DISCOVERY_MSG_PORT']=str(29800+2*args.instance)
        env['GZ_DISCOVERY_SRV_PORT']=str(29801+2*args.instance)
    env['JEPA_ATTITUDE_REFERENCE']=args.attitude_reference
    env['JEPA_STARTUP_ONLY']=str(int(args.startup_only))
    env['JEPA_INJECT_EXIT_AT']=str(args.inject_exit_at)
    env['JEPA_SMOOTH_TAKEOFF']='1' if args.smooth_takeoff else '0'
    env['JEPA_SHADOW_ONLY']=str(int(args.shadow_only))
    env['JEPA_GROUP']=args.group;env['JEPA_NOISE']=args.noise;env['JEPA_SEED']=str(args.seed)
    if args.group=='D':env['PX4_PARAM_HNTR_MASS']='5.39999998' if args.loaded else '4.49999998'
    if args.spatial_case:
        assert args.physics_step==.004, '250 Hz IMU requires matched 4 ms physics for discrete estimator'
        env['PX4_PARAM_HNTR_MASS']='5.4'
        if args.group=='D':
            import numpy as np
            truth=json.loads((ROOT.parent/'generated/truth_for_evaluation_only.json').read_text())
            case=truth[args.spatial_case];nom=truth['nominal']
            request=dict(delta_com=(np.array(case['c'])-nom['c']).tolist(),delta_inertia=(np.diag(case['j'])-np.diag(nom['j'])).tolist())
            (out/'oracle_request.json').write_text(json.dumps(request,indent=2)+'\n')
    startup=rootfs/'etc/init.d-posix/px4-rc.gzsim'
    startup.write_text(startup.read_text().replace('--verbose=${GZ_VERBOSE:=1} -r -s',f'--seed {args.seed} --verbose=${{GZ_VERBOSE:=1}} -r -s'))
    if args.speed!=1.:env['PX4_SIM_SPEED_FACTOR']=str(args.speed)
    procs={};logs=[];commands={}
    def launch(name,cmd,cwd):
        f=(out/(name+'.log')).open('w');logs.append(f);commands[name]=cmd
        procs[name]=subprocess.Popen(cmd,cwd=cwd,env=env,stdin=subprocess.PIPE,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,text=True)
        return procs[name]
    try:
        launch('agent',['MicroXRCEAgent','udp4','-p',str(port),'-v','4'],out)
        px=launch('px4',[str(BUILD/'bin/px4'),'-i',str(args.instance),'-w',str(rootfs)],rootfs)
        manifest=dict(kind='owned_local_sitl',namespace=token,GZ_PARTITION=token,environment={k:v for k,v in env.items() if k in ['ROS_DOMAIN_ID','ROS_AUTOMATIC_DISCOVERY_RANGE','PX4_UXRCE_DDS_PORT','GZ_IP','PX4_SIM_MODEL','GZ_DISCOVERY_MSG_PORT','GZ_DISCOVERY_SRV_PORT']},
                      pids={n:x.pid for n,x in procs.items()},commands=commands,source_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=PX4,text=True).strip())
        manifest.update(experiment={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},binary_sha256=hashlib.sha256((BUILD/'bin/px4').read_bytes()).hexdigest())
        manifest['controller_sources_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in REV2.glob('*.py')}
        snapshot=out/'source_snapshot';snapshot.mkdir()
        for file in [*ROOT.glob('*.py'),ROOT/'nominal_calibration.json']:
            shutil.copy2(file,snapshot/file.name)
        manifest['px4_consumer_sources_sha256']={str(p.relative_to(PX4)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [PX4/'src/modules/mc_rate_control/HnuterControl.cpp',PX4/'src/modules/mc_rate_control/HnuterControl.hpp',PX4/'msg/HnuterControlStatus.msg']}
        manifest['revision3_sources_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.iterdir()}
        manifest['truth_recorder_sha256']=hashlib.sha256((ROOT/'build/libJepaTruthRecorder.so').read_bytes()).hexdigest()
        (out/'launch.json').write_text(json.dumps(manifest,indent=2)+'\n')
        (ROOT/'latest_sitl_run.txt').write_text(str(out)+'\n')
        print('STARTED',out,flush=True)
        deadline=time.monotonic()+65
        while time.monotonic()<deadline:
            if px.poll() is not None: raise RuntimeError('PX4 exited during startup')
            if 'Startup script returned successfully' in (out/'px4.log').read_text(): break
            time.sleep(1)
        else: raise RuntimeError('PX4 startup timeout')
        px.stdin.write('ver all\nparam show HNTR_*\nparam show CA_AIRFRAME\nparam show UXRCE_DDS_*\n');px.stdin.flush()
        sensor=launch('sensors',['python3',str(ROOT/'record_sensors.py')],out)
        time.sleep(3)
        probe=ROOT/'spatial_probe.py' if args.spatial_case else REV2/'sitl_position_probe.py'
        cmd=['bash','-c',f'source /opt/ros/jazzy/setup.bash\nsource {REV2.parent.parent}/install/setup.bash\nexec python3 {probe}']
        node=launch('controller',cmd,out)
        manifest['commands']=commands;manifest['pids']={n:x.pid for n,x in procs.items()}
        (out/'launch.json').write_text(json.dumps(manifest,indent=2)+'\n')
        result=node.wait(timeout=170)
        print('CONTROLLER_EXIT',result,flush=True)
        return result
    finally:
        if 'px4' in procs and procs['px4'].poll() is None:
            try:
                procs['px4'].stdin.write('shutdown\n');procs['px4'].stdin.flush();time.sleep(2)
            except (BrokenPipeError,OSError): pass
        for name,proc in reversed(list(procs.items())):
            try: os.killpg(proc.pid,signal.SIGTERM)
            except ProcessLookupError: pass
        for proc in procs.values():
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try: os.killpg(proc.pid,signal.SIGKILL)
                except ProcessLookupError: pass
                proc.wait()
        for f in logs:f.close()
        (out/'process_exit.json').write_text(json.dumps({n:p.returncode for n,p in procs.items()},indent=2)+'\n')
        print('OWNED_PROCESSES_CLEANED',out,flush=True)


if __name__=='__main__': raise SystemExit(main())
