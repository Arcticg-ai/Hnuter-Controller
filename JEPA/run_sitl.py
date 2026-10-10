"""Own and reap an isolated nominal PX4/Gazebo/DDS baseline experiment."""
import argparse
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
PX4=Path('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl')
BUILD=PX4/'build/px4_sitl_default'


def main():
    p=argparse.ArgumentParser();p.add_argument('--label',default='nominal')
    p.add_argument('--loaded',action='store_true');p.add_argument('--instrument',action='store_true')
    p.add_argument('--speed',type=float,default=1.)
    p.add_argument('--smooth-takeoff',action='store_true')
    p.add_argument('--group',choices=['A','B','C','D','C_shadow'],default='C_shadow')
    p.add_argument('--noise',choices=['native','zero','low_noise','noisy_delayed'],default='native')
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--instance',type=int,default=0)
    args=p.parse_args()
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
        if args.loaded:source=ROOT/'generated/native_noise/hnuter_mass_cal_payload/model.sdf'
        tree=ET.parse(source);model=tree.getroot().find('model')
        if args.noise in ('zero','low_noise'):
            for noise in model.findall('.//sensor//noise'):
                for child in noise:
                    if child.tag in ('mean','stddev','bias_mean','bias_stddev','dynamic_bias_stddev'):
                        # Exact-zero gyro/accel/baro/mag trip unchanged-value validators.
                        child.text=str(float(child.text)*.01) if args.noise=='low_noise' and child.tag=='stddev' else '0'
        model_name='hnuter_jepa';model.set('name',model_name)
        for uri in model.findall('.//mesh/uri'):
            if not uri.text.startswith(('/', 'model://', 'file://')):
                uri.text=str(source.parent/uri.text)
        plugin=ET.SubElement(model,'plugin',filename='gz-sim-joint-state-publisher-system',name='gz::sim::systems::JointStatePublisher')
        ET.SubElement(plugin,'topic').text='/jepa_joint_sensors'
        models=out/'models';folder=models/model_name;folder.mkdir(parents=True)
        tree.write(folder/'model.sdf',encoding='utf-8',xml_declaration=True)
        export_geometry(PX4/'Tools/simulation/gz/models/hnuter/model.sdf',out/'actuator_geometry.json')
        with (rootfs/'gz_env.sh').open('a') as f:
            f.write(f'\nexport PX4_GZ_MODELS={models}\nexport GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:{models}\n')
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
    env['JEPA_SMOOTH_TAKEOFF']='1' if args.smooth_takeoff else '0'
    env['JEPA_GROUP']=args.group;env['JEPA_NOISE']=args.noise;env['JEPA_SEED']=str(args.seed)
    if args.group=='D':env['PX4_PARAM_HNTR_MASS']='5.39999998' if args.loaded else '4.49999998'
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
        manifest=dict(kind='owned_local_sitl',namespace=token,GZ_PARTITION=token,environment={k:v for k,v in env.items() if k in ['ROS_DOMAIN_ID','ROS_AUTOMATIC_DISCOVERY_RANGE','PX4_UXRCE_DDS_PORT','GZ_IP','PX4_SIM_MODEL']},
                      pids={n:x.pid for n,x in procs.items()},commands=commands,source_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=PX4,text=True).strip())
        manifest.update(experiment=vars(args),binary_sha256=hashlib.sha256((BUILD/'bin/px4').read_bytes()).hexdigest())
        manifest['controller_sources_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in ROOT.glob('*.py')}
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
        time.sleep(3)
        cmd=['bash','-c',f'source /opt/ros/jazzy/setup.bash\nsource {ROOT.parent}/install/setup.bash\nexec python3 {ROOT}/sitl_position_probe.py']
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
