"""Owned, serial PX4/Gazebo run. No host network, hardware or shared rootfs."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET
from model import PX4,WS,ROOT,write
from owned_cleanup import cleanup_owned,RunMonitor

def run(experiment,case_id,mode,seed,out,checkpoint=None,startup_only=False,clock_audit=False):
    if os.environ.get('HNUTER_CONTAINER')!='1':raise RuntimeError('Use the isolated Docker container')
    assert mode=='baseline' and os.environ.get('R12_ARM','00') in ('00','10','01','11')
    case=next(c for c in json.loads((experiment/'cases.json').read_text()) if c['id']==case_id)
    cfg=json.loads((experiment/'config.json').read_text())
    out.mkdir(parents=True,exist_ok=False);rootfs=out/'rootfs';rootfs.mkdir()
    snapshot=out/'source_snapshot';snapshot.mkdir()
    for source in ROOT.iterdir():
        if source.is_file() and source.suffix in ('.py','.sh','.json','.cc'):shutil.copy2(source,snapshot/source.name)
    build=PX4/'build/px4_sitl_diag';shutil.copytree(build/'etc',rootfs/'etc')
    (rootfs/'etc/init.d-posix/px4-rc.mavlink').write_text('#!/bin/sh\n# No MAVLink in this owned experiment.\n')
    tree=ET.parse(case['model']);model=tree.getroot().find('model');model.set('name','hnuter_jepa')
    # Diagnostic-only replacement: source parity verifies original control code.
    for servo in model.findall('plugin'):
        if servo.get('name')=='gz::sim::systems::JointPositionController':
            joint=servo.findtext('joint_name')
            servo.set('filename',str(ROOT/'build/libAuditedJointPositionController.so'))
            servo.set('name','gz::sim::systems::AuditedJointPositionController')
            ET.SubElement(servo,'diagnostic_output').text=str(out/('servo_'+joint+'.csv'))
    if cfg['noise']!='native':raise ValueError('First package uses native sensor noise only')
    plugin=ET.SubElement(model,'plugin',filename='gz-sim-joint-state-publisher-system',name='gz::sim::systems::JointStatePublisher')
    ET.SubElement(plugin,'topic').text='/jepa_joint_sensors'
    truth_plugin=ROOT/'build/libJepaTruthRecorder.so'
    plugin=ET.SubElement(model,'plugin',filename=str(truth_plugin),name='jepa::TruthRecorder')
    ET.SubElement(plugin,'output').text=str(out/'truth_diagnostic.csv')
    clock_audit=True
    if clock_audit:
        plugin=ET.SubElement(model,'plugin',filename=str(ROOT/'build/libJepaClockAudit.so'),name='jepa::ClockAudit')
        ET.SubElement(plugin,'output').text=str(out/'clock_audit.csv')
    arm=os.environ.get('R12_ARM','00');assert arm in ('00','10','01','11')
    disturbance=dict(case['disturbance']);disturbance['force_y']=.6 if arm[0]=='1' else 0.
    plugin=ET.SubElement(model,'plugin',filename=str(ROOT/'build/libJepaBoundedForce.so'),name='jepa::BoundedForce')
    for key,value in disturbance.items():ET.SubElement(plugin,key).text=str(value)
    ET.SubElement(plugin,'output').text=str(out/'external_force_truth.csv')
    ET.SubElement(plugin,'anchor_file').text=str(out/'reference_epoch.txt')
    if mode=='force_cancel':write(out/'diagnostic_force_schedule.json',dict(**disturbance,mass_kg=case['truth']['mass']))
    models=out/'models';folder=models/'hnuter_jepa';folder.mkdir(parents=True)
    tree.write(folder/'model.sdf',encoding='utf-8',xml_declaration=True)
    from actuator_sensors import export_geometry
    export_geometry(PX4/'Tools/simulation/gz/models/hnuter/model.sdf',out/'actuator_geometry.json')
    # Isolated 1 ms physics and native stock 1 ms PD; original 250 Hz IMU and gains.
    world=ET.parse(ROOT/'flight_worlds/default.sdf')
    assert abs(float(world.getroot().findtext('world/physics/max_step_size'))-.001)<1e-9
    (rootfs/'gz_env.sh').write_text(f'''export PX4_GZ_MODELS={models}
export PX4_GZ_WORLDS={ROOT}/flight_worlds
export PX4_GZ_PLUGINS={PX4}/build/px4_sitl_default/src/modules/simulation/gz_plugins
export PX4_GZ_SERVER_CONFIG={PX4}/src/modules/simulation/gz_bridge/server.config
export GZ_SIM_RESOURCE_PATH={models}:{PX4}/Tools/simulation/gz/models:{ROOT}/flight_worlds
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PX4_GZ_PLUGINS
export GZ_SIM_SERVER_CONFIG_PATH=$PX4_GZ_SERVER_CONFIG
''')
    token='adapt_'+uuid.uuid4().hex[:12]
    profile=out/'dds_loopback.xml'
    profile.write_text('''<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"><transport_descriptors><transport_descriptor><transport_id>local</transport_id><type>UDPv4</type><interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList></transport_descriptor></transport_descriptors><participant profile_name="local" is_default_profile="true"><rtps><userTransports><transport_id>local</transport_id></userTransports><useBuiltinTransports>false</useBuiltinTransports></rtps></participant></profiles>''')
    env={k:v for k,v in os.environ.items() if not k.startswith(('PX4_','ROS_','GZ_','HNUTER_','RMW_'))}
    # Restore only the container marker, never host flight-control settings.
    env.update(HNUTER_CONTAINER='1',HEADLESS='1',PX4_SIM_MODEL='gz_hnuter_jepa',PX4_SYS_AUTOSTART='4051',PX4_GZ_WORLD='default',
       GZ_IP='127.0.0.1',GZ_PARTITION=token,ROS_DOMAIN_ID='83',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
       PX4_UXRCE_DDS_PORT='29888',PX4_UXRCE_DDS_NS=token,PX4_PARAM_UXRCE_DDS_SYNCT='0',PX4_PARAM_HNTR_MASS=str(cfg['controller_mass_kg']),
       FASTDDS_DEFAULT_PROFILES_FILE=str(profile),FASTRTPS_DEFAULT_PROFILES_FILE=str(profile),RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
       ROS_LOG_DIR=str(out/'ros_logs'),JEPA_RUN_DIR=str(out),JEPA_GROUP='A',JEPA_NOISE='native',JEPA_SEED=str(seed),JEPA_GPS_SEED=str(seed),JEPA_GPS_AUDIT=str(out/'gps_noise_audit.csv'),
       JEPA_EXCITATION=case['excitation'],JEPA_ATTITUDE_REFERENCE='legacy',JEPA_SMOOTH_TAKEOFF='1',JEPA_STARTUP_ONLY=str(int(startup_only)),
       R12_ARM=arm,R6_MODE=mode,ADAPTATION_MODE=('oracle' if mode.startswith('oracle') else 'baseline'),ADAPTATION_CHECKPOINT=str(checkpoint or ''),PYTHONUNBUFFERED='1',
       SDL_VIDEODRIVER='dummy',SDL_AUDIODRIVER='dummy')
    if startup_only:env['JEPA_UNARMED_HOLD_S']=os.environ.get('JEPA_UNARMED_HOLD_S','0')
    startup=rootfs/'etc/init.d-posix/px4-rc.gzsim'
    from seed_launch import patch_startup
    startup.write_text(patch_startup(startup.read_text(), seed))
    if mode.startswith('oracle'):
        request=list(case['target'])
        if mode=='oracle_com':request[3:]=[0.,0.,0.]
        if mode=='oracle_inertia':request[1:3]=[0.,0.]
        write(out/'oracle_request.json',request)
    write(out/'experiment.json',dict(case_id=case_id,mode=mode,seed=seed,checkpoint=str(checkpoint),startup_only=startup_only,clock_audit=clock_audit))
    procs={};logs=[];commands={};manifest={};failure=None
    monitor=RunMonitor(out/'resource_monitor.jsonl',procs)
    def termination(signum,frame):raise RuntimeError(f'launcher_signal_{signum}')
    old_term=signal.signal(signal.SIGTERM,termination)
    def launch(name,cmd,cwd):
        f=(out/(name+'.log')).open('w');logs.append(f);commands[name]=cmd
        procs[name]=subprocess.Popen(cmd,cwd=cwd,env=env,stdin=subprocess.PIPE,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,text=True)
        return procs[name]
    try:
        launch('agent',['MicroXRCEAgent','udp4','-p','29888','-v','2'],out)
        px=launch('px4',[str(build/'bin/px4'),'-i','0','-w',str(rootfs)],rootfs)
        manifest=dict(kind='owned_local_sitl',namespace=token,GZ_PARTITION=token,experiment={'instance':0},
                      pids={k:p.pid for k,p in procs.items()},commands=commands,
                      binary_sha256=hashlib.sha256((build/'bin/px4').read_bytes()).hexdigest(),
                      source_manifest_sha256=hashlib.sha256((ROOT/'SOURCE_MANIFEST.json').read_bytes()).hexdigest(),
                      world_sha256=hashlib.sha256((ROOT/'flight_worlds/default.sdf').read_bytes()).hexdigest(),physics_step_s=.001,stock_servo_period_s=.001,imu_requested_hz=250,
                      model_sha256=hashlib.sha256((folder/'model.sdf').read_bytes()).hexdigest(),
                      local_source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.iterdir()},
                      platform_image='hnuter-adaptation:20261013',network='container-none',hardware_devices=[],
                      controller_mass_kg=cfg['controller_mass_kg'],clock_audit=clock_audit)
        write(out/'launch.json',manifest)
        deadline=time.monotonic()+80
        while time.monotonic()<deadline:
            if px.poll() is not None:raise RuntimeError('PX4 exited during startup')
            if 'Startup script returned successfully' in (out/'px4.log').read_text():break
            time.sleep(1)
        else:raise RuntimeError('PX4 startup timeout')
        receipt=json.loads((out/'seed_receipt.json').read_text())
        assert receipt['requested_seed']==seed and receipt['effective_seed_before_server']==seed and receipt['set_seed_called'] and receipt['draws_consumed_by_audit']==0
        assert f'JEPA GPS private MT19937 seed={seed}' in (out/'px4.log').read_text()
        px.stdin.write('ver all\nparam show HNTR_*\n');px.stdin.flush()
        launch('sensors',[sys.executable,str(WS/'JEPA/revision3/articulated/record_sensors.py')],out)
        # R20 readiness/arming are governed by physical clock and sample counts.
        if not startup_only:launch('pre_gate',[sys.executable,str(ROOT/'live_pre_gate.py'),str(out)],out)
        if not startup_only:launch('adaptation',[sys.executable,'/r6/online.py',str(out)],out)
        node=launch('controller',[sys.executable,str(ROOT/'r20_probe.py')],out)
        manifest.update(pids={k:p.pid for k,p in procs.items()},commands=commands);write(out/'launch.json',manifest)
        code=node.wait(timeout=180)
        if not startup_only and procs['adaptation'].wait(timeout=15)!=0:raise RuntimeError('online_adapter_failed')
        if code:failure=f'controller_exit_{code}'
        elif not startup_only:
            from network import MAX_OBSERVATION_GAP_S
            rows=[json.loads(line) for line in (out/'observations.jsonl').read_text().splitlines()]
            cal=[row for row in rows if 18<=row['elapsed']<=34]
            gap=max((b['t']-a['t'] for a,b in zip(cal,cal[1:])),default=999.)
            quality=dict(samples=len(cal),max_gap_s=gap,minimum_samples=280,
                         maximum_gap_s=MAX_OBSERVATION_GAP_S,usable=len(cal)>=280 and gap<=MAX_OBSERVATION_GAP_S)
            write(out/'observation_quality.json',quality)
            if not quality['usable']:failure='insufficient_or_fragmented_calibration_observations'
    except BaseException as exc:
        failure=type(exc).__name__+': '+str(exc)
    finally:
        cleanup=cleanup_owned(procs)
        monitor.stop();signal.signal(signal.SIGTERM,old_term)
        for f in logs:f.close()
        result_path=out/'flight_result.json'
        if not result_path.exists():write(result_path,dict(status='FAIL',failure=failure or 'missing_controller_result',landed=None,disarmed=None,startup_only=startup_only))
        result=json.loads(result_path.read_text())
        confirmed=bool(not failure and result.get('status')=='PASS' and result.get('landed') and result.get('disarmed') and cleanup['processes'].get('controller')==0)
        write(out/'process_exit.json',dict(failure=failure,**cleanup,flight_success_confirmed=confirmed,flight_result=result.get('status'),landed=result.get('landed'),disarmed=result.get('disarmed')))
        if not cleanup['all_launched_parents_exited']:failure=failure or 'owned_process_cleanup_incomplete'
    if failure:raise RuntimeError(failure)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('experiment',type=Path);p.add_argument('case');p.add_argument('mode',choices=['baseline','zero','oracle','oracle_com','oracle_inertia','baseline_force','oracle_force','force_cancel'])
    p.add_argument('--seed',type=int,default=0);p.add_argument('--out',type=Path,required=True);p.add_argument('--checkpoint',type=Path);p.add_argument('--clock-audit',action='store_true');p.add_argument('--startup-only',action='store_true')
    a=p.parse_args();run(a.experiment,a.case,a.mode,a.seed,a.out,a.checkpoint,a.startup_only,a.clock_audit)
