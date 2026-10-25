"""Frozen matched no-delay matrix after magnetometer and servo-DC repair."""
import hashlib
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tarfile
import time
import xml.etree.ElementTree as ET

ROOT=Path('/home/hnuter/px4_ws_ros2_consolidated')
OUT=Path(__file__).resolve().parent/'postfix_suite_20261012'
sys.path.insert(0,str(ROOT))
from tools.experiments import run_maneuver_validation as runner
from tools.experiments.run_no_delay_maneuver_experiments import DEFAULT_FIRMWARE,firmware_metadata


class OwnedProcess(runner.PtyProcess):
    def stop(self,*args,**kwargs):
        try:
            super().stop(*args,**kwargs)
        finally:
            # A successful make exit must not leave its Gazebo children alive.
            try:os.killpg(self.process.pid,signal.SIGTERM)
            except ProcessLookupError:pass


def main():
    global OUT
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=OUT)
    args=parser.parse_args();OUT=args.output.resolve()
    if (OUT/'manifest.json').exists():raise SystemExit('Use a new output directory; refusing to overwrite a run')
    runner.PtyProcess=OwnedProcess
    fw=Path(DEFAULT_FIRMWARE)
    # Finish any dependency-triggered rebuild before hashing or starting SITL.
    build_env=os.environ.copy();build_env['CCACHE_DISABLE']='1'
    subprocess.run(['make','px4_sitl_default'],cwd=fw,env=build_env,check=True)
    metadata=firmware_metadata(fw)
    assert not metadata['dynamic_actuator_tokens_present']
    for name in ['Tools/simulation/gz/server.config','src/modules/simulation/gz_bridge/server.config']:
        plugin=ET.parse(fw/name).getroot().find("plugins/plugin[@name='gz::sim::systems::Magnetometer']")
        assert plugin.findtext('use_units_gauss')=='true'
        assert plugin.findtext('use_earth_frame_ned')=='false'
    bridge=(fw/'src/modules/simulation/gz_bridge/GZBridge.cpp').read_text()
    section=bridge.split('void GZBridge::magnetometerCallback',1)[1].split('void GZBridge::airPressureCallback',1)[0]
    for line in ['report.x = msg.field_tesla().x();','report.y = -msg.field_tesla().y();','report.z = -msg.field_tesla().z();']:
        assert line in section
    for key in list(os.environ):
        if key.startswith('HNUTER_'):os.environ.pop(key)
    os.environ.pop('PX4_SIM_SPEED_FACTOR',None)
    os.environ.update(OPENBLAS_NUM_THREADS='1',CCACHE_DISABLE='1',HNUTER_CONTROL_MODE='direct',
        HNUTER_DIRECT_PUBLISH_ALLOCATOR_SETPOINTS='0',HNUTER_DIRECT_PUBLISH_THRUST_SETPOINT='1',
        HNUTER_NDA_GAIN='12',HNUTER_NDA_NULL_GAIN='2',HNUTER_NDA_POSTURE_GAIN='2',HNUTER_NDA_INVERSION='sampled',
        HNUTER_SWITCH_COMMON_PRELUDE='1',HNUTER_SWITCH_COMMON_AW_GAIN='0',HNUTER_DRCDA_REACTION_RATIO_M='0.016',
        HNUTER_REV_RAMP_S='0.75',HNUTER_REV_NEAR_GAIN='0.25',HNUTER_REV_TILT_S='15',
        HNUTER_REV_EARLY_HANDOVER='1',HNUTER_REV_SDF_GEOMETRY='1',HNUTER_REV_UNIT_DC_GAIN='1')
    methods=['direct','paper_nda','full','no_preview','no_horizon','no_nearfield']
    scenarios=['lissajous_7','lissajous_5','attitude_80_180']
    files=['src/modules/simulation/gz_bridge/GZBridge.cpp','src/modules/simulation/gz_bridge/server.config',
        'Tools/simulation/gz/server.config','Tools/simulation/gz/models/hnuter/model.sdf',
        'src/modules/land_detector/MulticopterLandDetector.cpp','src/modules/land_detector/MulticopterLandDetector.h',
        'src/modules/land_detector/HnuterDirectLandingGuard.hpp','build/px4_sitl_default/bin/px4']
    manifest=dict(firmware=metadata,firmware_sha256={f:hashlib.sha256((fw/f).read_bytes()).hexdigest() for f in files},
        methods=methods,scenarios=scenarios,repeats=1,cases=[],
        environment={k:v for k,v in os.environ.items() if k.startswith('HNUTER_')},
        scope='Matched post-fix SITL; one run per condition; no online retuning',started_unix=time.time(),
        gazebo_version=subprocess.check_output(['gz','sim','--versions'],text=True).strip())
    OUT.mkdir(exist_ok=True)
    (OUT/'provenance').mkdir(exist_ok=True)
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    with tarfile.open(OUT/'provenance/controller_source.tar.gz','w:gz') as tar:
        for folder in ['controllers','tools/experiments','config/experiments','tests']:
            for f in (ROOT/folder).rglob('*'):
                if f.is_file() and '__pycache__' not in str(f):tar.add(f,arcname=str(f.relative_to(ROOT)))
    with tarfile.open(OUT/'provenance/firmware_source.tar.gz','w:gz') as tar:
        for f in files:
            if not f.startswith('build/'):tar.add(fw/f,arcname=f)
    (OUT/'provenance/firmware.patch').write_text(subprocess.check_output(['git','diff'],cwd=fw,text=True))
    (OUT/'provenance/controller.patch').write_text(subprocess.check_output(['git','diff'],cwd=ROOT,text=True))
    for scenario in scenarios:
        for method in methods:
            assert all(hashlib.sha256((fw/f).read_bytes()).hexdigest()==v for f,v in manifest['firmware_sha256'].items())
            print(f'START {scenario}/{method}',flush=True)
            result=runner.run_case(OUT/'runs',scenario,method,1,False,
                '/home/hnuter/px4_ws_ros2/px4-venv/bin/python','controllers.experiments.maneuver_revision',
                190. if scenario.startswith('attitude') else 40.)
            manifest['cases'].append(result)
            (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
            if not result['csv'] or not result['metrics'].get('evaluated_duration_s'):
                raise RuntimeError('Infrastructure failed; retained evidence and halted matrix')
    manifest['finished_unix']=time.time()
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('POSTFIX_SUITE_DONE',flush=True)


if __name__=='__main__':main()
