"""Explicitly configured, no-delay-only revision experiments."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from tools.experiments.run_maneuver_validation import run_case
from tools.experiments.run_no_delay_maneuver_experiments import DEFAULT_FIRMWARE,firmware_metadata


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--method',action='append',choices=['direct','paper_nda','full','no_preview','no_horizon','no_nearfield'])
    p.add_argument('--scenario',action='append',choices=['lissajous_7','lissajous_5','attitude_60_60','attitude_80_180'])
    p.add_argument('--repeat',type=int,default=1)
    p.add_argument('--ramp',type=float,default=.75)
    p.add_argument('--near-gain',type=float,default=.25)
    p.add_argument('--tilt-seconds',type=float,default=15.)
    p.add_argument('--late-handover',action='store_true')
    p.add_argument('--legacy-geometry',action='store_true')
    p.add_argument('--gui',action='store_true')
    p.add_argument('--controller-python',default='/home/hnuter/px4_ws_ros2/px4-venv/bin/python')
    a=p.parse_args()
    if a.ramp<0 or a.ramp>=5 or a.tilt_seconds<=0: p.error('invalid trajectory timing')
    metadata=firmware_metadata(DEFAULT_FIRMWARE)
    if metadata['dynamic_actuator_tokens_present']:raise SystemExit('Refusing delayed firmware')
    a.output.mkdir(parents=True,exist_ok=False)
    os.environ.update(OPENBLAS_NUM_THREADS='1',CCACHE_DISABLE='1',HNUTER_CONTROL_MODE='direct',
        HNUTER_DIRECT_PUBLISH_ALLOCATOR_SETPOINTS='0',HNUTER_DIRECT_PUBLISH_THRUST_SETPOINT='1',
        HNUTER_NDA_GAIN='12',HNUTER_NDA_NULL_GAIN='2',HNUTER_NDA_POSTURE_GAIN='2',HNUTER_NDA_INVERSION='sampled',
        HNUTER_SWITCH_COMMON_PRELUDE='1',HNUTER_SWITCH_COMMON_AW_GAIN='0',HNUTER_DRCDA_REACTION_RATIO_M='0.016',
        HNUTER_REV_RAMP_S=str(a.ramp),HNUTER_REV_NEAR_GAIN=str(a.near_gain),
        HNUTER_REV_TILT_S=str(a.tilt_seconds),HNUTER_REV_EARLY_HANDOVER='0' if a.late_handover else '1')
    os.environ['HNUTER_REV_SDF_GEOMETRY']='0' if a.legacy_geometry else '1'
    os.environ.pop('HNUTER_SWITCH_JOINT_MODEL',None)
    fw=Path(DEFAULT_FIRMWARE)
    files=['src/modules/land_detector/MulticopterLandDetector.cpp',
           'src/modules/land_detector/MulticopterLandDetector.h',
           'src/modules/land_detector/HnuterDirectLandingGuard.hpp',
           'Tools/simulation/gz/models/hnuter/model.sdf','build/px4_sitl_default/bin/px4']
    manifest=dict(firmware=metadata,firmware_sha256={f:hashlib.sha256((fw/f).read_bytes()).hexdigest() for f in files},
        firmware_diff=subprocess.check_output(['git','-C',str(fw),'diff'],text=True),
        options=vars(a).copy(),cases=[])
    manifest['options']['output']=str(a.output)
    for s in a.scenario or ['lissajous_7','lissajous_5','attitude_80_180']:
        for n in range(1,a.repeat+1):
            methods=a.method or ['direct','paper_nda','full','no_preview','no_horizon','no_nearfield']
            for m in methods if n%2 else reversed(methods):
                r=run_case(a.output,s,m,n,a.gui,a.controller_python,
                    'controllers.experiments.maneuver_revision',8*a.tilt_seconds+65 if s.startswith('attitude') else 40.)
                manifest['cases'].append(r)
                (a.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
                if not r['csv'] or not r['metrics'].get('evaluated_duration_s'):
                    raise SystemExit('Infrastructure failure: no evaluated samples')


if __name__=='__main__':main()
