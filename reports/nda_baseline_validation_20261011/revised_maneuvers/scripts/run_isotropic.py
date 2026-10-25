import argparse
import json
import os
from pathlib import Path
import sys

ROOT=Path('/home/hnuter/px4_ws_ros2_consolidated');sys.path.insert(0,str(ROOT))
from tools.experiments.run_maneuver_validation import run_case

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
p.add_argument('--method',action='append',default=None)
p.add_argument('--module',choices=['isotropic_revision','unit_gain_revision'],default='isotropic_revision')
p.add_argument('--scenario',action='append',default=None);a=p.parse_args()
HERE=Path(__file__).resolve().parent;a.output.mkdir(exist_ok=False)
os.environ.update(OPENBLAS_NUM_THREADS='1',CCACHE_DISABLE='1',HNUTER_CONTROL_MODE='direct',
    HNUTER_DIRECT_PUBLISH_ALLOCATOR_SETPOINTS='0',HNUTER_DIRECT_PUBLISH_THRUST_SETPOINT='1',
    HNUTER_NDA_GAIN='12',HNUTER_NDA_NULL_GAIN='2',HNUTER_NDA_POSTURE_GAIN='2',HNUTER_NDA_INVERSION='sampled',
    HNUTER_SWITCH_COMMON_PRELUDE='1',HNUTER_SWITCH_COMMON_AW_GAIN='0',HNUTER_DRCDA_REACTION_RATIO_M='0.016',
    HNUTER_REV_RAMP_S='.75',HNUTER_REV_NEAR_GAIN='.25',HNUTER_REV_TILT_S='15',
    HNUTER_REV_EARLY_HANDOVER='1',HNUTER_REV_SDF_GEOMETRY='1')
os.environ.pop('HNUTER_SWITCH_JOINT_MODEL',None)
os.environ['PYTHONPATH']=str(HERE)+':'+str(ROOT)+':'+os.environ.get('PYTHONPATH','')
manifest=dict(definition='Single allocation-model/objective candidate; same final geometry and constraints',module=a.module,cases=[])
for scenario in a.scenario or ['attitude_80_180','lissajous_7','lissajous_5']:
    for method in a.method or ['full']:
        result=run_case(a.output,scenario,method,1,False,
            '/home/hnuter/px4_ws_ros2/px4-venv/bin/python',a.module,185. if scenario.startswith('attitude') else 40.)
        manifest['cases'].append(result)
        (a.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        if not result['metrics'].get('evaluated_duration_s'):raise SystemExit('No evaluated samples')
