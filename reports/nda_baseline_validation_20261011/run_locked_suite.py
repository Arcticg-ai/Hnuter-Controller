"""Run qualified, fixed-parameter no-delay comparison and repair ablations."""
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path('/home/hnuter/px4_ws_ros2_consolidated')
BASE = Path(__file__).resolve().parent
env = os.environ.copy()
env.update(OPENBLAS_NUM_THREADS='1', HNUTER_NDA_GAIN='12', HNUTER_NDA_POSTURE_GAIN='2',
           HNUTER_NDA_NULL_GAIN='2', HNUTER_NDA_INVERSION='sampled',
           HNUTER_SWITCH_COMMON_PRELUDE='1', HNUTER_SWITCH_COMMON_AW_GAIN='0',
           HNUTER_DRCDA_REACTION_RATIO_M='0.016')
env.pop('HNUTER_SWITCH_JOINT_MODEL', None)
plan = [
    ('qualification_hover_common', ['--method','paper_nda','--duration','3','--span','0',
      '--post-hold','10','--position-peak-limit','.15','--attitude-peak-limit-deg','3',
      '--roll-pitch-peak-limit-deg','3'], True),
    ('qualification_low_common', ['--method','paper_nda','--duration','3','--span','.4',
      '--post-hold','5','--position-peak-limit','.25','--attitude-peak-limit-deg','5',
      '--roll-pitch-peak-limit-deg','3'], True),
    ('comparison_common', ['--method','paper_nda','--method','full','--method','direct',
      '--method','no_preview','--method','no_horizon','--duration','2.0','--duration','1.5',
      '--duration','1.1','--span','1.2','--post-hold','5'], False),
    ('nda_ablations', ['--method','paper_nda_no_posture','--method','paper_nda_continuous',
      '--duration','3','--span','0','--post-hold','10',
      '--position-peak-limit','.15','--attitude-peak-limit-deg','3',
      '--roll-pitch-peak-limit-deg','3'], False),
]
(BASE/'locked_plan.json').write_text(json.dumps({'environment':{k:v for k,v in env.items()
    if k.startswith('HNUTER_') or k=='OPENBLAS_NUM_THREADS'}, 'plan':plan}, indent=2)+'\n')
for name, args, required in plan:
    print(f'BEGIN {name}', flush=True)
    command = [sys.executable, str(ROOT/'tools/experiments/run_fixed_attitude_switching.py'),
               '--output', str(BASE/'runs'/name),'--no-gate','--dwell','0','--repeat','2', *args]
    result = subprocess.run(command, cwd=ROOT, env=env)
    manifest = BASE/'runs'/name/'manifest.json'
    if required and (result.returncode != 0 or not manifest.exists()):
        raise SystemExit(f'Qualification failed: {name}; comparison not started')
    print(f'END {name} exit={result.returncode}', flush=True)
