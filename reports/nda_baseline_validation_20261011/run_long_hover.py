"""Supplement the locked comparison with a longer, unchanged NDA hover."""
import json
import os
from pathlib import Path
import subprocess
import sys

BASE = Path(__file__).resolve().parent
ROOT = Path('/home/hnuter/px4_ws_ros2_consolidated')
env = os.environ.copy()
env.update(json.loads((BASE/'locked_plan.json').read_text())['environment'])
env.pop('HNUTER_SWITCH_JOINT_MODEL', None)
command = [sys.executable, str(ROOT/'tools/experiments/run_fixed_attitude_switching.py'),
           '--output', str(BASE/'runs/qualification_hover_long'),
           '--method', 'paper_nda', '--duration', '3', '--span', '0', '--no-gate',
           '--dwell', '0', '--post-hold', '60', '--repeat', '1',
           '--position-peak-limit', '.15', '--attitude-peak-limit-deg', '3',
           '--roll-pitch-peak-limit-deg', '3']
(BASE/'long_hover_command.json').write_text(json.dumps(command, indent=2)+'\n')
raise SystemExit(subprocess.run(command, cwd=ROOT, env=env).returncode)
