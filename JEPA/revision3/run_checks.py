"""Serial offline validation entry point; never launches PX4/DDS/Gazebo."""
from pathlib import Path
import subprocess
import sys
import json

root=Path(__file__).resolve().parent
stages=[('model_generation',[sys.executable,str(root/'assembly.py')]),
        ('unit_tests',[sys.executable,'-m','unittest','discover','-s',str(root/'tests'),'-v']),
        ('offline_experiments',[sys.executable,str(root/'offline_experiments.py')]),
        ('replay',[sys.executable,str(root/'replay.py')]),
        ('report',[sys.executable,str(root/'make_report.py')])]
for name,cmd in stages:
    with (root/'results'/f'{name}.log').open('w') as f:
        result=subprocess.run(cmd,cwd=root,stdout=f,stderr=subprocess.STDOUT)
    print(name,result.returncode,flush=True)
    if result.returncode:raise SystemExit(result.returncode)
gate=json.loads((root/'results/sitl_gate.json').read_text())
print(gate['status'], 'new SITL launches:',gate['new_sitl_launches'])
raise SystemExit(2 if gate['status']=='BLOCKED_BEFORE_LAUNCH' else 0)
