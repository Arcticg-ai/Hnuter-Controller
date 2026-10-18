"""Probe installed RNG in fresh processes; no simulator or vehicle is created."""
from pathlib import Path
import json,subprocess
R=Path(__file__).resolve().parent
rows={}
for mode in ['legacy','fixed']:
 for seed in [0,1]:
  rows[f'{mode}_{seed}']=[json.loads(subprocess.check_output([str(R/'build/seeded_server'),'--probe',mode,str(seed)],text=True)) for _ in range(2)]
checks={'fixed_zero_repeatable':rows['fixed_0'][0]==rows['fixed_0'][1],'fixed_one_repeatable':rows['fixed_1'][0]==rows['fixed_1'][1],'one_matches_legacy':rows['fixed_1'][0]==rows['legacy_1'][0],'zero_explicit':all(x['effective']==0 and x['set_called'] for x in rows['fixed_0']),'legacy_zero_unset':all(not x['set_called'] for x in rows['legacy_0']),'different_registered_streams':rows['fixed_0'][0]['draws']!=rows['fixed_1'][0]['draws']}
result={'rows':rows,'checks':checks,'passed':all(checks.values())};(R/'results/seed_runtime.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));assert result['passed']
