"""Archive raw evidence, parameter deltas and scoped source changes."""
import difflib
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import numpy as np
from pyulog import ULog

ROOT=Path(__file__).resolve().parent
A=ROOT/'first_roll_audit_20261012'
REPO=Path('/home/hnuter/px4_ws_ros2_consolidated')
FW=Path('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl')
DEST=REPO/'reports/first_roll_diagnosis_20261012'
RUNS=[('before',ROOT/'visual_attitude_only_20261012_01','02_50_39'),
      ('sensor_fixed',A/'sensor_fixed','03_10_05'),
      ('sensor_unit_gain',A/'sensor_unit_gain','03_14_42'),
      ('final_entry',A/'final_entry','03_20_55')]


def dump(path,data):
    path.write_text(json.dumps(data,indent=2,default=lambda x:x.item() if isinstance(x,np.generic) else str(x))+'\n')


def main():
    DEST.mkdir(parents=True,exist_ok=True)
    shutil.copytree(A,DEST/'evidence',dirs_exist_ok=True)
    shutil.copytree(RUNS[0][1],DEST/'evidence/original_visual_run',dirs_exist_ok=True)
    params={};run_checks={}
    for label,run,stamp in RUNS:
        path=FW/f'build/px4_sitl_default/rootfs/log/2026-10-12/{stamp}.ulg'
        shutil.copy2(path,DEST/'evidence'/f'{label}.ulg')
        u=ULog(str(path));params[label]=u.initial_parameters
        dump(DEST/'evidence'/f'{label}_parameters.json',u.initial_parameters)
        modes=u.get_dataset('vehicle_control_mode').data
        mask=(modes['flag_control_offboard_enabled']==1)&(modes['flag_armed']==1)
        run_checks[label]={k:np.unique(v[mask]).tolist() for k,v in modes.items() if k.startswith('flag_')}
        run_checks[label]['offboard_armed_samples']=int(mask.sum())
        run_checks[label]['parameter_changes']=u.changed_parameters
    dump(DEST/'evidence/ulog_mode_checks.json',run_checks)
    parameter_deltas={}
    for label in params:
        if label=='before':continue
        parameter_deltas[label]={k:[params['before'].get(k),v] for k,v in params[label].items()
                                 if params['before'].get(k)!=v}
    dump(DEST/'evidence/parameter_deltas_from_original.json',parameter_deltas)
    configs={label:json.loads((run/'logs/runtime_config.json').read_text()) for label,run,_ in RUNS}
    def differences(old,new,prefix=''):
        result={}
        for key in old.keys()|new.keys():
            x,y=old.get(key),new.get(key);name=prefix+key
            if isinstance(x,dict) and isinstance(y,dict):result.update(differences(x,y,name+'.'))
            elif x!=y:result[name]=[x,y]
        return result
    dump(DEST/'evidence/runtime_config_deltas.json',{
        k:differences(configs['before'],v) for k,v in configs.items() if k!='before'})
    provenance=DEST/'evidence/provenance'
    for file in ['src/modules/simulation/gz_bridge/GZBridge.cpp',
                 'src/modules/simulation/gz_bridge/server.config','Tools/simulation/gz/server.config',
                 'Tools/simulation/gz/models/hnuter/model.sdf']:
        target=provenance/'firmware'/file;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(FW/file,target)
    source=REPO/'controllers/experiments/maneuver_revision.py'
    shutil.copy2(source,provenance/'maneuver_revision.py.after')
    before=(provenance/'maneuver_revision.py.before').read_text().splitlines(True)
    after=source.read_text().splitlines(True)
    (provenance/'controller_this_turn.patch').write_text(''.join(difflib.unified_diff(before,after,
        fromfile='a/controllers/experiments/maneuver_revision.py',tofile='b/controllers/experiments/maneuver_revision.py')))
    patch=subprocess.check_output(['git','diff','--','src/modules/simulation/gz_bridge/GZBridge.cpp',
        'src/modules/simulation/gz_bridge/server.config','Tools/simulation/gz/server.config'],cwd=FW,text=True)
    (provenance/'firmware_this_turn.patch').write_text(patch)
    dump(provenance/'identities.json',{
        'firmware_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=FW,text=True).strip(),
        'controller_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
        'px4_binary_sha256':hashlib.sha256((FW/'build/px4_sitl_default/bin/px4').read_bytes()).hexdigest(),
        'sdf_sha256':hashlib.sha256((FW/'Tools/simulation/gz/models/hnuter/model.sdf').read_bytes()).hexdigest(),
        'dirty_sources':'HEAD identities do not include working changes; source snapshots and patches are included.'})
    for name in ['audit_first_roll.py','plot_first_roll.py','archive_first_roll.py','visualize_revision.py']:
        shutil.copy2(ROOT/name,DEST/name)
    shutil.copy2(REPO/'tests/experiments/test_maneuver_revision.py',provenance/'test_maneuver_revision.py')
    print(DEST)


if __name__=='__main__':main()
