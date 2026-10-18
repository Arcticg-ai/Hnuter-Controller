from pathlib import Path
import argparse,json,hashlib
import numpy as np
from aligned import Aligner,old_events,rollout,basis
R=Path(__file__).resolve().parent
def main(raw):
    out=R/'results/data';out.mkdir(exist_ok=False);manifest=[]
    for i,name in enumerate(['s0_00_nominal','s1_00_mass','s2_00_com','s3_00_inertia','s4_00_combo']):
        source=raw/name/'deployable_observations.jsonl';rows=[json.loads(s) for s in source.read_text().splitlines()];align=Aligner();grid=[]
        for e in old_events(rows):grid.extend(align.feed(e))
        records=[]
        for j,s in enumerate(grid[:-4]):
            future=[grid[j+k] for k in [1,2,4]]
            if not 18<=s['elapsed']<67.5 or len(s['history'])!=16 or any(len(z['history'])!=16 for z in future) or grid[j+4]['t_us']-s['t_us']!=400000:continue
            acts=np.concatenate([grid[j+k]['actions_from_previous'] for k in range(1,5)]);physical=rollout(s['x'],acts);rate=(physical-s['state'])/np.array([.1,.2,.4])[:,None]
            records.append((s['history'],np.stack([z['history'] for z in future]),acts,s['state'],np.stack([z['state'] for z in future]),physical,basis(rate[0]),s['elapsed'],s['t_us']))
        keys=['xp','xf','actions','current','target','physical','phi','elapsed','t_us'];data={k:np.asarray(v,dtype=np.int64 if k=='t_us' else np.float32) for k,v in zip(keys,zip(*records))};np.savez_compressed(out/(name+'.npz'),**data);manifest.append(dict(name=name,split='validation' if i==4 else 'train',windows=len(records),trajectories=1,rejected_grid=align.rejects,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),source=str(source)));print(manifest[-1],flush=True)
    (out/'manifest.json').write_text(json.dumps(dict(groups=manifest,grid_s=.1,actions='completed_interval_ZOH_means_2x50ms',old_action_resolution='20Hz_snapshot_not_full_callback',truth_read=False,parameters_read=False),indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('raw',type=Path);main(p.parse_args().raw)
