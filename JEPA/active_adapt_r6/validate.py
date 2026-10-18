import json,time
from pathlib import Path
import numpy as np,torch
from online import Engine
R=Path(__file__).resolve().parent
e=Engine();d=np.load(R/'results/data/s4_00_combo.npz');x=(torch.tensor(d['xp'])-e.mean)/e.std;u=torch.tensor(d['actions']);h=np.array([.1,.2,.4])[None,:,None];rows=[]
for name,m in e.models.items():
    with torch.inference_mode():
        if m is None:pred=d['physical'];rank=None;change=None
        else:
            res=m(x,u).numpy()*e.ys+e.ym;pred=d['physical']+h*res;z=m.encoder(x)[0][:,-1].numpy();sv=np.linalg.svd(z-z.mean(0),compute_uv=False);p=sv**2/max(sum(sv**2),1e-20);rank=float(np.exp(-sum(p*np.log(p+1e-20))));changed=u.clone();changed[:,:,0]+=.01;change=float(abs(m(x,changed).numpy()-m(x,u).numpy()).max())
        assert np.isfinite(pred).all();err=pred-d['target'];rows.append(dict(method=name,velocity_rmse=np.sqrt(np.mean(np.sum(err[:,:,:3]**2,2),0)).tolist(),omega_rmse=np.sqrt(np.mean(np.sum(err[:,:,3:]**2,2),0)).tolist(),latent_effective_rank=rank,action_perturbation_changes_output=change))
(R/'results/validation.json').write_text(json.dumps(dict(trajectory='R4_combo_validation_only',horizons_s=[.1,.2,.4],selection_used=False,models=rows),indent=2));print(json.dumps(rows,indent=2))
