from pathlib import Path
import json,time,copy,hashlib
import numpy as np,torch
from network import AdaptiveForecast,latent_loss
R=Path(__file__).resolve().parent;torch.set_num_threads(2)
def main():
    meta=json.loads((R/'results/data/manifest.json').read_text());names=[g['name'] for g in meta['groups'] if g['split']=='train'];ds=[np.load(R/'results/data'/f'{n}.npz') for n in names];data={k:torch.tensor(np.concatenate([d[k] for d in ds])) for k in ['xp','xf','actions','current','target','physical','phi']};mean=data['xp'].mean((0,1));std=data['xp'].std((0,1)).clamp_min(.01);res=(data['target']-data['physical'])/torch.tensor([.1,.2,.4])[None,:,None];ym=res.mean((0,1));ys=res.std((0,1)).clamp_min(.1);phi_mean=data['phi'].mean(0);phi_std=data['phi'].std(0).clamp_min(.02);data['xp']=(data['xp']-mean)/std;data['xf']=(data['xf']-mean)/std;target=(res-ym)/ys
    out=R/'results/models';out.mkdir(exist_ok=False);norm=dict(train_groups=names,input_mean=mean.tolist(),input_std=std.tolist(),residual_mean=ym.tolist(),residual_std=ys.tolist(),phi_mean=phi_mean.tolist(),phi_std=phi_std.tolist(),adapter_scale=ys.tolist(),truth_read=False,validation_read=False);(out/'normalization.json').write_text(json.dumps(norm,indent=2));summary=[]
    for seed in [17,29,43]:
        torch.manual_seed(seed);init=copy.deepcopy(AdaptiveForecast().state_dict())
        for method in ['supervised','jepa']:
            torch.manual_seed(seed);rng=np.random.default_rng(seed);m=AdaptiveForecast();m.load_state_dict(init);opt=torch.optim.Adam(m.parameters(),lr=.001);curve=[];start=time.perf_counter()
            for epoch in range(24):
                if method=='jepa' and epoch==12:opt=torch.optim.Adam(m.parameters(),lr=.001)
                stage='latent' if method=='jepa' and epoch<12 else 'joint' if method=='jepa' else 'state';losses=[]
                for idx in np.array_split(rng.permutation(len(target)),int(np.ceil(len(target)/96))):
                    x=data['xp'][idx];u=data['actions'][idx]
                    if stage=='latent':loss=latent_loss(m,x,u,data['xf'][idx])
                    else:
                        loss=(m(x,u)-target[idx]).square().mean()
                        if stage=='joint':loss=loss+.1*latent_loss(m,x,u,data['xf'][idx])
                    assert torch.isfinite(loss);opt.zero_grad();loss.backward();torch.nn.utils.clip_grad_norm_(m.parameters(),1.);opt.step();losses.append(float(loss.detach()))
                curve.append(dict(epoch=epoch+1,stage=stage,loss=float(np.mean(losses))))
            m.eval();m.requires_grad_(False);file=out/f'{method}_{seed}.pt';torch.save(dict(state=m.state_dict(),method=method,seed=seed,normalization=norm,checkpoint='fixed_final_epoch',online_backbone_updates=False,control_enabled=False),file);(out/f'{method}_{seed}_curve.json').write_text(json.dumps(curve,indent=2));row=dict(method=method,seed=seed,parameters=sum(p.numel() for p in m.parameters()),wall_s=time.perf_counter()-start,sha256=hashlib.sha256(file.read_bytes()).hexdigest());summary.append(row);print(row,flush=True)
    (out/'summary.json').write_text(json.dumps(dict(models=summary,training_trajectories=4,validation_read=False,test_read=False),indent=2))
if __name__=='__main__':main()
