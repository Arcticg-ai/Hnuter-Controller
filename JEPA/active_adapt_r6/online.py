"""Independent shadow process. Reads receipts, updates only 6x4 coefficients.

No command publisher and no path to Gazebo truth/configuration. All labels arrive
before updates, and scores use the adapter snapshot BEFORE that label's update.
"""
from pathlib import Path
import argparse,json,time,hashlib
import numpy as np,torch
from aligned import Aligner,rollout,basis
from network import AdaptiveForecast
from adapter import Adapter
R=Path(__file__).resolve().parent
def weights_hash(m):return hashlib.sha256(b''.join(x.detach().cpu().numpy().tobytes() for x in m.state_dict().values())).hexdigest()
class Engine:
    def __init__(self):
        torch.set_num_threads(1);self.norm=json.loads((R/'results/models/normalization.json').read_text());n=self.norm;self.mean=torch.tensor(n['input_mean']);self.std=torch.tensor(n['input_std']);self.ym=np.array(n['residual_mean']);self.ys=np.array(n['residual_std']);self.pm=np.array(n['phi_mean']);self.ps=np.array(n['phi_std']);self.models={'physics':None};self.hashes={}
        for path in sorted((R/'results/models').glob('*.pt')):
            ck=torch.load(path,map_location='cpu',weights_only=True);m=AdaptiveForecast();m.load_state_dict(ck['state']);m.eval();m.requires_grad_(False);self.models[path.stem]=m;self.hashes[path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
        self.memory_hashes={k:weights_hash(m) for k,m in self.models.items() if m is not None};self.adapters={k:Adapter(n['adapter_scale']) for k in self.models};self.previous=None;self.align=Aligner();self.rows=0
    def step(self,e):
        outputs=[];aligned=[]
        for s in self.align.feed(e):
            aligned.append({k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in s.items() if k!='history'})
            p=self.previous;self.previous=s
            if p is None or s['t_us']-p['t_us']!=100000 or len(p['history'])!=16:continue
            before=time.perf_counter_ns();physical=rollout(p['x'],s['actions_from_previous']);base=(physical-p['state'])/.1;phi=(basis(base)-self.pm)/self.ps;x=(torch.tensor(p['history'],dtype=torch.float32)[None]-self.mean)/self.std;u=torch.tensor(s['actions_from_previous'],dtype=torch.float32)[None];valid=bool(not np.any(p['x'][41:43]) and not np.any(s['x'][41:43]) and np.max(s['x'][39:41])<=.12);target=(s['state']-p['state'])/.1
            batch=[]
            for name,m in self.models.items():
                t0=time.perf_counter_ns();a=self.adapters[name]
                with torch.inference_mode():res=np.zeros(6) if m is None else m(x,u).numpy()[0,0]*self.ys+self.ym
                baseline=base+res;on=baseline+a.predict(phi,True);off=baseline+a.predict(phi,False)
                row=dict(method=name,t_us=p['t_us'],end_us=s['t_us'],elapsed=p['elapsed'],target=s['state'].tolist(),pred_on=(p['state']+.1*on).tolist(),pred_off=(p['state']+.1*off).tolist(),bias_before=a.bias.tolist(),phi=phi.tolist(),phi0_before=a.phi0.tolist(),W_before=a.W.tolist(),rank_before=a.rank,updates_before=a.updates,valid=valid,received_through_ns=s['receive_ns'],processed_ns=time.monotonic_ns(),control_enabled=False)
                reason=a.update(phi,target-baseline,s['elapsed'],valid);row.update(update_reason=reason,updates_after=a.updates,rank_after=a.rank,core_ms=(time.perf_counter_ns()-t0)*1e-6);batch.append(row)
            duration=(time.perf_counter_ns()-before)*1e-6
            for row in batch:row['all_models_interval_ms']=duration
            outputs.extend(batch);self.rows+=1
        return outputs,aligned
    def summary(self):
        same=all(hashlib.sha256((R/'results/models'/f).read_bytes()).hexdigest()==h for f,h in self.hashes.items()) and all(weights_hash(self.models[k])==v for k,v in self.memory_hashes.items())
        return dict(intervals=self.rows,alignment_rejects=self.align.rejects,frozen_backbone_files_unchanged=same,hashes=self.hashes,control_enabled=False,adapters={k:dict(W=a.W.tolist(),bias=a.bias.tolist(),phi0=a.phi0.tolist(),rank=a.rank,updates=a.updates,reasons=a.reasons) for k,a in self.adapters.items()})
def main(out):
    engine=Engine();source=out/'adapt_receipts.jsonl';start=time.monotonic();done=False;pending=''
    while not source.exists():
        if time.monotonic()-start>70:raise RuntimeError('receipt_file_missing')
        time.sleep(.02)
    with source.open() as f,(out/'online_predictions.jsonl').open('x') as pred,(out/'aligned_states.jsonl').open('x') as aligned:
        while time.monotonic()-start<180:
            text=f.read(1048576)
            if text:
                pending+=text;lines=pending.split('\n');pending=lines.pop()
                for line in lines:
                    rows,states=engine.step(json.loads(line))
                    for row in rows:pred.write(json.dumps(row,allow_nan=False)+'\n')
                    for row in states:aligned.write(json.dumps(row,allow_nan=False)+'\n')
                pred.flush();aligned.flush()
            elif (out/'async_log_result.json').exists():done=True;break
            else:time.sleep(.005)
    result=engine.summary();result.update(completed=done,partial_line=bool(pending),wall_s=time.monotonic()-start);(out/'online_summary.json').write_text(json.dumps(result,indent=2))
    assert done and not pending and result['frozen_backbone_files_unchanged']
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);main(p.parse_args().out)
