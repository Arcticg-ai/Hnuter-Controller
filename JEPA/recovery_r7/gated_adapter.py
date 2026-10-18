"""Frozen readout adapter with two disjoint ID-block consistency gate.
Same 24 slow coefficients and clips as R6. Never claims disturbance identification.
"""
import numpy as np
class GatedAdapter:
 def __init__(self,scale):
  self.scale=np.asarray(scale);self.bias=np.zeros(6);self.phi0=np.zeros(4);self.W=np.zeros((6,4));self.trim=[];self.blocks=[[],[]];self.rank=0;self.updates=0;self.last_gate={};self.reasons={}
 def predict(self,phi,on=True):
  delta=self.W@(phi-self.phi0) if on else np.zeros(6)
  return self.bias+np.clip(delta*self.scale,[-.3]*3+[-1]*3,[.3]*3+[1]*3)
 def update(self,phi,error,elapsed,valid=True):
  reason='outside_id_window'
  if 34<=elapsed<44 and valid:
   self.trim.append((phi.copy(),error.copy()));self.phi0=np.mean([x[0] for x in self.trim],0);self.bias=np.mean([x[1] for x in self.trim],0);reason='trim'
  elif 48<=elapsed<56:
   y=(error-self.bias)/self.scale
   if not valid:reason='stale_or_saturated'
   elif np.linalg.norm(y)>12:reason='transient_residual'
   else:
    k=0 if elapsed<52 else 1
    if len(self.blocks[k])<32:self.blocks[k].append((phi.copy(),y.copy()))
    reason='insufficient_disjoint_blocks'
    if min(map(len,self.blocks))>=32:
     fits=[];projections=[]
     for b in self.blocks:
      x=np.array([v[0] for v in b]);v=np.array([v[1] for v in b]);x-=x.mean(0);v-=v.mean(0);cov=x.T@x/len(x);eig,vec=np.linalg.eigh(cov);keep=eig>=.05;q=vec[:,keep];projections.append(q@q.T);fits.append((v.T@x/len(x))@q@np.diag(1/(eig[keep]+.1))@q.T)
     a=projections[0]@projections[1]@projections[0];eig,vec=np.linalg.eigh((a+a.T)/2);q=vec[:,eig>=.95];P=q@q.T;self.rank=q.shape[1];f1,f2=[f@P for f in fits];candidate=(f1+f2)/2;diff=np.linalg.norm(f1-f2);den=max(np.linalg.norm(candidate),.1);cos=float(np.sum(f1*f2)/max(np.linalg.norm(f1)*np.linalg.norm(f2),1e-12));self.last_gate=dict(shared_rank=self.rank,relative_difference=float(diff/den),cosine=cos)
     reason='inconsistent_blocks'
     if self.rank and diff/den<=.5 and cos>=.8:
      candidate*=min(1.,2./max(np.linalg.norm(candidate),1e-12));step=candidate-self.W;step*=min(1.,.05/max(np.linalg.norm(step),1e-12));self.W+=step;self.updates+=1;reason='updated'
  self.reasons[reason]=self.reasons.get(reason,0)+1
  return reason
