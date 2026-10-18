"""Causal ENU acceleration limiter. No truth/model/data-file imports."""
import numpy as np
class SlewLimit:
 def __init__(self,bound=.3,rate=.2,max_gap=.05):
  self.bound=bound;self.rate=rate;self.max_gap=max_gap;self.last=None;self.value=np.zeros(3)
 def step(self,request,fresh,t):
  r=np.asarray(request,dtype=float);dt=None if self.last is None else t-self.last
  valid=bool(fresh and np.isfinite(t) and r.shape==(3,) and np.isfinite(r).all() and np.linalg.norm(r)<=self.bound+1e-9 and (dt is None or 0<=dt<=self.max_gap))
  if not valid:
   self.value=np.zeros(3);self.last=float(t) if np.isfinite(t) else None
   return self.value.copy(),dict(valid=False,limited=False,reason='invalid_stale_time_or_bound',emergency_zero=True)
  if dt is None:self.value=np.zeros(3)
  elif dt>0:
   delta=r-self.value;n=np.linalg.norm(delta);scale=min(1.,self.rate*dt/max(n,1e-30));self.value+=delta*scale
  self.last=float(t)
  return self.value.copy(),dict(valid=True,limited=bool(np.linalg.norm(self.value-r)>1e-10),reason='ready',emergency_zero=False)
