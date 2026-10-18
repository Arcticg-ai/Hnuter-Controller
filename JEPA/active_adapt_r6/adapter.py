"""24 bounded slow coefficients; fixed bias trim; observable subspace only."""
import numpy as np
class Adapter:
    def __init__(self,scale):
        self.scale=np.asarray(scale);self.bias=np.zeros(6);self.trim=[];self.phis=[];self.targets=[];self.W=np.zeros((6,4));self.rank=0;self.updates=0;self.phi0=np.zeros(4);self.reasons={}
    def predict(self,phi,on=True):
        delta=self.W@(phi-self.phi0) if on else np.zeros(6)
        return self.bias+np.clip(delta*self.scale,[-.3]*3+[-1.]*3,[.3]*3+[1.]*3)
    def update(self,phi,error,elapsed,valid=True):
        reason='outside_id_window'
        if 34<=elapsed<44 and valid:
            self.trim.append((phi.copy(),error.copy()));self.phi0=np.mean([x[0] for x in self.trim],0);self.bias=np.mean([x[1] for x in self.trim],0);reason='trim'
        elif 48<=elapsed<56:
            y=(error-self.bias)/self.scale
            if not valid:reason='stale_or_saturated'
            elif np.linalg.norm(y)>12:reason='transient_residual'
            else:
                self.phis.append(phi.copy());self.targets.append(y.copy());reason='insufficient_samples'
                if len(self.phis)>=32:
                    x=np.array(self.phis);y=np.array(self.targets);x-=x.mean(0);y-=y.mean(0);cov=x.T@x/len(x);d,v=np.linalg.eigh(cov);keep=d>=.05;self.rank=int(sum(keep));reason='unobservable'
                    if self.rank:
                        proj=v[:,keep];candidate=(y.T@x/len(x))@proj@np.diag(1/(d[keep]+.1))@proj.T
                        candidate*=min(1.,2./max(np.linalg.norm(candidate),1e-12));step=candidate-self.W;step*=min(1.,.05/max(np.linalg.norm(step),1e-12));self.W+=step;self.updates+=1;reason='updated'
        self.reasons[reason]=self.reasons.get(reason,0)+1
        return reason
