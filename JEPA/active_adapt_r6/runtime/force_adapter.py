"""Bounded nominal static geometry compensation; no learned model/unknown parameters."""
import json
from pathlib import Path
import numpy as np

class ForceAdapter:
    def __init__(self):
        d=json.loads(Path(__file__).with_name('nominal_force_map.json').read_text())
        self.center=np.array(d['center']);self.scale=np.array(d['scale']);self.coef=np.array(d['coefficients'])
        self.value=np.zeros(3)
    def step(self,force_frd,torque_frd,R_flu_enu,mass,dt,enabled):
        target=np.zeros(3);R=np.asarray(R_flu_enu);x=(np.r_[force_frd,torque_frd]-self.center)/self.scale
        valid=bool(enabled and 0<dt<=.02 and 3.5<=mass<=7 and np.isfinite(x).all() and np.max(abs(x))<=1. and R.shape==(3,3) and np.isfinite(R).all() and np.allclose(R.T@R,np.eye(3),atol=1e-5) and abs(np.linalg.det(R)-1)<1e-5)
        if valid:
            features=np.r_[1.,x,[x[i]*x[j] for i in range(6) for j in range(i,6)]]
            error=features@self.coef
            target=R@(-error*np.array([1.,-1.,-1.]))/mass
            target*=min(1.,.3/max(np.linalg.norm(target),1e-12))
        # Invalid/disabled source fails closed immediately; valid re-entry starts at zero.
        if not valid:self.value[:]=0.;return self.value.copy(),False
        # Common-vector slew applies only while the source is valid.
        if not 0<dt<=.02:self.value[:]=0.;return self.value.copy(),False
        desired=self.value+(target-self.value)*(-np.expm1(-dt/.3))
        delta=desired-self.value;delta*=min(1.,.15*dt/max(np.linalg.norm(delta),1e-12))
        self.value+=delta
        return self.value.copy(),valid
