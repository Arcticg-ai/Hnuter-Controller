"""Conservative offline action selection; no publisher or flight authorization."""
from dataclasses import dataclass
import numpy as np
from rigid_math import rotation
from integral_estimator import integral_window, Estimator


def reference(t, axis, amplitude=np.deg2rad(2), duration=6., frequency=.4):
    """C2 start/stop: angle and first two derivatives of one analytic function."""
    if t<=0 or t>=duration:return np.zeros(3)
    k=2*np.pi/duration;w=2*np.pi*frequency
    # sin^4 envelope has zero first/second/third derivative at boundaries.
    b=np.pi/duration;s=np.sin(b*t);c=np.cos(b*t)
    e=s**4;ed=4*b*s**3*c;edd=4*b*b*(3*s*s*c*c-s**4)
    return amplitude*np.array([e*np.sin(w*t),ed*np.sin(w*t)+e*w*np.cos(w*t),
                               edd*np.sin(w*t)+2*ed*w*np.cos(w*t)-e*w*w*np.sin(w*t)])


def synthetic_axis(axis, theta, offdiag, rz, dt=.004, duration=12., amplitude=np.deg2rad(2), bias=None):
    t=np.arange(0,duration+dt/2,dt);r=[];w=[];f=[];tau=[]
    j=np.diag(theta[2:])+offdiag;com=np.array([theta[0],theta[1],rz]);ei=np.eye(3)[axis]
    for tt in t:
        angle,rate,acc=reference(tt,axis,amplitude,duration)
        rr=rotation(ei,angle);ww=ei*rate;ff=rr.T@np.array([0.,0.,-52.92])
        torque=j@(ei*acc)+np.cross(ww,j@ww)+np.cross(com,ff)
        if bias is not None:torque-=bias # Unknown omitted external torque.
        r.append(rr);w.append(ww);f.append(ff);tau.append(torque)
    return t,np.array(r),np.array(w),np.array(f),np.array(tau)


def windows(data,rz,offdiag,offset=0.):
    t,r,w,f,tau=data;rows=[]
    step=int(round(.4/(t[1]-t[0])))
    for lo in range(0,len(t)-step,step):
        hi=lo+step+1
        rows.append(integral_window(t[lo:hi]+offset,r[lo:hi],w[lo:hi],f[lo:hi],tau[lo:hi],rz,offdiag))
    return rows


@dataclass
class CalibrationState:
    state: str='PASSIVE'
    selected_axis: int|None=None
    reason: str='waiting_for_information'

    def choose(self, information, candidates, safety_ready, model_gate, control_gate):
        if not safety_ready or not model_gate or not control_gate:
            self.state='HOLD';self.reason='preflight_model_or_control_gate';return self
        if information['rank']==5:
            self.state='VALIDATE';self.reason='passive_information_sufficient';return self
        viable=[x for x in candidates if x['within_bounds'] and x['actuator_margin_verified']
                and x['supported_new_directions']>0]
        if not viable:
            self.state='HOLD';self.reason='no_feasible_informative_action';return self
        best=max(viable,key=lambda x:x['new_information'])
        self.state='EXCITE';self.selected_axis=best['axis'];self.reason='missing_direction';return self

    def abort(self, reason):
        self.state='FROZEN_RECOVERY';self.selected_axis=None;self.reason=reason


def candidate_actions(prior,offdiag,rz,info):
    result=[];p=np.array(info['projector'])
    j=np.diag(prior[2:])+offdiag
    for axis in range(3):
        data=synthetic_axis(axis,prior,offdiag,rz,duration=6.)
        ww=windows(data,rz,offdiag);e=Estimator(prior,offdiag)
        inf=e.information(ww);a=inf['a']
        new=float(np.linalg.norm(a@(np.eye(5)-p),ord='fro')**2)
        new_s=np.linalg.svd(a@(np.eye(5)-p),compute_uv=False)
        rates=np.linalg.norm(data[2],axis=1)
        acc=max(abs(reference(t,axis,duration=6.)[2]) for t in data[0])
        # An analytical rigid-body budget, NOT actual allocator authority proof.
        torque=float(np.linalg.norm(j[:,axis])*acc)
        result.append(dict(axis=axis,amplitude_deg=2.,duration_s=6.,peak_rate_rad_s=float(max(rates)),
                           peak_accel_rad_s2=float(acc),incremental_torque_bound_Nm=torque,
                           new_information=new,predicted_rank=inf['rank'],
                           supported_new_directions=int(sum(new_s>=5.)),new_singular_values=new_s.tolist(),
                           within_bounds=bool(max(rates)<.15 and acc<.6 and torque<.15),
                           actuator_margin_verified=False))
    return result
