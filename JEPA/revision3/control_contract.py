"""Executable OFFLINE torque contract, not connected to Hnuter firmware.

Must first prove the real allocator maps this torque to the same fixed O.
Baseline-preserving incremental model compensation; original gains unchanged.
Integrator here is a PHYSICAL torque state i_tau = -Ki * integral_e_R.
"""
import numpy as np
from rigid_math import physical_inertia


def correction(r,j,r0,j0,force,omega,base_pd):
    alpha_base=np.linalg.solve(j0,base_pd)
    return np.cross(r-r0,force)+(j-j0)@alpha_base+np.cross(omega,(j-j0)@omega)


def smooth_bumpless(r,j,target_r,target_j,r0,j0,force,omega,base_pd,i_tau,dt,
                    integral_limit,torque_limit,base_trim=np.zeros(3)):
    if not (np.isfinite(dt) and 0<dt<=.02 and physical_inertia(target_j)):
        return r,j,i_tau,False,'physical_or_time'
    # One scalar step preserves symmetric physical tensors (convex physical cone).
    dr=target_r-r;dj=target_j-j
    fraction=min(1.,.002*dt/max(np.linalg.norm(dr),1e-12),.01*dt/max(np.linalg.norm(dj,2),1e-12))
    rn=r+fraction*dr;jn=j+fraction*dj
    old=correction(r,j,r0,j0,force,omega,base_pd)
    new=correction(rn,jn,r0,j0,force,omega,base_pd)
    proposed=i_tau+old-new
    tau=base_pd+base_trim+proposed+new
    # Reject, never clip the integral to claim continuity. Zero Ki axes require
    # zero allowed integral range; such axes can reject the requested transition.
    if np.any(abs(proposed)>integral_limit) or np.any(abs(tau)>torque_limit):
        return r,j,i_tau,False,'integral_or_torque_margin'
    return rn,jn,proposed,True,'accepted'


class TrackingTransition:
    """Alternative for the actual Ki=0 controller: separate torque tracking state.

    At a parameter step b <- b + old_correction - new_correction. The applied
    correction is therefore continuous; b decays afterwards with an explicit
    slew bound. This is an OFFLINE contract, not a claim of firmware integration.
    """
    def __init__(self):
        self.bias=np.zeros(3)
        self.frozen=False

    def step(self,old,new,base_torque,dt,limits,rate=.02):
        if self.frozen:return old+self.bias,False
        if not 0<dt<=.02 or not all(np.all(np.isfinite(x)) for x in (old,new,base_torque)):
            return old+self.bias,False
        bias=self.bias+old-new
        # No decay in the same cycle as a parameter change: exact continuity.
        if np.array_equal(old,new):bias-=np.clip(bias,-rate*dt,rate*dt)
        output=new+bias
        if np.any(abs(base_torque+output)>limits):return old+self.bias,False
        self.bias=bias
        return output,True

    def freeze(self):
        self.frozen=True
