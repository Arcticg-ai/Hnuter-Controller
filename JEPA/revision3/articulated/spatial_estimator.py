"""Shadow-only spatial parameter fit. No file I/O, simulation or truth imports."""
import numpy as np
from spatial_model import physical,from_parameters


def propose(a,b,prior,m,hz,torque_bias_bound_nm=None):
    a,b,prior=map(np.asarray,(a,b,prior));cut=len(a)//2
    result=dict(accepted=False,theta=prior.tolist(),train_windows=cut,holdout_windows=len(a)-cut)
    if a.shape!=(len(a),3,8) or b.shape!=(len(a),3) or not all(np.all(np.isfinite(x)) for x in (a,b,prior)):
        return dict(result,reason='shape_or_nonfinite')
    if cut<12 or len(a)-cut<5:return dict(result,reason='independent_sample_count')
    if torque_bias_bound_nm is None or not 0<=torque_bias_bound_nm<=.01:
        return dict(result,reason='unverified_constant_torque_bias')
    scale=np.array([m*.002]*2+[.05]*6);sigma=.004
    weight=.995**(np.arange(cut-1,-1,-1)/2)
    at=np.vstack(a[:cut]*weight[:,None,None])*scale/sigma
    bt=np.concatenate(b[:cut]*weight[:,None])/sigma
    _,s,v=np.linalg.svd(at,full_matrices=False);keep=(s>=5)&(s>=s[0]/100)
    result.update(rank=int(keep.sum()),singular_values=s.tolist())
    if not np.all(keep):return dict(result,reason='incomplete_spatial_information')
    delta=np.linalg.solve(at.T@at+.01*np.eye(8),at.T@(bt-at@(prior/scale)))
    candidate=prior+scale*delta
    c,j=from_parameters(candidate,m,hz)
    if not physical(candidate,m,hz) or np.any(abs(c)>.3) or np.any(np.linalg.eigvalsh(j)>1):
        return dict(result,reason='physical_constraints')
    held=np.einsum('nij,j->ni',a[cut:],candidate)-b[cut:]
    hrms=np.sqrt(np.mean(held**2,axis=0));result['holdout_axis_rms_Nms']=hrms.tolist()
    if np.any(hrms>2*sigma):return dict(result,reason='independent_residual')
    fits=[]
    for sl in (slice(0,cut//2),slice(cut//2,cut)):
        ah=np.vstack(a[sl])*scale/sigma;bh=np.concatenate(b[sl])/sigma
        sh=np.linalg.svd(ah,compute_uv=False)
        if sh[-1]<3:return dict(result,reason='split_information_insufficient')
        fits.append(np.linalg.lstsq(ah,bh-ah@(prior/scale),rcond=None)[0])
    split=float(np.linalg.norm(fits[0]-fits[1]));result['split_delta_normalized']=split
    if split>.5:return dict(result,reason='estimate_unstable')
    return dict(result,accepted=True,reason='shadow_accepted_only',theta=candidate.tolist(),com=c.tolist(),inertia=j.tolist())
