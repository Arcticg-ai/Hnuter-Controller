"""Deterministic synthetic experiments, explicitly separate from SITL replay."""
import json
from pathlib import Path
import numpy as np
from actions import synthetic_axis,windows,candidate_actions,CalibrationState
from integral_estimator import Estimator,SCALE,integral_window
from control_contract import correction
from rigid_math import rotation

ROOT=Path(__file__).resolve().parent


def fit_scenario(label,theta,prior,off,rz,noise=0.,delay=0,bias=0.,axes=(0,1,2),sigma=.001):
    est=Estimator(prior,off,sigma_impulse=sigma,torque_bias_bound_nm=0.);rng=np.random.default_rng(73)
    offset=0.;minimum_truth=theta.copy()
    # Repeat every axis in both training halves; reserve entire final cycle.
    for repeat in range(4):
        for axis in axes:
            t,r,w,f,tau=synthetic_axis(axis,theta,off,rz,duration=6.)
            if noise:
                w=w+rng.normal(0,noise,w.shape)
                tau=tau+rng.normal(0,noise*2,tau.shape)
            if bias:tau=tau-np.array([bias,0,0])
            if delay:
                # Undeclared delay corrupts the regression, distinct from a
                # declared age >25 ms which the window constructor rejects.
                w=np.vstack([np.repeat(w[:1],delay,axis=0),w[:-delay]])
            for win in windows((t,r,w,f,tau),rz,off,offset):est.add(win,holdout=repeat==3)
            offset+=6.
    p=est.propose();p.update(label=label,synthetic=True,angular_rate_noise_std=noise,undeclared_delay_s=delay*.004,
                            omitted_torque_bias_Nm=bias,truth=theta.tolist(),prior=prior.tolist(),
                            bias_assumption_deliberately_violated=bool(bias),
                            error=(np.array(p['theta'])-theta).tolist())
    return p,est


def oracle_rotation(case,nominal,compensate):
    """Reduced rigid-body oracle experiment; NOT PX4 allocator verification.

    Uses actual recorded Hnuter KR,D (Ki=0), full inertia, same gains/limits.
    A baseline trim r0×F is assumed exact solely to isolate model mismatch.
    Includes body attitude and world gravity-aligned force, no translation.
    """
    r=np.array(case['c']);j=np.array(case['j']);r0=np.array(nominal['c']);j0=np.array(nominal['j'])
    kr=np.array([8.,8.,3.2]);d=np.array([2.,2.,2.2]);limits=np.array([.9,8.,1.8])
    rot=np.eye(3);w=np.zeros(3);dt=.002;errors=[];saturated=0;rows=[]
    from scipy.spatial.transform import Rotation
    for k in range(10000):
        t=k*dt;target=np.deg2rad([2*np.sin(2*np.pi*.25*t),1.5*np.sin(2*np.pi*.31*t),2*np.sin(2*np.pi*.19*t)])
        rd=Rotation.from_rotvec(target).as_matrix()
        err=Rotation.from_matrix(rd.T@rot).as_rotvec()
        pd=-kr*err-d*w
        f=rot.T@np.array([0,0,-5.4*9.8])
        delta=correction(r,j,r0,j0,f,w,pd) if compensate else np.zeros(3)
        # Existing roll/yaw limits apply to incremental control. Baseline pitch
        # trim is included in the physical total torque before its own limit.
        desired=pd+np.cross(r0,f)+delta
        limited=np.clip(desired,-limits,limits);saturated+=int(np.any(abs(limited-desired)>1e-9))
        alpha=np.linalg.solve(j,limited-np.cross(r,f)-np.cross(w,j@w))
        mid=w+.5*dt*alpha
        rot=rot@Rotation.from_rotvec(mid*dt).as_matrix();w+=dt*alpha
        if t>4:errors.append(np.linalg.norm(err))
        if k%20==0:rows.append([t,*np.rad2deg(err)])
    return dict(attitude_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(np.square(errors))))),saturation_fraction=saturated/10000,trace=rows)


def main():
    truth=json.loads((ROOT/'generated/truth_for_evaluation_only.json').read_text());n=truth['nominal'];l=truth['offset_y']
    prior=np.r_[n['c'][:2],np.diag(n['j'])];theta=np.r_[l['c'][:2],np.diag(l['j'])]
    off=np.array(n['j'])-np.diag(np.diag(n['j']));rz=n['c'][2]
    results=[]
    for name,kw in [('known_clean',dict(sigma=.0002)),('noise',dict(noise=.001,sigma=.001)),
                    ('large_noise',dict(noise=.05,sigma=.001)),('undeclared_delay',dict(delay=30,sigma=.001)),
                    ('constant_bias',dict(bias=.1,sigma=.001)),('roll_only',dict(axes=(0,),sigma=.0002))]:
        p,e=fit_scenario(name,theta,prior,off,rz,**kw);results.append(p)
    # Passive hover supports COM but no inertia; evaluate candidate novelty
    # against that missing-inertia subspace, not against an empty data set.
    passive=Estimator(prior,off)
    tp=np.arange(101)*.004;rp=np.tile(np.eye(3),(101,1,1));wp=np.zeros((101,3))
    fp=np.tile([0.,0.,-52.92],(101,1));taup=np.tile(np.cross([*prior[:2],rz],fp[0]),(101,1))
    for k in range(24):passive.add(integral_window(tp+k*.4,rp,wp,fp,taup,rz,off))
    info=passive.information()
    candidates=candidate_actions(prior,off,rz,info)
    state=CalibrationState().choose(info,candidates,True,False,False)
    oracle={}
    for case in ('nominal','offset_y','symmetric_inertia'):
        oracle[case]={mode:oracle_rotation(truth[case],n,mode=='oracle') for mode in ('fixed','oracle')}
    output=dict(evidence='synthetic only; not PX4/Gazebo flights',estimation=results,candidates=candidates,state=vars(state),oracle=oracle,
                candidate_passive_rank=info['rank'],candidate_passive_projector=info['projector'].tolist(),
                fixed_scales=SCALE.tolist(),constant_bias_caveat='A stable constant external torque is structurally confounded with horizontal COM at hover; residual gates cannot guarantee detection.')
    (ROOT/'results/offline_experiments.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps({x['label']:dict(rank=x['rank'],accepted=x['accepted'],reason=x['reason'],error=x['error']) for x in results},indent=2))
    print('Oracle reduced rigid-body RMSE:',{k:{m:round(v['attitude_rmse_deg'],5) for m,v in row.items()} for k,row in oracle.items()})


if __name__=='__main__':main()
