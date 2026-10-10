"""Sensor-only spatial window construction followed by separate oracle scoring.

Uses native IMU / joint telemetry with original timestamps, and estimated PX4
attitude/velocity. Model mass/inertia truth is loaded AFTER window construction
for scoring only. No fitted time shifts. Window duration and residual gate fixed.
"""
import argparse
import sys
import json
from pathlib import Path
import numpy as np
import pandas as pd
from pyulog import ULog
from scipy.spatial.transform import Rotation, Slerp
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from assembly import Assembly, SOURCE
from wrench import measured_wrench
from spatial_model import window,discrete_window,origin_properties,parameters,from_parameters
from spatial_estimator import propose
from rigid_math import parallel_axis

D=np.diag([1.,-1.,-1.])


def interp(t,ts,vals):
    vals=np.asarray(vals)
    return np.column_stack([np.interp(t,ts,vals[:,i]) for i in range(vals.shape[1])])


def pack(run):
    """Measurement allowlist. Nominal moving assembly calibration is independent.

    Offline buffering is realizable with one bounded past-data processing lag;
    it is not proof that transport latency has been handled by a live node.
    """
    joint=pd.read_csv(run/'joints_native.csv').drop_duplicates('t').sort_values('t')
    imu=pd.read_csv(run/'imu_native.csv').drop_duplicates('t').sort_values('t')
    u=ULog(str(next((run/'rootfs/log').rglob('*.ulg'))))
    a=u.get_dataset('vehicle_attitude').data;v=u.get_dataset('vehicle_local_position').data
    tr=pd.read_csv(run/'trajectory.csv');lo=tr[tr.phase=='CALIBRATION'].px4_time_s.min();hi=tr[tr.phase=='EVALUATION'].px4_time_s.max()
    j=joint[(joint.t>=lo)&(joint.t<=hi)];t=j.t.to_numpy()
    assert len(imu)>1000 and len(t)>1000
    ts,idx=np.unique(a['timestamp_sample']*1e-6,return_index=True)
    qs=np.array([a[f'q[{i}]'][idx] for i in (1,2,3,0)]).T
    r=Slerp(ts,Rotation.from_quat(qs))(t).as_matrix()
    w=interp(t,imu.t,imu[['wx','wy','wz']])@D
    f=interp(t,imu.t,imu[['ax','ay','az']])@D
    vv=interp(t,v['timestamp_sample']*1e-6,np.array([v[k] for k in ('vx','vy','vz')]).T)
    n=Assembly(SOURCE);ref=n.state({});h0,jo0,_=origin_properties(ref['m'],ref['c'],ref['j'],ref['h'],ref['cdot'])
    dh=[];dj=[];rel=[];tau=[]
    for i,row in enumerate(j.itertuples(index=False)):
        d=row._asdict();q={k:(d[k+'_q'],d[k+'_qd']) for k in n.geometry['joints']}
        s=n.state(q);h,jo,hr=origin_properties(s['m'],s['c'],s['j'],s['h'],s['cdot'])
        dh.append(h-h0);dj.append(jo-jo0);rel.append(hr)
        tau.append(measured_wrench(n.geometry,q,w[i],r[i].T@vv[i])[1])
    return dict(t=t,r=r,w=w,f=f,dh=np.array(dh),dj=np.array(dj),rel=np.array(rel),tau=np.array(tau),
                reference=ref,nominal_h=h0,nominal_jo=jo0,
                timing=dict(joint_samples=len(joint),imu_samples=len(imu),joint_max_gap=float(np.max(np.diff(t))),
                            imu_max_gap=float(np.max(np.diff(imu.t)))))


def score(run,discrete=False):
    p=pack(run)
    integrate=discrete_window if discrete else window
    known_hz=5.4*p['reference']['c'][2]
    # Evaluation truth enters only here. These parameters never feed p or a fit.
    truth=Assembly(run/'models/hnuter_jepa/model.sdf').state({})
    h,jo,_=origin_properties(truth['m'],truth['c'],truth['j'],truth['h'],truth['cdot'])
    theta=parameters(h,jo);rows=[];aa=[];bb=[];lo=0
    for hi in range(1,len(p['t'])):
        if p['t'][hi]-p['t'][hi-1]>.03:lo=hi
        if p['t'][hi]-p['t'][lo]<.399999:continue
        sl=slice(lo,hi+1);t=p['t'][sl];duration=t[-1]-t[0]
        args=[p[k][sl] for k in ('r','w','f','tau')]
        a,b=integrate(t,*args,known_hz,p['dh'][sl],p['dj'][sl],p['rel'][sl])
        _,b0=integrate(t,*args,known_hz,np.zeros_like(p['dh'][sl]),np.zeros_like(p['dj'][sl]),np.zeros_like(p['rel'][sl]))
        _,bn=integrate(t,*args,known_hz,p['dh'][sl],p['dj'][sl],np.zeros_like(p['rel'][sl]))
        rows.append(dict(start=float(t[0]),end=float(t[-1]),corrected_Nm=((b-a@theta)/duration).tolist(),
            rigid_Nm=((b0-a@theta)/duration).tolist(),no_internal_Nm=((bn-a@theta)/duration).tolist()))
        aa.append(a);bb.append(b);lo=hi
    cut=len(rows)//2
    rms=lambda key,sl:np.sqrt(np.mean(np.array([r[key] for r in rows])[sl]**2,axis=0)).tolist()
    result=dict(kind='SENSOR_ONLY_WINDOWS_WITH_OFFLINE_TRUTH_SCORING',run=str(run),timing=p['timing'],
        n_windows=len(rows),fit_windows=cut,holdout_windows=len(rows)-cut,
        train={k:rms(k,slice(0,cut)) for k in ('corrected_Nm','rigid_Nm','no_internal_Nm')},
        holdout={k:rms(k,slice(cut,None)) for k in ('corrected_Nm','rigid_Nm','no_internal_Nm')},windows=rows)
    result['oracle_residual_gate_pass']=all(v<.02 for v in result['holdout']['corrected_Nm'])
    # Diagnose information with fixed scales, independent of truth parameter errors.
    a=np.vstack(aa[:cut]);b=np.concatenate(bb[:cut]);scales=np.array([5.4*.002]*2+[.05]*6)
    u,s,v=np.linalg.svd(a*scales/.004,full_matrices=False)
    result['data_singular_values']=s.tolist();result['retained_rank']=int(np.sum((s>=5)&(s>=s[0]/100)))
    suffix='_discrete' if discrete else ''
    result['quadrature']='derived_semiimplicit' if discrete else 'trapezoid'
    fitted=np.linalg.lstsq(np.vstack(aa[:cut]),np.concatenate(bb[:cut]),rcond=None)[0]
    # All comparison cases have the same declared fixed 5.4 kg mass.
    mass=5.4;hz=mass*p['reference']['c'][2]
    prior=parameters(mass*p['reference']['c'],p['reference']['j']+parallel_axis(mass,p['reference']['c']))
    shadow=propose(np.array(aa),np.array(bb),prior,mass,hz,torque_bias_bound_nm=0.)
    result['shadow']=shadow
    result['bias_missing_negative_control']=propose(np.array(aa),np.array(bb),prior,mass,hz)['reason']
    if shadow['accepted']:
        c,j=from_parameters(np.array(shadow['theta']),mass,hz)
        result['shadow_truth_error']=dict(com_m=(c-truth['c']).tolist(),inertia_diag_kg_m2=(np.diag(j)-np.diag(truth['j'])).tolist(),inertia_frobenius=float(np.linalg.norm(j-truth['j'])))
    result['offline_fit_error']=dict(theta=(fitted-theta).tolist(),note='Truth scoring only; not an accepted online update.')
    (run/('sensor_balance'+suffix+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    np.savez_compressed(run/('sensor_windows'+suffix+'.npz'),a=np.array(aa),b=np.array(bb),truth=theta)
    print(json.dumps({k:v for k,v in result.items() if k!='windows'},indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--discrete',action='store_true');args=p.parse_args();score(args.run,args.discrete)
