"""Separate-process, bounded-latency shadow fit using only allowlisted sensors.

Inputs: native IMU/joint sensor CSVs, ROS attitude/velocity capture NPZ, fixed
nominal calibration JSON. No SDF, truth recorder, oracle request or run manifest.
"""
import argparse
import json
import io
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation,Slerp
from nominal_dynamics import NominalDynamics,D
from spatial_model import origin_properties,parameters,discrete_window
from spatial_estimator import propose
from rigid_math import parallel_axis
from wrench import measured_wrench


def complete_csv(path):
    text=path.read_text();text=text[:text.rfind('\n')+1]
    return pd.read_csv(io.StringIO(text)).drop_duplicates('t').sort_values('t')


def estimate(imu_path,joint_path,ros_path,calibration_path):
    ros=np.load(ros_path);lo,hi=ros['interval'];imu=complete_csv(imu_path);joint=complete_csv(joint_path)
    j=joint[(joint.t>=lo)&(joint.t<=hi)];t=j.t.to_numpy()
    it=imu.t.to_numpy();ids=np.searchsorted(it,t)
    if len(t)<100 or np.any(ids>=len(it)) or np.max(abs(it[ids]-t))>1e-6:
        return dict(accepted=False,reason='imu_joint_cadence_mismatch')
    if np.max(np.diff(t))>.004001:return dict(accepted=False,reason='missing_physics_step_sensor')
    im=imu.iloc[ids];w=im[['wx','wy','wz']].to_numpy()@D;f=im[['ax','ay','az']].to_numpy()@D
    at=ros['attitude'];vt=ros['velocity'];ts,idx=np.unique(at[:,0],return_index=True)
    if ts[0]>t[0] or ts[-1]<t[-1] or np.max(np.diff(ts))>.03:return dict(accepted=False,reason='attitude_coverage')
    r=Slerp(ts,Rotation.from_quat(at[idx][:,[2,3,4,1]]))(t).as_matrix()
    v=np.array([np.interp(t,vt[:,0],vt[:,i]) for i in (1,2,3)]).T
    n=NominalDynamics(json.loads(calibration_path.read_text()));ref=n.state({});h0,jo0,_=origin_properties(ref['m'],ref['c'],ref['j'],ref['h'],ref['cdot'])
    dh=[];dj=[];hr=[];tau=[]
    for i,row in enumerate(j.itertuples(index=False)):
        d=row._asdict();q={k:(d[k+'_q'],d[k+'_qd']) for k in n.geometry['joints']}
        s=n.state(q);h,jo,rel=origin_properties(s['m'],s['c'],s['j'],s['h'],s['cdot'])
        dh.append(h-h0);dj.append(jo-jo0);hr.append(rel);tau.append(measured_wrench(n.geometry,q,w[i],r[i].T@v[i])[1])
    dh,dj,hr,tau=map(np.array,(dh,dj,hr,tau));mass=5.4;hz=mass*ref['c'][2];aa=[];bb=[];start=0
    for end in range(1,len(t)):
        if t[end]-t[start]<.399999:continue
        sl=slice(start,end+1);a,b=discrete_window(t[sl],r[sl],w[sl],f[sl],tau[sl],hz,dh[sl],dj[sl],hr[sl]);aa.append(a);bb.append(b);start=end
    prior=parameters(mass*ref['c'],ref['j']+parallel_axis(mass,ref['c']))
    result=propose(np.array(aa),np.array(bb),prior,mass,hz,torque_bias_bound_nm=0.)
    # Diagnostic only; does not alter fit, information threshold or acceptance.
    cut=len(aa)//2;weights=.995**(np.arange(cut-1,-1,-1)/2)
    scaled=np.vstack(np.array(aa[:cut])*weights[:,None,None])*np.array([mass*.002]*2+[.05]*6)/.004
    _,sv,vh=np.linalg.svd(scaled,full_matrices=False);keep=(sv>=5)&(sv>=sv[0]/100)
    projector=vh[keep].T@vh[keep]
    steps=int(round(.4/np.median(np.diff(t))))
    endpoint=[r[k+steps]@w[k+steps]-r[k]@w[k] for k in range(0,min(len(t)-steps,cut*steps),steps)]
    result['information_diagnostic']=dict(window_count=len(aa),individual_window_rejections=0,
        unused_tail_s=float(t[-1]-t[start]),training_column_norms=np.linalg.norm(scaled,axis=0).tolist(),
        supported_projector_diagonal=np.diag(projector).tolist(),
        measured_rate_rms_rad_s=np.sqrt(np.mean(w*w,axis=0)).tolist(),
        training_endpoint_world_rate_change_rms_rad_s=np.sqrt(np.mean(np.array(endpoint)**2,axis=0)).tolist(),
        note='Data-only information, prior excluded; endpoint changes determine inertia information in integral windows.')
    result['input_interval']=list(map(float,(t[0],t[-1])));result['sensor_only']=True
    if result['accepted']:
        dr=np.array(result['com'])-ref['c'];delta=np.diag(result['inertia'])-np.diag(ref['j'])
        # Consumer supports additive diagonal payload inertia only. Never clip
        # an out-of-contract candidate to manufacture an accepted command.
        off=np.array(result['inertia'])-ref['j']-np.diag(delta)
        if np.linalg.norm(dr)>.02 or np.any(delta<0) or np.any(delta>.03) or np.linalg.norm(off)>.002:
            result.update(accepted=False,reason='consumer_parameter_contract')
        else:result['request']=dict(delta_com=dr.tolist(),delta_inertia=delta.tolist())
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('imu',type=Path);p.add_argument('joints',type=Path);p.add_argument('ros',type=Path);p.add_argument('calibration',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    try:result=estimate(a.imu,a.joints,a.ros,a.calibration)
    except Exception as ex:result=dict(accepted=False,reason='sensor_processing_exception',error=str(ex))
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
