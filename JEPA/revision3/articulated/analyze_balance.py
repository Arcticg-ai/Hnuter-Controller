"""Offline oracle only: independent per-link state versus reconstructed wrench.

All active windows are retained. No fit, optimized delay, or threshold changes.
TruthRecorder pre state is at simTime-dt; post state is at simTime.
"""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from assembly import Assembly, SOURCE
from rigid_math import kinematics
from wrench import measured_wrench

D=np.diag([1.,-1.,-1.])


def vec(df, name, fields):
    return df[[name+'_'+x for x in fields]].to_numpy()


def rot(df, name):
    return Rotation.from_quat(vec(df,name,['qx','qy','qz','qw'])).as_matrix()


def mv(r,v):return np.einsum('nij,nj->ni',r,v)


def states(df, assembly):
    parts=[]; mtotal=sum(x[0] for x in assembly.inertials.values())
    for name,(m,c,j) in assembly.inertials.items():
        p=vec(df,name,['px','py','pz']);r=rot(df,name)
        w=vec(df,name,['wx','wy','wz']);v=vec(df,name,['vx','vy','vz'])
        off=np.einsum('nij,j->ni',r,c);pc=p+off;vc=v+np.cross(w,off)
        jw=r@j@r.transpose(0,2,1)
        f=vec(df,name,['fx','fy','fz']);tau=vec(df,name,['tx','ty','tz'])
        parts.append((m,p,pc,vc,mv(jw,w),f,tau))
    c=sum(m*pc for m,p,pc,v,h,f,tau in parts)/mtotal
    mom=sum(m*v for m,p,pc,v,h,f,tau in parts)
    h=sum(h+np.cross(pc-c,m*v) for m,p,pc,v,h,f,tau in parts)
    f=sum(f for m,p,pc,v,h,f,tau in parts)
    # ExternalWorldWrenchCmd stores torque ABOUT LINK ORIGIN. AddWorldForce
    # already contributes the link-COM offset moment. Do not add it twice.
    tc=sum(tau+np.cross(p-c,ff) for m,p,pc,v,h,ff,tau in parts)
    o=vec(df,'base_link',['px','py','pz'])
    to=tc+np.cross(c-o,f)
    return dict(c=c,p=mom,h=h,f=f,tc=tc,to=to,m=mtotal)


def rms(x):return np.sqrt(np.mean(np.asarray(x)**2,axis=0)).tolist()


def analyze(run):
    a=Assembly(run/'models/hnuter_jepa/model.sdf');nom=Assembly(SOURCE)
    raw=pd.read_csv(run/'truth_diagnostic.csv')
    pre=raw[raw.stage==0].set_index('iteration');post=raw[raw.stage==1].set_index('iteration')
    ix=pre.index.intersection(post.index);pre=pre.loc[ix];post=post.loc[ix]
    tr=pd.read_csv(run/'trajectory.csv')
    # Include ALL calibration/check/evaluation windows, predeclared phases.
    lo=tr[tr.phase=='CALIBRATION'].px4_time_s.min();hi=tr[tr.phase=='EVALUATION'].px4_time_s.max()
    mask=(pre.t>=lo)&(post.t<=hi);pre=pre[mask];post=post[mask]
    s=states(pre,a);z=states(post,a);dt=pre.dt.to_numpy()
    windows=[];start=0
    for end in range(len(pre)):
        if post.t.iloc[end]-pre.t.iloc[start]<.399999:continue
        sl=slice(start,end+1);duration=float(np.sum(dt[sl]))
        residual=(np.sum(s['tc'][sl]*dt[sl,None],axis=0)-z['h'][end]+s['h'][start])/duration
        fr=(np.sum((s['f'][sl]+np.array([0,0,-9.8*s['m']]))*dt[sl,None],axis=0)-z['p'][end]+s['p'][start])/duration
        windows.append(dict(start=float(pre.t.iloc[start]),end=float(post.t.iloc[end]),residual_Nm=residual.tolist(),force_residual_N=fr.tolist()))
        start=end+1
    checks=[]
    # Fixed every tenth step; used for kinematics identity, not selecting windows.
    for i in range(0,len(pre),10):
        row=pre.iloc[i];r=rot(pre.iloc[i:i+1],'base_link')[0];o=vec(pre.iloc[i:i+1],'base_link',['px','py','pz'])[0]
        w=vec(pre.iloc[i:i+1],'base_link',['wx','wy','wz'])[0];v=vec(pre.iloc[i:i+1],'base_link',['vx','vy','vz'])[0]
        joints={n:(row[n+'_q'],row[n+'_qd']) for n in a.geometry['joints']}
        kin=kinematics(a.geometry,joints);poserr=[];velerr=[];werr=[];roterr=[]
        for name,(tt,wr,vr) in kin.items():
            truthp=vec(pre.iloc[i:i+1],name,['px','py','pz'])[0]
            truthv=vec(pre.iloc[i:i+1],name,['vx','vy','vz'])[0]
            truthw=vec(pre.iloc[i:i+1],name,['wx','wy','wz'])[0]
            truthr=rot(pre.iloc[i:i+1],name)[0]
            poserr.append(np.linalg.norm(o+r@tt[:3,3]-truthp))
            velerr.append(np.linalg.norm(v+np.cross(w,r@tt[:3,3])+r@vr-truthv))
            werr.append(np.linalg.norm(w+r@wr-truthw))
            roterr.append(np.linalg.norm(r@tt[:3,:3]-truthr))
        ar=a.state(joints);predh=r@D@(ar['j']@(D@r.T@w)+ar['h'])
        f,tau,_=measured_wrench(nom.geometry,joints,D@r.T@w,D@r.T@v)
        rigid=a.state({});h0=r@D@rigid['j']@D@r.T@w
        nointernal=r@D@ar['j']@D@r.T@w
        checks.append(dict(t=float(row.t),position_error_m=max(poserr),velocity_error_m_s=max(velerr),
            angular_velocity_error_rad_s=max(werr),rotation_matrix_error=max(roterr),
            h_error_Nms=(predh-s['h'][i]).tolist(),rigid_h_error_Nms=(h0-s['h'][i]).tolist(),
            no_internal_h_error_Nms=(nointernal-s['h'][i]).tolist(),
            force_reconstruction_error_N=(r@D@f-s['f'][i]).tolist(),
            torque_reconstruction_error_Nm=(r@D@tau-s['to'][i]).tolist()))
    result=dict(run=str(run),kind='OFFLINE_TRUTH_DIAGNOSTIC_ONLY',window_count=len(windows),
        phase_interval_px4_s=[float(lo),float(hi)],dt_s=float(np.median(dt)),
        balance_rms_Nm=rms([x['residual_Nm'] for x in windows]),
        balance_max_abs_Nm=np.max(np.abs([x['residual_Nm'] for x in windows]),axis=0).tolist(),
        force_balance_rms_N=rms([x['force_residual_N'] for x in windows]),
        reconstruction={k:rms([x[k] for x in checks]) for k in checks[0] if k!='t'},
        windows=windows,checks=checks)
    (run/'balance_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('checks','windows')},indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args();analyze(args.run)
