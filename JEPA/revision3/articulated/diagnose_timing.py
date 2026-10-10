"""Offline dynamics diagnostic; truth parameters used ONLY for residual scoring.

Do not export the best fitted time shift as an online calibration. It is a
diagnostic selected on first-half windows and checked on later held windows.
No groundtruth angular-velocity topic: PX4 implements Euler finite differences!
"""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import json,csv
import numpy as np
from pyulog import ULog
from rigid_math import quat_rotation,parallel_axis
from assembly import Assembly,SOURCE
from wrench import measured_wrench
from replay import RUNS
from spatial_model import origin_properties,parameters,window

ROOT=Path(__file__).resolve().parent


def interp(d,keys,t,stamp='timestamp'):
    ts=d[stamp]*1e-6
    return np.array([np.interp(t,ts,d[k]) for k in keys]).T


def main():
    output=[]
    for name in RUNS[:2]:
        run=ROOT.parents[1]/'revision2/runs'/name
        u=ULog(str(next((run/'rootfs/log').rglob('*.ulg'))))
        jointrows=[x for x in csv.DictReader((run/'actuator_sensor.csv').open()) if x['phase'] in ('CALIBRATION','CHECK','EVALUATION')]
        t=np.array([float(x['sensor_t']) for x in jointrows])
        nominal=Assembly(SOURCE);loaded=Assembly(run/'models/hnuter_jepa/model.sdf')
        ref=loaded.state({});h,jo,hr=origin_properties(ref['m'],ref['c'],ref['j'],ref['h'],ref['cdot'])
        theta=parameters(h,jo)
        n=nominal.state({});h0,jo0,_=origin_properties(n['m'],n['c'],n['j'],n['h'],n['cdot'])
        delta_h=[];delta_j=[];rel=[];joints=[]
        for x in jointrows:
            q={key:(float(x[key+'_angle']),float(x[key+'_velocity'])) for key in nominal.geometry['joints']}
            joints.append(q);s=nominal.state(q)
            hq,jq,hrel=origin_properties(s['m'],s['c'],s['j'],s['h'],s['cdot'])
            delta_h.append(hq-h0);delta_j.append(jq-jo0);rel.append(hrel)
        delta_h=np.array(delta_h);delta_j=np.array(delta_j);rel=np.array(rel)
        imu=u.get_dataset('sensor_combined').data
        att=u.get_dataset('vehicle_attitude').data
        filtered=u.get_dataset('vehicle_angular_velocity').data
        vp=u.get_dataset('vehicle_local_position').data
        # Joint samples give fixed historical evaluation grid. Interpolation
        # below is diagnostic, not the stage1 causal past-only estimator.
        starts=[];lo=0
        for hi in range(1,len(t)):
            if t[hi]-t[hi-1]>.03:lo=hi
            if t[hi]-t[lo]>=.384:starts.append((lo,hi+1));lo=hi
        variants=[]
        for rate_source in ('filtered','raw_imu'):
            for shift in np.arange(-.024,.0241,.004):
                sample_t=t+shift
                qs=interp(att,[f'q[{i}]' for i in range(4)],sample_t,'timestamp_sample')
                r=np.array([quat_rotation(q) for q in qs])
                w=(interp(filtered,[f'xyz[{i}]' for i in range(3)],sample_t,'timestamp_sample') if rate_source=='filtered'
                   else interp(imu,[f'gyro_rad[{i}]' for i in range(3)],sample_t))
                # Account for per-message accelerometer sample offset.
                acc_t=imu['timestamp']*1e-6+imu['accelerometer_timestamp_relative']*1e-6
                fO=np.column_stack([np.interp(sample_t,acc_t,imu[f'accelerometer_m_s2[{i}]']) for i in range(3)])
                vel=interp(vp,['vx','vy','vz'],sample_t,'timestamp_sample')
                tau=np.array([measured_wrench(nominal.geometry,q,ww,rr.T@vv)[1] for q,ww,rr,vv in zip(joints,w,r,vel)])
                residual=[];nointernal=[]
                for lo,hi in starts:
                    a,b=window(t[lo:hi],r[lo:hi],w[lo:hi],fO[lo:hi],tau[lo:hi],h[2],delta_h[lo:hi],delta_j[lo:hi],rel[lo:hi])
                    residual.append((b-a@theta)/(t[hi-1]-t[lo]))
                    a,b=window(t[lo:hi],r[lo:hi],w[lo:hi],fO[lo:hi],tau[lo:hi],h[2],delta_h[lo:hi],delta_j[lo:hi],np.zeros_like(rel[lo:hi]))
                    nointernal.append((b-a@theta)/(t[hi-1]-t[lo]))
                residual=np.array(residual);nointernal=np.array(nointernal);cut=len(residual)//2
                rms=lambda x:np.sqrt(np.mean(x*x,axis=0)).tolist()
                variants.append(dict(rate_source=rate_source,shift_s=float(shift),count=len(residual),
                    train_rms_Nm=rms(residual[:cut]),holdout_rms_Nm=rms(residual[cut:]),all_rms_Nm=rms(residual),
                    no_internal_holdout_rms_Nm=rms(nointernal[cut:]),residuals=residual.tolist()))
        best=min(variants,key=lambda x:np.linalg.norm(x['train_rms_Nm']))
        zero=[x for x in variants if abs(x['shift_s'])<1e-6]
        output.append(dict(run=name,variants=variants,training_selected_best={k:v for k,v in best.items() if k!='residuals'},
                           zero_shift=[{k:v for k,v in x.items() if k!='residuals'} for x in zero],
                           caution='Diagnostic interpolation/shift only; do not treat as identified online delay.',
                           sample_counts=dict(joint=len(t),raw_imu=len(imu['timestamp']),filtered=len(filtered['timestamp']))))
        print(name, best['rate_source'],best['shift_s'],best['holdout_rms_Nm'],flush=True)
    (ROOT/'results/timing_diagnostic.json').write_text(json.dumps(output,indent=2)+'\n')


if __name__=='__main__':main()
