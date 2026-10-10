"""Offline shadow estimation and separately-labelled truth evaluation of old logs.

No flight, no command publisher. Explicitly NOT new revision3 SITL evidence.
Preselected: nominal fixed-group A/s0, successful C/s0, failed C/s1 (legacy).
"""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from pyulog import ULog
from rigid_math import quat_rotation
from integral_estimator import Estimator,integral_window
from wrench import measured_wrench
from assembly import Assembly,SOURCE,ROOT

RUNS=['sitl_matrix_0_A_s0_20261010_191338','sitl_matrix_2_C_s0_20261010_191338','sitl_matrix_7_C_s1_20261010_191649']


def past(dataset,fields,t,max_age=.025):
    times=dataset.get('timestamp_sample',dataset['timestamp'])*1e-6
    idx=np.searchsorted(times,t,side='right')-1
    if idx<0 or t-times[idx]>max_age:return None
    return np.array([dataset[k][idx] for k in fields])


def sensor_pack(run,geometry,u):
    # Explicit allowlist; no groundtruth topics, mass or requested forces.
    att=u.get_dataset('vehicle_attitude').data
    rates=u.get_dataset('vehicle_angular_velocity').data
    vel=u.get_dataset('vehicle_local_position').data
    rows=list(csv.DictReader((run/'actuator_sensor.csv').open()))
    trajectory=list(csv.DictReader((run/'trajectory.csv').open()))
    zero=float(trajectory[0]['px4_time_s'])-float(trajectory[0]['t'])
    out=[]
    for row in rows:
        t=float(row['sensor_t'])
        if not 18<=t-zero<=55:continue
        q=past(att,[f'q[{i}]' for i in range(4)],t)
        w=past(rates,[f'xyz[{i}]' for i in range(3)],t)
        v=past(vel,['vx','vy','vz'],t)
        if any(x is None for x in (q,w,v)):continue
        joints={n:(float(row[n+'_angle']),float(row[n+'_velocity'])) for n in geometry['joints']}
        r=quat_rotation(q)
        f,tau,drag=measured_wrench(geometry,joints,w,r.T@v)
        out.append(dict(t=t,r=r,w=w,v=v,f=f,tau=tau,joints=joints,drag=drag,phase=row['phase']))
    return out


def main():
    geometry=json.loads((ROOT/'generated/nominal_sensor_calibration.json').read_text())
    nominal=Assembly(SOURCE);n=nominal.state({})
    prior=np.r_[n['c'][:2],np.diag(n['j'])];offdiag=n['j']-np.diag(np.diag(n['j']))
    result=[]
    for name in RUNS:
        run=ROOT.parent/'revision2/runs'/name
        ulg=next((run/'rootfs/log').rglob('*.ulg'));u=ULog(str(ulg))
        sensors=sensor_pack(run,geometry,u)
        est=Estimator(prior,offdiag)
        # Groundtruth assembly is constructed ONLY AFTER measurement extraction.
        # It will never be passed to the estimator; it tests model applicability.
        loaded=Assembly(run/'models/hnuter_jepa/model.sdf');ref=loaded.state({})
        truth_theta=np.r_[ref['c'][:2],np.diag(ref['j'])]
        truth_off=ref['j']-np.diag(np.diag(ref['j']))
        windows=[];buf=[];accepted=[];i=0
        tilt_names=['lj1','lj2','rj1','rj2']
        for sample in sensors:
            if buf and sample['t']-buf[-1]['t']>.03:buf=[]
            buf.append(sample)
            if buf[-1]['t']-buf[0]['t']<.384:continue
            if len(buf)<5:buf=[];continue
            b=buf;buf=[b[-1]]
            times=np.array([x['t'] for x in b]);r=np.array([x['r'] for x in b]);w=np.array([x['w'] for x in b]);f=np.array([x['f'] for x in b]);tau=np.array([x['tau'] for x in b])
            q=np.array([[x['joints'][key][0] for key in tilt_names] for x in b]);qd=np.array([[x['joints'][key][1] for key in tilt_names] for x in b])
            peak_q=float(np.max(abs(q)));peak_qd=float(np.max(abs(qd)));motion=float(np.max(np.ptp(q,axis=0)))
            velocity=float(max(np.linalg.norm(x['v']) for x in b));rates_peak=float(np.max(np.linalg.norm(w,axis=1)))
            # These engineering bounds are fixed before fitting record data.
            active=all(x['phase'] in ('HOVER','CALIBRATION','CHECK','EVALUATION') for x in b)
            low_motion=active and peak_q<.05 and peak_qd<.05 and motion<.02 and velocity<.3 and rates_peak<.3
            row=dict(start_s=float(times[0]),end_s=float(times[-1]),peak_tilt_joint_rad=peak_q,peak_tilt_joint_rate_rad_s=peak_qd,
                     joint_span_rad=motion,body_rate_rad_s=rates_peak,speed_m_s=velocity,active_phase=active,low_motion=bool(low_motion))
            try:window=integral_window(times,r,w,f,tau,n['c'][2],offdiag)
            except ValueError as ex:row['rejection']=str(ex);windows.append(row);continue
            if low_motion:accepted.append(window)
            # Independent offline evaluation: varying assembly C,J and internal h.
            states=[loaded.state(x['joints']) for x in b]
            c=np.array([x['c'] for x in states]);j=np.array([x['j'] for x in states]);h=np.array([x['h'] for x in states])
            impulse=np.trapz(np.einsum('nij,nj->ni',r,tau-np.cross(c,f)),times,axis=0)
            momentum=np.einsum('nij,nj->ni',r,np.einsum('nij,nj->ni',j,w)+h)
            dynamic_residual=(impulse-momentum[-1]+momentum[0])/window.duration
            # Constant-reference rigid model residual with true parameters.
            tw=integral_window(times,r,w,f,tau,ref['c'][2],truth_off)
            rigid_residual=(tw.b-tw.a@truth_theta)/window.duration
            spin_world=np.einsum('nij,nj->ni',r,h)
            hrate=(spin_world[-1]-spin_world[0])/window.duration
            # Separate slowed physical rotor spin from gimbal internal momentum.
            rotor_joints={m['joint'] for m in geometry['motors']}
            spin_end=[]
            for x in (b[0],b[-1]):
                spin_joints={key:(val[0],val[1] if key in rotor_joints else 0.) for key,val in x['joints'].items()}
                spin_end.append(x['r']@loaded.state(spin_joints)['h'])
            rotor_rate=(spin_end[-1]-spin_end[0])/window.duration
            joint_correction=rigid_residual-dynamic_residual
            center_torque=tau-np.cross(c,f)
            row.update(rigid_truth_residual_Nm=rigid_residual.tolist(),articulated_truth_residual_Nm=dynamic_residual.tolist(),
                       internal_momentum_equivalent_Nm=hrate.tolist(),articulation_correction_Nm=joint_correction.tolist(),
                       rotor_spin_equivalent_Nm=rotor_rate.tolist(),tilt_internal_equivalent_Nm=(hrate-rotor_rate).tolist(),
                       external_center_torque_rms_Nm=np.sqrt(np.mean(center_torque**2,axis=0)).tolist(),
                       max_com_shift_m=float(np.max(np.linalg.norm(c-ref['c'],axis=1))),
                       max_inertia_change_fraction=float(max(np.linalg.norm(x-ref['j'],2) for x in j)/np.linalg.norm(ref['j'],2)),
                       omitted_terms_pass=bool(np.linalg.norm(joint_correction)<.01),
                       physical_truth_residual_pass=bool(np.linalg.norm(dynamic_residual)<.02))
            windows.append(row);i+=1
        # Last third (at least five) reserved chronologically; no refitting.
        cut=max(0,len(accepted)-max(5,len(accepted)//3))
        for k,window in enumerate(accepted):est.add(window,holdout=k>=cut)
        proposal=est.propose()
        folder=ROOT/'results'/name;folder.mkdir(exist_ok=True)
        (folder/'windows.json').write_text(json.dumps(windows,indent=2)+'\n')
        (folder/'shadow_estimate.json').write_text(json.dumps(proposal,indent=2)+'\n')
        np.savez_compressed(folder/'sensor_pack.npz',t=np.array([x['t'] for x in sensors]),
                            rotation=np.array([x['r'] for x in sensors]),omega=np.array([x['w'] for x in sensors]),
                            force=np.array([x['f'] for x in sensors]),torque_o=np.array([x['tau'] for x in sensors]))
        selected=[x for x in windows if x.get('low_motion') and 'rigid_truth_residual_Nm' in x]
        allvalid=[x for x in windows if 'rigid_truth_residual_Nm' in x]
        stats={}
        for label,rows in [('all_including_recovery',allvalid),('active_only',[x for x in allvalid if x['active_phase']]),('low_motion',selected)]:
            stats[label]=dict(count=len(rows))
            if rows:
                for field in ['rigid_truth_residual_Nm','articulated_truth_residual_Nm','internal_momentum_equivalent_Nm','articulation_correction_Nm',
                              'rotor_spin_equivalent_Nm','tilt_internal_equivalent_Nm','external_center_torque_rms_Nm']:
                    a=np.array([x[field] for x in rows]);stats[label][field]=dict(rms_components=np.sqrt(np.mean(a*a,axis=0)).tolist(),max_norm=float(np.max(np.linalg.norm(a,axis=1))))
                stats[label]['omitted_terms_pass']=sum(x['omitted_terms_pass'] for x in rows)
                stats[label]['physical_truth_residual_pass']=sum(x['physical_truth_residual_pass'] for x in rows)
        result.append(dict(run=name,source_ulog=str(ulg),sha256=hashlib.sha256(ulg.read_bytes()).hexdigest(),
                           sensor_samples=len(sensors),total_windows=len(windows),admissible_sensor_windows=len(accepted),
                           shadow_proposal=proposal,evaluation_only=stats))
        print(name,'windows',len(windows),'admissible',len(accepted),'proposal',proposal['reason'],flush=True)
    (ROOT/'results/replay_summary.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
