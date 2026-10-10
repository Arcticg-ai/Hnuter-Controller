"""ULog evidence that the actual Hnuter consumer changed, froze and stayed bounded."""
import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd
from pyulog import ULog


def analyze(run):
    u=ULog(str(next((run/'rootfs/log').rglob('*.ulg'))))
    d=u.get_dataset('hnuter_control_status').data;t=d['timestamp']*1e-6
    traj=pd.read_csv(run/'trajectory.csv');ev=traj[traj.phase=='EVALUATION']
    result=json.loads((run/'flight_result.json').read_text());manifest=json.loads((run/'launch.json').read_text())
    result.update(run=run.name,case=manifest['experiment'].get('spatial_case'))
    result['legacy_mass_estimator_fields_unused']=True
    if not len(ev):return result
    mask=(t>=ev.px4_time_s.min())&(t<=ev.px4_time_s.max())
    vector=lambda prefix:np.array([d[f'{prefix}[{i}]'] for i in range(3)]).T
    dr=vector('spatial_delta_com');dj=vector('spatial_delta_inertia');bias=vector('spatial_tracking_bias');corr=vector('spatial_correction')
    torque=vector('torque_command');att=vector('attitude_error');limits=np.array([.9,8.,1.8])
    result.update(evaluation_attitude_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(np.sum(att[mask]**2,axis=1))))),
        evaluation_attitude_axis_rmse_deg=np.rad2deg(np.sqrt(np.mean(att[mask]**2,axis=0))).tolist(),
        evaluation_torque_saturation_fraction=float(np.mean(np.any(abs(torque[mask])>=limits*.999,axis=1))),
        max_parameter_transition_error_Nm=float(np.max(d['spatial_transition_error'])),
        effective_delta_com_at_eval_start=dr[mask][0].tolist(),effective_delta_inertia_at_eval_start=dj[mask][0].tolist(),
        effective_delta_com_final=dr[-1].tolist(),effective_delta_inertia_final=dj[-1].tolist(),
        max_effective_delta_com_norm_m=float(np.max(np.linalg.norm(dr,axis=1))),
        max_effective_delta_inertia_norm_kg_m2=float(np.max(np.linalg.norm(dj,axis=1))),
        tracking_bias_max_evaluation_Nm=np.max(abs(bias[mask]),axis=0).tolist(),
        correction_mean_evaluation_Nm=np.mean(corr[mask],axis=0).tolist(),
        update_rejection_fraction=float(np.mean(d['spatial_update_rejected'])),
        hntr_mass=float(u.initial_parameters['HNTR_MASS']),
        gains={k:float(v) for k,v in u.initial_parameters.items() if k.startswith(('HNTR_ATT_KR','HNTR_ATT_D_','HNTR_ATT_I_','HNTR_ATT_ILIM'))})
    cal=traj[traj.phase=='CALIBRATION'];check=traj[traj.phase=='CHECK'];cm=(t>=cal.px4_time_s.min())&(t<=cal.px4_time_s.max())
    result['calibration']=dict(duration_s=float(cal.px4_time_s.max()-cal.px4_time_s.min()),
        position_rmse_m=float(np.sqrt(np.mean(cal.error**2))),max_error_m=float(cal.error.max()),
        vertical_reference_velocity_cost_m2_s=float(np.trapz(cal.vz_ref**2,cal.px4_time_s)),
        vertical_reference_acceleration_cost_m2_s3=float(np.trapz(cal.az_ref**2,cal.px4_time_s)),
        attitude_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(np.sum(att[cm]**2,axis=1))))),
        update_rejection_fraction=float(np.mean(d['spatial_update_rejected'][cm])))
    result['evaluation_update_rejection_fraction']=float(np.mean(d['spatial_update_rejected'][mask]))
    km=(t>=check.px4_time_s.min())&(t<=check.px4_time_s.max())
    result['check_update_rejection_fraction']=float(np.mean(d['spatial_update_rejected'][km]))
    for field,values,bound in [('com',dr,.002),('inertia',dj,.01)]:
        dt=np.diff(t);valid=dt>0;rate=np.linalg.norm(np.diff(values,axis=0)[valid],axis=1)/dt[valid]
        result[field+'_max_logged_change_rate']=float(np.max(rate))
        result[field+'_rate_bound']=bound
    active=np.flatnonzero(np.linalg.norm(dr,axis=1)+np.linalg.norm(dj,axis=1)>1e-7)
    if len(active):
        k=active[0];result['first_update_px4_s']=float(t[k]);result['first_update_correction_norm_Nm']=float(np.linalg.norm(corr[k]));result['first_update_logged_torque_step_Nm']=float(np.linalg.norm(torque[k]-torque[k-1]))
    land=traj[traj.phase=='LAND'];events=json.loads((run/'spatial_events.json').read_text()) if (run/'spatial_events.json').exists() else []
    frozen=[e['t'] for e in events if e['freeze']]
    result['freeze_sent']=bool(frozen)
    # Frozen in-memory model after LAND; compare all subsequent status samples.
    if frozen:
        zero=float(traj.px4_time_s.iloc[0]-traj.t.iloc[0]);fm=t>zero+min(frozen)+.1
        result['frozen_parameter_span']=float(np.max(np.ptp(np.c_[dr,dj][fm],axis=0))) if np.any(fm) else None
    if (run/'online_shadow.json').exists():result['online_shadow']=json.loads((run/'online_shadow.json').read_text())
    (run/'consumer_analysis.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args();analyze(args.run)
