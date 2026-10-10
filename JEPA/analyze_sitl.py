"""Offline evaluation: this file may read mass truth and ULog, never controls."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import numpy as np


def time_mean(values,times):
    """Time weighting prevents callback jitter from over-weighting dense bursts."""
    return float(np.trapz(values,times)/(times[-1]-times[0])) if len(times)>1 and times[-1]>times[0] else float(np.mean(values))


def analyze(out,truth_mass):
    os.environ['MPLCONFIGDIR']=str(out/'mpl_cache')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from pyulog import ULog
    result=json.loads((out/'flight_result.json').read_text())
    with (out/'trajectory.csv').open() as f:rows=list(csv.DictReader(f))
    metrics={'flight':result,'truth_mass_kg_evaluation_only':truth_mass,'phases':{}}
    for phase in dict.fromkeys(r['phase'] for r in rows):
        selected=[r for r in rows if r['phase']==phase]
        a=lambda key:np.array([float(r[key]) for r in selected])
        metrics['phases'][phase]=dict(samples=len(selected),duration_s=float(np.ptp(a('t'))),
                                     position_rmse_m=float(np.sqrt(time_mean(a('error')**2,a('t')))),
                                     position_max_m=float(max(a('error'))),
                                     vertical_rmse_m=float(np.sqrt(time_mean((a('z_up')-a('z_ref'))**2,a('t')))),
                                     tilt_max_deg=float(max(a('tilt_deg'))),
                                     saturation_fraction=time_mean((a('motor_max')>=.95).astype(float),a('t')),
                                     reference_velocity_energy_m2_s=float(np.trapz(a('vz_ref')**2,a('t'))))
    path=out/'mass_windows.csv'
    if path.exists():
        with path.open() as f:windows=list(csv.DictReader(f))
        valid=[r for r in windows if r['mode']=='validation']
        metrics['mass']=dict(estimate_kg=result['mass_estimate_kg'],
                             error_percent=100*(result['mass_estimate_kg']/truth_mass-1),
                             training_windows=sum(r['mode']=='training' for r in windows),
                             validation_windows=len(valid),
                             accepted=result['mass_accepted'],
                             holdout_relative_rms=float(np.sqrt(np.mean([(float(r['residual_Ns'])/float(r['Q_Ns']))**2 for r in valid]))) if valid else None)
        calibration=[r for r in rows if r['phase']=='CALIBRATION']
        if calibration and 'px4_time_s' in calibration[0]:
            start=float(calibration[0]['px4_time_s'])
            accepted=result.get('mass_accepted_at_px4_s')
            metrics['mass']['acceptance_time_s']=accepted-start if accepted is not None else None
            accurate=[r for r in windows if abs(float(r['mass_kg'])/truth_mass-1)<.02]
            metrics['mass']['first_within_2pct_s']=float(accurate[0]['t'])-start if accurate else None
    logs=list((out/'rootfs/log').rglob('*.ulg'))
    for log in logs:
        u=ULog(str(log))
        metrics['ulog']=dict(path=str(log),sha256=hashlib.sha256(log.read_bytes()).hexdigest(),
                              firmware=u.msg_info_dict.get('ver_sw','unknown'),
                              params={k:v for k,v in u.initial_parameters.items() if k.startswith(('HNTR_','UXRCE_DDS_')) or k=='CA_AIRFRAME'})
        (out/'ulog_parameters.json').write_text(json.dumps(u.initial_parameters,indent=2)+'\n')
        if rows and 'px4_time_s' in rows[0]:
            gt=u.get_dataset('vehicle_local_position_groundtruth').data
            stamps=np.array([float(r['px4_time_s']) for r in rows]);gt_t=gt['timestamp']*1e-6
            truth=np.column_stack([np.interp(stamps,gt_t,gt['y']),np.interp(stamps,gt_t,gt['x']),-np.interp(stamps,gt_t,gt['z'])])
            initial=np.array([float(rows[0]['x']),float(rows[0]['y']),float(rows[0]['z_up'])])
            truth=truth-truth[0]+initial
            reference=np.array([[float(r[k]) for k in ('x_ref','y_ref','z_ref')] for r in rows])
            truth_error=np.linalg.norm(truth-reference,axis=1)
            for phase in metrics['phases']:
                mask=np.array([r['phase']==phase for r in rows])
                metrics['phases'][phase]['truth_position_rmse_m']=float(np.sqrt(time_mean(truth_error[mask]**2,stamps[mask])))
                metrics['phases'][phase]['truth_position_max_m']=float(np.max(truth_error[mask]))
            metrics['truth_alignment']='Fixed translation aligns Gazebo ENU to logged local/reference frame at first armed sample; no scale/rotation/time fitting.'
        status=u.get_dataset('hnuter_control_status').data
        if 'effective_mass' in status:
            mass=status['effective_mass'];updated=status['mass_updated'].astype(bool)
            force=np.column_stack([status[f'force_body[{i}]'] for i in range(3)])
            jumps=np.linalg.norm(np.diff(force,axis=0),axis=1)
            metrics['compensation']=dict(initial_effective_mass_kg=float(mass[0]),final_effective_mass_kg=float(mass[-1]),
                updated_logged_samples=int(np.sum(updated)),rejected_logged_samples=int(np.sum(status['mass_update_rejected'])),
                max_algebraic_update_force_error_N=float(np.max(status['mass_update_force_error'][updated])) if np.any(updated) else None,
                max_adjacent_logged_force_change_during_update_N=float(np.max(jumps[updated[1:]])) if np.any(updated[1:]) else None,
                note='Algebraic error isolates the mass change; adjacent samples also include normal feedback/trajectory dynamics.')
            if rows and 'px4_time_s' in rows[0]:
                t0=float(rows[0]['px4_time_s']);times=status['timestamp']*1e-6
                transition=(times[1:]>=t0+35)&(times[1:]<=t0+39)
                metrics['compensation']['common_transition_max_force_step_N']=float(np.max(jumps[transition])) if np.any(transition) else None
                metrics['compensation']['common_transition_p95_force_step_N']=float(np.percentile(jumps[transition],95)) if np.any(transition) else None
            if np.any(updated) and rows and 'px4_time_s' in rows[0]:
                first=float(status['timestamp'][updated][0])*1e-6;last=float(status['timestamp'][updated][-1])*1e-6
                transient=[r for r in rows if first-1<=float(r['px4_time_s'])<=last+2]
                status_dt=np.diff(status['timestamp'].astype(float))*1e-6
                positive_dt=status_dt>0
                # ULog publication times can duplicate or bunch; distinguish this
                # diagnostic slope from the firmware's angular-sample dt limiter.
                metrics['compensation'].update(update_duration_s=last-first,
                    transient_position_max_m=max(float(r['error']) for r in transient),
                    observed_mass_rate_max_kg_s=float(np.max(np.abs(np.diff(mass)[positive_dt])/status_dt[positive_dt])),
                    transition_average_mass_rate_kg_s=float((mass[updated][-1]-mass[updated][0])/(last-first)),
                    nonincreasing_status_timestamps=int(np.sum(~positive_dt)),
                    rate_note='Maximum is a publication-timestamp finite difference, affected by log timing. Limiter is 0.3 kg/s in the control sample clock.')
        sensor_file=out/'actuator_sensor.csv'
        if sensor_file.exists():
            with sensor_file.open() as f:sensors=list(csv.DictReader(f))
            sensors=[r for r in sensors if r['phase']=='CALIBRATION']
            gt=u.get_dataset('vehicle_local_position_groundtruth').data
            gt_t=gt['timestamp']*1e-6
            gt_vz=gt['vz']
            residuals=[]
            start=0
            for end in range(1,len(sensors)):
                t0=float(sensors[start]['t']);t1=float(sensors[end]['t'])
                if t1-t0<.4-1e-6:continue
                chunk=sensors[start:end+1];ts=np.array([float(r['t']) for r in chunk]);fs=np.array([float(r['thrust_up']) for r in chunk])
                if np.max(np.diff(ts))<.05 and gt_t[0]<=t0<t1<=gt_t[-1]:
                    q=float(np.trapz(fs,ts));y=9.8*(t1-t0)-(np.interp(t1,gt_t,gt_vz)-np.interp(t0,gt_t,gt_vz))
                    residuals.append((q-truth_mass*y)/(t1-t0))
                start=end
            metrics['offline_thrust_balance']=dict(windows=len(residuals),
                rms_residual_N=float(np.sqrt(np.mean(np.square(residuals)))) if residuals else None,
                mean_residual_N=float(np.mean(residuals)) if residuals else None,
                reference='Evaluation-only m*(g*dt-delta_vz_groundtruth); includes unmodeled aerodynamic force, not a direct force sensor.')
    a=lambda key:np.array([float(r[key]) for r in rows])
    fig,axes=plt.subplots(4,1,figsize=(10,10),sharex=True)
    axes[0].plot(a('t'),a('z_ref'),label='reference');axes[0].plot(a('t'),a('z_up'),label='PX4 estimated altitude');axes[0].legend();axes[0].set_ylabel('Height (m)')
    axes[1].plot(a('t'),a('error'));axes[1].set_ylabel('3D error (m)')
    axes[2].plot(a('t'),a('tilt_deg'));axes[2].set_ylabel('Tilt (deg)')
    axes[3].plot(a('t'),a('motor_max'));axes[3].set_ylabel('Max |motor cmd|');axes[3].set_xlabel('Time after arming (s)')
    for ax in axes:ax.grid(alpha=.3)
    fig.suptitle(f'Actual PX4/Gazebo SITL — {out.name}\n{result["status"]}, controller mass {metrics["ulog"]["params"]["HNTR_MASS"]:.1f} kg / physical mass {truth_mass:.1f} kg')
    fig.tight_layout();fig.savefig(out/'flight.png',dpi=140);plt.close(fig)
    metrics['rmse_definition']='Trapezoidal time-weighted RMS over each entire phase; maximum retains all samples.'
    (out/'metrics.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')
    return metrics


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--truth-mass',type=float,required=True);a=p.parse_args()
    m=analyze(a.run,a.truth_mass);print(json.dumps({'status':m['flight']['status'],'phases':m['phases'],'mass':m.get('mass')},indent=2))
