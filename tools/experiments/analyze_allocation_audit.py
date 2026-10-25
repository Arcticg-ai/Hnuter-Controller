#!/usr/bin/env python3
"""Archive and plot the October allocation audit; no simulation data synthesis."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

os.environ.setdefault('MPLCONFIGDIR', '/tmp/hnuter-audit-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation

ROOT = Path(os.environ.get('HNUTER_CONTROLLER_ROOT', Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(ROOT))
from tools.experiments.run_fixed_attitude_switching import analyze_csv

THRESHOLDS = dict(position_peak_m=.75, attitude_peak_deg=8.,
                  roll_pitch_peak_deg=5., minimum_altitude_m=.5,
                  gate_clearance_m=.05, feedback_ratio=.95)
COLORS = dict(full='#D94F44', direct='#3978AD', no_preview='#B1781A',
              no_horizon='#52946D', paper_nda='#8362A6', no_nearfield='#18949C')
LABELS = dict(full='Full (repaired)', direct='Direct', no_preview='No preview',
              no_horizon='One step', paper_nda='NDA adaptation', no_nearfield='No near-stage (pilot)')


def aligned(path):
    with path.open() as stream:
        reader = csv.reader(stream)
        header = next(reader)
        rows = []
        for row in reader:
            if len(row) != len(header):
                if row[1] not in {'preflight_direct', 'takeoff_wait_arm_direct'}:
                    raise ValueError(f'Unexpected malformed flight row in {path}')
                continue
            rows.append(row)
    df = pd.DataFrame(rows,columns=header)
    for name in df.columns:
        if name not in ['mode','switch_phase','drcda_status','auto_traj_mode','direct_safety_reason','manual_attitude_axis']:
            df[name] = pd.to_numeric(df[name],errors='coerce')
    df = df.sort_values('time_s')
    first = df.loc[df.switch_phase == 'a_to_b'].iloc[0]
    t0 = first.time_s - first.switch_elapsed_s + 4.
    df['t'] = df.time_s-t0
    df['ep'] = np.linalg.norm(df[['position_x_enu_m','position_y_enu_m','position_z_rel_m']].values
                            - df[['target_x_enu_m','target_y_enu_m','target_z_rel_m']].values, axis=1)
    return df


def save(fig, out, name):
    fig.savefig(out/f'{name}.png', dpi=210, bbox_inches='tight')
    fig.savefig(out/f'{name}.pdf', bbox_inches='tight')
    plt.close(fig)


def archive(inputs, out):
    records = []
    for cohort in inputs:
        for path in sorted(cohort.rglob('result.json')):
            result = json.loads(path.read_text())
            csv = Path(result.get('csv') or '')
            if not csv.is_file():
                continue
            relative = Path(cohort.name) / path.parent.relative_to(cohort)
            destination = out/'data'/relative
            destination.mkdir(parents=True, exist_ok=True)
            shutil.copy2(csv, destination/'diagnostic.csv')
            shutil.copy2(path, destination/'original_result.json')
            if (path.parent/'console').is_dir():
                shutil.copytree(path.parent/'console', destination/'console', dirs_exist_ok=True)
            metrics = analyze_csv(csv, THRESHOLDS)
            with csv.open() as stream:
                raw = list(__import__('csv').reader(stream))
            malformed = [row for row in raw[1:] if len(row) != len(raw[0])]
            metrics['malformed_startup_rows'] = len(malformed)
            valid_rows = [dict(zip(raw[0],row)) for row in raw[1:] if len(row)==len(raw[0])]
            solve_times = [float(row['drcda_solve_ms']) for row in valid_rows
                           if row.get('switch_phase') in {'a_to_b','b_to_a','a_to_c'}
                           and row.get('drcda_solve_ms') not in (None,'','nan')]
            metrics['solve_ms_p95'] = float(np.percentile(solve_times,95)) if solve_times else None
            tuning = Path(result.get('tuning', ''))
            if tuning.is_file():
                shutil.copy2(tuning, destination/'effective_tuning.json')
            profile = Path(result.get('joint_model_profile', ''))
            if profile.is_file():
                shutil.copy2(profile, destination/'joint_response_profile.json')
            record = dict(cohort=cohort.name, method=result['method'],
                          T=result['switch_duration_s'],
                          dwell=result.get('dwell_s', 0.5),
                          joint_model_profile=result.get('joint_model_profile','zero_gravity_step'),
                          repeat=next((p for p in path.parts if p.startswith('repeat_')), 'repeat_1'),
                          original_status=result['status'],
                          csv=str((destination/'diagnostic.csv').relative_to(out)),
                          sha256=hashlib.sha256(csv.read_bytes()).hexdigest(), **metrics)
            (destination/'reevaluated.json').write_text(json.dumps(record, indent=2)+'\n')
            records.append(record)
    (out/'audit_metrics.json').write_text(json.dumps(records, indent=2)+'\n')
    pd.DataFrame(records).drop(columns=['failures']).to_csv(out/'audit_metrics.csv', index=False)
    return records


def plot_validation(records, out):
    rows = [r for r in records if r['cohort']=='final']
    if not rows: return
    for duration in sorted({r['T'] for r in rows}, reverse=True):
        chosen = [r for r in rows if r['T']==duration and r['method'] in ['full','direct']]
        fig, axs = plt.subplots(3,2, figsize=(12,8.7), sharex=True, layout='constrained')
        for r in chosen:
            df = aligned(out/r['csv'])
            df = df.loc[df.t.between(-.15,3*duration+1.8)]
            color=COLORS[r['method']]
            label=LABELS[r['method']] if r['repeat']=='repeat_1' else '_nolegend_'
            ls='-' if r['repeat']=='repeat_1' else '--'
            arrays=[df.position_y_enu_m, df.ep, df.attitude_error_angle_deg,
                    df.angular_r_frd_rps,df.position_z_rel_m,df.switch_measured_joint_b_l_rad]
            for ax, values in zip(axs.flat,arrays):
                ax.plot(df.t, values, color=color, ls=ls, lw=1.2, label=label)
        if chosen:
            axs[0,0].plot(df.t,df.target_y_enu_m, color='black', ls=':', label='Reference')
        for i, (ax,label) in enumerate(zip(axs.flat,['y (m)','3D position error (m)',
                'SO(3) error (deg)','Yaw rate (rad/s)','Altitude (m)',r'Measured $\beta_L$ (rad)'])):
            ax.set_ylabel(label)
            ax.set_xlabel('Time from maneuver start (s)')
            ax.grid(alpha=.22)
            for k in range(1,4): ax.axvline(k*duration,color='.5',lw=.6,ls=':')
            ax.set_xlim(-.15,3*duration+1.8)
        axs[1,0].axhline(8,color='#999999',ls='--',lw=.8)
        axs[0,0].legend(loc='best', fontsize=8)
        fig.suptitle(f'Continuous A-B-A-C | T={duration:.1f} s | 2 repeats: solid / dashed')
        save(fig,out/'figures',f'01_continuous_T{duration:.1f}')

    fig, axs=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    specs=[('position_error_peak_m','Peak 3D error (m)'),('attitude_error_peak_deg','Peak SO(3) error (deg)'),
           ('yaw_rate_peak_rad_s','Peak yaw rate (rad/s)'),('roll_pitch_peak_deg','Peak roll/pitch (deg)')]
    methods=[m for m in ['direct','paper_nda','no_horizon','no_preview','full']
             if any(r['method']==m for r in rows)]
    for ax,(key,label) in zip(axs.flat,specs):
        for i,method in enumerate(methods):
            for duration,offset,marker in [(1.1,0.,'o')]:
                vals=[r[key] for r in rows if r['method']==method and r['T']==duration]
                for j,val in enumerate(vals):
                    ax.scatter(i+offset+(j-.5)*.06,val,color=COLORS[method],marker=marker,s=32)
        ax.set_xticks(range(len(methods)),[LABELS[m] for m in methods],rotation=15)
        ax.set_yscale('log')
        ax.set_ylabel(label+' [log scale]')
        ax.grid(axis='y',alpha=.25)
    fig.suptitle('Final matched-constraint comparison | T=1.1 s | both repeats shown')
    save(fig,out/'figures','02_ablations_all_runs')

    fig,axs=plt.subplots(2,2,figsize=(12,7),sharex=True,layout='constrained')
    for r in rows:
        if r['repeat']!='repeat_1': continue
        df=aligned(out/r['csv'])
        df=df.loc[df.t.between(-.15,3*r['T']+1.8)]
        ys=[df.ep,df.attitude_error_angle_deg,df.switch_commanded_joint_b_l_rad,
            df.switch_measured_joint_b_l_rad]
        for ax,y in zip(axs.flat,ys):
            ax.plot(df.t,y,color=COLORS[r['method']],label=LABELS[r['method']],lw=1.1)
    for ax,label in zip(axs.flat,['3D position error (m)','SO(3) error (deg)',
                                r'Commanded $\beta_L$ (rad)',r'Measured $\beta_L$ (rad)']):
        ax.set_ylabel(label); ax.set_xlabel('Time from maneuver start (s)')
        ax.grid(alpha=.25)
        for k in range(1,4): ax.axvline(k*1.1,color='.5',ls=':',lw=.6)
    axs[0,0].legend(fontsize=8)
    fig.suptitle('Matched ablations | T=1.1 s, first repeat | same trajectory and limits')
    save(fig,out/'figures','05_ablation_mechanism')


def motion_crosscheck(records,out):
    rows=[r for r in records if r['cohort']=='final' and r['method']=='full']
    if not rows: return
    df=aligned(out/rows[0]['csv'])
    t=df.t.to_numpy()
    grid=np.arange(t.min(),t.max(),.02)
    velocities=np.stack([np.interp(grid,t,df[f'velocity_{a}_enu_mps']) for a in 'xyz'],axis=1)
    angles=np.stack([np.interp(grid,t,np.unwrap(np.deg2rad(df[f'{a}_deg']))) for a in ['roll','pitch','yaw']],axis=1)
    rot=Rotation.from_euler('xyz',angles).as_matrix()
    accel=savgol_filter(velocities,7,2,deriv=1,delta=.02,axis=0)
    force_enu=4.5*(accel+np.array([0.,0.,9.81]))
    force_body=np.einsum('nji,nj->ni',rot,force_enu)
    fy=np.interp(t,grid,force_body[:,1])
    active=df.switch_phase.isin(['a_to_b','b_to_a','a_to_c']).to_numpy()
    fig,axs=plt.subplots(3,1,figsize=(12,8),sharex=True,layout='constrained')
    axs[0].plot(t,df.drcda_estimated_wrench_fy_n,label='Allocator model estimate',color='#3978AD')
    axs[0].plot(t,fy,label='Motion-derived net force (offline)',color='#555555',ls='--')
    metrics={}
    for h,n,color in [(.05,1,'#C28A21'),(.1,2,'#D94F44')]:
        predicted=df[f'drcda_predicted_h{n}_wrench_fy_n'].to_numpy()
        future_model=np.interp(t+h,t,df.drcda_estimated_wrench_fy_n)
        future_motion=np.interp(t+h,t,fy)
        for ax,ref,label in [(axs[1],future_model,'model'),(axs[2],future_motion,'motion')]:
            error=np.where(active,predicted-ref,np.nan)
            rmse=float(np.sqrt(np.nanmean(error**2)))
            metrics[f'h{n}_{label}_rmse_N']=rmse
            ax.plot(t,error,color=color,label=f'H{n}={h:.2f} s; RMSE={rmse:.2f} N')
    for ax,label in zip(axs,['Fy (N)','Prediction - future model (N)','Prediction - future net force (N)']):
        ax.set_ylabel(label); ax.legend(fontsize=8); ax.grid(alpha=.25)
        ax.axhline(0,color='k',lw=.5); ax.set_xlim(0,3*rows[0]['T'])
    axs[-1].set_xlabel('Time from maneuver start (s)')
    fig.suptitle('Prediction evidence audit | net force includes drag; differentiated EKF velocity')
    save(fig,out/'figures','03_prediction_independent_crosscheck')
    metrics['caveat']='Offline 0.14 s Savitzky-Golay velocity derivative; not a force sensor or aerodynamic rotor-wrench ground truth.'
    (out/'prediction_crosscheck.json').write_text(json.dumps(metrics,indent=2)+'\n')


def joint_replay_check(records,out):
    records = [r for r in records if r['cohort']=='final' and r['method']=='full']
    if not records: return
    profile=json.loads((ROOT/'config/experiments/gazebo_loaded_joint_response.json').read_text())
    models={
        'Zero-gravity fit': ([.9939,.9018,.994,.9152],[.994,.9119,.994,.9042],
                            [.1384,.0891,.1408,.092],[.1396,.0874,.1422,.0868]),
        'Loaded fit': tuple(profile[n] for n in ['servo_gain_positive','servo_gain_negative',
                                                'servo_tau_positive_s','servo_tau_negative_s'])}
    metrics={}
    fig,axs=plt.subplots(1,2,figsize=(11,4),layout='constrained')
    for ax,horizon in zip(axs,[.05,.1]):
        for name,parameters in models.items():
            errors=[]
            gp,gn,tp,tn = map(np.asarray,parameters)
            for r in records:
                df=aligned(out/r['csv'])
                t=df.t.to_numpy()
                q=df[[f'switch_measured_joint_{j}_rad' for j in ['a_l','b_l','a_r','b_r']]].to_numpy(float)
                u=df[[f'switch_commanded_joint_{j}_rad' for j in ['a_l','b_l','a_r','b_r']]].to_numpy(float)
                active=df.switch_phase.isin(['a_to_b','b_to_a','a_to_c']).to_numpy()
                for i in np.flatnonzero(active):
                    target=t[i]+horizon
                    if target>t[-1]: continue
                    prediction=q[i].copy(); cursor=t[i]; index=i
                    while cursor<target-1e-10:
                        next_time=min(target,t[index+1])
                        dt=next_time-cursor
                        gains=np.where(u[index]>=0,gp,gn)
                        taus=np.where(u[index]>=0,tp,tn)
                        dq=(1-np.exp(-dt/taus))*(gains*u[index]-prediction)
                        prediction+=np.clip(dq,-10*dt,10*dt)
                        cursor=next_time; index+=1
                    truth=np.array([np.interp(target,t,q[:,j]) for j in range(4)])
                    errors.append(prediction-truth)
            rmse=np.sqrt(np.mean(np.square(errors),axis=0))
            metrics[f'{name}_{horizon:.2f}s']=rmse.tolist()
            x=np.arange(4)+(0 if name=='Zero-gravity fit' else .38)
            ax.bar(x,rmse,width=.35,label=name,color='#888888' if name=='Zero-gravity fit' else '#D94F44')
        ax.set_xticks(np.arange(4)+.19,[r'$\alpha_L$',r'$\beta_L$',r'$\alpha_R$',r'$\beta_R$'])
        ax.set_ylabel('Joint prediction RMSE (rad)'); ax.set_title(f'Command replay, H={horizon:.2f} s')
        ax.legend(fontsize=8); ax.grid(axis='y',alpha=.2)
    fig.suptitle('Held-out final runs: both models replay the same recorded command sequence')
    save(fig,out/'figures','04_heldout_joint_model_check')
    (out/'joint_model_crosscheck.json').write_text(json.dumps(metrics,indent=2)+'\n')


def plot_default_experiment(records, out):
    rows = [r for r in records if r['cohort'] in {'model_ablation', 'default_ablations'}
            or (r['cohort'] == 'final' and r['method'] == 'direct')]
    if not rows:
        return
    fig, axs = plt.subplots(3, 2, figsize=(13, 9), sharex=True, layout='constrained')
    for r in rows:
        df = aligned(out/r['csv'])
        df = df.loc[df.t.between(-.1, 5.3)]
        label = LABELS[r['method']] if r['repeat'] == 'repeat_1' else '_nolegend_'
        values = [df.ep, df.attitude_error_angle_deg, df.angular_r_frd_rps,
                  df.switch_commanded_joint_b_l_rad, df.switch_measured_joint_b_l_rad,
                  df.position_z_rel_m]
        for ax, series in zip(axs.flat, values):
            ax.plot(df.t, series, color=COLORS[r['method']], lw=1.15,
                    ls='-' if r['repeat'] == 'repeat_1' else '--', label=label)
    names = ['3D position error (m)', 'SO(3) error (deg)', 'Body yaw rate (rad/s)',
             'Left beta command (rad)', 'Left beta measured (rad)', 'Altitude (m)']
    for ax, name in zip(axs.flat, names):
        ax.set_ylabel(name)
        ax.set_xlim(-.1, 5.3)
        ax.grid(alpha=.25)
        for time in [1.1, 2.2, 3.3]:
            ax.axvline(time, color='#777777', lw=.7, ls=':')
    axs[0, 0].legend(fontsize=8, ncol=2)
    axs[0, 1].axhline(8, color='#555555', ls='--', lw=.8)
    for ax in axs[-1]:
        ax.set_xlabel('Time from maneuver start (s)')
    fig.suptitle('Default joint model | T=1.1 s, zero dwell, two runs per method\n'
                 'Solid: run 1; dashed: run 2. Direct runs reused from the matched final cohort.')
    save(fig, out/'figures', '06_default_model_comparison')


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--input',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'figures').mkdir(exist_ok=True)
    plt.rcParams.update({'font.size':9,'axes.titlesize':10})
    records=archive(args.input,args.output)
    plot_validation(records,args.output)
    motion_crosscheck(records,args.output)
    joint_replay_check(records,args.output)
    plot_default_experiment(records,args.output)
    print(f'Archived and evaluated {len(records)} actual SITL runs')
    for r in records:
        print(r['cohort'],r['repeat'],r['T'],r['method'],r['passed'],
              round(r['position_error_peak_m'],3),round(r['attitude_error_peak_deg'],2),
              round(r['roll_pitch_peak_deg'],2),round(r['yaw_rate_peak_rad_s'],3))


if __name__=='__main__': main()
