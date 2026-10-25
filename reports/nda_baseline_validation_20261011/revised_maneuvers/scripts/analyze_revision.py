"""Archive actual revision trials and plot unclipped matched comparisons."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil

os.environ.setdefault('MPLCONFIGDIR', '/tmp/hnuter-revision-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pyulog import ULog

LABELS = {'direct':'Direct', 'paper_nda':'NDA (Hnuter adaptation)',
          'full':'Full DRCDA', 'no_preview':'No future reference',
          'no_horizon':'One-step DRCDA', 'no_nearfield':'No near-term objective'}
COLORS = dict(zip(LABELS, ['#277da1','#2b9348','#d1495b','#b58105','#7554a3','#454545']))


def load_frame(path):
    d = pd.read_csv(path,low_memory=False)
    d = d[~d.switch_phase.isin(['idle','ingress','settle_a'])].copy()
    d['t'] = d.time_s-d.time_s.iloc[0]
    p = d[['position_x_enu_m','position_y_enu_m','position_z_rel_m']].to_numpy()
    ref = d[['target_x_enu_m','target_y_enu_m','target_z_rel_m']].to_numpy()
    d['ep'] = np.linalg.norm(p-ref,axis=1)
    d['tilt_error_deg'] = np.degrees(np.arccos(np.clip(d.thrust_axis_alignment,-1,1)))
    return d


def ulog_audit(source, dest):
    u = ULog(source)
    def topic(name):
        return u.get_dataset(name).data
    gt, lp, land, status = map(topic, ['vehicle_local_position_groundtruth',
        'vehicle_local_position','vehicle_land_detected','vehicle_status'])
    t = gt['timestamp'].astype(float)
    def sample(data, key):
        i = np.searchsorted(data['timestamp'],t,side='right')-1
        a = np.asarray(data[key],dtype=float)[np.maximum(i,0)]
        a[i<0]=np.nan
        return a
    armed = sample(status,'arming_state')==2
    anchor = np.flatnonzero(armed)[0] if armed.any() else 0
    diag = pd.DataFrame(dict(timestamp_s=t/1e6,time_from_arm_s=(t-t[anchor])/1e6,
        truth_height_m=-gt['z'],estimated_height_m=-sample(lp,'z'),
        landed=sample(land,'landed'),maybe_landed=sample(land,'maybe_landed'),
        ground_contact=sample(land,'ground_contact'),armed=armed.astype(int)))
    attitude,attitude_truth=topic('vehicle_attitude'),topic('vehicle_attitude_groundtruth')
    qe=np.column_stack([sample(attitude,f'q[{i}]') for i in range(4)])
    qt=np.column_stack([sample(attitude_truth,f'q[{i}]') for i in range(4)])
    dot=np.sum(qe*qt,axis=1)/(np.linalg.norm(qe,axis=1)*np.linalg.norm(qt,axis=1))
    diag['estimator_truth_so3_deg']=2*np.degrees(np.arccos(np.clip(np.abs(dot),0,1)))
    diag['quat_reset_counter']=sample(attitude,'quat_reset_counter')
    csv=pd.read_csv(dest/'diagnostic.csv',low_memory=False)
    evaluated=csv[~csv.switch_phase.isin(['idle','ingress','settle_a'])]
    first_csv_arm=float(csv.loc[csv.armed==1,'time_s'].iloc[0])
    first_ulog_arm=float(status['timestamp'][status['arming_state']==2][0])/1e6
    offset=first_ulog_arm-first_csv_arm
    start=float(evaluated.time_s.iloc[0])+offset;end=float(evaluated.time_s.iloc[-1])+offset
    in_task=(diag.timestamp_s>=start)&(diag.timestamp_s<=end)
    diag['time_from_motion_s']=diag.timestamp_s-start
    diag['truth_relative_height_m']=diag.truth_height_m-float(diag.truth_height_m.iloc[anchor])
    lost=diag[in_task & (diag.truth_relative_height_m<.5)]
    diag.to_csv(dest/'ulog_flight_diagnostics.csv',index=False)
    off,mode = topic('offboard_control_mode'),topic('vehicle_control_mode')
    active=mode['flag_control_offboard_enabled']==1
    events=[]
    for msg in u.logged_messages:
        if 'Disarmed by' in msg.message:
            ix=np.clip(np.searchsorted(t,msg.timestamp)-1,0,len(t)-1)
            events.append(dict(message=msg.message.strip(),timestamp_s=msg.timestamp/1e6,
                               truth_height_m=float(-gt['z'][ix])))
    fields=['position','velocity','acceleration','attitude','body_rate','thrust_and_torque']
    flags=['flag_control_position_enabled','flag_control_velocity_enabled',
           'flag_control_attitude_enabled','flag_control_rates_enabled','flag_control_allocation_enabled']
    out=dict(source=str(source),sha256=hashlib.sha256(Path(source).read_bytes()).hexdigest(),
        direct_actuator_all=bool(np.all(off['direct_actuator'])),
        forbidden_offboard_flags={k:bool(np.any(off[k])) for k in fields},
        internal_control_flags_in_offboard={k:bool(np.any(mode[k][active])) for k in flags},
        elevated_landing_disarms=sum('landing' in e['message'] and e['truth_height_m']>.5 for e in events),
        airborne_landed_samples=int(np.sum(armed & (diag.truth_height_m>.5) & (diag.landed==1))),
        csv_ulog_alignment='first armed edge, approximately one status/log sample uncertainty',
        csv_to_ulog_offset_s=offset,
        task_truth_min_relative_height_m=float(diag.loc[in_task,'truth_relative_height_m'].min()),
        first_physical_altitude_loss_s=float(lost.time_from_motion_s.iloc[0]) if len(lost) else None,
        task_estimator_truth_so3_peak_deg=float(diag.loc[in_task,'estimator_truth_so3_deg'].max()),
        task_quat_reset_count=int(diag.loc[in_task,'quat_reset_counter'].max()-diag.loc[in_task,'quat_reset_counter'].min()),
        events=events,initial_parameters=u.initial_parameters,info=u.msg_info_dict)
    (dest/'ulog_metadata.json').write_text(json.dumps(out,indent=2,default=str)+'\n')
    return out


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--runs',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=True)
    figs=a.output/'figures';figs.mkdir(exist_ok=True)
    records=[]; metrics=[]; phases=[]
    for file in sorted(a.runs.rglob('result.json')):
        r=json.loads(file.read_text())
        if (file.parent/'diagnostic.csv').exists():
            r['csv']=str(file.parent/'diagnostic.csv')
        if not r.get('csv') or not r['metrics'].get('evaluated_duration_s'):continue
        dest=a.output/'data'/r['scenario']/f"repeat_{r['repeat']}"/r['method']
        dest.mkdir(parents=True,exist_ok=True)
        for name,src in [('result.json',file),('diagnostic.csv',Path(r['csv'])),
                         ('runtime_config.json',file.parent/'runtime_config.json' if (file.parent/'runtime_config.json').exists()
                          else file.parent/'logs/runtime_config.json'),
                         ('effective_tuning.json',file.parent/'effective_tuning.json')]:
            if src.resolve()!=(dest/name).resolve():shutil.copy2(src,dest/name)
        shutil.copytree(file.parent/'console',dest/'console',dirs_exist_ok=True)
        if (file.parent/'ulog_metadata.json').exists():
            for name in ['ulog_metadata.json','ulog_flight_diagnostics.csv']:
                if (file.parent/name).resolve()!=(dest/name).resolve():shutil.copy2(file.parent/name,dest/name)
            audit=json.loads((dest/'ulog_metadata.json').read_text())
        else:
            audit=ulog_audit(r['ulog_source'],dest)
        r['archived_csv']=str((dest/'diagnostic.csv').relative_to(a.output))
        records.append(r);d=load_frame(r['csv'])
        m=dict(scenario=r['scenario'],method=r['method'],repeat=r['repeat'],**r['metrics'])
        m.update(position_p95_m=float(d.ep.quantile(.95)),so3_p95_deg=float(d.attitude_error_angle_deg.quantile(.95)),
            tilt_rms_deg=float(np.sqrt(np.mean(d.tilt_error_deg**2))),tilt_peak_deg=float(d.tilt_error_deg.max()),
            entry_so3_peak_deg=float(d[d.t<1].attitude_error_angle_deg.max()),
            minimum_takeoff_elapsed_s=float(d.takeoff_elapsed_s.min()),
            completed_airborne=bool(m['finished'] and m['remained_armed'] and not m['safety_cutoff'] and m['minimum_altitude_m']>.5),
            elevated_landing_disarms=audit['elevated_landing_disarms'])
        for key in ['task_truth_min_relative_height_m','task_estimator_truth_so3_peak_deg','task_quat_reset_count']:
            m[key]=audit[key]
        m['first_physical_altitude_loss_s']=audit['first_physical_altitude_loss_s']
        m['completed_airborne']=bool(m['completed_airborne'] and audit['task_truth_min_relative_height_m']>.5)
        pre=d if audit['first_physical_altitude_loss_s'] is None else d[d.t<audit['first_physical_altitude_loss_s']]
        m['pre_height_loss_duration_s']=float(pre.t.iloc[-1]) if len(pre) else 0.
        m['pre_height_loss_position_rms_m']=float(np.sqrt(np.mean(pre.ep**2))) if len(pre) else None
        m['pre_height_loss_so3_rms_deg']=float(np.sqrt(np.mean(pre.attitude_error_angle_deg**2))) if len(pre) else None
        for kind in ['commanded','measured']:
            cols=[f'switch_{kind}_joint_{j}_rad' for j in ['a_l','b_l','a_r','b_r']]
            m[f'{kind}_joint_total_variation_rad']=float(d[cols].diff().abs().sum().sum())
            m[f'{kind}_joint_sampled_rate_peak_rad_s']=float(d[cols].diff().abs().div(d.time_s.diff(),axis=0).max().max())
        metrics.append(m)
        for name,g in d.groupby('switch_phase',sort=False):
            phases.append(dict(scenario=r['scenario'],method=r['method'],repeat=r['repeat'],phase=name,
                duration_s=float(g.t.iloc[-1]-g.t.iloc[0]),position_rms_m=float(np.sqrt(np.mean(g.ep**2))),
                position_peak_m=float(g.ep.max()),so3_rms_deg=float(np.sqrt(np.mean(g.attitude_error_angle_deg**2))),
                so3_peak_deg=float(g.attitude_error_angle_deg.max()),
                tilt_rms_deg=float(np.sqrt(np.mean(g.tilt_error_deg**2))),tilt_peak_deg=float(g.tilt_error_deg.max())))
            phases[-1]['after_physical_altitude_loss']=bool(audit['first_physical_altitude_loss_s'] is not None
                and g.t.iloc[0]>=audit['first_physical_altitude_loss_s'])
    pd.DataFrame(metrics).to_csv(a.output/'metrics.csv',index=False)
    pd.DataFrame(phases).to_csv(a.output/'phase_metrics.csv',index=False)
    (a.output/'results.json').write_text(json.dumps(records,indent=2)+'\n')
    manifest=next((p for p in [a.runs/'manifest.json',a.runs/'run_manifest.json',a.runs.parent/'run_manifest.json'] if p.exists()),None)
    if manifest is not None and manifest.resolve()!=(a.output/'run_manifest.json').resolve():
        shutil.copy2(manifest,a.output/'run_manifest.json')
    def write_figure(fig,name):
        fig.set_layout_engine('none')
        fig.tight_layout(rect=(0,0,1,.96),pad=1.2)
        fig.savefig(figs/(name+'.png'),dpi=170,bbox_inches='tight')
    def save(fig,name):
        write_figure(fig,name);plt.close(fig)
    for s in sorted({r['scenario'] for r in records}):
        rows=[r for r in records if r['scenario']==s]
        for group,methods in [('comparison',{'direct','paper_nda','full'}),
                              ('ablation',{'full','no_preview','no_horizon','no_nearfield'})]:
            chosen=[r for r in rows if r['method'] in methods]
            fig,ax=plt.subplots(4,2,figsize=(13,11),sharex=True,layout='constrained')
            for r in chosen:
                d=load_frame(r['csv']);style=dict(color=COLORS[r['method']],lw=1,
                    ls='-' if r['repeat']==1 else '--',label=LABELS[r['method']]+f" r{r['repeat']}")
                ys=[d.ep,d.attitude_error_angle_deg,d.angular_r_frd_rps,d.position_z_rel_m,
                    d.switch_commanded_joint_b_l_rad,d.switch_measured_joint_b_l_rad,
                    d.tilt_error_deg,d.yaw_authority_scale]
                for axis,y in zip(ax.flat,ys):axis.plot(d.t,y,**style)
            for axis,label in zip(ax.flat,['3D position error (m)','SO(3) error (deg)',
                'Body yaw rate (rad/s)','Estimated height (m)','Left beta command (rad)','Left beta measured (rad)',
                'Thrust-axis direction error (deg)','Common yaw gain scale']):
                axis.set_ylabel(label);axis.grid(alpha=.25)
            ax[0,0].axhline(.75,color='gray',ls=':',lw=.8);ax[0,1].axhline(12,color='gray',ls=':',lw=.8)
            ax[0,0].legend(fontsize=8)
            for axis in ax[-1]:axis.set_xlabel('Time from reference motion start (s)')
            fig.suptitle(s+' | '+group+' | No-delay firmware; identical outer loop and limits')
            write_figure(fig,s+'_'+group)
            ax[0,0].set_ylim(0,1.5);ax[0,1].set_ylim(0,25)
            ax[1,0].set_ylim(-1,1);ax[1,1].set_ylim(0,5)
            fig.suptitle(s+' | '+group+' | CLIPPED detail; consult full-scale failures')
            save(fig,s+'_'+group+'_detail')
        fig,ax=plt.subplots(3,2,figsize=(13,9),sharex=True,layout='constrained')
        for r in rows:
            d=load_frame(r['csv']);d=d[d.t<2]
            ys=[d.target_ax_enu_mps2,d.target_ay_enu_mps2,d.attitude_error_angle_deg,d.angular_q_frd_rps,
                d.wrench_tau_y_nm,d.switch_commanded_joint_b_l_rad]
            for axis,y in zip(ax.flat,ys):axis.plot(d.t,y,color=COLORS[r['method']],lw=1,label=LABELS[r['method']])
        for axis,label in zip(ax.flat,['Reference ax (m/s2)','Reference ay (m/s2)','SO(3) error (deg)',
            'Body pitch rate (rad/s)','Requested pitch moment (Nm)','Left beta command (rad)']):
            axis.set_ylabel(label);axis.grid(alpha=.25)
        ax[0,0].legend(fontsize=7);fig.suptitle(s+' | Entry detail, first 2 s');save(fig,s+'_entry')
        if s.startswith('attitude'):
            fig,ax=plt.subplots(3,2,figsize=(13,9),sharex=True)
            for r in rows:
                d=load_frame(r['csv']);d=d[d.t<=25]
                values=[d.ep,d.attitude_error_angle_deg,d.position_z_rel_m,d.tilt_error_deg,
                        d.switch_commanded_joint_a_l_rad,d.switch_commanded_joint_b_l_rad]
                for axis,y in zip(ax.flat,values):axis.plot(d.t,y,color=COLORS[r['method']],lw=1,label=LABELS[r['method']])
            for axis,label in zip(ax.flat,['3D position error (m)','SO(3) error (deg)','Estimated height (m)',
                'Thrust-axis direction error (deg)','Left alpha command (rad)','Left beta command (rad)']):
                axis.set_ylabel(label);axis.grid(alpha=.25)
            ax[0,0].legend(fontsize=7);fig.suptitle(s+' | First +80 deg roll: transition, hold and start of return')
            save(fig,s+'_initial_roll')
            fig,ax=plt.subplots(2,2,figsize=(13,8),sharex=True,layout='constrained')
            for r in rows:
                d=load_frame(r['csv'])
                for axis,y in zip(ax.flat,[d.target_roll_deg,d.target_pitch_deg,
                    d.roll_deg.where(d.switch_phase.str.startswith('roll')),
                    d.continuous_test_pitch_deg.where(d.switch_phase.str.startswith('pitch'))]):
                    axis.plot(d.t,y,color=COLORS[r['method']],lw=1,label=LABELS[r['method']])
            for axis,label in zip(ax.flat,['Target roll (deg)','Target pitch (deg)',
                'Measured roll in roll phases (deg)','Continuous pitch in pitch phases (deg)']):
                axis.set_ylabel(label);axis.grid(alpha=.25)
            ax[0,0].legend(fontsize=7);fig.suptitle(s+' | 15 s transition, 5 s hold, 5 s level settle')
            save(fig,s+'_orientation')
            fig,ax=plt.subplots(3,1,figsize=(12,9),sharex=True,layout='constrained')
            for r in rows:
                ud=pd.read_csv(a.output/Path(r['archived_csv']).parent/'ulog_flight_diagnostics.csv')
                ud=ud[ud.time_from_motion_s>=0]
                for axis,y in zip(ax,[ud.truth_relative_height_m,ud.estimator_truth_so3_deg,ud.quat_reset_counter]):
                    axis.plot(ud.time_from_motion_s,y,color=COLORS[r['method']],lw=1,label=LABELS[r['method']])
            for axis,label in zip(ax,['Gazebo true relative height (m)','EKF vs truth orientation (deg)','EKF quaternion reset counter']):
                axis.set_ylabel(label);axis.grid(alpha=.25)
            ax[0].legend(fontsize=7,ncol=3);ax[-1].set_xlabel('Time from motion start (s), armed-edge alignment')
            fig.suptitle(s+' | Physical altitude and estimator audit; not allocation-only causes')
            save(fig,s+'_truth_audit')
        else:
            fig,ax=plt.subplots(3,1,figsize=(12,9),sharex=True,layout='constrained')
            for r in rows:
                d=load_frame(r['csv'])
                for i,(pos,ref) in enumerate(zip(['position_x_enu_m','position_y_enu_m','position_z_rel_m'],
                    ['target_x_enu_m','target_y_enu_m','target_z_rel_m'])):
                    offset=d[ref].iloc[0] if i<2 else 0
                    ax[i].plot(d.t,d[pos]-offset,color=COLORS[r['method']],lw=1,label=LABELS[r['method']])
                    if r is rows[0]:ax[i].plot(d.t,d[ref]-offset,'k--',lw=1,label='Reference')
            for axis,label in zip(ax,['ENU x relative (m)','ENU y relative (m)','Altitude (m)']):
                axis.set_ylabel(label);axis.grid(alpha=.25)
            ax[0].legend(fontsize=7,ncol=3);ax[-1].set_xlabel('Time from motion start (s)')
            fig.suptitle(s+' | All methods, full scale');save(fig,s+'_xyz')
            fig=plt.figure(figsize=(10,8),layout='constrained')
            axis=fig.add_subplot(111,projection='3d')
            for r in rows:
                d=load_frame(r['csv'])
                offset=d[['target_x_enu_m','target_y_enu_m','target_z_rel_m']].iloc[0].to_numpy()
                offset[2]=0
                xyz=d[['position_x_enu_m','position_y_enu_m','position_z_rel_m']].to_numpy()-offset
                axis.plot(*xyz.T,color=COLORS[r['method']],lw=1,label=LABELS[r['method']])
                if r is rows[0]:
                    ref=d[['target_x_enu_m','target_y_enu_m','target_z_rel_m']].to_numpy()-offset
                    axis.plot(*ref.T,'k--',lw=1,label='Reference')
            axis.set(xlabel='Relative ENU x (m)',ylabel='Relative ENU y (m)',zlabel='Altitude (m)')
            axis.legend(fontsize=7);fig.suptitle(s+' | Full 3D paths, including loss of tracking')
            save(fig,s+'_3d')
        fig,ax=plt.subplots(4,2,figsize=(13,10),sharex=True,layout='constrained')
        for col,method in enumerate(['full','no_nearfield']):
            for r in rows:
                if r['method']!=method:continue
                d=load_frame(r['csv'])
                for i,joint in enumerate(['a_l','b_l','a_r','b_r']):
                    ax[i,col].plot(d.t,d[f'switch_commanded_joint_{joint}_rad'],color=COLORS[method],lw=1,label='Command')
                    ax[i,col].plot(d.t,d[f'switch_measured_joint_{joint}_rad'],'k--',lw=.8,label='Gazebo measured')
                    ax[i,col].set_ylabel(joint+' (rad)');ax[i,col].grid(alpha=.25)
            ax[0,col].set_title(LABELS[method]);ax[0,col].legend(fontsize=8)
            ax[-1,col].set_xlabel('Time from motion start (s)')
        fig.suptitle(s+' | Four-joint command / measurement detail')
        save(fig,s+'_four_joints')
    print(pd.DataFrame(metrics)[['scenario','method','passed','completed_airborne','position_rms_m','position_peak_m','so3_peak_deg','hold_so3_rms_deg']].to_string(index=False))


if __name__=='__main__':main()
