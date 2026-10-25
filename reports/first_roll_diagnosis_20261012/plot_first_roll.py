"""Reproduce the estimator/allocation isolation figures from aligned SITL logs."""
from pathlib import Path
import argparse
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    p=argparse.ArgumentParser()
    p.add_argument('root',type=Path)
    a=p.parse_args()
    out=a.root/'figures';out.mkdir(exist_ok=True)
    names=['before','after_sensor_only','after_both','final_entry_analysis']
    labels=['Before fixes','Sensor frame fixed','Sensor + DC gain fixed','Final entry repeat']
    colors=['#b63636','#bb7b10','#167b65','#3969bc']
    data=[pd.read_csv(a.root/n/'aligned.csv',low_memory=False) for n in names]
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.22})
    def save(fig,name):
        fig.savefig(out/f'{name}.png',dpi=170)
        fig.savefig(out/f'{name}.pdf')
        plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(13,10),layout='constrained')
    specs=[('attitude_error_angle_deg','Controller SO(3) error [deg]'),
           ('est_truth_deg','EKF versus Gazebo truth [deg]'),
           ('truth_ref_deg','True attitude versus reference [deg]'),
           ('position_error_m','Estimated position tracking error [m]'),
           ('gyro_truth_error','Gyro versus quaternion-derived body rate [rad/s]'),
           ('truth_relative_height_m','Gazebo true height above armed origin [m]')]
    for ax,(col,title) in zip(axes.flat,specs):
        for d,label,color in zip(data,labels,colors):
            s=d[d.motion_time.between(0,20)]
            ax.plot(s.motion_time,s[col],label=label,color=color,lw=1.1)
        ax.axvline(15,color='.4',ls='--',lw=.8)
        ax.set(xlabel='Time from first positive roll [s]',ylabel=title)
    axes[0,0].legend(fontsize=9)
    fig.suptitle('First +80 deg roll: 15 s outbound, 5 s hold; unchanged outer-loop gains')
    save(fig,'01_first_roll_isolation')
    fig,axes=plt.subplots(2,3,figsize=(15,8),layout='constrained')
    for j in range(3):
        d=data[j];s=d[d.motion_time.between(0,20)]
        for i,axis in enumerate(['roll','yaw']):
            ax=axes[i,j]
            for col,label,color in [(f'target_{axis}_deg','Reference','black'),
                (f'est_{axis}_deg','EKF','#bb7b10'),(f'truth_{axis}_deg','Gazebo truth','#167b65')]:
                values=s[col].to_numpy()
                if axis=='yaw':
                    reference=s.target_yaw_deg.to_numpy()
                    values=(values-reference+180)%360-180
                ax.plot(s.motion_time,values,label=label,color=color,lw=1.2)
            ax.axvline(15,color='.4',ls='--')
            ax.set(xlabel='Time from first positive roll [s]',
                   ylabel=f'{axis.title()}'+(' relative to reference' if axis=='yaw' else '')+' [deg]')
        axes[0,j].set_title(labels[j]);axes[0,j].legend()
    save(fig,'02_reference_estimate_truth')
    fig,axes=plt.subplots(3,2,figsize=(13,10),layout='constrained')
    for j,n in enumerate([1,2]):
        s=data[n][data[n].motion_time.between(4,20)]
        config=json.loads((a.root/(['sensor_fixed','sensor_unit_gain'][j])/'logs/runtime_config.json').read_text())['allocator']
        for i,joint in enumerate(['b_l','b_r']):
            ax=axes[i,j];u=s[f'switch_commanded_joint_{joint}_rad'].to_numpy()
            q=s[f'switch_measured_joint_{joint}_rad'].to_numpy();idx=1+2*i
            gain=np.where(u>=0,config['servo_gain_positive'][idx],config['servo_gain_negative'][idx])
            for values,label,color in [(u,'Command u','black'),(q,'Measured q','#167b65'),(gain*u,'Predictor equilibrium K*u','#bb7b10')]:
                ax.plot(s.motion_time,np.degrees(values),label=label,color=color,lw=1.1)
            ax.set(ylabel=f'{joint} joint angle [deg]',xlabel='Time from first roll [s]');ax.legend(fontsize=8)
        ax=axes[2,j]
        for col,label,color in [('wrench_tau_z_nm','Requested yaw torque','black'),
            ('drcda_estimated_wrench_tz_nm','Current model estimate','#167b65'),
            ('drcda_predicted_wrench_tz_nm','Terminal model prediction','#bb7b10')]:
            ax.plot(s.motion_time,s[col],label=label,color=color,lw=1)
        ax.set(ylabel='Yaw torque [Nm]',xlabel='Time from first roll [s]');ax.legend(fontsize=8)
        axes[0,j].set_title(labels[n])
    fig.suptitle('Residual allocation error: measured equilibrium is q=u, not q=0.9u\nWrench traces are model estimates, not independent force measurements')
    save(fig,'03_joint_dc_gain_and_wrench')
    fig,axes=plt.subplots(4,1,figsize=(14,11),sharex=True,layout='constrained')
    for d,label,color in zip(data[2:],labels[2:],colors[2:]):
        s=d[d.motion_time.between(0,160)]
        for ax,col in zip(axes,['attitude_error_angle_deg','truth_ref_deg','position_error_m','truth_relative_height_m']):
            ax.plot(s.motion_time,s[col],label=label,color=color,lw=1.1)
    for ax,title in zip(axes,['Controller SO(3) [deg]','True reference error [deg]','Position error [m]','True relative height [m]']):
        ax.set_ylabel(title)
        for start in [0,40,80,120]:
            ax.axvspan(start+15,start+20,color='#9c9999',alpha=.15)
            ax.axvline(start,color='.4',lw=.7)
    for x,title in [(20,'+80 roll'),(60,'+180 pitch'),(100,'-80 roll'),(140,'-180 pitch')]:
        axes[0].text(x,1.02,title,transform=axes[0].get_xaxis_transform(),ha='center')
    axes[0].legend();axes[-1].set_xlabel('Time from maneuver start [s]; shading = peak hold')
    fig.suptitle('Full 160 s maneuver: negative-pitch transient remains; no crash or airborne disarm')
    save(fig,'04_full_maneuver_residual')
    s=data[1]
    cfg=json.loads((a.root/'sensor_fixed/logs/runtime_config.json').read_text())['allocator']
    equilibrium={}
    for i,joint in enumerate(['a_l','b_l','a_r','b_r']):
        u=s[f'switch_commanded_joint_{joint}_rad'].to_numpy()
        q=s[f'switch_measured_joint_{joint}_rad'].to_numpy()
        t=s.time_s.to_numpy()
        k=np.where(u>=0,cfg['servo_gain_positive'][i],cfg['servo_gain_negative'][i])
        mask=(s.motion_time.between(13,20)&(np.abs(u)>.4)&
              (np.abs(np.gradient(u,t))<.15)&(np.abs(np.gradient(q,t))<.15))
        equilibrium[joint]={'samples':int(mask.sum()),
            'q_minus_u_rms_deg':float(np.degrees(np.sqrt(np.mean((q[mask]-u[mask])**2)))),
            'q_minus_Ku_rms_deg':float(np.degrees(np.sqrt(np.mean((q[mask]-k[mask]*u[mask])**2))))}
    (out/'quasistatic_joint_errors.json').write_text(json.dumps(equilibrium,indent=2)+'\n')


if __name__=='__main__':main()
