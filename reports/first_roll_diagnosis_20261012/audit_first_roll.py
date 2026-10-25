"""Align direct-controller logs with PX4 estimated and Gazebo true attitude."""
import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd
from pyulog import ULog
from scipy.spatial.transform import Rotation


def main():
    p=argparse.ArgumentParser()
    p.add_argument('run',type=Path)
    p.add_argument('ulog',type=Path)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    d=pd.read_csv(next(a.run.rglob('switching_*.csv')),low_memory=False)
    u=ULog(str(a.ulog))
    topic=lambda n:u.get_dataset(n).data
    status=topic('vehicle_status')
    offset=status['timestamp'][status['arming_state']==2][0]/1e6-d.loc[d.armed==1,'time_s'].iloc[0]
    times=(d.time_s.to_numpy()+offset)*1e6
    def sample(name,fields):
        raw=topic(name);indices=np.clip(np.searchsorted(raw['timestamp'],times,side='right')-1,0,len(raw['timestamp'])-1)
        return np.column_stack([raw[f][indices] for f in fields])
    def rotation(name):
        q=sample(name,[f'q[{i}]' for i in range(4)])
        return Rotation.from_quat(q[:,[1,2,3,0]])
    est=rotation('vehicle_attitude');truth=rotation('vehicle_attitude_groundtruth')
    ne=np.array([[0,1,0],[1,0,0],[0,0,-1]])
    ff=np.diag([1,-1,-1])
    ref=Rotation.from_euler('xyz',d[['target_roll_deg','target_pitch_deg','target_yaw_deg']].values,degrees=True)
    ref=Rotation.from_matrix(ne@ref.as_matrix()@ff)
    d['est_truth_deg']=(truth.inv()*est).magnitude()*180/np.pi
    d['truth_ref_deg']=(ref.inv()*truth).magnitude()*180/np.pi
    for tag,rot in [('truth',truth),('est',est)]:
        e=Rotation.from_matrix(ne@rot.as_matrix()@ff).as_euler('xyz',degrees=True)
        for i,k in enumerate(['roll','pitch','yaw']):d[f'{tag}_{k}_deg']=e[:,i]
    raw_truth=topic('vehicle_attitude_groundtruth')
    tq=np.column_stack([raw_truth[f'q[{i}]'] for i in [1,2,3,0]])
    tr=Rotation.from_quat(tq)
    delta_t=np.diff(raw_truth['timestamp'])/1e6
    true_omega=(tr[:-1].inv()*tr[1:]).as_rotvec()/delta_t[:,None]
    mid_t=(raw_truth['timestamp'][1:].astype(float)+raw_truth['timestamp'][:-1])/2
    for tag,name in [('truth',None),('est','vehicle_angular_velocity')]:
        omega=(np.column_stack([np.interp(times,mid_t,true_omega[:,i]) for i in range(3)])
               if name is None else sample(name,[f'xyz[{i}]' for i in range(3)]))
        for i,k in enumerate('pqr'):d[f'{tag}_{k}']=omega[:,i]
    d['gyro_truth_error']=np.linalg.norm(d[['truth_p','truth_q','truth_r']].values-d[['est_p','est_q','est_r']].values,axis=1)
    pos=sample('vehicle_local_position_groundtruth',['x','y','z'])
    for i,k in enumerate('xyz'):d[f'truth_pos_{k}']=pos[:,i]
    d['position_error_m']=np.linalg.norm(d[['position_x_enu_m','position_y_enu_m','position_z_rel_m']].values-
        d[['target_x_enu_m','target_y_enu_m','target_z_rel_m']].values,axis=1)
    arm_index=d.index[d.armed==1][0]
    d['truth_relative_height_m']=-pos[:,2]+pos[arm_index,2]
    first=d.index[d.switch_phase=='roll_pos_out'][0]
    d['motion_time']=d.time_s-d.time_s.iloc[first]
    d.to_csv(a.output/'aligned.csv',index=False)
    fields=['motion_time','target_roll_deg','roll_deg','truth_roll_deg','attitude_error_angle_deg',
            'est_truth_deg','truth_ref_deg','gyro_truth_error','wrench_tau_x_nm',
            'drcda_estimated_wrench_tx_nm','drcda_predicted_wrench_tx_nm']
    print(d[d.motion_time.between(0,17)].iloc[::43][fields].round(3).to_string(index=False))
    summary={'ulog':str(a.ulog),'offset_s':float(offset),'windows':{},
        'angular_rate_truth':'SO(3) logarithm of successive true quaternions, not Euler-angle differences'}
    for lo,hi in [(0,2),(2,4),(4,8),(8,12),(12,15),(15,20)]:
        s=d[d.motion_time.between(lo,hi)]
        summary['windows'][f'{lo}-{hi}']={k:float(s[k].max()) for k in ['est_truth_deg','truth_ref_deg','gyro_truth_error']}
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    rows=[]
    for phase,s in d[d.motion_time>=0].groupby('switch_phase',sort=False):
        rows.append(dict(phase=phase,samples=len(s),position_peak_m=float(s.position_error_m.max()),
            so3_rms_deg=float(np.sqrt(np.mean(s.attitude_error_angle_deg**2))),
            so3_peak_deg=float(s.attitude_error_angle_deg.max()),
            estimator_truth_peak_deg=float(s.est_truth_deg.max()),truth_ref_peak_deg=float(s.truth_ref_deg.max()),
            truth_height_min_m=float(s.truth_relative_height_m.min()),armed_all=bool((s.armed==1).all())))
    pd.DataFrame(rows).to_csv(a.output/'phases.csv',index=False)


if __name__=='__main__':main()
