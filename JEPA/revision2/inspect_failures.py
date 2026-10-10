"""Read original logs only; compare estimated vs true attitude at safety exits."""
import csv
import json
from pathlib import Path
import numpy as np
from pyulog import ULog

ROOT=Path(__file__).resolve().parent
CASES=['sitl_noisy_C_shadow_s3_20261010_173019','sitl_noisy_C_s4_20261010_173110','sitl_noisy_B_s4_20261010_173202']


def main():
    results=[]
    for case in CASES:
        p=ROOT.parent/'runs'/case
        with (p/'trajectory.csv').open() as f:rows=list(csv.DictReader(f))
        end=float(rows[-1]['px4_time_s']);begin=float(rows[0]['px4_time_s'])
        u=ULog(str(next((p/'rootfs/log').rglob('*.ulg'))))
        a=u.get_dataset('vehicle_attitude').data;g=u.get_dataset('vehicle_attitude_groundtruth').data
        t=a['timestamp']*1e-6;mask=(t>=end-5)&(t<=end)
        q=np.array([a[f'q[{i}]'][mask] for i in range(4)]).T
        gt=np.array([np.interp(t[mask],g['timestamp']*1e-6,g[f'q[{i}]']) for i in range(4)]).T
        gt/=np.linalg.norm(gt,axis=1)[:,None]
        attitude_err=np.degrees(2*np.arccos(np.clip(np.abs(np.sum(q*gt,axis=1)),0,1)))
        tilt=lambda v:np.degrees(np.arccos(np.clip(1-2*(v[:,1]**2+v[:,2]**2),-1,1)))
        pos=u.get_dataset('vehicle_local_position_groundtruth').data;pt=pos['timestamp']*1e-6
        st=u.get_dataset('hnuter_control_status').data;stm=(st['timestamp']*1e-6>=end-5)&(st['timestamp']*1e-6<=end)
        nav=u.get_dataset('vehicle_status').data
        result=dict(case=case,last_reference=rows[-1],last5s_attitude_error_max_deg=float(max(attitude_err)),
            last5s_attitude_error_mean_deg=float(np.mean(attitude_err)),estimated_tilt_last_deg=float(tilt(q)[-1]),true_tilt_last_deg=float(tilt(gt)[-1]),
            true_height_change_from_arming_m=float(-np.interp(end,pt,pos['z'])+np.interp(begin,pt,pos['z'])),
            desired_attitude_last=[float(st[f'attitude_setpoint_q[{i}]'][stm][-1]) for i in range(4)],
            active_mass_last=float(st['effective_mass'][stm][-1]),
            nav_states_after_exit=np.unique(nav['nav_state'][nav['timestamp']*1e-6>=end]).tolist(),
            original_result=json.loads((p/'flight_result.json').read_text()))
        results.append(result)
    (ROOT/'original_failure_audit.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results,indent=2))


if __name__=='__main__':main()
