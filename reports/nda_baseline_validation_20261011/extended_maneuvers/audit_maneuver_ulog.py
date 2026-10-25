"""Audit termination and estimator/ground-truth discrepancy without changing scores."""
import argparse
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/hnuter-maneuver-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pyulog import ULog

BASE = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--report', type=Path, default=BASE/'maneuver_report')
args = parser.parse_args()
out = args.report
rows = []
for r in json.loads((out/'results.json').read_text()):
    parent = out/Path(r['archived_csv']).parent
    reduced = parent/'ulog_flight_diagnostics.csv'
    events_path = parent/'ulog_termination.json'
    if reduced.exists() and events_path.exists():
        d = pd.read_csv(reduced)
        events = json.loads(events_path.read_text())
    else:
        u = ULog(r['ulog_source'])
        gt = u.get_dataset('vehicle_local_position_groundtruth').data
        lp = u.get_dataset('vehicle_local_position').data
        land = u.get_dataset('vehicle_land_detected').data
        mot = u.get_dataset('actuator_motors').data
        stat = u.get_dataset('vehicle_status').data
        t = gt['timestamp'].astype(float)
        def sample(topic, key):
            ix = np.searchsorted(topic['timestamp'], t, side='right')-1
            values = np.asarray(topic[key], dtype=float)[np.maximum(ix, 0)]
            values[ix < 0] = np.nan
            return values
        d = pd.DataFrame({'timestamp_s':t/1e6, 'truth_height_m':-gt['z'],
                          'estimated_height_m':-sample(lp,'z'),
                          'landed':sample(land,'landed'),
                          'low_throttle':sample(land,'has_low_throttle'),
                          'arming_state':sample(stat,'arming_state'),
                          'main_motor_mean':np.mean([sample(mot,f'control[{i}]') for i in range(4)],axis=0)})
        d['velocity_discrepancy_m_s'] = np.linalg.norm(
            np.array([gt[k]-sample(lp,k) for k in ['vx','vy','vz']]),axis=0)
        # Positions can have different estimator origins: compare displacement from
        # the first armed sample, not the absolute simulator/EKF coordinates.
        armed = np.flatnonzero(d.arming_state.to_numpy()==2)
        anchor = int(armed[0]) if len(armed) else 0
        dp = np.array([gt[k]-sample(lp,k) for k in ['x','y','z']])
        dp -= dp[:,anchor,None]
        d['relative_position_discrepancy_m'] = np.linalg.norm(dp,axis=0)
        d['time_from_arm_s'] = (t-t[anchor])/1e6
        p = u.initial_parameters
        threshold = p['HNTR_MOT_HOV']*p['HNTR_LND_MIN_R']
        d['minimum_throttle_threshold'] = threshold
        events = []
        for msg in u.logged_messages:
            if 'Disarmed by' not in msg.message: continue
            ix = max(0,int(np.searchsorted(t,msg.timestamp)-1))
            events.append(dict(message=msg.message.strip(),timestamp_s=msg.timestamp/1e6,
                               time_from_arm_s=float(d.time_from_arm_s.iloc[ix]),
                               truth_height_m=float(d.truth_height_m.iloc[ix]),
                               estimated_height_m=float(d.estimated_height_m.iloc[ix]),
                               velocity_discrepancy_m_s=float(d.velocity_discrepancy_m_s.iloc[ix]),
                               motor_mean=float(d.main_motor_mean.iloc[ix]),
                               minimum_throttle_threshold=float(threshold)))
        d.to_csv(reduced,index=False)
        events_path.write_text(json.dumps(events,indent=2)+'\n')
    airborne = d[(d.arming_state==2)&(d.truth_height_m>.5)]
    landing_events = [e for e in events if 'landing' in e['message']]
    elevated = [e for e in landing_events if e['truth_height_m']>.5]
    rows.append(dict(scenario=r['scenario'],method=r['method'],repeat=r['repeat'],
                     landing_disarm_count=len(landing_events),elevated_landing_disarm_count=len(elevated),
                     elevated_disarm_heights_m=';'.join(f"{e['truth_height_m']:.3f}" for e in elevated),
                     airborne_velocity_discrepancy_peak_m_s=float(airborne.velocity_discrepancy_m_s.max()),
                     airborne_relative_position_discrepancy_peak_m=float(airborne.relative_position_discrepancy_m.max())))
    if r['method']=='full' and r['repeat']==1 and r['scenario'].startswith('attitude'):
        fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True,layout='constrained')
        axes[0].plot(d.time_from_arm_s,d.truth_height_m,label='Gazebo truth')
        axes[0].plot(d.time_from_arm_s,d.estimated_height_m,label='PX4 estimate')
        axes[0].set_ylabel('Height (m)'); axes[0].legend()
        axes[1].plot(d.time_from_arm_s,d.main_motor_mean,label='Mean main motor command')
        axes[1].plot(d.time_from_arm_s,d.minimum_throttle_threshold,':',label='Minimum-throttle threshold')
        axes[1].plot(d.time_from_arm_s,d.landed*.5,'--',label='Landed flag x 0.5')
        axes[1].set_ylabel('Normalized value');axes[1].legend(fontsize=8)
        axes[2].plot(d.time_from_arm_s,d.velocity_discrepancy_m_s,label='Estimator vs truth')
        axes[2].set_ylabel('Velocity difference (m/s)');axes[2].set_xlabel('Time from arming (s)')
        for ax in axes:
            ax.grid(alpha=.25)
            for e in events: ax.axvline(e['time_from_arm_s'],color='red',ls=':',lw=1)
        fig.suptitle(r['scenario']+' | Full repeat 1 | ULog termination audit\n'
                     'Red line: logged disarm; includes takeoff, not just maneuver')
        fig.savefig(out/'figures'/f"{r['scenario']}_termination.png",dpi=180,bbox_inches='tight')
        plt.close(fig)
pd.DataFrame(rows).to_csv(out/'termination_audit.csv',index=False)
print(pd.DataFrame(rows).to_string(index=False))
