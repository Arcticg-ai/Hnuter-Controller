"""Archive and audit new post-fix runs without mixing historical results."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import numpy as np
import pandas as pd
from pyulog import ULog
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
REPO=Path('/home/hnuter/px4_ws_ros2_consolidated')
SOURCE=HERE/'postfix_suite_20261012'
OUTPUT=REPO/'reports/postfix_matched_20261012'
helper=HERE/'analyze_revision.py'
if not helper.exists():helper=HERE/'followup_20261011/analyze_revision.py'
spec=importlib.util.spec_from_file_location('revision_analysis',helper)
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)


def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def write_json(p,x):p.write_text(json.dumps(x,indent=2,default=lambda a:a.item() if isinstance(a,np.generic) else str(a))+'\n')


def enrich(d,path,audit):
    u=ULog(str(path),message_name_filter_list=['vehicle_attitude_groundtruth'])
    raw=u.get_dataset('vehicle_attitude_groundtruth').data
    times=(d.time_s.values+audit['csv_to_ulog_offset_s'])*1e6
    indices=np.clip(np.searchsorted(raw['timestamp'],times,side='right')-1,0,len(raw['timestamp'])-1)
    quat=np.column_stack([raw[f'q[{i}]'][indices] for i in [1,2,3,0]])
    truth=Rotation.from_quat(quat)
    ne=np.array([[0,1,0],[1,0,0],[0,0,-1]])
    ff=np.diag([1,-1,-1])
    ref=Rotation.from_euler('xyz',d[['target_roll_deg','target_pitch_deg','target_yaw_deg']],degrees=True)
    ref=Rotation.from_matrix(ne@ref.as_matrix()@ff)
    d['true_reference_so3_deg']=np.degrees((ref.inv()*truth).magnitude())
    return d


def main():
    global SOURCE,OUTPUT
    p=argparse.ArgumentParser();p.add_argument('--partial',action='store_true')
    p.add_argument('--source',type=Path,default=SOURCE)
    p.add_argument('--output',type=Path,default=OUTPUT)
    args=p.parse_args();SOURCE=args.source.resolve();OUTPUT=args.output.resolve()
    manifest=json.loads((SOURCE/'manifest.json').read_text())
    if not args.partial:assert len(manifest['cases'])==18 and 'finished_unix' in manifest
    OUTPUT.mkdir(parents=True,exist_ok=True);rows=[];phases=[];checks=[];reference=None;allocator_ref=None
    for result in manifest['cases']:
        scenario,method=result['scenario'],result['method']
        src=SOURCE/'runs'/scenario/'repeat_1'/method
        dest=OUTPUT/'data'/scenario/'repeat_1'/method;dest.mkdir(parents=True,exist_ok=True)
        for name,path in [('diagnostic.csv',Path(result['csv'])),('result.json',src/'result.json'),
            ('runtime_config.json',src/'logs/runtime_config.json'),('effective_tuning.json',src/'effective_tuning.json')]:
            shutil.copy2(path,dest/name)
        shutil.copytree(src/'console',dest/'console',dirs_exist_ok=True)
        cfg=json.loads((dest/'runtime_config.json').read_text())
        common={k:cfg[k] for k in ['outer','shared_force_command_rate_n_s','direct_output','geometry']}
        if reference is None:reference=common
        assert common==reference,(scenario,method,'outer/constraints/output mismatch')
        assert cfg['geometry']['gazebo_unit_dc_gain'] is True
        assert cfg['direct_output']==dict(position=False,allocator_setpoints=False,landing_hint=True)
        allocator=cfg['allocator']
        if allocator:
            for key in ['servo_gain_positive','servo_gain_negative']:assert allocator[key]==[1.]*4
            for key in ['servo_delay_positive_s','servo_delay_negative_s']:assert allocator[key]==[0.]*4
            keys=[k for k in allocator if k.startswith(('servo_','motor_','thrust_'))]
            common_allocator={k:allocator[k] for k in keys}
            if allocator_ref is None:allocator_ref=common_allocator
            assert common_allocator==allocator_ref,(scenario,method,'allocator constraints/model mismatch')
        for key,value in dict(tilt_s=15.,peak_hold_s=5.,level_settle_s=5.,phase_ramp_s=.75,
            early_handover=True,takeoff_guard_end_s=12.5).items():assert cfg['trajectory'][key]==value
        if scenario.startswith('attitude'):np.testing.assert_allclose(cfg['trajectory']['attitude_amplitudes_deg'],[80,180,0])
        assert result['source_sha256']==manifest['cases'][0]['source_sha256'],'controller changed during batch'
        assert result['tuning_sha256']==manifest['cases'][0]['tuning_sha256'],'tuning changed during batch'
        raw=Path(result['ulog_source']);target=dest/'flight.ulg'
        if not target.exists():shutil.copy2(raw,target)
        assert digest(raw)==digest(target)
        if (dest/'ulog_metadata.json').exists():audit=json.loads((dest/'ulog_metadata.json').read_text())
        else:audit=old.ulog_audit(str(target),dest)
        assert audit['direct_actuator_all']
        assert not any(audit['forbidden_offboard_flags'].values())
        assert not any(audit['internal_control_flags_in_offboard'].values())
        d=old.load_frame(dest/'diagnostic.csv');d=enrich(d,target,audit)
        if method in ['full','no_preview','no_nearfield']:
            np.testing.assert_allclose(d.drcda_prediction_h1_s,.05)
            np.testing.assert_allclose(d.drcda_prediction_h2_s,.10)
        if method=='no_horizon':
            np.testing.assert_allclose(d.drcda_prediction_h1_s,.01)
            assert d.drcda_prediction_h2_s.isna().all()
        ud=pd.read_csv(dest/'ulog_flight_diagnostics.csv')
        for name in ['truth_relative_height_m','estimator_truth_so3_deg']:
            d[name]=np.interp(d.t,ud.time_from_motion_s,ud[name])
        d.to_csv(dest/'plot_data.csv',index=False)
        m=dict(scenario=scenario,method=method,repeat=1,**result['metrics'])
        m.update({k:audit[k] for k in ['first_physical_altitude_loss_s','task_truth_min_relative_height_m',
            'task_estimator_truth_so3_peak_deg','task_quat_reset_count','elevated_landing_disarms']})
        m['completed_airborne']=bool(m['finished'] and m['remained_armed'] and not m['safety_cutoff']
            and audit['first_physical_altitude_loss_s'] is None)
        if audit['first_physical_altitude_loss_s'] is not None:
            m['passed']=False;m['failures']=list(m['failures'])+['true height below .5 m']
        m.update(position_p95_m=float(d.ep.quantile(.95)),so3_p95_deg=float(d.attitude_error_angle_deg.quantile(.95)),
            true_so3_rms_deg=float(np.sqrt(np.mean(d.true_reference_so3_deg**2))),
            true_so3_peak_deg=float(d.true_reference_so3_deg.max()),
            entry_so3_peak_deg=float(d.loc[d.t<2,'attitude_error_angle_deg'].max()))
        for kind in ['commanded','measured']:
            cols=[f'switch_{kind}_joint_{j}_rad' for j in ['a_l','b_l','a_r','b_r']]
            m[f'{kind}_joint_total_variation_rad']=float(d[cols].diff().abs().sum().sum())
        loss=audit['first_physical_altitude_loss_s']
        pre=d if loss is None else d[d.t<loss]
        m['pre_loss_position_rms_m']=float(np.sqrt(np.mean(pre.ep**2)))
        m['pre_loss_duration_s']=float(pre.t.iloc[-1]) if len(pre) else 0.
        m['initial_reference_yaw_deg']=float(d.target_yaw_deg.iloc[0])
        m['diagnostic_median_dt_s']=float(d.time_s.diff().median())
        for phase,g in d.groupby('switch_phase',sort=False):
            phases.append(dict(scenario=scenario,method=method,phase=phase,samples=len(g),
                start_s=float(g.t.iloc[0]),end_s=float(g.t.iloc[-1]),
                position_rms_m=float(np.sqrt(np.mean(g.ep**2))),position_peak_m=float(g.ep.max()),
                so3_rms_deg=float(np.sqrt(np.mean(g.attitude_error_angle_deg**2))),so3_peak_deg=float(g.attitude_error_angle_deg.max()),
                true_so3_rms_deg=float(np.sqrt(np.mean(g.true_reference_so3_deg**2))),
                true_so3_peak_deg=float(g.true_reference_so3_deg.max()),
                after_height_loss=bool(loss is not None and g.t.iloc[-1]>=loss)))
        rows.append(m);checks.append(dict(scenario=scenario,method=method,common_configuration=True,
            unit_gain=True,pure_delay=False,direct_actuation_verified=True,
            ulog_sha256=digest(target),controller_sha256=result['source_sha256'],tuning_sha256=result['tuning_sha256']))
        print(scenario,method,'archived and audited',flush=True)
    pd.DataFrame(rows).to_csv(OUTPUT/'metrics.csv',index=False)
    pd.DataFrame(phases).to_csv(OUTPUT/'phase_metrics.csv',index=False)
    write_json(OUTPUT/'verification.json',dict(complete_matrix=len(rows)==18,case_count=len(rows),
        checks=checks,common_runtime=reference,common_actuator_model=allocator_ref))
    shutil.copy2(SOURCE/'manifest.json',OUTPUT/'run_manifest.json')
    shutil.copytree(SOURCE/'provenance',OUTPUT/'provenance',dirs_exist_ok=True)
    scripts=OUTPUT/'scripts';scripts.mkdir(exist_ok=True)
    for name in ['run_postfix_suite.py','finalize_postfix_suite.py']:
        if (HERE/name).resolve()!=(scripts/name).resolve():shutil.copy2(HERE/name,scripts/name)
    if helper.resolve()!=(scripts/'analyze_revision.py').resolve():shutil.copy2(helper,scripts/'analyze_revision.py')
    shutil.copy2(SOURCE/'batch.log',OUTPUT/'batch.log')
    if (SOURCE/'regression_tests.xml').exists():shutil.copy2(SOURCE/'regression_tests.xml',OUTPUT/'regression_tests.xml')
    print(pd.DataFrame(rows)[['scenario','method','passed','completed_airborne','position_rms_m','so3_peak_deg','true_so3_peak_deg']].to_string(index=False))


if __name__=='__main__':main()
