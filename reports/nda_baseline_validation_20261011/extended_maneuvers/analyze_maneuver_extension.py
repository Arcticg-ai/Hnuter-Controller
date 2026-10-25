"""Build figures from the actual matched no-delay maneuver logs."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil

os.environ.setdefault('MPLCONFIGDIR', '/tmp/hnuter-maneuver-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

BASE = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--runs', type=Path, default=BASE/'maneuver_matched')
parser.add_argument('--output', type=Path, default=BASE/'maneuver_report')
args = parser.parse_args()
OUT = args.output.resolve()
OUT.mkdir(parents=True, exist_ok=True)
(OUT/'figures').mkdir(exist_ok=True)
LABELS = dict(direct='Direct', paper_nda='NDA (Hnuter)', full='Full DRCDA',
              no_preview='DRCDA no preview', no_horizon='DRCDA one step')
COLORS = dict(direct='#2879ad', paper_nda='#267a50', full='#cc453c',
              no_preview='#b38518', no_horizon='#7e57a2')


def read(r):
    d = pd.read_csv(r['csv'])
    d = d[~d.switch_phase.isin(['idle','ingress','settle_a'])].copy()
    if d.empty: return d
    d['t'] = d.time_s-d.time_s.iloc[0]
    d['ep'] = np.linalg.norm(d[['position_x_enu_m','position_y_enu_m','position_z_rel_m']].values
                           - d[['target_x_enu_m','target_y_enu_m','target_z_rel_m']].values, axis=1)
    return d


records = []
for p in sorted(args.runs.rglob('result.json')):
    r = json.loads(p.read_text())
    local_csv = p.parent/'diagnostic.csv'
    if local_csv.exists(): r['csv'] = str(local_csv)
    if r.get('csv') and Path(r['csv']).exists():
        dest = OUT/'data'/r['scenario']/f'repeat_{r["repeat"]}'/r['method']
        dest.mkdir(parents=True, exist_ok=True)
        sources = {'result.json':p, 'diagnostic.csv':Path(r['csv']),
                   'effective_tuning.json':p.parent/'effective_tuning.json',
                   'runtime_config.json':p.parent/'runtime_config.json' if local_csv.exists()
                                          else p.parent/'logs/runtime_config.json'}
        for name in ['ulog_flight_diagnostics.csv','ulog_termination.json','ulog_metadata.json']:
            sources[name] = p.parent/name
        for name, source in sources.items():
            if source.exists() and source.resolve() != (dest/name).resolve(): shutil.copy2(source, dest/name)
        if (p.parent/'console').exists() and (p.parent/'console').resolve() != (dest/'console').resolve():
            shutil.copytree(p.parent/'console', dest/'console', dirs_exist_ok=True)
        r['archived_csv'] = str((dest/'diagnostic.csv').relative_to(OUT))
        r['csv_sha256'] = hashlib.sha256(Path(r['csv']).read_bytes()).hexdigest()
    records.append(r)
(OUT/'results.json').write_text(json.dumps(records, indent=2)+'\n')
pd.DataFrame([dict(scenario=r['scenario'], repeat=r['repeat'], method=r['method'], **r['metrics'])
              for r in records]).to_csv(OUT/'metrics.csv', index=False)


def save(fig, name):
    fig.savefig(OUT/'figures'/f'{name}.png', dpi=180, bbox_inches='tight')
    plt.close(fig)


def curves(rows, name, clipped=False):
    if not rows: return
    fig, axes = plt.subplots(3,2,figsize=(13,9),sharex=True,layout='constrained')
    for r in rows:
        d = read(r)
        if d.empty: continue
        ys = [d.ep, d.attitude_error_angle_deg, d.angular_r_frd_rps, d.position_z_rel_m,
              d.switch_commanded_joint_b_l_rad, d.switch_measured_joint_b_l_rad]
        for ax, y in zip(axes.flat, ys):
            ax.plot(d.t, y, color=COLORS[r['method']], lw=1,
                    ls='-' if r['repeat']==1 else '--',
                    label=LABELS[r['method']] if r['repeat']==1 else '_nolegend_')
    for ax, label in zip(axes.flat, ['3D position error (m)','SO(3) error (deg)',
                                    'Body yaw rate (rad/s)','Altitude (m)',
                                    'Left beta command (rad)','Left beta measured (rad)']):
        ax.set_ylabel(label); ax.grid(alpha=.25)
    axes[0,0].axhline(.75,color='gray',ls=':',lw=.8)
    axes[0,1].axhline(12.,color='gray',ls=':',lw=.8)
    axes[0,0].legend(fontsize=8,ncol=2)
    for ax in axes[-1]: ax.set_xlabel('Time since allocator takeover (s)')
    title = rows[0]['scenario']+' | No pure actuator delay | solid: repeat 1; dashed: repeat 2'
    if clipped:
        axes[0,0].set_ylim(0,2); axes[0,1].set_ylim(0,40)
        axes[1,0].set_ylim(-1,1); axes[1,1].set_ylim(0,5)
        title += '\nClipped diagnostic view; consult full-scale figure for failures'
    fig.suptitle(title)
    save(fig,name)


phase_rows = []
window_rows = []
for scenario in sorted({r['scenario'] for r in records}):
    rows = [r for r in records if r['scenario']==scenario and r.get('csv')]
    curves([r for r in rows if r['method'] in {'direct','paper_nda','full'}], scenario+'_comparison')
    curves([r for r in rows if r['method'] in {'direct','paper_nda','full'}], scenario+'_detail',True)
    curves([r for r in rows if r['method'] in {'full','no_preview','no_horizon'}], scenario+'_ablation')
    if scenario.startswith('lissajous'):
        duration = float(scenario.rsplit('_',1)[1])
        for r in rows:
            d=read(r)
            if d.empty: continue
            for label,mask in [('entry_0_to_1s',d.t<1.),('motion_after_1s',(d.t>=1.)&(d.t<duration)),
                               ('post_hold',d.t>=duration)]:
                g=d[mask]
                if g.empty: continue
                peak_index=g.attitude_error_angle_deg.idxmax()
                window_rows.append(dict(scenario=scenario,method=r['method'],repeat=r['repeat'],window=label,
                    position_rms_m=float(np.sqrt(np.mean(g.ep**2))),position_peak_m=float(g.ep.max()),
                    so3_peak_deg=float(g.attitude_error_angle_deg.max()),
                    so3_peak_time_s=float(g.loc[peak_index,'t']),
                    yaw_rate_peak_rad_s=float(g.angular_r_frd_rps.abs().max())))
        fig = plt.figure(figsize=(10,8),layout='constrained')
        ax = fig.add_subplot(111,projection='3d')
        reference_drawn = False
        for r in rows:
            if r['method'] not in {'direct','paper_nda','full'}: continue
            d = read(r)
            if d.empty: continue
            x0,y0 = d.target_x_enu_m.iloc[0], d.target_y_enu_m.iloc[0]
            ax.plot(d.position_x_enu_m-x0,d.position_y_enu_m-y0,d.position_z_rel_m,
                    color=COLORS[r['method']],ls='-' if r['repeat']==1 else '--',lw=1,
                    label=LABELS[r['method']] if r['repeat']==1 else '_nolegend_')
            if not reference_drawn:
                ax.plot(d.target_x_enu_m-x0,d.target_y_enu_m-y0,d.target_z_rel_m,'k:',label='Reference')
                reference_drawn = True
        ax.set(xlabel='Relative ENU x (m)',ylabel='Relative ENU y (m)')
        ax.text2D(.96,.50,'Altitude (m)',rotation=90,va='center',transform=ax.transAxes)
        ax.legend(); fig.suptitle(scenario+' | Full trajectories, including failed tracking')
        save(fig,scenario+'_3d')
        fig = plt.figure(figsize=(10,8),layout='constrained')
        ax = fig.add_subplot(111,projection='3d')
        references=[]
        for r in rows:
            d=read(r)
            if d.empty: continue
            ref=d[['target_x_enu_m','target_y_enu_m','target_z_rel_m']].to_numpy()
            ref[:,:2]-=ref[0,:2]
            references.append(ref)
        if references:
            reference=np.vstack(references)
            lower=reference.min(axis=0)-[.7,.7,.5]
            upper=reference.max(axis=0)+[.7,.7,.5]
            for r in rows:
                d=read(r)
                if d.empty: continue
                xyz=d[['position_x_enu_m','position_y_enu_m','position_z_rel_m']].to_numpy()
                xyz[:,:2]-=d[['target_x_enu_m','target_y_enu_m']].iloc[0].to_numpy()
                keep=np.all((xyz>=lower)&(xyz<=upper),axis=1)
                xyz[~keep]=np.nan
                ax.plot(*xyz.T,color=COLORS[r['method']],lw=1,
                        ls='-' if r['repeat']==1 else '--',
                        label=LABELS[r['method']] if r['repeat']==1 else '_nolegend_')
            ax.plot(*references[0].T,'k:',label='Reference')
            ax.set(xlim=(lower[0],upper[0]),ylim=(lower[1],upper[1]),zlim=(lower[2],upper[2]),
                   xlabel='Relative ENU x (m)',ylabel='Relative ENU y (m)')
            ax.set_box_aspect(upper-lower)
            ax.text2D(.96,.50,'Altitude (m)',rotation=90,va='center',transform=ax.transAxes)
            ax.legend(fontsize=8)
            fig.suptitle(scenario+' | Reference-neighborhood view, all methods\n'
                         'Clipped paths; use full-scale figure and metrics for failures')
            save(fig,scenario+'_3d_detail')
    else:
        fig, axes = plt.subplots(2,2,figsize=(13,7),sharex=True,layout='constrained')
        for r in rows:
            d = read(r)
            if d.empty: continue
            for phase, g in d.groupby('switch_phase',sort=False):
                phase_rows.append(dict(scenario=scenario,method=r['method'],repeat=r['repeat'],phase=phase,
                    duration_s=float(g.t.iloc[-1]-g.t.iloc[0]),position_peak_m=float(g.ep.max()),
                    so3_peak_deg=float(g.attitude_error_angle_deg.max()),
                    min_altitude_m=float(g.position_z_rel_m.min()),armed=bool((g.armed==1).all())))
            if r['method'] not in {'direct','paper_nda','full'}: continue
            style = dict(color=COLORS[r['method']], ls='-' if r['repeat']==1 else '--',lw=1)
            label = LABELS[r['method']] if r['repeat']==1 else '_nolegend_'
            axes[0,0].plot(d.t,d.target_roll_deg,**style,label=label)
            axes[0,1].plot(d.t,d.target_pitch_deg,**style,label=label)
            axes[1,0].plot(d.t,d.roll_deg.where(d.switch_phase.str.startswith('roll')),**style)
            axes[1,1].plot(d.t,d.continuous_test_pitch_deg.where(d.switch_phase.str.startswith('pitch')),**style)
        for ax,label in zip(axes.flat,['Requested roll (deg)','Requested pitch (deg)',
                                      'Measured roll, roll phases (deg)','Measured continuous pitch, pitch phases (deg)']):
            ax.set_ylabel(label);ax.grid(alpha=.25)
        axes[0,0].legend(fontsize=8)
        for ax in axes[-1]:ax.set_xlabel('Time since allocator takeover (s)')
        fig.suptitle(scenario+' | Orientation traces; SO(3) is the tracking metric\n'
                     'Later phases after loss of tracking are not independent capability tests')
        save(fig,scenario+'_attitude')
pd.DataFrame(phase_rows).to_csv(OUT/'attitude_phase_metrics.csv',index=False)
pd.DataFrame(window_rows).to_csv(OUT/'lissajous_window_diagnostics.csv',index=False)

actuation_rows=[]
for r in records:
    if not r.get('csv'): continue
    d=read(r)
    row=dict(scenario=r['scenario'],method=r['method'],repeat=r['repeat'])
    for kind in ['commanded','measured']:
        columns=[f'switch_{kind}_joint_{axis}_rad' for axis in ['a_l','b_l','a_r','b_r']]
        row[f'joint_{kind}_total_variation_rad']=float(d[columns].diff().abs().sum().sum())
    row['logged_duration_s']=float(d.t.iloc[-1]) if len(d) else 0.
    actuation_rows.append(row)
pd.DataFrame(actuation_rows).to_csv(OUT/'actuation_diagnostics.csv',index=False)

sections=[]
for scenario in sorted({r['scenario'] for r in records}):
    lines=[f'### {scenario}', '', '| 方法 | 通过 | 位置峰值/m | SO(3)峰值/deg | 偏航速度峰值/(rad/s) |',
           '|---|---:|---:|---:|---:|']
    for method,label in LABELS.items():
        rows=[r for r in records if r['scenario']==scenario and r['method']==method]
        if not rows:continue
        cells=[]
        for key in ['position_peak_m','so3_peak_deg','yaw_rate_peak_rad_s']:
            a=[r['metrics'].get(key,np.nan) for r in rows]
            cells.append(f'{np.nanmin(a):.3f}–{np.nanmax(a):.3f}')
        lines.append(f'| {label} | {sum(r["metrics"]["passed"] for r in rows)}/{len(rows)} | '+ ' | '.join(cells)+' |')
    lines.extend(['',f'![{scenario} main comparison](figures/{scenario}_comparison.png)',
                  '',f'![{scenario} ablation](figures/{scenario}_ablation.png)',
                  '',f'[局部误差放大图](figures/{scenario}_detail.png)'])
    if scenario.startswith('lissajous'):
        lines.append(f'[三维全轨迹](figures/{scenario}_3d.png) · [三维局部视图](figures/{scenario}_3d_detail.png)')
    else:
        lines.append(f'[姿态参考与实测曲线](figures/{scenario}_attitude.png) · [上锁审计](figures/{scenario}_termination.png)')
    sections.append('\n'.join(lines))
(OUT/'RESULTS.md').write_text('\n\n'.join(sections)+'\n')
print(f'Processed {len(records)} logged runs')
print('\n\n'.join(sections))
