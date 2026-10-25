"""Summarize actual logged runs; never synthesize trajectory results."""
import csv
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil

os.environ.setdefault('MPLCONFIGDIR', '/tmp/nda-validation-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

BASE = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--runs', type=Path, default=BASE/'runs')
parser.add_argument('--output', type=Path, default=BASE/'report')
args = parser.parse_args()
RUNS = args.runs.resolve()
OUT = args.output.resolve()
OUT.mkdir(exist_ok=True)
(OUT/'figures').mkdir(exist_ok=True)
LABELS = {'paper_nda': 'NDA (Hnuter adaptation)', 'full': 'Full DRCDA', 'direct': 'Direct',
          'no_preview': 'DRCDA no preview', 'no_horizon': 'DRCDA one step',
          'paper_nda_no_posture': 'NDA no posture', 'paper_nda_continuous': 'NDA continuous inverse'}
COLORS = {'paper_nda': '#267a50', 'full': '#cc453c', 'direct': '#2879ad',
          'no_preview': '#b38518', 'no_horizon': '#7e57a2',
          'paper_nda_no_posture': '#ad5879', 'paper_nda_continuous': '#695044'}

def load(result):
    df = pd.read_csv(result['csv'])
    df = df.sort_values('time_s')
    start = df.loc[df.switch_phase == 'a_to_b']
    if start.empty:
        return None
    df['t'] = df.time_s - start.iloc[0].time_s
    df['ep'] = np.linalg.norm(df[['position_x_enu_m','position_y_enu_m','position_z_rel_m']].values
                             - df[['target_x_enu_m','target_y_enu_m','target_z_rel_m']].values, axis=1)
    return df

def copy_file(source, target):
    if Path(source).resolve() != Path(target).resolve():
        shutil.copy2(source, target)


records = []
for path in sorted(RUNS.rglob('result.json')):
    r = json.loads(path.read_text())
    cohort = path.relative_to(RUNS).parts[0]
    r['cohort'] = cohort
    r['repeat'] = next((p for p in path.parts if p.startswith('repeat_')), 'repeat_1')
    if (path.parent/'diagnostic.csv').is_file():
        r['csv'] = str(path.parent/'diagnostic.csv')
        r['tuning'] = str(path.parent/'effective_tuning.json')
    if r.get('csv') and Path(r['csv']).is_file():
        destination = OUT/'data'/path.parent.relative_to(RUNS)
        destination.mkdir(parents=True, exist_ok=True)
        copy_file(r['csv'], destination/'diagnostic.csv')
        copy_file(path, destination/'result.json')
        copy_file(r['tuning'], destination/'effective_tuning.json')
        if (path.parent/'console').resolve() != (destination/'console').resolve():
            shutil.copytree(path.parent/'console', destination/'console', dirs_exist_ok=True)
        r['csv_sha256'] = hashlib.sha256(Path(r['csv']).read_bytes()).hexdigest()
        r['archived_csv'] = str((destination/'diagnostic.csv').relative_to(OUT))
    records.append(r)
(OUT/'all_results.json').write_text(json.dumps(records, indent=2, ensure_ascii=False)+'\n')
flat = []
for r in records:
    flat.append(dict(cohort=r['cohort'], method=r['method'], repeat=r['repeat'],
                     duration=r['switch_duration_s'], span=r.get('span_m'),
                     status=r['status'], csv=r.get('archived_csv'), error=r.get('error'),
                     **r.get('metrics', {})))
pd.DataFrame(flat).to_csv(OUT/'all_metrics.csv', index=False)

diagnostics = []
for r in records:
    if r['cohort'] not in {'comparison_common', 'qualification_hover_common', 'qualification_hover_long',
                           'qualification_low_common', 'nda_ablations'}:
        continue
    df = load(r)
    if df is None:
        continue
    df = df[df.t >= 0]
    row = dict(cohort=r['cohort'], method=r['method'], repeat=r['repeat'],
               duration=r['switch_duration_s'])
    for name in ['drcda_solve_ms', 'control_dt_s', 'nda_saturation_scale',
                 'nda_normalized_condition']:
        if name not in df:
            continue
        values = df[name].replace([np.inf, -np.inf], np.nan).dropna()
        for label, quantile in [('median', .5), ('p95', .95), ('max', 1)]:
            row[f'{name}_{label}'] = values.quantile(quantile) if len(values) else np.nan
    row['nda_saturation_fraction'] = (float((df.nda_saturation_scale < .999).mean())
                                      if df.nda_saturation_scale.notna().any() else np.nan)
    row['measured_joint_abs_peak_rad'] = float(np.nanmax(np.abs(df[
        [f'switch_measured_joint_{j}_rad' for j in ('a_l', 'b_l', 'a_r', 'b_r')]])))
    diagnostics.append(row)
pd.DataFrame(diagnostics).to_csv(OUT/'allocator_diagnostics.csv', index=False)

def save(fig, name):
    fig.savefig(OUT/'figures'/f'{name}.png', dpi=180, bbox_inches='tight')
    plt.close(fig)

def curves(selected, name, title, detail=False):
    if not selected:
        return
    fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=True, layout='constrained')
    for r in selected:
        df = load(r)
        if df is None:
            continue
        df = df[df.t >= -.1]
        method = r['method']
        series = [df.ep, df.attitude_error_angle_deg, df.angular_r_frd_rps,
                  df.position_z_rel_m, df.switch_commanded_joint_b_l_rad,
                  df.switch_measured_joint_b_l_rad]
        label = LABELS[method] if r['repeat']=='repeat_1' else '_nolegend_'
        for ax, y in zip(axes.flat, series):
            ax.plot(df.t, y, color=COLORS[method], ls='-' if r['repeat']=='repeat_1' else '--',
                    lw=1.1, label=label)
    for ax, label in zip(axes.flat, ['3D position error (m)', 'SO(3) error (deg)',
                                    'Body yaw rate (rad/s)', 'Altitude (m)',
                                    'Left beta command (rad)', 'Left beta measured (rad)']):
        ax.set_ylabel(label); ax.grid(alpha=.25)
    axes[0,0].legend(fontsize=8, ncol=2)
    if detail:
        axes[0,0].set_ylim(0, .8)
        axes[0,1].set_ylim(0, 12)
        axes[1,0].set_ylim(-.6, .6)
        axes[1,1].set_ylim(1.25, 1.75)
        title += ' | Clipped detail; use full-scale figure for failure excursions'
    for ax in axes[-1]: ax.set_xlabel('Time since tested allocator activation (s)')
    fig.suptitle(title+'\nSolid: repeat 1; dashed: repeat 2. No gate; no pure actuator delay.')
    save(fig, name)

final = [r for r in records if r['cohort']=='comparison_common']
for duration in sorted({r['switch_duration_s'] for r in final}):
    selected = [r for r in final if r['switch_duration_s']==duration]
    curves([r for r in selected if r['method'] in {'paper_nda','full','direct'}],
           f'comparison_T{duration:.1f}', f'Matched comparison | 1.2 m per leg, T={duration:.1f} s')
    curves([r for r in selected if r['method'] in {'paper_nda','full','direct'}],
           f'comparison_T{duration:.1f}_detail', f'Detail comparison | T={duration:.1f} s', detail=True)
    curves([r for r in selected if r['method'] in {'full','no_preview','no_horizon'}],
           f'drcda_ablation_T{duration:.1f}', f'DRCDA ablations | T={duration:.1f} s')

for cohort, name in [('qualification_hover_common','qualification_hover'), ('qualification_low_common','qualification_low'),
                     ('qualification_hover_long', 'qualification_hover_long')]:
    curves([r for r in records if r['cohort']==cohort], name,
           'NDA qualification | common Direct takeoff, NDA throughout evaluated window')
curves([r for r in records if r['cohort'] in {'nda_ablations', 'qualification_hover_common'}], 'nda_ablations',
       'NDA repair ablations | stationary hover after common initialization')

for r in final:
    if r['method'] != 'paper_nda' or r['repeat'] != 'repeat_1':
        continue
    df = load(r)
    df = df[df.t >= 0]
    fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=True, layout='constrained')
    for j in ('a_l', 'b_l', 'a_r', 'b_r'):
        axes[0,0].plot(df.t, df[f'switch_commanded_joint_{j}_rad'], label=j)
        axes[0,1].plot(df.t, df[f'switch_measured_joint_{j}_rad'], label=j)
    axes[1,0].plot(df.t, df.nda_saturation_scale)
    axes[1,1].semilogy(df.t, df.nda_normalized_condition)
    for suffix in ('left_1', 'left_2', 'right_1', 'right_2', 'tail'):
        axes[2,0].plot(df.t, df[f'drcda_q_f_{suffix}_n'], label=suffix)
    axes[2,1].plot(df.t, df.attitude_error_angle_deg, label='SO(3)')
    for ax, label in zip(axes.flat, ['Joint commands (rad)', 'Measured joints (rad)',
                                    'Uniform saturation scale', 'Normalized Jacobian condition',
                                    'Model-estimated rotor force (N)', 'SO(3) error (deg)']):
        ax.set_ylabel(label); ax.grid(alpha=.25)
    for ax in [axes[0,0], axes[0,1], axes[2,0]]:
        ax.legend(fontsize=8, ncol=2)
    for ax in axes[-1]: ax.set_xlabel('Time since NDA activation (s)')
    fig.suptitle(f'NDA diagnostics | T={r["switch_duration_s"]:.1f} s | repeat 1\n'
                 'Rotor forces are model estimates, not measured thrust')
    save(fig, f'nda_diagnostics_T{r["switch_duration_s"]:.1f}')

def table(selected):
    lines=['| 方法 | 通过 | 位置峰值/m | SO(3)峰值/deg | 偏航角速度峰值/(rad/s) |',
           '|---|---:|---:|---:|---:|']
    for method in LABELS:
        rows=[r for r in selected if r['method']==method]
        if not rows: continue
        cells=[]
        for key in ['position_error_peak_m','attitude_error_peak_deg','yaw_rate_peak_rad_s']:
            values=[r.get('metrics',{}).get(key, np.nan) for r in rows]
            cells.append(f'{np.nanmin(values):.3f}–{np.nanmax(values):.3f}')
        lines.append(f'| {LABELS[method]} | {sum(r["status"]=="complete" for r in rows)}/{len(rows)} | '+' | '.join(cells)+' |')
    return '\n'.join(lines)

sections=[]
for cohort in ['qualification_hover_common', 'qualification_low_common', 'qualification_hover_long', 'comparison_common', 'nda_ablations']:
    selected=[r for r in records if r['cohort']==cohort]
    if cohort == 'nda_ablations' and selected:
        selected += [r for r in records if r['cohort']=='qualification_hover_common']
    if not selected: continue
    for duration in sorted({r['switch_duration_s'] for r in selected}):
        sections.append(f'### {cohort}, T={duration:.1f} s\n\n'+table([r for r in selected if r['switch_duration_s']==duration]))
(OUT/'RESULTS.md').write_text('\n\n'.join(sections)+'\n')
print(f'Archived {len(records)} actual runs; {len(final)} matched comparison runs')
print('\n\n'.join(sections))
