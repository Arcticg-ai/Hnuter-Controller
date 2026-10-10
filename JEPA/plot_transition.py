"""Offline transition evidence from recorded ULog and estimator windows."""
import argparse
import csv
import json
import os
from pathlib import Path
import numpy as np
from pyulog import ULog


def main():
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);args=parser.parse_args();run=args.run
    out=Path(__file__).resolve().parent/'results';out.mkdir(exist_ok=True)
    os.environ['MPLCONFIGDIR']=str(out/'mpl_cache')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    with (run/'trajectory.csv').open() as f:rows=list(csv.DictReader(f))
    with (run/'mass_windows.csv').open() as f:windows=list(csv.DictReader(f))
    log=next((run/'rootfs/log').rglob('*.ulg'));data=ULog(str(log)).get_dataset('hnuter_control_status').data
    t0=float(rows[0]['px4_time_s']);t=data['timestamp']*1e-6-t0
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
    axes[0].plot([float(r['t'])-t0 for r in windows],[float(r['mass_kg']) for r in windows],label='Independent estimate')
    axes[0].plot(t,data['effective_mass'],label='Firmware effective mass')
    truth=json.loads((run/'metrics.json').read_text())['truth_mass_kg_evaluation_only']
    axes[0].axhline(truth,linestyle='--',color='gray',label='Truth (evaluation only)');axes[0].set_ylabel('Mass (kg)');axes[0].legend()
    shown=(t>=16)&(t<=57)
    axes[1].plot(t[shown],-data['force_body[2]'][shown],linewidth=.6,label='Desired body upward force')
    axes[1].set_ylabel('Force (N)');axes[1].legend()
    shown_rows=[r for r in rows if 16<=float(r['t'])<=57]
    axes[2].plot([float(r['t']) for r in shown_rows],[float(r['error'])*100 for r in shown_rows],linewidth=.8)
    axes[2].set_ylabel('Position error (cm)');axes[2].set_xlabel('Time after arming (s)')
    for ax in axes:
        ax.axvspan(18,34,alpha=.08,color='blue');ax.axvspan(35,38.1,alpha=.1,color='orange');ax.grid(alpha=.25);ax.set_xlim(16,57)
    fig.suptitle(f'Actual SITL: {run.name}\nBlue: calibration; orange: approximate mass transition; evaluation: 40–56 s')
    fig.tight_layout();fig.savefig(out/'transition.png',dpi=160)
    (out/'transition_source.json').write_text(json.dumps(dict(run=str(run),ulog=str(log),note='Representative preselected noisy C seed 0; all trials are reported separately.'),indent=2)+'\n')


if __name__=='__main__':main()
