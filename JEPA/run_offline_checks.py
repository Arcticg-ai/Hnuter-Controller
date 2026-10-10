"""Reproduce preparatory checks and archive outputs; never starts a flight."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parent


def main():
    stamp = datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S')
    out = ROOT/'runs'/('offline_'+stamp)
    out.mkdir(parents=True,exist_ok=False)
    commands = [
        ('preflight',[sys.executable,str(ROOT/'preflight.py')]),
        ('model_audit',[sys.executable,str(ROOT/'model_audit.py')]),
        ('unit_tests',[sys.executable,'-m','unittest','discover','-s',str(ROOT),'-p','test_offline.py','-v']),
        ('sdf_native',['gz','sdf','-k',str(ROOT/'generated/native_noise/hnuter_mass_cal_payload/model.sdf')]),
        ('sdf_zero',['gz','sdf','-k',str(ROOT/'generated/zero_sensor_noise/hnuter_mass_cal_payload/model.sdf')]),
    ]
    results = {}
    for name, cmd in commands:
        r = subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,timeout=45)
        (out/(name+'.log')).write_text(r.stdout+r.stderr)
        results[name] = dict(argv=cmd,returncode=r.returncode,log=str(out/(name+'.log')))
    os.environ['MPLCONFIGDIR'] = str(out/'mpl_cache')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    from estimator import vertical_reference
    t = np.linspace(0,16,1601)
    reference = np.array([vertical_reference(x) for x in t])
    fig, axes = plt.subplots(3,1,figsize=(9,7),sharex=True)
    for i,label in enumerate(['z offset (m), NED','v_z (m/s), NED','a_z (m/s^2), NED']):
        axes[i].plot(t,reference[:,i]); axes[i].set_ylabel(label); axes[i].grid(alpha=.3)
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('Planned calibration reference — analytic only, NOT flight data')
    fig.tight_layout()
    fig.savefig(out/'planned_reference.png',dpi=150)
    plt.close(fig)
    # Persist source manifest for this exact offline run.
    manifest = {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in ROOT.glob('*.py')}
    summary = dict(evidence='OFFLINE_ONLY',sitl_flights=0,commands=results,source_sha256=manifest,
                   status='BLOCKED_SITL' if results['preflight']['returncode']==2 else 'OFFLINE_CHECKS_ONLY',
                   offline_checks_passed=all(v['returncode']==0 for k,v in results.items() if k!='preflight'))
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    (ROOT/'latest_offline_run.txt').write_text(str(out)+'\n')
    print(json.dumps(summary,indent=2))
    if not summary['offline_checks_passed']:
        return 1
    return 2 if summary['status']=='BLOCKED_SITL' else 0


if __name__ == '__main__':
    raise SystemExit(main())
