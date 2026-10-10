"""Run matched trials on owned, isolated PX4/DDS/Gazebo instances at 1x only."""
import argparse
import concurrent.futures
import datetime
import json
from pathlib import Path
import signal
import subprocess
import sys
from zoneinfo import ZoneInfo

ROOT=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser();p.add_argument('--phase',choices=['low','noisy'],required=True)
    p.add_argument('--workers',type=int,default=3);args=p.parse_args()
    if not 1<=args.workers<=3:p.error('workers must be 1..3')
    groups=['A','B','C','D','C_shadow']
    seeds=[0] if args.phase=='low' else list(range(5))
    jobs=[(group,seed) for seed in seeds for group in groups]
    stamp=datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S')
    out=ROOT/'runs'/f'matrix_{args.phase}_{stamp}';out.mkdir()
    results=[]
    def worker(instance):
        local=[]
        for group,seed in jobs[instance::args.workers]:
            label=f'{args.phase}_{group}_s{seed}'
            cmd=[sys.executable,str(ROOT/'run_sitl.py'),'--label',label,'--loaded','--instrument','--smooth-takeoff',
                 '--group',group,'--noise','low_noise' if args.phase=='low' else 'noisy_delayed','--seed',str(seed),'--instance',str(instance)]
            proc=subprocess.Popen(cmd,cwd=ROOT.parent,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            try:stdout,_=proc.communicate(timeout=240)
            except subprocess.TimeoutExpired:
                proc.send_signal(signal.SIGINT);stdout,_=proc.communicate(timeout=25)
            (out/(label+'.log')).write_text(stdout)
            run=None
            for line in stdout.splitlines():
                if line.startswith('STARTED '):run=Path(line.removeprefix('STARTED '))
            result=dict(group=group,seed=seed,instance=instance,returncode=proc.returncode,run=str(run) if run else None)
            if run and (run/'flight_result.json').exists():
                result['flight']=json.loads((run/'flight_result.json').read_text())
                if (run/'trajectory.csv').exists():
                    analysis=subprocess.run([sys.executable,str(ROOT/'analyze_sitl.py'),str(run),'--truth-mass','5.39999998'],capture_output=True,text=True)
                    (run/'analysis.log').write_text(analysis.stdout+analysis.stderr)
                    result['analysis_returncode']=analysis.returncode
            local.append(result)
            print(json.dumps(result),flush=True)
        return local
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures=[pool.submit(worker,i) for i in range(args.workers)]
        for future in concurrent.futures.as_completed(futures):
            results.extend(future.result())
            (out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    (ROOT/f'latest_matrix_{args.phase}.txt').write_text(str(out)+'\n')
    print('MATRIX_COMPLETE',out,flush=True)


if __name__=='__main__':main()
