"""Explicit, append-only batches. Every launch attempt retained, including prearm failure."""
import argparse, concurrent.futures, datetime, json, signal, subprocess, sys, threading
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser();p.add_argument('--batch',choices=['targeted','ablation','readiness','matrix','recovery'],required=True)
    p.add_argument('--workers',type=int,default=3);p.add_argument('--discovery',choices=['shared','isolated'],default='shared')
    p.add_argument('--attitude-reference',choices=['legacy','level'],default='level')
    a=p.parse_args();assert 1<=a.workers<=3
    if a.batch=='targeted':jobs=[('A',3),('B',1),('C',3),('D',2),('B',4),('C',4),('C_shadow',3)]
    elif a.batch=='ablation':jobs=[('B',4),('C',4),('C_shadow',3)]
    elif a.batch=='readiness':jobs=[('A',s) for s in [3,1,3,2,3,1,3,2,3,1,3,2]]
    elif a.batch=='recovery':jobs=[('A',0),('C',0)]
    else:jobs=[(g,s) for s in range(5) for g in ['A','B','C','D','C_shadow']]
    out=ROOT/'runs'/('batch_'+a.batch+'_'+a.discovery+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True)
    (out/'plan.json').write_text(json.dumps(dict(options=vars(a),jobs=jobs),indent=2))
    results=[];lock=threading.Lock()
    def worker(instance):
        for index in range(instance,len(jobs),a.workers):
            g,s=jobs[index];label=f'{a.batch}_{index}_{g}_s{s}'
            cmd=[sys.executable,str(ROOT/'run_sitl.py'),'--label',label,'--loaded','--instrument','--smooth-takeoff','--group',g,'--noise','noisy_delayed','--seed',str(s),'--instance',str(instance),'--discovery',a.discovery,'--attitude-reference',a.attitude_reference]
            if a.batch=='ablation':cmd+=['--attitude-reference','legacy']
            if a.batch=='readiness':cmd+=['--startup-only']
            if a.batch=='recovery':cmd+=['--inject-exit-at',str(36 if g=='C' else 7)]
            with (out/(label+'.log')).open('w') as f:
                proc=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT)
                try:rc=proc.wait(timeout=240)
                except subprocess.TimeoutExpired:
                    proc.send_signal(signal.SIGINT);rc=proc.wait(timeout=25)
            output=(out/(label+'.log')).read_text();run=next((Path(x[8:]) for x in output.splitlines() if x.startswith('STARTED ')),None)
            r=dict(index=index,group=g,seed=s,instance=instance,returncode=rc,command=cmd,run=str(run) if run else None)
            if run and (run/'flight_result.json').exists():
                r['flight']=json.loads((run/'flight_result.json').read_text())
                if (run/'trajectory.csv').exists():
                    with (run/'analysis.log').open('w') as f:r['analysis_returncode']=subprocess.run([sys.executable,str(ROOT/'analyze_sitl.py'),str(run),'--truth-mass','5.39999998'],stdout=f,stderr=subprocess.STDOUT).returncode
            with lock:
                results.append(r);(out/'results.json').write_text(json.dumps(sorted(results,key=lambda x:x['index']),indent=2))
                print(json.dumps(r),flush=True)
    print('BATCH_STARTED',out,flush=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=a.workers) as pool:list(pool.map(worker,range(a.workers)))
    (ROOT/('latest_'+a.batch+'_'+a.discovery+'.txt')).write_text(str(out)+'\n')
    print('BATCH_COMPLETE',out,flush=True)
if __name__=='__main__':main()
