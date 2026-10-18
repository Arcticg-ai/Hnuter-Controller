"""Serial continuation after first audited flight; stops on any real admission failure."""
from pathlib import Path
import json,subprocess,time
R=Path(__file__).resolve().parent;runtime=R/'runtime'
def main():
    order=json.loads((runtime/'MATRIX.json').read_text())['order'];assert json.loads((runtime/'results'/f'{order[0]}_outcome.json').read_text())['continue_batch'];completed=[order[0]];start=time.monotonic()
    for name in order[1:]:
        (R/'results/campaign_status.json').write_text(json.dumps(dict(stage='running',run=name,completed=completed),indent=2))
        for cmd,label,timeout in [(['python3',str(runtime/'run_bounded.py'),name],'run',210),(['python3',str(runtime/'post_run.py'),name],'audit',90)]:
            with (runtime/'results'/f'{name}_{label}_supervisor.log').open('x') as log:q=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,timeout=timeout)
            if q.returncode:
                (R/'results/campaign_status.json').write_text(json.dumps(dict(stage='stopped',run=name,failed_step=label,completed=completed,returncode=q.returncode,wall_s=time.monotonic()-start),indent=2));raise SystemExit(q.returncode)
        assert json.loads((runtime/'results'/f'{name}_outcome.json').read_text())['continue_batch'];completed.append(name);print(name,'safety/data PASS',flush=True)
    (R/'results/campaign_status.json').write_text(json.dumps(dict(stage='completed',completed=completed,wall_s=time.monotonic()-start),indent=2))
if __name__=='__main__':main()
