"""Adapter falsification on recorded inputs; synthetic label pulses are NOT wind SITL."""
from pathlib import Path
import json,numpy as np
from adapter import Adapter
R=Path(__file__).resolve().parent
def main():
    rows=[];scale=json.loads((R/'results/models/normalization.json').read_text())['adapter_scale']
    for folder in sorted((R/'runtime/results/experiment/runs').iterdir()):
        if not (folder/'online_summary.json').exists():continue
        logs=[json.loads(s) for s in (folder/'online_predictions.jsonl').read_text().splitlines()];end={x['t_us']:x['elapsed'] for x in map(json.loads,(folder/'aligned_states.jsonl').read_text().splitlines())};final=json.loads((folder/'online_summary.json').read_text())
        for name in final['adapters']:
            records=[x for x in logs if x['method']==name];clean=Adapter(scale);pulse=Adapter(scale)
            for x in records:
                elapsed=end[x['end_us']];phi=np.array(x['phi']);error=(np.array(x['target'])-x['pred_off'])/.1+np.array(x['bias_before']);perturb=np.zeros(6)
                if 51<elapsed<53:perturb[0]=.3*np.sin(np.pi*(elapsed-51)/2)**2
                clean.update(phi,error,elapsed,x['valid']);pulse.update(phi,error+perturb,elapsed,x['valid'])
            parity=float(abs(clean.W-np.array(final['adapters'][name]['W'])).max());assert parity<1e-8
            changes=[np.linalg.norm(pulse.predict(np.array(x['phi']))[:3]-clean.predict(np.array(x['phi']))[:3]) for x in records if 58<=x['elapsed']<68];rows.append(dict(run=folder.name,method=name,replay_W_max_difference=parity,pulse_peak_rate_error_m_s2=.3,pulse_window_s=[51,53],persistent_W_difference=float(np.linalg.norm(pulse.W-clean.W)),independent_task_rate_correction_rms_difference=float(np.sqrt(np.mean(np.array(changes)**2))),synthetic_label_perturbation_not_physical_wind=True))
    result=dict(rows=rows,interpretation='A short bounded residual during identification can alter frozen coefficients. Centering only removes constant offsets; disturbance/parameter separation remains unqualified. This is a software sensitivity test, not a Gazebo wind experiment.',control_release=False);(R/'results/software_stress.json').write_text(json.dumps(result,indent=2));print('adapter replay max diff',max(x['replay_W_max_difference'] for x in rows),'pulse persistent W max',max(x['persistent_W_difference'] for x in rows))
if __name__=='__main__':main()
