from pathlib import Path
import pandas as pd,numpy as np

def audit(run,seed):
 p=Path(run);d=pd.read_csv(p/'gps_noise_audit.csv');delay=d.receive_us.to_numpy()*1000-d.source_ns.to_numpy()
 checks={'audit_close_ack':'JEPA GPS audit closed' in (p/'px4.log').read_text(),'complete_last_row':(p/'gps_noise_audit.csv').read_bytes().endswith(b'\n'),'finite_all_fields':bool(np.isfinite(d.to_numpy()).all()),'nonempty':len(d)>100,'seed_recorded':bool((d.seed==seed).all()),'six_normals_per_callback':bool((d.normal_draws==6*(d.callback_index-1)).all()),'contiguous_callback_index':bool((np.diff(d.callback_index)==1).all()),'strictly_increasing_source_ns':bool((np.diff(d.source_ns)>0).all()),'finite_noise':bool(np.isfinite(d[['pos_n','pos_e','pos_d','vel_n','vel_e','vel_d']]).all().all()),'new_rng_log':f'JEPA GPS private MT19937 seed={seed}' in (p/'px4.log').read_text()}
 return {'passed':all(checks.values()),'checks':checks,'callbacks_logged':len(d),'first_source_ns':int(d.source_ns.iloc[0]),'last_source_ns':int(d.source_ns.iloc[-1]),'receive_minus_source_ms':{'min':float(delay.min()/1e6),'median':float(np.median(delay)/1e6),'max':float(delay.max()/1e6),'negative_count':int((delay<0).sum())},'note':'Source and receipt are distinct; delay statistics are diagnostic, no timestamp rewriting.'}
