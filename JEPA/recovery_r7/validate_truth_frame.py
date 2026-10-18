from pathlib import Path
import json,numpy as np,pandas as pd
from pyulog import ULog
from audit_r6 import project
R=Path(__file__).resolve().parent;out=[]
for p in (R/'runtime/results/experiment/runs').iterdir():
 if not (p/'flight_result.json').exists():continue
 u=ULog(str(next((p/'rootfs/log').rglob('*.ulg'))),message_name_filter_list=['vehicle_local_position']);lp=u.get_dataset('vehicle_local_position').data;g=pd.read_csv(p/'gps_noise_audit.csv');t=g.source_ns.to_numpy()*1e-9;ids=np.maximum(np.searchsorted(lp['timestamp_sample']*1e-6,t,side='right')-1,0);geo=np.c_[project(g.lat_in.to_numpy(),g.lon_in.to_numpy(),lp['ref_lat'][ids],lp['ref_lon'][ids]),g.alt_in.to_numpy()-lp['ref_alt'][ids]];truth=pd.read_csv(p/'truth_diagnostic.csv',usecols=['t','stage','base_link_px','base_link_py','base_link_pz']);truth=truth[truth.stage==1];xyz=np.c_[[np.interp(t,truth.t,truth[k]) for k in ['base_link_px','base_link_py','base_link_pz']]].T;ep=json.loads((p/'diagnostic_epoch.json').read_text())['epoch'];mask=(t>=ep+34)&(t<ep+92);delta=geo[mask]-xyz[mask];delta-=delta.mean(0);out.append(dict(run=p.name,projection_vs_gazebo_motion_centered_rmse_m=float(np.sqrt(np.mean(np.sum(delta**2,1)))),max_m=float(np.linalg.norm(delta,axis=1).max())))
(R/'results/truth_frame_validation.json').write_text(json.dumps(out,indent=2));assert max(x['max_m'] for x in out)<.001;print(out)
