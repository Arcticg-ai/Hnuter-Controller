"""Offline relative drift diagnosis; never resets or corrects the estimator."""
from pathlib import Path
import json,numpy as np,pandas as pd
R=Path(__file__).resolve().parent
out=[]
for name in json.loads((R/'runtime/MATRIX.json').read_text())['order']:
    p=R/'runtime/results/experiment/runs'/name;ep=json.loads((p/'diagnostic_epoch.json').read_text())['epoch'];tr=pd.read_csv(p/'trajectory.csv');t=tr.px4_time_s.to_numpy()-ep;estimated=tr[['x','y','z_up']].to_numpy();error=estimated-tr[['x_ref','y_ref','z_ref']].to_numpy();truth=pd.read_csv(p/'truth_diagnostic.csv',usecols=['t','stage','base_link_px','base_link_py','base_link_pz']);truth=truth[truth.stage==1];gt=truth.t.to_numpy()-ep;xyz=truth[['base_link_px','base_link_py','base_link_pz']].to_numpy();pre=(t>=45)&(t<48);end=(t>=71)&(t<72);gp=(gt>=45)&(gt<48);ge=(gt>=71)&(gt<72);dp=estimated[end].mean(0)-estimated[pre].mean(0);dg=xyz[ge].mean(0)-xyz[gp].mean(0);out.append(dict(run=name,pre_mean_estimated_error_enu_m=error[pre].mean(0).tolist(),final_mean_estimated_error_enu_m=error[end].mean(0).tolist(),estimated_position_change_m=dp.tolist(),physical_base_position_change_m=dg.tolist(),estimated_minus_physical_change_m=(dp-dg).tolist(),interpretation='Relative pre-to-final drift in established ENU convention; not absolute physical tracking error and not a unique causal diagnosis. No truth correction used online.'))
(R/'results/recovery_diagnosis.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
