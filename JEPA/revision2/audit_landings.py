"""Offline ground-truth check independent of landed/disarmed messages."""
from pathlib import Path
import json,csv,numpy as np
from pyulog import ULog
R=Path(__file__).resolve().parent
out=[]
for p in (R/'runs').glob('sitl_*'):
 f=p/'flight_result.json'
 if not f.exists():continue
 r=json.loads(f.read_text())
 if not r.get('samples'):continue
 u=ULog(str(next((p/'rootfs/log').rglob('*.ulg'))));g=u.get_dataset('vehicle_local_position_groundtruth').data
 tr=list(csv.DictReader((p/'trajectory.csv').open()));start=float(tr[0]['px4_time_s']);tt=g['timestamp']*1e-6
 h=float(-g['z'][-1]+np.interp(start,tt,g['z']));v=float(g['vz'][-1]);out.append(dict(run=p.name,status=r['status'],landed=r['landed'],disarmed=r['disarmed'],final_true_height_relative_arm_m=h,final_true_vertical_speed_m_s=v))
(R/'results/landing_truth_audit.json').write_text(json.dumps(out,indent=2))
yes=[x for x in out if x['landed'] and x['disarmed']];no=[x for x in out if not (x['landed'] and x['disarmed'])]
print(json.dumps(dict(flight_attempts=len(out),confirmed=len(yes),unconfirmed=no,max_confirmed_final_height_m=max(abs(x['final_true_height_relative_arm_m']) for x in yes),max_confirmed_final_speed_m_s=max(abs(x['final_true_vertical_speed_m_s']) for x in yes)),indent=2))
