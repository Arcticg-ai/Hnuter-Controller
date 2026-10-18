import unittest,ast,math,json,hashlib
from pathlib import Path
import numpy as np
from excitation import reference
R=Path(__file__).resolve().parent;OLD=R.parent.parent/'control_benefit_r2'
class Collection(unittest.TestCase):
 def test_reference_bound_smooth_zero_and_yaw_derivative(self):
  for t in [0,12,45,47.999,48,68,70,72]:self.assertFalse(np.any(reference(t)[0]));self.assertEqual(reference(t)[1],0.)
  ts=np.arange(48,68,.004);a=np.array([reference(t)[0] for t in ts]);self.assertLessEqual(abs(a).max(),math.radians(1.5));self.assertLess(abs(a[1]).max(),1e-7);self.assertLess(abs(a[-1]).max(),1e-7)
  for t in [49.,53.,57.,64.]:self.assertAlmostEqual(reference(t)[1],(reference(t+1e-5)[0][2]-reference(t-1e-5)[0][2])/2e-5,places=8)
 def test_only_declared_trajectory_insert_and_logger_changes(self):
  old=(OLD/'r20_probe.py').read_text();new=(R/'r20_probe.py').read_text();insert="""# R4 baseline-only, smooth attitude reference after the unchanged initialization interval.
        if elapsed>=48:
            angles,yawrate=reference(elapsed)
            self.roll_ref=float(angles[0]);self.pitch_ref=float(angles[1]);self.manual_des_yaw=self.initial_yaw+float(angles[2]);self.target_attitude_rate[2]=yawrate"""
  new=new.replace(insert,'# R20 no intervention: fixed hover throughout; no 2cm reference excursion.').replace('self.rows[-1].update(roll_ref=self.roll_ref,pitch_ref=self.pitch_ref)','self.rows[-1].update(roll_ref=0.,pitch_ref=0.)')
  get=lambda s,n:ast.dump(next(x for x in ast.walk(ast.parse(s)) if isinstance(x,ast.FunctionDef) and x.name==n))
  for n in ['update_trajectory','watchdog','publish_pending_nominal','publish_px4_trajectory_setpoint','on_control','observe_source']:self.assertEqual(get(old,n),get(new,n),n)
 def test_truth_recorder_only_decimates_reads(self):
  original=Path('/home/hnuter/px4_ws_ros2/JEPA/revision3/articulated/TruthRecorder.cc').read_text();new=(R/'TruthRecorder.cc').read_text();self.assertEqual(new.replace(' || info.iterations%10!=0',''),original)
 def test_models_have_separate_groups_and_no_force(self):
  cases=json.loads((R/'results/experiment/cases.json').read_text());self.assertEqual(len(cases),5);self.assertEqual(len({c['model_sha256'] for c in cases}),5);self.assertEqual(sum(c['split']=='validation' for c in cases),1)
  for c in cases:
   self.assertEqual(c['disturbance']['force_y'],0);J=np.array(c['truth']['J_com_model']);self.assertTrue(np.all(np.linalg.eigvalsh(J)>0))
  self.assertNotEqual(cases[2]['truth']['com_model'],cases[0]['truth']['com_model']);self.assertNotEqual(cases[3]['truth']['J_com_model'],cases[0]['truth']['J_com_model'])
 def test_no_truth_as_reference_input(self):
  src=(R/'excitation.py').read_text();self.assertNotIn('seed',src);self.assertNotIn('mass',src);self.assertNotIn('truth',src)
if __name__=='__main__':unittest.main()
