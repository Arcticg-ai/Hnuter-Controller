import unittest,numpy as np
from gated_adapter import GatedAdapter
class GateBehavior(unittest.TestCase):
 def drive(self,mode):
  a=GatedAdapter(np.ones(6));W=np.arange(24).reshape(6,4)*.002
  for k in range(100):a.update(np.zeros(4),np.zeros(6),34+k*.1)
  for k in range(80):
   phi=np.array([np.sin(2*np.pi*k/8),np.cos(2*np.pi*k/8),np.sin(2*np.pi*k/10),np.cos(2*np.pi*k/10)]);error=W@phi
   if mode=='reversing' and k>=40:error=-error
   a.update(phi,error,48+k*.1)
  return a
 def test_consistent_effect_can_update(self):
  a=self.drive('parameter');self.assertGreater(a.updates,0);self.assertGreater(np.linalg.norm(a.W),0)
 def test_reversing_effect_keeps_slow_coefficients(self):
  a=self.drive('reversing');self.assertEqual(a.updates,0);self.assertEqual(np.linalg.norm(a.W),0)
 def test_after_id_frozen_and_no_target_required_for_prediction(self):
  a=self.drive('parameter');before=a.W.copy();pred=a.predict(np.ones(4));a.update(np.ones(4),np.ones(6)*100,59,True);np.testing.assert_array_equal(a.W,before);np.testing.assert_array_equal(a.predict(np.ones(4)),pred)
 def test_correlated_disturbance_indistinguishable(self):
  # Same visible X and residual: even a consistency gate must make same decision.
  parameter=self.drive('parameter');disturbance=self.drive('disturbance');np.testing.assert_array_equal(parameter.W,disturbance.W);self.assertGreater(disturbance.updates,0)
if __name__=='__main__':unittest.main()
