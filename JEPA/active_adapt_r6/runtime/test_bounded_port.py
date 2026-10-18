import unittest,numpy as np
from bounded_port import SlewLimit
from causal_clock import LatestCausalClock
from residual_port import apply_residual,force_profile
class Port(unittest.TestCase):
 def test_zero_bits(self):
  q=SlewLimit()
  for i in range(100):
   b,m=q.step(np.zeros(3),True,i*.004);a=np.array([0.,-0.,.012]);f,_,_=apply_residual(a,b,m['valid']);self.assertTrue(np.array_equal(a.view(np.uint64),f.view(np.uint64)))
 def test_step_slew(self):
  q=SlewLimit();last=np.zeros(3)
  for i in range(100):
   b,m=q.step(np.array([.3,0,0]),True,i*.004);self.assertLessEqual(np.linalg.norm(b-last),.0008000001);self.assertLessEqual(np.linalg.norm(b),.300000001);last=b
 def test_failure_zero(self):
  for r,f,t in [([float('nan'),0,0],True,.004),([.4,0,0],True,.004),([.1,0,0],False,.004),([.1,0,0],True,-.1),([.1,0,0],True,.1)]:
   q=SlewLimit();q.step([0,0,0],True,0);b,m=q.step(r,f,t);self.assertFalse(m['valid']);self.assertFalse(np.any(b))
 def test_profile_unaffected(self):
  q=SlewLimit();s=dict(start=50.,end=58.,force_y=.6);errors=[]
  for t in np.arange(0,72,.004):
   r=np.array([0.,-force_profile(t,s)/5.39999998,0.]);b,m=q.step(r,True,t);errors.append(np.max(abs(b-r)));self.assertTrue(m['valid'])
  self.assertLess(max(errors),1e-12)
 def test_same_time_hold(self):
  q=SlewLimit();q.step([0,0,0],True,1);b,m=q.step([.2,0,0],True,1);self.assertFalse(np.any(b));self.assertTrue(m['valid'])
 def test_diagnostic_units(self):
  r=np.array([0.,-.6/5.4,0.]);T=np.array([[0,1,0],[1,0,0],[0,0,-1]])
  self.assertTrue(np.allclose(T@r,[-.6/5.4,0,0]));self.assertTrue(np.allclose(5.4*r+[0,.6,0],0))
 def test_actual_publisher_land_zero(self):
  import ast
  from pathlib import Path
  tree=ast.parse((Path(__file__).parent/'r20_probe.py').read_text());method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='publish_pending_nominal')
  class Base:
   def publish_px4_trajectory_setpoint(self):return self.target_acceleration.copy()
  cls=ast.ClassDef(name='P',bases=[ast.Name(id='Base',ctx=ast.Load())],keywords=[],body=[method],decorator_list=[]);mod=ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[]));ns=dict(Base=Base,np=np,apply_residual=apply_residual,force_profile=force_profile);exec(compile(mod,'actual_publisher','exec'),ns)
  n=ns['P']();n.port_clock=LatestCausalClock();n.receipts={};n.target_acceleration=np.array([0.,-0.,0.]);n.r6_mode='baseline';n.r12_arm='11';n.phase='LAND';n.force_schedule=dict(start=50,end=58,force_y=.6,mass_kg=5.4);n.px4_timestamp=51004000;n.residual_limiter=SlewLimit();n.residual_limiter.last=51.;n.residual_limiter.value=np.array([0.,-.1,0.]);n.zero_checks=0;n.command_sequence=0
  out=n.publish_pending_nominal();self.assertFalse(np.any(out));self.assertFalse(n.port_record['residual_valid']);self.assertTrue(n.port_record['limiter']['emergency_zero'])
 def test_received_max_handles_skew_without_extrapolation(self):
  c=LatestCausalClock();q=SlewLimit();seen=[]
  for row in [[1000000,1004000],[1001000],[1008000,1004000],[1008000]]:
   seen+=row;t=c.update(row);self.assertEqual(t,max(seen));b,m=q.step([0,0,0],True,t*1e-6);self.assertTrue(m['valid'])
 def test_received_clock_invalid_rejected(self):
  c=LatestCausalClock()
  for row in [[],[float('nan')],[-1],[True],[1.5]]:
   with self.assertRaises(ValueError):c.update(row)
if __name__=='__main__':unittest.main()
