import sys
import unittest
from pathlib import Path
import numpy as np
import json
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from assembly import Assembly,SOURCE
from allocation_contract import forward,jacobian
from control_contract import TrackingTransition
from spatial_model import discrete_window,window
from spatial_estimator import propose
from nominal_dynamics import NominalDynamics
from actions import reference


class ConsumerTests(unittest.TestCase):
    def test_serialized_nominal_calibration_matches_known_assembly(self):
        root=Path(__file__).resolve().parent
        n=NominalDynamics(json.loads((root/'nominal_calibration.json').read_text()))
        a=Assembly(SOURCE);rng=np.random.default_rng(7)
        for _ in range(8):
            q={name:(rng.normal(0,.2),rng.normal(0,2)) for name in n.geometry['joints']}
            for key,value in n.state(q).items():np.testing.assert_allclose(value,a.state(q)[key],atol=1e-12)

    def test_shadow_excitation_derivative_and_moment_bounds(self):
        n=Assembly(SOURCE);j=n.state({})['j'];samples=[]
        for t in np.arange(0,16.001,.004):
            samples.append([reference(t,i,amplitude=np.deg2rad(.75),duration=16,frequency=f) for i,f in enumerate((.4,.5,.6))])
        x=np.array(samples)
        self.assertLess(np.max(np.linalg.norm(x[:,:,0],axis=1)),np.deg2rad(2))
        self.assertLess(np.max(np.linalg.norm(x[:,:,1],axis=1)),.15)
        self.assertLess(np.max(np.linalg.norm(x[:,:,2],axis=1)),.6)
        self.assertLess(np.max(np.linalg.norm(x[:,:,2]@j.T,axis=1)),.15)

    def test_allocator_scale_from_geometry_not_empirical_flight_fit(self):
        n=Assembly(SOURCE);f=np.array([0.,0.,-5.4*9.81]);t=np.array([0.,-.105*5.4*9.81,0.])
        g=jacobian(n,f,t)
        compiled=np.array([[1.24212546,-.000126503,0],[-.000280509,.998578762,-.000023648],[-.009441720,-.031515954,1.241330617]])
        np.testing.assert_allclose(g,compiled,atol=2e-8)
        # Primary force levers are ~0.410 m, normalized using HNTR_L1=0.330 m.
        self.assertAlmostEqual(g[0,0],.410/.330,delta=.001)
        c=n.state({})['c'];df=np.array([.1,-.03,.04])
        f0,t0=forward(n.geometry,f,t);f1,t1=forward(n.geometry,f,t+np.linalg.solve(g,df))
        np.testing.assert_allclose((t1-t0)-np.cross(c,f1-f0),df,atol=2e-5)

    def test_incremental_oracle_preserves_nominal_dynamics_and_zero_case(self):
        n=Assembly(SOURCE);j=n.state({})['j'];g=jacobian(n,np.array([0.,0.,-52.974]),np.array([0.,-5.56227,0.]))
        pd=np.array([.1,-.2,.04]);f=np.array([.1,-.1,-52.9]);w=np.array([.05,-.03,.01]);dr=np.array([0,-.006666667,0]);dj=np.diag([.0018,.00087,.00243])
        alpha=np.linalg.solve(j,g@pd)
        delta=np.linalg.solve(g,np.cross(dr,f)+dj@alpha+np.cross(w,dj@w))
        # Gyroscopic nominal term is common to the before/after comparison.
        actual=np.linalg.solve(j+dj,g@(pd+delta)-np.cross(dr,f)-np.cross(w,dj@w))
        np.testing.assert_allclose(actual,alpha,atol=1e-12)
        np.testing.assert_array_equal(np.linalg.solve(g,np.zeros(3)),np.zeros(3))

    def test_zero_integral_bumpless_then_rate_limited_tracking(self):
        tr=TrackingTransition();old=np.zeros(3);new=np.array([.28,.01,-.04])
        out,ok=tr.step(old,new,np.array([.1,-5.5,0]),.004,np.array([.9,8,1.8]))
        self.assertTrue(ok);np.testing.assert_allclose(out,old,atol=1e-15)
        out2,ok=tr.step(new,new,np.array([.1,-5.5,0]),.004,np.array([.9,8,1.8]))
        self.assertTrue(ok);self.assertLessEqual(np.max(abs(out2-out)),.02*.004+1e-15)
        tr.freeze();_,ok=tr.step(new,new,np.zeros(3),.004,np.ones(3));self.assertFalse(ok)

    def test_independent_bias_gate_cannot_be_replaced_by_rank(self):
        rng=np.random.default_rng(5);a=rng.normal(size=(30,3,8));prior=np.array([-.54,0,.25,.2,.35,0,0,0]);b=a@prior
        self.assertEqual(propose(a,b,prior,5.4,.02)['reason'],'unverified_constant_torque_bias')
        result=propose(a,b,prior,5.4,.02,torque_bias_bound_nm=0.)
        self.assertTrue(result['accepted']);np.testing.assert_allclose(result['theta'],prior,atol=1e-12)


if __name__=='__main__':unittest.main(verbosity=2)
