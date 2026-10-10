import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import unittest
import numpy as np
from assembly import Assembly,SOURCE,D,audit
from rigid_math import kinematics
from wrench import measured_wrench
from control_contract import TrackingTransition


class GeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.a=Assembly(SOURCE)

    def test_com_tensor_matches_independent_static_audit(self):
        ref=audit(self.a.model);s=self.a.state({})
        np.testing.assert_allclose(s['c'],D@ref['com_model_m'],atol=1e-12)
        np.testing.assert_allclose(s['j'],D@np.array(ref['inertia_com_model_kg_m2'])@D,atol=1e-12)

    def test_joint_kinematic_velocity_finite_difference(self):
        q={n:(.1 if 'xy' not in n else .3,.04 if 'xy' not in n else 30.) for n in self.a.geometry['joints']}
        k=kinematics(self.a.geometry,q);eps=1e-6
        kp=kinematics(self.a.geometry,{n:(a+eps*v,v) for n,(a,v) in q.items()})
        km=kinematics(self.a.geometry,{n:(a-eps*v,v) for n,(a,v) in q.items()})
        for name,(t,w,v) in k.items():
            np.testing.assert_allclose((kp[name][0][:3,3]-km[name][0][:3,3])/(2*eps),v,atol=1e-8)
            dr=(kp[name][0][:3,:3]-km[name][0][:3,:3])/(2*eps)@t[:3,:3].T
            np.testing.assert_allclose([dr[2,1],dr[0,2],dr[1,0]],w,atol=1e-7)

    def test_wrench_zero_rpm_and_no_desired_force_input(self):
        joints={n:(0.,0.) for n in self.a.geometry['joints']}
        f,tau,drag=measured_wrench(self.a.geometry,joints,np.zeros(3),np.ones(3))
        np.testing.assert_array_equal(f,np.zeros(3));np.testing.assert_array_equal(tau,np.zeros(3))

    def test_tail_reversal_reverses_thrust_and_drag_torque(self):
        joints={n:(0.,0.) for n in self.a.geometry['joints']};joints['xyj5']=(0.,30.)
        f,tau,_=measured_wrench(self.a.geometry,joints,np.zeros(3),np.zeros(3))
        joints['xyj5']=(0.,-30.)
        fm,tm,_=measured_wrench(self.a.geometry,joints,np.zeros(3),np.zeros(3))
        np.testing.assert_allclose(fm,-f,atol=1e-12);np.testing.assert_allclose(tm,-tau,atol=1e-12)

    def test_rotor_momentum_uses_slowed_joint_speed_not_aerodynamic_speed(self):
        q={n:(0.,0.) for n in self.a.geometry['joints']};q['xyj5']=(0.,30.)
        a=self.a.state(q)['h'];q['xyj5']=(0.,60.);b=self.a.state(q)['h']
        self.assertGreater(np.linalg.norm(a),0.)
        np.testing.assert_allclose(b,2*a,atol=1e-12)

    def test_zero_integral_tracking_transition_continuous_and_rate_limited(self):
        transition=TrackingTransition();old=np.zeros(3);new=np.array([.1,-.2,.05]);base=np.zeros(3)
        out,ok=transition.step(old,new,base,.004,np.ones(3))
        self.assertTrue(ok);np.testing.assert_array_equal(out,np.zeros(3))
        nxt,ok=transition.step(new,new,base,.004,np.ones(3))
        self.assertLessEqual(np.max(abs(nxt-out)),.02*.004+1e-12)
        transition.freeze();frozen,_=transition.step(new,new,base,.004,np.ones(3));np.testing.assert_array_equal(frozen,nxt)


if __name__=='__main__':unittest.main(verbosity=2)
