import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import unittest
import numpy as np
from rigid_math import rotation,physical_inertia
from integral_estimator import Estimator,integral_window,SCALE
from actions import synthetic_axis,windows,reference,candidate_actions,CalibrationState
from control_contract import correction,smooth_bumpless

THETA=np.array([-.1,.015,.18,.16,.24]);OFF=np.array([[0,.001,-.002],[.001,0,.0003],[-.002,.0003,0]])
PRIOR=np.array([-.105,0.,.15,.14,.21]);RZ=.014


class PhysicsTests(unittest.TestCase):
    def test_physical_full_tensor_and_triangle(self):
        self.assertTrue(physical_inertia(np.diag(THETA[2:])+OFF))
        self.assertFalse(physical_inertia(np.diag([1.,1.,3.])))
        self.assertFalse(physical_inertia([[.1,.2,0],[.2,.1,0],[0,0,.1]]))

    def test_hover_only_two_com_directions_no_prior_information(self):
        t=np.arange(101)*.004;r=np.tile(np.eye(3),(101,1,1));w=np.zeros((101,3));f=np.tile([0,0,-52.92],(101,1))
        tau=np.tile(np.cross([*THETA[:2],RZ],f[0]),(101,1));est=Estimator(PRIOR,OFF,torque_bias_bound_nm=0.)
        for i in range(30):est.add(integral_window(t+i*.4,r,w,f,tau,RZ,OFF),holdout=i>=25)
        p=est.propose();self.assertEqual(p['rank'],2);self.assertTrue(p['accepted'])
        np.testing.assert_array_equal(np.array(p['theta'])[2:],PRIOR[2:])

    def test_each_axis_adds_only_corresponding_inertia(self):
        for axis in range(3):
            est=Estimator(PRIOR,OFF,sigma_impulse=.0002)
            ws=windows(synthetic_axis(axis,THETA,OFF,RZ),RZ,OFF)
            info=est.information(ws)
            self.assertEqual(info['rank'],3)
            self.assertGreater(info['projector'][axis+2,axis+2],.99)
            for k in set(range(3))-{axis}:self.assertLess(info['projector'][k+2,k+2],1e-8)

    def test_short_windows_recover_known_parameters(self):
        est=Estimator(PRIOR,OFF,sigma_impulse=.0002,torque_bias_bound_nm=0.)
        for axis in range(3):
            for w in windows(synthetic_axis(axis,THETA,OFF,RZ),RZ,OFF,offset=axis*12):est.add(w)
        # Independent subsequent multi-axis validation, no reuse of fit windows.
        for axis in range(3):
            for w in windows(synthetic_axis(axis,THETA,OFF,RZ),RZ,OFF,offset=36+axis*12):est.add(w,holdout=True)
        p=est.propose();self.assertEqual(p['rank'],5)
        # Axis-block training lacks split stability on all axes; guard must reject.
        self.assertFalse(p['accepted']);self.assertEqual(p['reason'],'split_information_insufficient')
        a=np.vstack([w.a for w in est.train]);b=np.concatenate([w.b for w in est.train])
        fit=np.linalg.lstsq(a,b,rcond=None)[0]
        np.testing.assert_allclose(fit,THETA,atol=5e-5)

    def test_body_and_world_integrals_agree(self):
        t,r,w,f,tau=synthetic_axis(0,THETA,OFF,RZ,dt=.001)
        sl=slice(1700,2101);t,r,w,f,tau=[x[sl] for x in (t,r,w,f,tau)]
        window=integral_window(t,r,w,f,tau,RZ,OFF)
        self.assertLess(np.linalg.norm(window.a@THETA-window.b),1e-7)
        j=np.diag(THETA[2:])+OFF
        # Body form retains cross(omega,J omega); independent sign check.
        lhs=j@(w[-1]-w[0])+np.trapz(np.cross(w,w@j.T),t,axis=0)
        rhs=np.trapz(tau-np.cross(np.array([*THETA[:2],RZ]),f),t,axis=0)
        self.assertLess(np.linalg.norm(lhs-rhs),1e-7)

    def test_whole_cycle_information_cancels(self):
        t,r,w,f,tau=synthetic_axis(2,THETA,OFF,RZ)
        full=r[-1][:,2]*w[-1,2]-r[0][:,2]*w[0,2]
        self.assertLess(np.linalg.norm(full),1e-12)
        self.assertGreater(sum(np.linalg.norm(x.a[:,4]) for x in windows((t,r,w,f,tau),RZ,OFF)),.1)

    def test_unknown_constant_torque_confounds_com(self):
        # A constant .1 Nm roll bias is indistinguishable from ~1.89 mm ry.
        f=np.array([0.,0.,-52.92]);dr=np.array([0.,.1/52.92,0.])
        np.testing.assert_allclose(np.cross(dr,f),[-.1,0,0])

    def test_unverified_bias_never_accepts_despite_good_rank(self):
        e=Estimator(PRIOR,OFF)
        t=np.arange(101)*.004;r=np.tile(np.eye(3),(101,1,1));w=np.zeros((101,3));f=np.tile([0,0,-52.92],(101,1))
        tau=np.tile(np.cross([*THETA[:2],RZ],f[0]),(101,1))
        for i in range(30):e.add(integral_window(t+i*.4,r,w,f,tau,RZ,OFF),holdout=i>=25)
        p=e.propose();self.assertEqual(p['rank'],2);self.assertFalse(p['accepted'])
        self.assertEqual(p['reason'],'unverified_constant_torque_bias')

    def test_known_internal_momentum_endpoint_correction(self):
        t=np.linspace(0,.4,101);r=np.tile(np.eye(3),(101,1,1));w=np.zeros((101,3));f=np.tile([0,0,-52.92],(101,1))
        h=np.column_stack([.01*t,.02*t,np.zeros(len(t))])
        tau=np.tile(np.cross([*THETA[:2],RZ],f[0])+[.01,.02,0],(len(t),1))
        a=integral_window(t,r,w,f,tau,RZ,OFF)
        b=integral_window(t,r,w,f,tau,RZ,OFF,nuisance_world=h)
        self.assertGreater(np.linalg.norm(a.a@THETA-a.b),.008)
        self.assertLess(np.linalg.norm(b.a@THETA-b.b),1e-12)

    def test_delayed_reversed_and_overlapping_data_reject(self):
        d=synthetic_axis(0,THETA,OFF,RZ);data=[x[:101] for x in d]
        with self.assertRaisesRegex(ValueError,'timestamp'):integral_window(*data,RZ,OFF,max_age=.05)
        with self.assertRaisesRegex(ValueError,'window_duration'):integral_window(*[x[::-1] for x in data],RZ,OFF)
        e=Estimator(PRIOR,OFF);w=integral_window(*data,RZ,OFF);e.add(w)
        with self.assertRaisesRegex(ValueError,'overlap'):e.add(w)

    def test_analytic_reference_derivatives(self):
        for t in [.1,1.,3.,5.9]:
            h=1e-4;v=reference(t,0);a=reference(t+h,0);b=reference(t-h,0)
            self.assertAlmostEqual((a[0]-b[0])/(2*h),v[1],places=7)
            self.assertAlmostEqual((a[1]-b[1])/(2*h),v[2],places=7)
        np.testing.assert_array_equal(reference(0,0),np.zeros(3))
        np.testing.assert_array_equal(reference(6,0),np.zeros(3))

    def test_baseline_identity_and_bumpless_rejection(self):
        r0=np.array([-.1,0,.014]);j0=np.diag(PRIOR[2:])+OFF;f=np.array([0,0,-53.]);w=np.array([.02,.01,.03]);pd=np.array([.01,.02,.03]);i=np.zeros(3)
        np.testing.assert_array_equal(correction(r0,j0,r0,j0,f,w,pd),np.zeros(3))
        rn,jn,inew,ok,_=smooth_bumpless(r0,j0,r0+[.001,.002,0],j0*1.1,r0,j0,f,w,pd,i,.004,np.ones(3)*.2,np.ones(3))
        self.assertTrue(ok)
        np.testing.assert_allclose(inew+correction(rn,jn,r0,j0,f,w,pd),i,atol=1e-15)
        *_,ok,reason=smooth_bumpless(r0,j0,r0+[.001,.002,0],j0*1.1,r0,j0,f,w,pd,i,.004,np.zeros(3),np.ones(3))
        self.assertFalse(ok)

    def test_action_gate_prevents_unvalidated_flight(self):
        e=Estimator(PRIOR,OFF);info=e.information();candidates=candidate_actions(PRIOR,OFF,RZ,info)
        state=CalibrationState().choose(info,candidates,True,False,False)
        self.assertEqual(state.state,'HOLD')
        # Full information does not trigger a gratuitous action.
        info['rank']=5
        self.assertEqual(CalibrationState().choose(info,candidates,True,True,True).state,'VALIDATE')
        state.abort('tilt');self.assertEqual(state.state,'FROZEN_RECOVERY')

    def test_action_uses_supported_missing_direction_and_verified_margin(self):
        info=dict(rank=4,projector=np.diag([1,1,0,1,1]))
        # Synthetic interface fixtures, not evidence of real actuator authority.
        choices=[dict(axis=i,within_bounds=True,actuator_margin_verified=True,
                      supported_new_directions=int(i==0),new_information=9. if i==0 else 0.) for i in range(3)]
        state=CalibrationState().choose(info,choices,True,True,True)
        self.assertEqual(state.state,'EXCITE');self.assertEqual(state.selected_axis,0)
        choices[0]['actuator_margin_verified']=False
        self.assertEqual(CalibrationState().choose(info,choices,True,True,True).state,'HOLD')


if __name__=='__main__':unittest.main(verbosity=2)
