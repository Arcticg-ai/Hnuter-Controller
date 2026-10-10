"""Synthetic mathematical checks, explicitly NOT a PX4/Gazebo experiment."""
import unittest
import numpy as np
from estimator import MassEstimator, thrust_up_from_rotors, vertical_reference, bumpless_candidate
from model_audit import audit, PX4
import xml.etree.ElementTree as ET


class CoreTests(unittest.TestCase):
    def feed(self, mass=5.4, vz_noise=0., seed=0):
        estimator = MassEstimator()
        rng = np.random.default_rng(seed)
        for t in np.arange(0,16.001,.004):
            v, a = .1*np.sin(t), .1*np.cos(t)
            # Deliberately synthetic; this is the fixture, not thrust reconstruction.
            estimator.push(t,v+rng.normal(0,vz_noise),mass*(9.8-a))
        return estimator

    def test_nominal_and_loaded_are_distinct(self):
        nominal, loaded = self.feed(4.5), self.feed(5.4)
        self.assertTrue(nominal.accepted and loaded.accepted)
        self.assertLess(abs(nominal.mass-4.5),.01)
        self.assertLess(abs(loaded.mass-5.4),.01)

    def test_five_noise_seeds_only_synthetic(self):
        for seed in range(5):
            estimator = self.feed(vz_noise=.01,seed=seed)
            self.assertTrue(estimator.accepted)
            self.assertLess(abs(estimator.mass-5.4),.03)

    def test_hover_informative(self):
        e = MassEstimator()
        for t in np.arange(0,16.001,.004):
            e.push(t,0,5.4*9.8)
        self.assertTrue(e.accepted)
        self.assertLess(abs(e.mass-5.4),.01)

    def test_independent_holdout_rejects_changed_force(self):
        e = MassEstimator()
        for t in np.arange(0,8.001,.004):
            e.push(t,0,5.4*9.8)
        self.assertIsNotNone(e.candidate)
        self.assertFalse(e.accepted)
        frozen = e.mass
        for t in np.arange(8.004,10.001,.004):
            e.push(t,0,6.2*9.8)
        self.assertFalse(e.accepted)
        self.assertEqual(e.mass,frozen)
        self.assertEqual(e.reason,'holdout_residual_rejected')

    def test_timestamp_and_gate_reset(self):
        e = self.feed()
        e.push(17,0,53)
        self.assertFalse(e.accepted)
        self.assertEqual(e.reason,'timestamp_gap_or_reversal')
        e.push(17.004,0,53,False)
        self.assertIsNone(e.start)
        self.assertEqual(e.reason,'invalid_or_ineligible_sample')

    def test_bounds_not_silently_clipped(self):
        e = MassEstimator()
        for t in np.arange(0,.405,.004):
            e.push(t,0,100)
        self.assertFalse(e.accepted)
        self.assertEqual(e.reason,'outside_physical_bounds')

    def test_rotation_and_units(self):
        omega = np.array([100.,200.])
        axes = np.array([[0,0,-1.],[0,0,-1.]])
        k = np.array([.001,.001])
        self.assertAlmostEqual(thrust_up_from_rotors(omega,axes,np.eye(3),k),50.)
        angle = np.pi/3
        r = np.array([[1,0,0],[0,np.cos(angle),-np.sin(angle)],[0,np.sin(angle),np.cos(angle)]])
        self.assertAlmostEqual(thrust_up_from_rotors(omega,axes,r,k),25.)
        with self.assertRaises(ValueError):
            thrust_up_from_rotors(omega,axes,-np.eye(3),k)

    def test_reference_derivatives_and_endpoints(self):
        h = 1e-4
        for t in np.linspace(.01,15.99,99):
            p, v, a = vertical_reference(t)
            lo, hi = vertical_reference(t-h), vertical_reference(t+h)
            self.assertAlmostEqual(v,(hi[0]-lo[0])/(2*h),places=7)
            self.assertAlmostEqual(a,(hi[1]-lo[1])/(2*h),places=7)
        for t in (0,16):
            np.testing.assert_array_equal(vertical_reference(t),[0,0,0])

    def test_bumpless_force_and_rejection(self):
        a = np.array([.2,-.1,.3]); old = np.array([.1,.1,-1.9]); g = np.array([0,0,9.81])
        new = bumpless_candidate(4.5,5.4,a,old)
        self.assertIsNotNone(new)
        np.testing.assert_allclose(4.5*(a+old-g),5.4*(a+new-g),atol=1e-12)
        self.assertIsNone(bumpless_candidate(4.5,7.,[0,0,0],[0,0,0]))

    def test_source_mass_and_frame_resolution(self):
        model = ET.parse(PX4/'Tools/simulation/gz/models/hnuter/model.sdf').getroot().find('model')
        result = audit(model)
        self.assertAlmostEqual(result['total_mass_kg'],4.5,places=6)
        self.assertEqual(len(result['links']),10)
        self.assertTrue(all(x>0 for x in result['principal_inertias_kg_m2']))


if __name__ == '__main__':
    unittest.main(verbosity=2)
