import unittest,numpy as np
from adapter import Adapter
from aligned import Aligner
from runtime.excitation_r6 import reference
class Contract(unittest.TestCase):
    def test_no_future_extrapolation(self):
        a=Aligner();values=dict(position=[0,0,0],attitude=np.eye(3).ravel().tolist(),rate=[0,0,0],motor=[0]*5,servo=[0]*4)
        out=[]
        for t in [100000,200000]:
            for k,v in values.items():out+=a.feed(dict(key=k,t_us=t,receive_ns=t*1000+1,value=v,elapsed=t*1e-6,phase='TEST'))
        self.assertEqual([x['t_us'] for x in out],[100000,200000]);self.assertTrue(all(x['receive_ns']>=x['t_us']*1000 for x in out))
        # One fast stream cannot fabricate a common future state.
        self.assertEqual(a.feed(dict(key='motor',t_us=400000,receive_ns=400000001,value=[0]*5,elapsed=.4,phase='TEST')),[])
    def test_partial_rank_and_freeze(self):
        a=Adapter(np.ones(6));rng=np.random.default_rng(4)
        for i in range(80):
            phi=np.array([rng.normal(),0,0,0]);a.update(phi,np.r_[phi[0],np.zeros(5)],48+i*.099)
        self.assertEqual(a.rank,1);self.assertGreater(a.updates,0);self.assertTrue(np.all(a.W[:,1:]==0));w=a.W.copy()
        for i in range(100):a.update(np.ones(4),np.ones(6)*1e6,60+i*.1)
        np.testing.assert_array_equal(a.W,w);self.assertLessEqual(np.linalg.norm(a.W),2.)
    def test_constant_disturbance_not_slope(self):
        a=Adapter(np.ones(6));rng=np.random.default_rng(4)
        for i in range(70):a.update(rng.normal(size=4),np.ones(6)*.5,48+i*.1)
        self.assertLess(np.linalg.norm(a.W),1e-10)
    def test_bounds_and_independent_task(self):
        for mode in ['active','conventional']:
            for t in np.linspace(0,72,7201):
                r,_=reference(t,mode);self.assertLessEqual(abs(r).max(),np.deg2rad(1.5)+1e-12)
                if t<48 or 56<=t<=58 or t>=68:self.assertEqual(abs(r).max(),0)
        for t in np.linspace(58,68,101):np.testing.assert_array_equal(reference(t,'active')[0],reference(t,'conventional')[0])
if __name__=='__main__':unittest.main()
