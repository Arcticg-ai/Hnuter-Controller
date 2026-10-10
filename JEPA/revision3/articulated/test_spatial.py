import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import unittest
import numpy as np
from assembly import Assembly,SOURCE
from actions import reference
from rigid_math import rotation
from spatial_model import origin_properties,parameters,from_parameters,window,physical


class SpatialTests(unittest.TestCase):
    def test_parallel_axis_full_tensor_roundtrip(self):
        m=5.4;c=np.array([-.1,.02,.004]);jc=np.array([[.25,.01,-.002],[.01,.13,.001],[-.002,.001,.33]])
        h,jo,_=origin_properties(m,c,jc,np.zeros(3),np.zeros(3))
        theta=parameters(h,jo);cr,jr=from_parameters(theta,m,h[2])
        np.testing.assert_allclose(cr,c);np.testing.assert_allclose(jr,jc)
        self.assertTrue(physical(theta,m,h[2]))

    def test_nominal_moving_terms_are_payload_independent(self):
        nominal=Assembly(SOURCE);n0=nominal.state({});hn0,jn0,_=origin_properties(n0['m'],n0['c'],n0['j'],n0['h'],n0['cdot'])
        rng=np.random.default_rng(5)
        for case in ('offset_y','symmetric_inertia'):
            load=Assembly(Path(__file__).resolve().parents[1]/'generated'/case/'model.sdf');l0=load.state({})
            hl0,jl0,_=origin_properties(l0['m'],l0['c'],l0['j'],l0['h'],l0['cdot'])
            for trial in range(8):
                q={name:(rng.uniform(-.3,.3),rng.uniform(-2,2) if 'xy' not in name else rng.uniform(-60,60)) for name in nominal.geometry['joints']}
                n=nominal.state(q);l=load.state(q)
                hn,jn,rn=origin_properties(n['m'],n['c'],n['j'],n['h'],n['cdot'])
                hl,jl,rl=origin_properties(l['m'],l['c'],l['j'],l['h'],l['cdot'])
                np.testing.assert_allclose(hl-hl0,hn-hn0,atol=1e-12)
                np.testing.assert_allclose(jl-jl0,jn-jn0,atol=1e-12)
                np.testing.assert_allclose(rl,rn,atol=1e-12)

    def test_full_articulated_integral_known_parameter_recovery(self):
        m=5.4;c=np.array([-.1,.01,.004]);jc=np.array([[.25,.01,-.002],[.01,.13,.001],[-.002,.001,.33]])
        h,jo,_=origin_properties(m,c,jc,np.zeros(3),np.zeros(3));theta=parameters(h,jo)
        aa=[];bb=[];without_internal=[]
        for axis in range(3):
            t=np.arange(0,6.0001,.001);r=[];w=[];f=[];tau=[];dh=[];dj=[];hr=[]
            for tt in t:
                angle,rate,acc=reference(tt,axis)
                rr=rotation(np.eye(3)[axis],angle);ww=np.eye(3)[axis]*rate;alpha=np.eye(3)[axis]*acc
                hh=np.array([.002*np.sin(.7*tt),.001*np.cos(.9*tt),0.])
                jj=np.diag([.001*np.sin(.8*tt),.001*np.cos(.6*tt),.0005*np.sin(1.1*tt)])
                jdot=np.diag([.0008*np.cos(.8*tt),-.0006*np.sin(.6*tt),.00055*np.cos(1.1*tt)])
                hhrel=np.array([.01*np.sin(1.9*tt),.02*np.cos(1.3*tt),.003*np.sin(.8*tt)])
                hrdot=np.array([.019*np.cos(1.9*tt),-.026*np.sin(1.3*tt),.0024*np.cos(.8*tt)])
                ff=rr.T@np.array([0.,0.,-9.8])
                torque=(jo+jj)@alpha+jdot@ww+hrdot+np.cross(ww,(jo+jj)@ww+hhrel)+np.cross(h+hh,ff)
                r.append(rr);w.append(ww);f.append(ff);tau.append(torque);dh.append(hh);dj.append(jj);hr.append(hhrel)
            r,w,f,tau,dh,dj,hr=map(np.array,(r,w,f,tau,dh,dj,hr))
            for lo in range(0,len(t)-400,400):
                sl=slice(lo,lo+401)
                a,b=window(t[sl],r[sl],w[sl],f[sl],tau[sl],h[2],dh[sl],dj[sl],hr[sl]);aa.append(a);bb.append(b)
                _,b0=window(t[sl],r[sl],w[sl],f[sl],tau[sl],h[2],dh[sl],dj[sl],np.zeros_like(hr[sl]))
                without_internal.append(np.linalg.norm(b0-a@theta))
        a=np.vstack(aa);b=np.concatenate(bb)
        self.assertEqual(np.linalg.matrix_rank(a),8)
        fitted=np.linalg.lstsq(a,b,rcond=None)[0]
        np.testing.assert_allclose(fitted,theta,atol=2e-6)
        self.assertLess(np.sqrt(np.mean((a@theta-b)**2)),1e-7)
        self.assertGreater(max(without_internal),.005)


if __name__=='__main__':unittest.main(verbosity=2)
