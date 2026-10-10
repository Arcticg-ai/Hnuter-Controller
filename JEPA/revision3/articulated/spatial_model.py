"""Fixed-origin spatial inertia regression with known moving-link corrections.

No truth/model loader. Inputs are sensor values and independently calibrated
nominal moving-link terms. Unknowns: hx,hy, JOxx,JOyy,JOzz,JOxy,JOxz,JOyz.
Mass and hz are fixed. Jc is recovered by the parallel-axis theorem.
"""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from rigid_math import skew,parallel_axis,physical_inertia

BASES=[]
for i,j in [(0,0),(1,1),(2,2),(0,1),(0,2),(1,2)]:
    a=np.zeros((3,3));a[i,j]=1.;a[j,i]=1.;BASES.append(a)


def origin_properties(m,c,jc,hrel_c,cdot):
    return m*c,jc+parallel_axis(m,c),hrel_c+np.cross(c,m*cdot)


def from_parameters(theta,m,hz):
    h=np.array([theta[0],theta[1],hz]);c=h/m
    jo=sum(b*x for b,x in zip(BASES,theta[2:]));jc=jo-parallel_axis(m,c)
    return c,jc


def physical(theta,m,hz):
    c,jc=from_parameters(theta,m,hz)
    return bool(np.all(np.isfinite(theta)) and np.linalg.norm(c)<.5 and physical_inertia(jc))


def parameters(h,jo):
    return np.array([h[0],h[1],jo[0,0],jo[1,1],jo[2,2],jo[0,1],jo[0,2],jo[1,2]])


def window(t,r,w,specific_acc_o,tau_o,hz,delta_h,delta_jo,hrel_o):
    """∫R τO = Δ R(JO ω+hrelO) + ∫R[h × fO].

    Here JO=JOref+known_delta_JO(q), h=href+known_delta_h(q).
    fO is IMU specific acceleration at fixed origin O, not COM acceleration.
    """
    t=np.asarray(t)
    if len(t)<5 or np.any(np.diff(t)<=0) or np.max(np.diff(t))>.03:
        raise ValueError('time_gap')
    arrays=[r,w,specific_acc_o,tau_o,delta_h,delta_jo,hrel_o]
    if not all(np.all(np.isfinite(x)) for x in arrays):raise ValueError('nonfinite')
    lever=np.trapz(np.array([-rr@skew(f) for rr,f in zip(r,specific_acc_o)]),t,axis=0)
    a=np.column_stack([lever[:,0],lever[:,1]]+[r[-1]@b@w[-1]-r[0]@b@w[0] for b in BASES])
    known_h=np.einsum('nij,nj->ni',r,np.einsum('nij,nj->ni',delta_jo,w)+hrel_o)
    b=np.trapz(np.einsum('nij,nj->ni',r,tau_o-np.cross(delta_h,specific_acc_o)),t,axis=0)
    b-=known_h[-1]-known_h[0]+lever[:,2]*hz
    return a,b


def discrete_window(t,r,w,specific_acc_o,tau_o,hz,delta_h,delta_jo,hrel_o):
    """Gazebo semi-implicit step quadrature, derived from measurement semantics.

    Forces at t[k] are applied over the NEXT step. The IMU acceleration at
    t[k+1] represents (v[k+1]-v[k])/dt, expressed at the END orientation.
    Rotate that sample back to the beginning frame before taking h cross f.
    No tuned shift or future sample: finish a window only after its endpoint
    samples have arrived. This is a simulator discretization model, not a
    generally valid sensor correction for hardware.
    """
    # Same validation contract as continuous quadrature.
    window(t,r,w,specific_acc_o,tau_o,hz,delta_h,delta_jo,hrel_o)
    dt=np.diff(t)
    fend_world=np.einsum('nij,nj->ni',r[1:],specific_acc_o[1:])
    fstart=np.einsum('nji,nj->ni',r[:-1],fend_world)
    lever=np.sum(np.array([-rr@skew(f) for rr,f in zip(r[:-1],fstart)])*dt[:,None,None],axis=0)
    a=np.column_stack([lever[:,0],lever[:,1]]+[r[-1]@b@w[-1]-r[0]@b@w[0] for b in BASES])
    known_h=np.einsum('nij,nj->ni',r,np.einsum('nij,nj->ni',delta_jo,w)+hrel_o)
    b=np.sum(np.einsum('nij,nj->ni',r[:-1],tau_o[:-1]-np.cross(delta_h[:-1],fstart))*dt[:,None],axis=0)
    b-=known_h[-1]-known_h[0]+lever[:,2]*hz
    return a,b
