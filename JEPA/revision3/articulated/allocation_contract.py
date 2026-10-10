"""Offline local torque Jacobian of the existing allocator and nominal geometry.

Source equations: ActuatorEffectivenessHnuter.cpp, c0d57b04. This is a static
small-signal calibration, not a command publisher or a replacement allocator.
"""
import sys
import json
import itertools
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from assembly import Assembly,SOURCE
from wrench import measured_wrench


def forward(geometry,f,t):
    fx,fy,fz=f*np.array([1,-1,-1]);tx,ty,tz=t*np.array([1,-1,-1])
    l1=.33;l2=.664;rx=.105;rz=-.013
    ft=ty/(rx+l2);fz-=ft;tx-= -rz*fy
    us=[np.array([fx/2-tz/(2*l1),fz/2+tx/(2*l1),-fy/2]),
        np.array([fx/2+tz/(2*l1),fz/2-tx/(2*l1),-fy/2])]
    q={k:(0,0) for k in geometry['joints']}
    for u,side in zip(us,['l','r']):
        q[side+'j2']=(np.arctan2(u[0],u[1]),0)
        q[side+'j1']=(np.arcsin(u[2]/np.linalg.norm(u)),0)
    for m in geometry['motors']:
        force=ft if m['index']==4 else np.linalg.norm(us[1 if m['index']<2 else 0])/2
        q[m['joint']]=(0,np.sign(force)*np.sqrt(abs(force)/m['k'])/m['slowdown']*m['direction'])
    f,t,_=measured_wrench(geometry,q,np.zeros(3),np.zeros(3))
    return f,t


def jacobian(n,f,t):
    c=n.state({})['c'];eps=.001;cols=[]
    for i in range(3):
        fp,tp=forward(n.geometry,f,t+np.eye(3)[i]*eps)
        fm,tm=forward(n.geometry,f,t-np.eye(3)[i]*eps)
        cols.append(((tp-tm)-np.cross(c,fp-fm))/(2*eps))
    return np.column_stack(cols)


def main():
    n=Assembly(SOURCE);f=np.array([0.,0.,-5.4*9.81]);t=np.array([0.,-.105*5.4*9.81,0.])
    g=jacobian(n,f,t);rng=np.random.default_rng(14);errors=[]
    for signs in itertools.product((-1.,0.,1.),repeat=6):
        ff=f+np.array(signs[:3])*[6,6,3];tt=t+np.array(signs[3:])*[.85,.6,.6]
        gg=jacobian(n,ff,tt);errors.append(float(np.linalg.norm(gg@np.linalg.inv(g)-np.eye(3),2)))
    report=dict(kind='static_nominal_geometry_calibration_only',center_jacobian=g.tolist(),inverse=np.linalg.inv(g).tolist(),
        max_relative_operator_error=max(errors),identity_mapping_error=float(np.linalg.norm(g-np.eye(3),2)),
        envelope=dict(force_delta_bounds_N=[6,6,3],torque_delta_bounds_Nm=[.85,.6,.6],grid_count=729),
        caveat='Excludes actuator lag, saturation, rate limiting, and large tilt. Actual consumer still requires SITL oracle.')
    path=Path(__file__).resolve().parent/'results/allocation_contract_expanded.json';path.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
