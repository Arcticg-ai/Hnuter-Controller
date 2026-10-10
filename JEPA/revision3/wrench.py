"""Reconstruct external actuator wrench from measured joints + fixed calibration.

No vehicle mass, vehicle COM, inertia, desired wrench, SDF or truth data.
Gazebo 8.11 motor model: signed thrust, drag, reaction and rolling moments.
Internal rotor angular momentum is NOT an external aerodynamic torque.
"""
import numpy as np
from rigid_math import kinematics

D=np.diag([1.,-1.,-1.])


def measured_wrench(geometry,joints,omega_frd,velocity_origin_frd,wind_frd=None):
    required=[n for n,j in geometry['joints'].items() if j['type']!='fixed']
    if any(n not in joints for n in required):raise ValueError('missing_joint')
    if not all(np.all(np.isfinite(joints[n])) for n in required):raise ValueError('invalid_joint')
    kin=kinematics(geometry,joints)
    w=D@np.asarray(omega_frd);v=D@np.asarray(velocity_origin_frd)
    wind=np.zeros(3) if wind_frd is None else D@np.asarray(wind_frd)
    force=np.zeros(3);moment=np.zeros(3);drag_sum=np.zeros(3)
    for motor in geometry['motors']:
        t,wr,vr=kin[motor['link']]
        axis=t[:3,2]
        speed=joints[motor['joint']][1]*motor['slowdown']
        thrust=motor['direction']*speed*abs(speed)*motor['k']
        point=t[:3,3]+t[:3,:3]@np.asarray(motor['application_point'])
        # WorldLinearVelocity is at the link origin. Its transverse velocity
        # includes body motion and relative articulated motion; wind is zero.
        rel=v+np.cross(w,t[:3,3])+vr-wind
        transverse=rel-axis*(rel@axis)
        drag=-abs(speed)*motor['drag']*transverse
        f=axis*thrust+drag
        reaction=-motor['direction']*thrust*motor['km']*axis
        rolling=-abs(speed)*motor['rolling']*transverse
        force+=f;drag_sum+=drag
        moment+=np.cross(point,f)+reaction+rolling
    return D@force,D@moment,D@drag_sum
