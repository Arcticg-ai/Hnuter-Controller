"""Known moving-link calibration. No SDF reader or payload truth dependency."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rigid_math import kinematics,parallel_axis
D=np.diag([1.,-1.,-1.])


class NominalDynamics:
    def __init__(self,calibration):
        self.geometry=calibration['geometry']
        self.inertials={k:(v[0],np.array(v[1]),np.array(v[2])) for k,v in calibration['inertials'].items()}

    def state(self,joints):
        parts=[];kin=kinematics(self.geometry,joints)
        for name,(m,c,j) in self.inertials.items():
            t,w,v=kin[name]
            off=t[:3,:3]@c
            parts.append((m,t[:3,3]+off,t[:3,:3]@j@t[:3,:3].T,w,v+np.cross(w,off)))
        m=sum(x[0] for x in parts);c=sum(x[0]*x[1] for x in parts)/m;cd=sum(x[0]*x[4] for x in parts)/m
        j=sum(x[2]+parallel_axis(x[0],x[1]-c) for x in parts)
        h=sum(x[2]@x[3]+np.cross(x[1]-c,x[0]*(x[4]-cd)) for x in parts)
        return dict(m=m,c=D@c,j=D@j@D,h=D@h,cdot=D@cd)
