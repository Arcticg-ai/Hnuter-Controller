"""SITL-only articulated rotor geometry read from the actual no-delay SDF.

This is an external rotor wrench model about the assembly COM, not a model
of joint reaction inertia, aerodynamic drag, or flexible-body dynamics.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from controllers.common.hnuter_drcda import HnuterWrenchModel


def rotation_y(angle):
    c,s=np.cos(angle),np.sin(angle)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])


def rotation_y_derivative(angle):
    c,s=np.cos(angle),np.sin(angle)
    return np.array([[-s,0,c],[0,0,0],[-c,0,-s]])


def pose(element):
    p=element.find('pose')
    values=np.zeros(6) if p is None else np.fromstring(p.text,sep=' ')
    if values.shape!=(6,):raise ValueError('SDF pose must have six entries')
    r,pitch,y=values[3:]
    cr,sr,cy,sy=np.cos(r),np.sin(r),np.cos(y),np.sin(y)
    rx=np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    rz=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return values[:3],rz@rotation_y(pitch)@rx


class SDFWrenchModel(HnuterWrenchModel):
    ROTORS=('xy3','xy4','xy1','xy2','xy5')
    TILTS={'lj2':0,'lj1':1,'rj2':2,'rj1':3}

    def __init__(self,path):
        self.path=Path(path)
        model=ET.parse(self.path).getroot().find('model')
        self.frames=[];self.inertials=[]
        known={'base_link'}
        pending=[e for e in model if e.tag in ('link','joint') and e.get('name')!='base_link']
        while pending:
            ready=[]
            for e in pending:
                pe=e.find('pose');parent=pe.get('relative_to') if pe is not None else None
                if parent not in known:continue
                t,r=pose(e);name=e.get('name')
                index=self.TILTS.get(name)
                if index is not None:
                    axis=np.fromstring(e.findtext('axis/xyz'),sep=' ')
                    if not np.allclose(r@axis,[0,1,0],atol=1e-7):
                        raise ValueError('joint feedback coordinates no longer match SDF')
                self.frames.append((name,parent,t,r,index));known.add(name);ready.append(e)
            if not ready:raise ValueError('unresolved SDF frame graph')
            pending=[e for e in pending if e not in ready]
        for link in model.findall('link'):
            inertia=link.find('inertial');mass=float(inertia.findtext('mass'))
            offset,_=pose(inertia)
            self.inertials.append((link.get('name'),mass,offset))
        self.mass=sum(i[1] for i in self.inertials)
        plugins={p.findtext('linkName'):p for p in model.findall('plugin') if p.find('motorNumber') is not None}
        self.reaction_ratios=np.array([float(plugins[n].findtext('momentConstant')) for n in self.ROTORS])
        # Gazebo reaction opposes the propeller spin in its local thrust axis.
        self.spin_sign=np.array([-1. if plugins[n].findtext('turningDirection')=='ccw' else 1. for n in self.ROTORS])
        self.reaction_torque_ratio_m=float(self.reaction_ratios[0])
        self.positions,self.zero_directions,_,_=self.geometry(np.zeros(4))

    def geometry(self,angles):
        states={'base_link':(np.eye(3),np.zeros(3),np.zeros((4,3,3)),np.zeros((4,3)))}
        for name,parent,t,r,index in self.frames:
            pr,pp,pdr,pdp=states[parent]
            local=r if index is None else rotation_y(angles[index])@r
            dr=pdr@local
            if index is not None:dr[index]+=pr@rotation_y_derivative(angles[index])@r
            states[name]=(pr@local,pp+pr@t,dr,pdp+pdr@t)
        com=np.zeros(3);dcom=np.zeros((4,3))
        for name,m,offset in self.inertials:
            r,p,dr,dp=states[name]
            com+=m*(p+r@offset);dcom+=m*(dp+dr@offset)
        com/=self.mass;dcom/=self.mass
        positions=[];directions=[];dpositions=[];ddirections=[]
        for name in self.ROTORS:
            r,p,dr,dp=states[name]
            positions.append(p-com);directions.append(r[:,2])
            dpositions.append(dp-dcom);ddirections.append(dr[:,:,2])
        return tuple(np.asarray(a) for a in (positions,directions,dpositions,ddirections))

    def _effectiveness(self,angles):
        p,d,dp,dd=self.geometry(angles)
        reaction=(self.spin_sign*self.reaction_ratios)[:,None]
        moments=np.cross(p,d)+reaction*d
        dm=np.cross(dp,d[:,None,:])+np.cross(p[:,None,:],dd)+reaction[:,None,:]*dd
        matrix=np.concatenate((d,moments),axis=1).T
        derivatives=np.concatenate((dd,dm),axis=2).transpose(1,2,0)
        return matrix,derivatives

    def wrench(self,state):
        state=np.asarray(state,dtype=float)
        matrix,_=self._effectiveness(state[:4])
        return matrix@state[4:]

    def jacobian(self,state):
        state=np.asarray(state,dtype=float)
        matrix,derivatives=self._effectiveness(state[:4])
        return np.column_stack(((derivatives@state[4:]).T,matrix))
