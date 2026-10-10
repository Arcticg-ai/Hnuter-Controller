"""Sensor-state wrench reconstruction; configuration has no mass/inertia fields."""
import json
from pathlib import Path
import threading
from collections import deque
import numpy as np
from gz.transport13 import Node
from gz.msgs10.model_pb2 import Model
from model_audit import pose
from estimator import thrust_up_from_rotors
import xml.etree.ElementTree as ET


def export_geometry(source, destination):
    model=ET.parse(source).getroot().find('model')
    geometry=dict(links={},joints={},motors=[])
    for e in model.findall('link')+model.findall('joint'):
        pe=e.find('pose')
        row=dict(pose=pose(pe).tolist(),relative=pe.get('relative_to','__model__') if pe is not None else '__model__')
        if e.tag=='joint':
            row.update(child=e.findtext('child'),axis=[float(v) for v in e.findtext('axis/xyz','0 0 1').split()])
        geometry[e.tag+'s'][e.get('name')]=row
    for e in model.findall('plugin'):
        if 'MulticopterMotorModel' in e.get('name',''):
            geometry['motors'].append(dict(index=int(e.findtext('motorNumber')),link=e.findtext('linkName'),joint=e.findtext('jointName'),
                                           k=float(e.findtext('motorConstant')),slowdown=float(e.findtext('rotorVelocitySlowdownSim')),
                                           direction=1 if e.findtext('turningDirection')=='ccw' else -1))
    geometry['motors'].sort(key=lambda v:v['index'])
    Path(destination).write_text(json.dumps(geometry,indent=2)+'\n')


class JointSensors:
    def __init__(self,geometry_path,topic):
        self.geometry=json.loads(Path(geometry_path).read_text())
        self.samples=deque(maxlen=3000)
        self.lock=threading.Lock()
        self.node=Node()
        self.node.subscribe(Model,topic,self.callback)

    def callback(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
        joints={j.name:(j.axis1.position,j.axis1.velocity) for j in msg.joint}
        with self.lock:self.samples.append((stamp,joints))

    def at(self,t,rng=None):
        with self.lock:
            samples=list(self.samples)
        if not samples:return None
        # Last measurement at/before the velocity time; no future information.
        valid=[s for s in samples if s[0]<=t+1e-6]
        if not valid:return None
        stamp,joints=valid[-1]
        if t-stamp>.025:return None
        if not all(n in joints for n in self.geometry['joints']):return None
        if rng is not None:
            joints={name:(p+rng.normal(0,np.deg2rad(.1)),v) for name,(p,v) in joints.items()}
        transforms={'__model__':np.eye(4)}
        children={j['child']:name for name,j in self.geometry['joints'].items()}
        entities={**self.geometry['links'],**self.geometry['joints']}
        def transform(name):
            if name in transforms:return transforms[name]
            e=entities[name]
            matrix=transform(e['relative'])@np.array(e['pose'])
            if name in children:
                jname=children[name]; axis=np.array(self.geometry['joints'][jname]['axis'],dtype=float)
                axis/=np.linalg.norm(axis);theta=joints[jname][0]
                a=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
                rotation=np.eye(4);rotation[:3,:3]=np.eye(3)+np.sin(theta)*a+(1-np.cos(theta))*(a@a)
                # Child pose is relative to joint in this audited SDF.
                matrix=transform(e['relative'])@rotation@np.array(e['pose'])
            transforms[name]=matrix
            return matrix
        axes=[];speeds=[];ks=[]
        for m in self.geometry['motors']:
            # GZ physical rotor velocity = slowed joint velocity times slowdown.
            # Direction restores signed command convention for CW rotors.
            speeds.append(joints[m['joint']][1]*m['slowdown']*m['direction'])
            if rng is not None:speeds[-1]+=rng.normal(0,2.)
            axes.append(np.diag([1.,-1.,-1.])@transform(m['link'])[:3,2])
            ks.append(m['k'])
        return stamp,np.array(speeds),np.array(axes),np.array(ks)

    def thrust(self,t,rotation,rng=None):
        sample=self.at(t,rng)
        if sample is None:return None
        stamp,omega,axes,k=sample
        return dict(sensor_t=stamp,thrust_up=thrust_up_from_rotors(omega,axes,rotation,k),omega=omega.tolist())
