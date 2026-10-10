"""OFFLINE/setup only: SDF composition, test payloads and articulation audit.

Never import this module from the estimator or flight node. Full tensors stay
in experiment/evaluation files. The sensor export strips inertial properties.
"""
import copy
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
from rigid_math import kinematics, parallel_axis, physical_inertia

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent/'revision2'))
from model_audit import pose, audit

SOURCE = Path('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl/Tools/simulation/gz/models/hnuter/model.sdf')
D = np.diag([1.,-1.,-1.])


class Assembly:
    def __init__(self, source):
        self.model = ET.parse(source).getroot().find('model')
        audit(self.model)  # Fail closed on unsupported frames/nonphysical links.
        self.links = {e.get('name'): e for e in self.model.findall('link')}
        entities={e.get('name'):e for e in self.model if e.tag in ('link','joint','frame')}
        cache={'__model__':np.eye(4)}
        def transform(n):
            if n in cache:return cache[n]
            e=entities[n];pe=e.find('pose')
            default=e.findtext('child') if e.tag=='joint' else e.get('attached_to','__model__')
            relative=pe.get('relative_to',default) if pe is not None else default
            cache[n]=transform(relative)@pose(pe)
            return cache[n]
        self.geometry=dict(links={},joints={},motors=[])
        self.inertials={}
        for n,e in self.links.items():
            self.geometry['links'][n]={'T0':transform(n).tolist()}
            inertial=e.find('inertial');p=pose(inertial.find('pose'))
            vals={k:float(inertial.findtext('inertia/'+k,'0')) for k in ('ixx','iyy','izz','ixy','ixz','iyz')}
            j=np.array([[vals['ixx'],vals['ixy'],vals['ixz']], [vals['ixy'],vals['iyy'],vals['iyz']], [vals['ixz'],vals['iyz'],vals['izz']]])
            self.inertials[n]=(float(inertial.findtext('mass')),p[:3,3],p[:3,:3]@j@p[:3,:3].T)
        for e in self.model.findall('joint'):
            axis=e.find('axis/xyz')
            if axis is not None and axis.get('expressed_in'):
                raise ValueError('unsupported axis frame')
            n=e.get('name');kind=e.get('type')
            if kind not in ('fixed','revolute'):raise ValueError('unsupported joint type')
            self.geometry['joints'][n]=dict(name=n,parent=e.findtext('parent'),child=e.findtext('child'),type=kind,
                                           T0=transform(n).tolist(),axis=list(map(float,e.findtext('axis/xyz','0 0 1').split())))
        for e in self.model.findall('plugin'):
            if 'MulticopterMotorModel' not in e.get('name',''):continue
            link=e.findtext('linkName')
            # Application point is rotor link COM in Gazebo AddWorldForce.
            # Fixed nominal rotor-COM calibration only, not total vehicle COM.
            self.geometry['motors'].append(dict(index=int(e.findtext('motorNumber')),link=link,joint=e.findtext('jointName'),
                k=float(e.findtext('motorConstant')),km=float(e.findtext('momentConstant')),
                drag=float(e.findtext('rotorDragCoefficient')),rolling=float(e.findtext('rollingMomentCoefficient')),
                slowdown=float(e.findtext('rotorVelocitySlowdownSim')),direction=1 if e.findtext('turningDirection')=='ccw' else -1,
                application_point=self.inertials[link][1].tolist()))
        self.geometry['motors'].sort(key=lambda x:x['index'])

    def state(self, joints, omega=np.zeros(3)):
        kin=kinematics(self.geometry,joints);parts=[]
        for n,(mass,c,j) in self.inertials.items():
            t,wr,vo=kin[n];cb=t[:3,:3]@c;pc=t[:3,3]+cb
            parts.append((mass,pc,t[:3,:3]@j@t[:3,:3].T,wr,vo+np.cross(wr,cb)))
        m=sum(x[0] for x in parts)
        c=sum(x[0]*x[1] for x in parts)/m
        cdot=sum(x[0]*x[4] for x in parts)/m
        j=sum(x[2]+parallel_axis(x[0],x[1]-c) for x in parts)
        # Internal COM angular momentum includes rotor spins AND tilt motion.
        h=sum(x[2]@x[3]+np.cross(x[1]-c,x[0]*(x[4]-cdot)) for x in parts)
        return dict(m=m,c=D@c,j=D@j@D,h=D@h,cdot=D@cdot)


def create_models():
    nominal=Assembly(SOURCE);ref=nominal.state({});out={}
    # Both cases 5.4 kg; same mass in every group. One-axis horizontal offset
    # at the nominal COM height preserves the known reference products of J.
    for name,offset,size in [('offset_y',np.array([0.,.04,0.]),[.10,.08,.04]),
                             ('symmetric_inertia',np.zeros(3),[.36,.32,.04])]:
        model=copy.deepcopy(nominal.model);model.set('name','hnuter_r3_'+name)
        c=D@ref['c']+offset
        link=ET.SubElement(model,'link',name='revision3_payload')
        ET.SubElement(link,'pose').text=' '.join(map(str,c))+' 0 0 0'
        inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'mass').text='.9'
        tensor=ET.SubElement(inertial,'inertia');a,b,d=size
        diag=.9/12*np.array([b*b+d*d,a*a+d*d,a*a+b*b])
        for key,val in zip(('ixx','iyy','izz','ixy','ixz','iyz'),[*diag,0,0,0]):ET.SubElement(tensor,key).text=str(val)
        joint=ET.SubElement(model,'joint',name='revision3_mount',type='fixed')
        ET.SubElement(joint,'parent').text='base_link';ET.SubElement(joint,'child').text='revision3_payload'
        for uri in model.findall('.//mesh/uri'):
            if not uri.text.startswith(('/','model://','file://')):uri.text=str(SOURCE.parent/uri.text)
        for link in model.findall('link'):
            wind=link.find('enable_wind')
            if wind is None:wind=ET.SubElement(link,'enable_wind')
            wind.text='false'
        folder=ROOT/'generated'/name;folder.mkdir(exist_ok=True)
        root=ET.Element('sdf',version='1.11');root.append(model);ET.indent(root)
        ET.ElementTree(root).write(folder/'model.sdf',encoding='utf-8',xml_declaration=True)
        (folder/'model.config').write_text(f'<model><name>hnuter_r3_{name}</name><version>1</version><sdf version="1.11">model.sdf</sdf></model>\n')
        loaded=Assembly(folder/'model.sdf').state({})
        assert physical_inertia(loaded['j'])
        assert np.allclose(loaded['j']-np.diag(np.diag(loaded['j'])),ref['j']-np.diag(np.diag(ref['j'])),atol=1e-9)
        out[name]=dict(payload_offset_flu_m=offset.tolist(),box_size_m=size,**{k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in loaded.items()})
    (ROOT/'generated/nominal_sensor_calibration.json').write_text(json.dumps(nominal.geometry,indent=2)+'\n')
    out['nominal']={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in ref.items()}
    (ROOT/'generated/truth_for_evaluation_only.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps(out,indent=2))


if __name__=='__main__':create_models()
