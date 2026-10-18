"""One canonical model: current 4.5 kg SITL assembly, not hardware tuning."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parent
WS=Path('/opt/ws') if Path('/opt/ws').is_dir() else ROOT/'vendor/ws'
PX4=Path('/opt/px4') if Path('/opt/px4').is_dir() else ROOT/'vendor/px4'
for p in [WS/'JEPA/revision2',WS/'JEPA/revision3']:
    sys.path.insert(0,str(p))
from model_audit import audit

SOURCE=PX4/'Tools/simulation/gz/models/hnuter/model.sdf'
D=np.diag([1.,-1.,-1.])

def write(path,obj):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')

def properties(model):
    a=audit(model)
    return dict(mass=a['total_mass_kg'],com_frd=(D@a['com_model_m']).tolist(),
                inertia_frd=(D@np.array(a['inertia_com_model_kg_m2'])@D).tolist(),
                principal_inertias=a['principal_inertias_kg_m2'])

def add_payload(original,mass,offset,size):
    model=copy.deepcopy(original)
    if mass==0:return model
    nominal=properties(original)
    center=D@np.array(nominal['com_frd'])+np.array(offset)
    link=ET.SubElement(model,'link',name='adaptation_payload')
    ET.SubElement(link,'pose').text=' '.join(map(str,center))+' 0 0 0'
    inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'mass').text=str(mass)
    inertia=ET.SubElement(inertial,'inertia');a,b,c=size
    diag=mass/12*np.array([b*b+c*c,a*a+c*c,a*a+b*b])
    for key,value in zip(('ixx','iyy','izz','ixy','ixz','iyz'),[*diag,0,0,0]):
        ET.SubElement(inertia,key).text=str(value)
    joint=ET.SubElement(model,'joint',name='adaptation_payload_mount',type='fixed')
    ET.SubElement(joint,'parent').text='base_link';ET.SubElement(joint,'child').text='adaptation_payload'
    return model

def consumer_target(nominal,loaded):
    dr=np.array(loaded['com_frd'])-nominal['com_frd']
    dj=np.array(loaded['inertia_frd'])-nominal['inertia_frd']
    if abs(dr[2])>1e-7 or np.linalg.norm(dr)>.02 or np.linalg.norm(dj-np.diag(np.diag(dj)))>1e-7:
        raise ValueError('Payload outside preserved consumer: horizontal COM and diagonal inertia only')
    if np.any(np.diag(dj)<-1e-8) or np.any(np.diag(dj)>.03):raise ValueError('Inertia increment outside contract')
    return [loaded['mass'],*dr[:2],*np.maximum(np.diag(dj),0)]

def prepare(out,config):
    out=Path(out)
    required={'sample_hz':20,'calibration_start_s':18,'calibration_end_s':34,'adaptation_start_s':35,'evaluation_start_s':60}
    if any(config.get(k)!=v for k,v in required.items()):
        raise ValueError('This package fixes sensor/timing conventions; changing them requires a new schema')
    if config['payload_mass_kg']!=[0.0,0.9] or config['payload_horizontal_offset_m']>.04:
        raise ValueError('Pilot envelope is fixed at 0..0.9 kg and <=4 cm payload displacement')
    if (out/'cases.json').exists():raise ValueError('Refuse to overwrite an existing experiment')
    original=ET.parse(SOURCE).getroot().find('model');nominal=properties(original)
    assert abs(nominal['mass']-4.5)<1e-5
    original_hash=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    rng=np.random.default_rng(config['case_seed']);cases=[]
    for split,count in [('train',config['train_cases']),('validation',config['validation_cases']),('test',config['test_cases'])]:
        for i in range(count):
            mass=float(rng.uniform(*config['payload_mass_kg']))
            offset=np.zeros(3)
            size=rng.uniform(*np.array(config['payload_box_size_m']))
            family=['mass','com_x','com_y','inertia'][i%4]
            # Spatial consumer was audited near 5.4 kg. Keep spatial cases
            # nearby, while mass-only cases explore the broader 0..0.9 kg range.
            if family!='mass':mass=float(rng.uniform(.75,.9))
            if family.startswith('com_'):offset[0 if family=='com_x' else 1]=rng.uniform(-1,1)*config['payload_horizontal_offset_m']
            if family!='inertia':size=np.array(config['payload_box_size_m'][0])
            # Include a nominal test; do not train only on loaded aircraft.
            if split=='test' and i==0:mass=0.;family='nominal'
            model=add_payload(original,mass,offset,size);loaded=properties(model)
            target=consumer_target(nominal,loaded)
            case_id=f'{split}_{i:03d}_{family}';model.set('name','hnuter_jepa')
            for uri in model.findall('.//mesh/uri'):
                if not uri.text.startswith(('model://','file://','/')):uri.text=str(SOURCE.parent/uri.text)
            # Disable wind consistently; uncertainty here is attached mass geometry.
            for link in model.findall('link'):
                e=link.find('enable_wind')
                if e is None:e=ET.SubElement(link,'enable_wind')
                e.text='false'
            path=out/'models'/case_id/'model.sdf';path.parent.mkdir(parents=True)
            root=ET.Element('sdf',version='1.11');root.append(model);ET.indent(root)
            ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
            cases.append(dict(id=case_id,split=split,family=family,payload_kg=mass,
                              offset_flu_m=offset.tolist(),box_size_m=size.tolist(),target=target,
                              truth=loaded,model=str(path.resolve()),sdf_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    write(out/'nominal.json',dict(**nominal,sdf_sha256=original_hash,
         convention='FRD properties at zero joint pose; runtime retains articulated Gazebo dynamics',
         discrepancies={'hardware_mass_kg':4.8,'chosen_sitl_mass_kg':4.5,
                        'hardware_tail_arm_m':.72,'chosen_sitl_tail_arm_m':.664,
                        'hardware_secondary_gear':2.,'gazebo_secondary_gear':1.},
         unchanged_control=True,unchanged_allocator=True,
         limitations=['Payload limits are experiment settings, not measured hardware limits',
                     'SITL main motor rise/fall 1/2 ms retained, not an identified hardware actuator model',
                     'Existing spatial consumer has a narrow force/moment envelope; activation must be measured']))
    write(out/'cases.json',cases);write(out/'config.json',config)
    return cases
