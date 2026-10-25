"""Compare SDF forward kinematics with live Gazebo link poses (not forces)."""
import argparse
import json
from pathlib import Path
import sys
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,'/home/hnuter/px4_ws_ros2_consolidated')
from controllers.experiments.sdf_wrench_model import SDFWrenchModel,pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
p.add_argument('--seconds',type=float,default=45.);a=p.parse_args()
sdf='/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl/Tools/simulation/gz/models/hnuter/model.sdf'
model=SDFWrenchModel(sdf);root=ET.parse(sdf).getroot().find('model')
zeros=[pose(root.find(f"joint[@name='{j}']"))[1] for j in model.TILTS]
pairs=[('base_link','l2'),('l2','l1'),('base_link','r2'),('r2','r1')]
rows=[];lock=threading.Lock()


def callback(message):
    poses={p.name.rsplit('::',1)[-1]:p for p in message.pose}
    if not all(n in poses for n,_,_ in model.inertials):return
    rotations={};positions={}
    for name,p in poses.items():
        rotations[name]=Rotation.from_quat([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]).as_matrix()
        positions[name]=np.array([p.position.x,p.position.y,p.position.z])
    angles=[]
    for (pa,ch),zero in zip(pairs,zeros):
        delta=rotations[pa].T@rotations[ch]@zero.T
        angles.append(np.arctan2(delta[0,2],delta[0,0]))
    com=sum(m*(positions[n]+rotations[n]@offset) for n,m,offset in model.inertials)/model.mass
    rb=rotations['base_link'].T
    actual_p=np.array([rb@(positions[n]-com) for n in model.ROTORS])
    actual_d=np.array([rb@rotations[n][:,2] for n in model.ROTORS])
    predicted_p,predicted_d,_,_=model.geometry(angles)
    row=dict(angles_rad=angles,position_error_max_m=float(np.linalg.norm(predicted_p-actual_p,axis=1).max()),
             axis_vector_error_max=float(np.linalg.norm(predicted_d-actual_d,axis=1).max()))
    with lock:rows.append(row)


node=Node();node.subscribe(Pose_V,'/world/default/dynamic_pose/info',callback)
time.sleep(a.seconds)
with lock:data=list(rows)
summary=dict(samples=len(data),scope='kinematic consistency only; not force or stability validation')
if data:
    for k in ['position_error_max_m','axis_vector_error_max']:summary[k]=max(r[k] for r in data)
    q=np.array([r['angles_rad'] for r in data]);summary['joint_angle_min_rad']=q.min(axis=0).tolist();summary['joint_angle_max_rad']=q.max(axis=0).tolist()
a.output.write_text(json.dumps(dict(summary=summary,samples=data),indent=2)+'\n')
print(json.dumps(summary,indent=2))
