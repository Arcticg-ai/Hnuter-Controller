"""Independent Gazebo sensor telemetry, preserving sensor timestamps.

Only IMU and joint sensor messages. No ECM, model inertia, requested wrench or
groundtruth topics. Buffered logging is an observer, with no command publishers.
"""
import csv
import json
import os
import signal
import threading
import time
from pathlib import Path
from gz.transport13 import Node
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.model_pb2 import Model


def main():
    run=Path(os.environ['JEPA_RUN_DIR'])
    manifest=json.loads((run/'launch.json').read_text())
    assert manifest['kind']=='owned_local_sitl'
    assert manifest['GZ_PARTITION']==os.environ['GZ_PARTITION']
    lock=threading.Lock(); stop=threading.Event(); counts={'imu':0,'joints':0}
    files={name:(run/(name+'_native.csv')).open('w') for name in counts}
    writers={}; node=Node()
    def write(kind,row):
        with lock:
            if kind not in writers:
                writers[kind]=csv.DictWriter(files[kind],fieldnames=list(row));writers[kind].writeheader()
            writers[kind].writerow(row);counts[kind]+=1
            if counts[kind]%250==0:files[kind].flush()
    def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
    def imu(msg):
        w=msg.angular_velocity;a=msg.linear_acceleration
        write('imu',dict(t=stamp(msg),wall=time.monotonic(),wx=w.x,wy=w.y,wz=w.z,ax=a.x,ay=a.y,az=a.z))
    def joints(msg):
        write('joints',dict(t=stamp(msg),wall=time.monotonic(),**{j.name+'_'+k:v for j in msg.joint for k,v in [('q',j.axis1.position),('qd',j.axis1.velocity)]}))
    model='hnuter_jepa_'+str(manifest['experiment']['instance'])
    topic='/world/default/model/'+model+'/link/base_link/sensor/imu_sensor/imu'
    assert node.subscribe(IMU,topic,imu)
    assert node.subscribe(Model,'/jepa_joint_sensors',joints)
    signal.signal(signal.SIGTERM,lambda *_:stop.set())
    signal.signal(signal.SIGINT,lambda *_:stop.set())
    (run/'native_sensor_topics.json').write_text(json.dumps({'imu':topic,'joints':'/jepa_joint_sensors'})+'\n')
    while not stop.wait(.25):pass
    node.unsubscribe(topic);node.unsubscribe('/jepa_joint_sensors')
    with lock:
        for f in files.values():f.close()
    (run/'native_sensor_counts.json').write_text(json.dumps(counts,indent=2)+'\n')


if __name__=='__main__':main()
