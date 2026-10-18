"""Deployment-only nominal approximate dynamics at fixed PX4 origin. No case/SDF access."""
import json
from pathlib import Path
import numpy as np
C=json.loads(Path(__file__).with_name('nominal_physics.json').read_text());B=np.array(C['B']);DB=np.array(C['dB']);DDB=np.array(C['ddB_diag']);M=C['mass'];J=np.array(C['J_flu']);JI=np.linalg.inv(J);COM=np.array(C['com_flu']);K=np.array(C['motor_k'])
SCALE_Q=np.deg2rad([185,185,180,180])
def motor_target(u):return np.r_[10+990*np.maximum(u[:4],0),1000*u[4]]
def observer(q,speed,u,dt):
 q=q+np.clip((u[5:]*SCALE_Q-q)*(1-np.exp(-dt/np.array(C['servo_tau_s']))),-10*dt,10*dt)
 speed=speed+(motor_target(u)-speed)*(1-np.exp(-dt/np.array(C['motor_tau_s'])))
 return q,speed

def derivatives(R,v,w,q,speed):
 f=K*speed*abs(speed);map_=B+np.einsum('q,qij->ij',q,DB)+.5*np.einsum('q,qij->ij',q*q,DDB)
 wrench=map_@f;force=wrench[:3];torque=wrench[3:]-np.cross(COM,force)
 alpha=JI@(torque-np.cross(w,J@w))
 acom=R@force/M-[0,0,9.81]-.025*v
 aorigin=acom-R@(np.cross(alpha,COM)+np.cross(w,np.cross(w,COM)))
 return aorigin,alpha

def predict(row,q,speed,u,dtvec):
 R=np.asarray(row['R_flu_enu']);v=np.array(row['velocity_enu']);w=np.array(row['omega_flu']);v0=v.copy();w0=w.copy()
 # Hold-current-action nominal rollout. Approximate common state clock; per-channel
 # elapsed corrections retain actual target/source timestamps in the residual loss.
 duration=float(max(dtvec[0],dtvec[3]));n=max(1,int(np.ceil(duration/.02)));h=duration/n
 for _ in range(n):
  q,speed=observer(q,speed,u,h);a,alpha=derivatives(R,v,w,q,speed)
  v+=h*a;w+=h*alpha
  angle=w*h;theta=np.linalg.norm(angle)
  if theta>1e-12:
   axis=angle/theta;x,y,z=axis;S=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
   R=R@(np.eye(3)+np.sin(theta)*S+(1-np.cos(theta))*(S@S))
 return np.r_[v0+(v-v0)*dtvec[:3]/duration,w0+(w-w0)*dtvec[3:]/duration]
