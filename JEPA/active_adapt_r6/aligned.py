"""Receipt-ordered, delayed common-clock states and completed-action intervals.

Never extrapolates states; action clocks are publication clocks. This is a delayed
identification stream, NOT a claim of prospective actuator-action availability.
"""
from collections import deque
import numpy as np
from physics import observer, derivatives
KEYS=('position','attitude','rate','motor','servo')
DT=.1

def old_events(rows):
    seen={}
    for r in rows:
        values=dict(position=r['velocity_enu'],attitude=np.array(r['R_flu_enu']).ravel().tolist(),rate=r['omega_flu'],motor=r['motors_normalized'],servo=r['servos_normalized'])
        events=[]
        for key in KEYS:
            s=r['stream_timing'][key]
            if seen.get(key)==s['source_publish_us']:continue
            seen[key]=s['source_publish_us']
            events.append(dict(key=key,t_us=s['source_publish_us'] if key in ('motor','servo') else s['source_sample_us'],receive_ns=s['receive_monotonic_ns'],value=values[key],elapsed=r['elapsed'],phase=r['phase']))
        yield from sorted(events,key=lambda e:e['receive_ns'])

class Aligner:
    def __init__(self):
        self.buf={k:deque(maxlen=400) for k in KEYS};self.next_us=None;self.q=np.zeros(4);self.speed=np.zeros(5);self.deriv=np.zeros(6);self.prev=None;self.history=deque(maxlen=16);self.rejects=0;self.last_receive=0
    def at(self,key,t,hold=False):
        a=list(self.buf[key]);ts=np.array([x['t_us'] for x in a]);i=np.searchsorted(ts,t,side='right')-1
        if i<0:raise ValueError('no_left_bracket')
        age=(t-ts[i])*1e-6
        if age>.12:raise ValueError('stale')
        left=np.array(a[i]['value'],float)
        if hold or ts[i]==t:return left,age
        if i+1>=len(a) or ts[i+1]-ts[i]>120000:raise ValueError('no_right_bracket')
        f=(t-ts[i])/(ts[i+1]-ts[i]);v=left*(1-f)+np.array(a[i+1]['value'])*f
        if key=='attitude':
            u,_,vt=np.linalg.svd(v.reshape(3,3));v=(u@np.diag([1,1,np.linalg.det(u@vt)])@vt).ravel()
        return v,age
    def action_mean(self,t0,t1):
        result=[]
        for key in ('motor','servo'):
            ticks=[t0]+[x['t_us'] for x in self.buf[key] if t0<x['t_us']<t1]+[t1]
            value=sum(self.at(key,a,True)[0]*(b-a) for a,b in zip(ticks,ticks[1:]))/(t1-t0)
            result.extend(value)
        return np.array(result)
    def feed(self,e):
        key=e['key'];b=self.buf[key]
        if b and e['t_us']<=b[-1]['t_us']:return []
        b.append(e);self.last_receive=max(self.last_receive,e['receive_ns'])
        if not all(self.buf.values()):return []
        if self.next_us is None:self.next_us=((max(v[0]['t_us'] for v in self.buf.values())+99999)//100000)*100000
        out=[]
        while min(v[-1]['t_us'] for v in self.buf.values())>=self.next_us:
            t=self.next_us;self.next_us+=100000
            try:
                v=self.at('position',t)[0];w=self.at('rate',t)[0];rot=self.at('attitude',t)[0].reshape(3,3);um,am=self.at('motor',t,True);us,ass=self.at('servo',t,True);u=np.r_[um,us];state=np.r_[v,w]
                acts=np.stack([self.action_mean(t-100000,t-50000),self.action_mean(t-50000,t)]) if self.prev is not None else np.stack([u,u])
                for action in acts:self.q,self.speed=observer(self.q,self.speed,action,.05)
                if self.prev is not None:self.deriv+=(1-np.exp(-DT/.3))*(np.clip((state-self.prev['state'])/DT,[-5]*3+[-20]*3,[5]*3+[20]*3)-self.deriv)
                feat=np.r_[state,rot.ravel(),u,self.q,self.speed/1000,self.deriv,am,ass,float(np.any(abs(um)>=.95)),float(np.any(abs(us)>=.95)),DT]
                assert len(feat)==44 and np.isfinite(feat).all()
                row=dict(t_us=t,state=state,x=feat,actions_from_previous=acts,receive_ns=self.last_receive,elapsed=float(e['elapsed'])-(e['t_us']-t)*1e-6,phase=e['phase'])
                self.history.append(feat.copy());row['history']=np.stack(self.history);out.append(row);self.prev=row
            except (ValueError,AssertionError):self.prev=None;self.history.clear();self.rejects+=1
        return out

def rollout(x,actions):
    v=x[:3].copy();w=x[3:6].copy();rot=x[6:15].reshape(3,3).copy();q=x[24:28].copy();speed=x[28:33].copy()*1000;states=[]
    for action in actions:
        for _ in range(5):
            q,speed=observer(q,speed,action,.01);a,alpha=derivatives(rot,v,w,q,speed);v+=.01*a;w+=.01*alpha
            z=w*.01;theta=np.linalg.norm(z)
            if theta>1e-12:
                xx,yy,zz=z/theta;s=np.array([[0,-zz,yy],[zz,0,-xx],[-yy,xx,0]]);rot=rot@(np.eye(3)+np.sin(theta)*s+(1-np.cos(theta))*(s@s))
        states.append(np.r_[v,w])
    return np.array(states)[[1,3,7]] if len(actions)==8 else np.array(states)[-1]

def basis(rate):return np.r_[rate[2]+9.81,rate[3:6]]
