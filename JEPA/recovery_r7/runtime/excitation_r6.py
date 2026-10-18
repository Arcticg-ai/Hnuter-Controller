"""Bounded predetermined identification; no unvalidated information optimizer."""
import os,math,numpy as np
def pulse(s,length,amp,cycles):
    if not 0<s<length:return 0.,0.
    w=2*math.pi*cycles/length;e=math.sin(math.pi*s/length)**2;de=math.pi/length*math.sin(2*math.pi*s/length)
    return amp*e*math.sin(w*s),amp*(de*math.sin(w*s)+e*w*math.cos(w*s))
def reference(elapsed,mode=None):
    mode=mode or os.environ.get('JEPA_EXCITATION','active')
    if mode=='hover':return np.zeros(3),0.
    angles=np.zeros(3);rates=np.zeros(3);s=elapsed-48
    if 0<s<8:
        if mode=='active':
            for i in range(3):angles[i],rates[i]=pulse(s,8,math.radians(1.5)/math.sqrt(3),3+i)
        elif mode=='conventional':
            i=min(2,int(s/(8/3)));angles[i],rates[i]=pulse(s-i*8/3,8/3,math.radians(1.5),2)
        else:raise ValueError(mode)
    elif 58<elapsed<68:
        for i in range(3):angles[i],rates[i]=pulse(elapsed-58,10,math.radians(1.),4+i)
    return angles,float(rates[2])
