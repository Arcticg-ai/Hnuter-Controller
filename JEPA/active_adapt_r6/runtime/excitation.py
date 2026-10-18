import math,numpy as np
def reference(elapsed):
 s=elapsed-48.
 if not 0<s<20:return np.zeros(3),0.
 e=math.sin(math.pi*s/20)**2;de=math.pi/20*math.sin(2*math.pi*s/20);A=math.radians(1.5);w=2*math.pi/np.array([5.,7.,9.]);angles=A*e*np.sin(w*s);yawrate=A*(de*math.sin(w[2]*s)+e*w[2]*math.cos(w[2]*s));return angles,float(yawrate)
