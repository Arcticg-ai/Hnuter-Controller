"""Shared prospective JEPA/diagnostic acceleration port. No parameter injection."""
import numpy as np

def apply_residual(original,residual,fresh=True):
 a=np.asarray(original,dtype=np.float64);r=np.asarray(residual,dtype=np.float64)
 if a.shape!=(3,) or r.shape!=(3,) or not np.isfinite(a).all():raise ValueError('invalid command')
 valid=bool(fresh and np.isfinite(r).all() and np.linalg.norm(r)<=.300000001)
 applied=r.copy() if valid else np.zeros(3)
 # Explicit zero bypass preserves IEEE signed-zero bits exactly.
 final=a.copy() if not np.any(applied) else a+applied
 return final,applied,valid

def force_profile(t,schedule):
 start,end=schedule['start'],schedule['end']
 return schedule['force_y']*np.sin(np.pi*(t-start)/(end-start))**2 if start<t<end else 0.
