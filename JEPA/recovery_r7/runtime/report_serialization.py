"""Prospective report writer, validated offline only; not injected into frozen flights."""
from pathlib import Path
import json,math,os,tempfile
import numpy as np

def normalize(value):
 if isinstance(value,np.ndarray):return normalize(value.tolist())
 if isinstance(value,np.generic):
  item=value.item()
  if isinstance(item,np.generic):raise TypeError('Extended precision NumPy scalar is unsupported')
  return normalize(item)
 if value is None or isinstance(value,(str,bool,int)):return value
 if isinstance(value,float):
  if not math.isfinite(value):raise ValueError('Nonfinite report value')
  return value
 if isinstance(value,(list,tuple)):return [normalize(x) for x in value]
 if isinstance(value,dict):
  if any(not isinstance(k,str) for k in value):raise TypeError('Report keys must be strings')
  return {k:normalize(v) for k,v in value.items()}
 raise TypeError('Unsupported report type: '+type(value).__name__)

def dumps(value,indent=2,allow_nan=False):
 if allow_nan:raise ValueError("Nonfinite reports are forbidden")
 return json.dumps(normalize(value),indent=indent,allow_nan=False)

def atomic_write(path,value):
 path=Path(path);payload=dumps(value);temporary=None
 try:
  with tempfile.NamedTemporaryFile(mode='w',encoding='utf8',dir=path.parent,prefix=path.name+'.',suffix='.tmp',delete=False) as f:
   temporary=Path(f.name);f.write(payload);f.flush();os.fsync(f.fileno())
  os.replace(temporary,path);temporary=None
 finally:
  if temporary is not None:temporary.unlink(missing_ok=True)
