"""Non-extrapolating maximum of already received PX4 publication timestamps."""
import math
class LatestCausalClock:
 def __init__(self):self.latest_us=0
 def update(self,values):
  values=list(values)
  if not values or any(isinstance(v,bool) or not math.isfinite(v) or int(v)!=v or v<0 for v in values):raise ValueError('invalid_received_timestamp')
  self.latest_us=max(self.latest_us,max(map(int,values)))
  return self.latest_us
