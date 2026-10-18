"""Causal integer-nanosecond readiness. Does not relax the 20 deg/s bound."""
import math

def joint_readiness(raw,cutoff_us,names=('rj2','lj2','rj1','lj1')):
 result=dict(ok=False,cutoff_us=int(cutoff_us),sensor_t_s=None,age_ns=None,max_rate_deg_s=None,reason='missing')
 if raw is None:return result
 stamp,joints=raw
 if not math.isfinite(stamp):result['reason']='nonfinite_stamp';return result
 age=int(cutoff_us)*1000-int(round(stamp*1e9));result.update(sensor_t_s=stamp,age_ns=age)
 if age<0:result['reason']='future';return result
 if age>=25_000_000:result['reason']='stale';return result
 if any(n not in joints for n in names):return result
 rates=[abs(joints[n][1]) for n in names]
 if not all(math.isfinite(x) for x in rates):result['reason']='nonfinite_rate';return result
 result['max_rate_deg_s']=math.degrees(max(rates));result['ok']=max(rates)<=math.radians(20);result['reason']='ready' if result['ok'] else 'rate';return result
