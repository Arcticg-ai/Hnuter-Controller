"""Offline membership check, never a causal sequence or online lookup."""
import numpy as np

def match_payloads(commands, stamps, payloads, mask):
    groups={}
    for c in commands:
        groups.setdefault(int(c['px4_message_timestamp_us']),[]).append(c['actual_acceleration_ned'])
    exact=np.array([int(s) in groups for s in stamps])
    errors=[float(np.max(abs(np.asarray(payloads[i])-np.asarray(groups[int(stamps[i])])),axis=1).min())
            for i in np.where(np.asarray(mask)&exact)[0]]
    ambiguous=sum(len(set(tuple(v) for v in vals))>1 for vals in groups.values())
    return exact,errors,ambiguous
