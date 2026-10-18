"""Versioned experiment-start fix; activated only in the yaw_init_fix batch.
Latch the already-received attitude at the existing physical-readiness release.
No estimator, controller gains, integral reset, frame realignment or truth input.
"""
import math,time
import numpy as np

def capture_ready_yaw(node,readiness):
    if readiness['action']!='release' or node.armed:raise ValueError('only existing ready-unarmed release may capture reference')
    matrix=np.asarray(node.R,dtype=float)
    if matrix.shape!=(3,3) or not np.isfinite(matrix).all():raise ValueError('invalid received attitude')
    received=dict(node.receipts['attitude']);capture_ns=time.monotonic_ns()
    if received['receive_monotonic_ns']>capture_ns:raise ValueError('attitude not yet received')
    yaw=math.atan2(float(matrix[1,0]),float(matrix[0,0]))
    audit=dict(old_initial_yaw_rad=float(node.initial_yaw),old_manual_yaw_rad=float(node.manual_des_yaw),captured_yaw_rad=yaw,
               source_attitude=received,release_physical_s=float(readiness['physical_s']),R_flu_enu=matrix.tolist(),capture_monotonic_ns=capture_ns,first_attitude=getattr(node,"yaw_first_attitude",None),truth_used=False,integrals_modified=False)
    node.initial_yaw=yaw;node.manual_des_yaw=yaw;node.target_attitude[2]=yaw;node._yaw_initialized=True
    return audit
