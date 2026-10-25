#!/usr/bin/env python3
"""Fit a zero-delay joint response from command/measurement CSVs.

This is an exploratory closed-loop fit, not independent actuator identification.
Use new SITL runs to validate the exported profile before deployment.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares


def fit(paths):
    datasets = []
    for path in paths:
        with path.open() as stream:
            datasets.append([row for row in csv.DictReader(stream)
                if row.get('switch_phase') in {'settle_a', 'a_to_b', 'dwell_b',
                    'b_to_a', 'dwell_a', 'a_to_c'}])
    estimates = []
    for joint in ('a_l','b_l','a_r','b_r'):
        batches = []
        for rows in datasets:
            t = np.array([float(row['time_s']) for row in rows])
            q = np.array([float(row[f'switch_measured_joint_{joint}_rad']) for row in rows])
            u = np.array([float(row[f'switch_commanded_joint_{joint}_rad']) for row in rows])
            dt = np.diff(t)
            valid = (dt>0) & (dt<.1) & np.isfinite(q[:-1]+q[1:]+u[:-1]+u[1:])
            batches.append((dt[valid], q[:-1][valid], q[1:][valid],
                            .5*(u[:-1][valid]+u[1:][valid])))

        def residual(x):
            gain,tau = x
            return np.concatenate([q+(1-np.exp(-dt/tau))*(gain*u-q)-qn
                                   for dt,q,qn,u in batches])
        result = least_squares(residual,[1.,.1],bounds=([.5,.005],[1.5,.4]))
        estimates.append({'joint':joint, 'gain':float(result.x[0]),
                          'tau_s':float(result.x[1]),
                          'one_step_rmse_rad':float(np.sqrt(np.mean(result.fun**2))),
                          'samples':len(result.fun)})
    return {'description':'Exploratory loaded closed-loop fit, zero pure delay; validate on fresh runs.',
            'servo_gain_positive':[row['gain'] for row in estimates],
            'servo_gain_negative':[row['gain'] for row in estimates],
            'servo_tau_positive_s':[row['tau_s'] for row in estimates],
            'servo_tau_negative_s':[row['tau_s'] for row in estimates],
            'physical_joint_velocity_limit_rad_s':10., 'pure_delay_s':0.,
            'fit':estimates,
            'training_sources':[{'path':str(path), 'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
                                for path in paths]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv',type=Path,action='append',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    profile = fit(args.csv)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(profile,indent=2)+'\n')
    print(json.dumps(profile,indent=2))


if __name__=='__main__':
    main()
