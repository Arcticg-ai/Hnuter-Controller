# Joint-PID Predictive DRCDA Upgrade

Date: 2026-10-02

## Scope

This upgrade keeps the existing measured-state correction, 0.10 s horizon,
move blocking, and two-iteration Gauss-Newton solve. It changes only the
actuator predictor and the Full DRCDA objective. The position and attitude
outer loops, trajectory, limits, anti-windup path, and Direct baseline are not
governed or modified by reachable-wrench feedback.

No pure-delay model is used anywhere in this validation.

## 1. Gazebo joint-PID model

The four joints were excited independently in a zero-gravity Gazebo world with
positive and negative 0.4 rad steps at 250 Hz. Link-relative poses, rather than
command echoes, supplied the measured joint angles. The fitted model is

```text
tau_i * dq_i/dt + q_i = K_i * u_i
```

in logical order `alpha_L, beta_L, alpha_R, beta_R`:

| Direction | K | tau (s) |
|---|---|---|
| positive | 0.9939, 0.9018, 0.9940, 0.9152 | 0.1384, 0.0891, 0.1408, 0.0920 |
| negative | 0.9940, 0.9119, 0.9940, 0.9042 | 0.1396, 0.0874, 0.1422, 0.0868 |

The fitted constants and mapping are stored in
`joint_pid_identification.json`. The old 1.404/0.705 historical command map is
not applied to Gazebo a second time. The allocation variable is now the actual
JointPositionController input, while the measured K belongs only to the
predictor.

At each allocation cycle the measured joint angles overwrite the predictor
initial state. The discrete propagation and sensitivity are

```text
a_i = 1 - exp(-Delta_p / tau_i)
q_{i,j+1} = q_{i,j} + a_i (K_i u_i - q_{i,j})
s_{i,j+1} = (1-a_i) s_{i,j} + a_i K_i
```

followed by the existing angle and rate limits. Active clipping sets or reduces
the corresponding sensitivity through the active branch.

## 2. Two-point future-wrench objective

Full DRCDA predicts the same move-blocked command at `H1=0.05 s` and
`H2=0.10 s`. The objective is

```text
J = 0.5 ||W (w_hat_H1 - w_d(k+H1))||^2
  + 1.0 ||W (w_hat_H2 - w_d(k+H2))||^2
  + ||u-u_prev||^2_Rdelta + ||u-u_pref||^2_Rpref.
```

Future minimum-jerk position, velocity, and acceleration are evaluated through
the same position-control law. The nominal vehicle state follows the reference
increments over H, preserving the currently measured tracking error. This
avoids incorrectly treating all future position and velocity change as a new
feedback error. Integral and attitude states are read without mutation.

When no explicit future trajectory is active, Full still uses the two-point
objective with the current wrench held at H1/H2. No-horizon retains a one-step
prediction and receives no future-wrench samples.

## 3. Compared methods

| Method | Allocation behavior |
|---|---|
| Direct | Existing analytic instantaneous allocation |
| Full predictive | Measured-state joint-PID prediction plus H1/H2 future wrench |
| No horizon | Same identified actuator model and constraints, one prediction step, no future wrench |

All methods use the same A-B-A-C minimum-jerk trajectory, fixed level attitude,
outer-loop gains, motor limits, gate geometry, and no-delay firmware commit
`49b60a5d`.

## 4. Results

Strict pass limits remain 0.75 m peak position error, 8 deg SO(3) attitude
error, 5 deg roll/pitch, 0.50 m minimum altitude, positive gate clearance, all
targets reached, no disarm, and no safety cutoff.

| T (s) | Method | Result | Peak pos (m) | Peak attitude (deg) | Peak yaw rate (rad/s) |
|---:|---|---|---:|---:|---:|
| 1.5 | Direct | PASS | 0.237 | 5.27 | 0.406 |
| 1.5 | Full | PASS | 0.171 | 2.78 | 0.061 |
| 1.5 | No horizon | FAIL | 0.185 | 8.24 | 0.169 |
| 1.2 | Direct | PASS | 0.349 | 7.60 | 0.594 |
| 1.2 | Full | PASS | 0.201 | 2.84 | 0.115 |
| 1.2 | No horizon | FAIL | 13.137 | 116.92 | 3.496 |
| 1.0 | Direct | FAIL | 0.908 | 176.90 | 5.082 |
| 1.0 | Full | FAIL | 0.311 | 9.86 | 0.446 |
| 1.0 | No horizon | FAIL | 0.956 | 11.88 | 0.936 |
| 0.8 | Direct | FAIL | 0.780 | 63.33 | 3.246 |
| 0.8 | Full | FAIL | 0.534 | 16.85 | 0.913 |
| 0.8 | No horizon | FAIL | 5.325 | 178.94 | 5.958 |

At T=1.2 s, Full reduces peak position error by 42%, peak attitude error by
63%, and peak yaw rate by 81% relative to Direct. Its aligned future Fy
prediction RMSE is 0.74 N at H1 and 1.98 N at H2. The mean secondary-joint
command decreases by 0.078 rad in the final 0.12 s before reversal, showing the
requested pre-reversal mechanism rather than only a small aggregate RMSE gain.

T=1.0 s is not claimed as a pass. Full avoids the Direct flip and completes the
target sequence, but violates the attitude limits. Near this boundary the
outer loop requests a dynamically unreachable lateral wrench. Because this
stage intentionally does not add a reachable-wrench governor, the allocator
cannot make that reference feasible by itself.

## 5. Figures and data

- `figures/performance_sweep.png`: four-speed peak metric comparison.
- `figures/t12_method_comparison.png`: T=1.2 trajectory, attitude, and beta command.
- `figures/full_t12_future_wrench.png`: current/future reference, estimated, and predicted Fy.
- `data/final_metrics.csv`: compact final metric table.
- `data/T_1p2_*_diagnostic.csv`: 50 Hz mechanism logs for all three methods.
- `data/*_result.json`: per-case pass/fail evidence and thresholds.

## 6. Code locations

- `controllers/common/hnuter_drcda.py`: identified model, multi-horizon state and sensitivity propagation, and multi-point Gauss-Newton residual.
- `controllers/simulation/hnuter_external_direct_controller_debug.py`: non-mutating future outer-loop wrench evaluation.
- `controllers/simulation/hnuter_external_direct_drcda.py`: Full-only future-reference hook and H1/H2 diagnostics.
- `controllers/experiments/drcda_closed_loop/controller.py`: measured-state reset and direct Gazebo joint command mapping.
- `controllers/experiments/fixed_attitude_switching/controller.py`: future minimum-jerk samples and experiment-only 50 Hz diagnostics.
- `tools/experiments/run_fixed_attitude_switching.py`: sweep, aligned prediction error, and pre-reversal metrics.

## Conclusion

The change fixes the two original structural mismatches: the predictor now
represents the actual Gazebo joint-PID response, and the finite-horizon state is
matched to a finite-horizon wrench reference. The strict validated operating
point improves from a marginal Direct T=1.2 pass to a substantially lower-error
Full T=1.2 pass. Faster references remain an outer-loop feasibility problem and
are deliberately reported as failures rather than hidden by threshold changes.
