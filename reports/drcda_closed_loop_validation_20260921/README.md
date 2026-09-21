# Measured-state DRCDA and reachable-wrench contract

## Scope

This is a SITL-only experimental controller. It does not modify the hardware
controller or the no-delay PX4 firmware. The firmware used here is
`/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl` at `49b60a5d`.
No pure-delay actuator model was used.

Run with ROS Jazzy and the local `px4_msgs` workspace sourced:

```bash
source /opt/ros/jazzy/setup.bash
source /home/hnuter/px4_ws_ros2/install/setup.bash
python3 -m controllers.experiments.drcda_closed_loop.controller
```

The controller requires Gazebo `/world/default/dynamic_pose/info`. It
calibrates link-relative zero orientations while disarmed, extracts four
physical joint angles, and refuses prearm when that signal is missing or
stale. Joint rotation is recovered about the parent-link +Y axis after
accounting for each joint's nonidentity zero orientation. PX4
`actuator_servos` and `actuator_motors` are command echoes, not physical
measurements, and are not used as actuator-state feedback.

The base DRCDA solver is retained. Its command box includes servo and thrust
limits plus per-step command slew. On each solve, the interface checks that
the chosen 9D command is in that box, recomputes a nonlinear predicted
6D wrench, and returns the predicted wrench rate
`(W_future - W_estimated) / horizon`. This command is a witness for
**model** reachability. The controller uses the feasible-minus-requested
wrench residual for position anti-windup. Predicted yaw rate starts from
PX4's measured body yaw rate. The four measured joint angles replace the
predictor's servo state; the physical servo *target* is kept separate and is
sent to Gazebo. No artificial pure delay or first-order lag was added.

This is not a proof that the physical aircraft can realize the returned
wrench: motor force remains model-predicted because this SITL setup does not
publish measured rotor RPM. The joint PID can also lag the target.

## Results

The strict runner requires the aircraft to remain armed after the
post-trajectory observation, no safety cutoff, altitude at least 0.3 m
during the trajectory, yaw-rate peak at most 1.0 rad/s, and horizontal
tracking RMS at most 0.8 m. A completion message alone is not a pass.

Two consecutive 7 s 3D Lissajous trajectories, each with X/Y/Z amplitudes
1.2/0.8/0.30 m, followed by 12 s hover:

| Metric | v1 | Measured-state contract |
| --- | ---: | ---: |
| Both trajectories and post-hold | pass | pass |
| Trajectory horizontal RMS error | 0.453 m | 0.403 m |
| Trajectory yaw-rate peak | 0.800 rad/s | 0.668 rad/s |
| First handover, 0-4 s yaw-rate RMS | 0.229 rad/s | 0.134 rad/s |
| Second handover, 0-2 s yaw-rate RMS | 0.292 rad/s | 0.161 rad/s |
| Second handover, 2-6 s yaw-rate RMS | 0.0788 rad/s | 0.0039 rad/s |
| Second handover, 6-12 s yaw-rate RMS | 0.0068 rad/s | 0.0056 rad/s |

Joint feedback was available in every logged trajectory sample, with a
maximum age of 0.017 s. These are single-run SITL comparisons, not flight
statistics or evidence of significance. The peak transient is reduced but
not eliminated.

See [second_handover.png](second_handover.png) and the selected diagnostic
CSVs plus provenance manifests under [data](data). The fast-trajectory v1
manifest predates the strict yaw/tracking checks and labels the case
`complete` solely from the completion message; its CSV fails the current
criteria.

A 5 s trajectory with reduced amplitudes 0.7/0.5/0.20 m remains **failed**:
v1 had trajectory yaw-rate peak 2.60 rad/s and horizontal RMS 1.24 m;
the corrected measured-state version had peak 1.55 rad/s and RMS 1.08 m.
Both violate the strict criteria. An attempted 2.8 rad/s predictive servo
rate cap worsened both scenarios and was removed; that test showed why the
predictor and actual servo target must be made dynamically consistent
before a rate model can be trusted. Earlier pre-allocation wrench clipping
also caused instability and was removed.

The present contract constrains and certifies the *allocated* command; it
does not time-scale an infeasible high-level trajectory. In the failed 5 s
case, yaw command reached the 5 N m limit and the predicted yaw-wrench
residual had 1.78 N m RMS. The next structural step is to pass directional
reachable wrench-rate limits to a reference governor that time-parameterizes
the path, with velocity and acceleration transformed consistently. It also
requires identifying the Gazebo joint PID response from physical joint
state, rather than imposing a rate cap only inside the predictor.

## Reproduction

```bash
source /opt/ros/jazzy/setup.bash
source /home/hnuter/px4_ws_ros2/install/setup.bash
python3 tools/experiments/run_no_delay_maneuver_experiments.py \
  --firmware /home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl \
  --output /tmp/drcda_closed_loop_recheck \
  --scenario nominal_repeat \
  --method drcda_v1 --method drcda_closed_loop
```

Do not run both controllers simultaneously. The runner starts and stops
PX4/Gazebo and the DDS Agent for each case. The plot uses only logged
controller diagnostics; raw ULogs from these runs remain under the
temporary run directories and were not committed.
