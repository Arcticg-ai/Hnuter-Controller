# No-delay maneuver experiments

Run the complete comparison and ablation matrix against the no-delay Hnuter
firmware model:

```bash
cd ~/px4_ws_ros2
python3 tools/experiments/run_no_delay_maneuver_experiments.py \
  --output hnuter_logs/no_delay_maneuver_validation_YYYYMMDD
python3 tools/experiments/analyze_no_delay_maneuver_experiments.py \
  hnuter_logs/no_delay_maneuver_validation_YYYYMMDD
```

The runner starts and stops PX4 SITL, Gazebo, and Micro XRCE-DDS Agent for each
case. It verifies that the selected `hnuter/model.sdf` does not contain the
delayed actuator plugin, records the firmware commit, applies one common
closed-loop tuning file, and archives controller CSV, ULog, and console logs.

The default matrix contains:

- 7 s aggressive 3D maneuver using the fixed
  `identified_gain_no_delay` servo model. The model retains the identified
  directional static gains but removes the unsupported pure delay and old
  first-order-lag fit.
- Sequential `+80 deg` roll, `+180 deg` pitch, `-80 deg` roll, and `-180 deg`
  pitch position-hold test. Each move takes 15 s, holds the peak for 5 s, and
  then waits level for 10 s before the next axis.
- Original direct, basic differential allocation, full DRCDA, no prediction
  horizon, and no actuator-rate limits.

The analyzer reports timed completion separately from valid tracking. A case
that finishes its reference timer after contact or loss of tracking is not
classified as successful. Attitude commands and feedback errors use rotation
matrices/SO(3). Euler angles are diagnostic only; figures split the four
maneuvers and map displayed pitch to the requested half-turn branch to avoid
misleading 180-degree jumps.

Figures are saved as vector PDF and 600 dpi PNG using a compact journal style.
The experiment output remains under `hnuter_logs/` and is intentionally not
tracked by Git.
