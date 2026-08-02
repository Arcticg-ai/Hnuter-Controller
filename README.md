# Hnuter Controller

ROS 2 offboard controllers for the Hnuter PX4/Gazebo setup.

## Main Files

- `hnuter_external_controller.py`: PX4 position-offboard controller with hover and trajectory modes.
- `hnuter_external_controller_px4_position.py`: preserved PX4 position-control baseline.
- `hnuter_external_direct_controller_debug.py`: direct actuator debug controller for checking motor/tilt command paths.
- `hnuter_external_direct_drcda.py`: dynamic-reachability-constrained differential allocator for direct actuator control.
- `hnuter_drcda.py`: ROS-independent DRCDA wrench model, actuator predictor, and short-horizon solver.
- `hnuter_external_setpoint_gamepad.py`: setpoint-only gamepad controller. It publishes position, velocity, attitude, and optional body-rate references while leaving the controller and allocator inside PX4.

## Dependencies

This workspace expects the PX4 ROS 2 message packages under `src/`:

- `src/px4_msgs`
- `src/px4_ros_com`

They are tracked as submodules. After cloning:

```bash
git submodule update --init --recursive
```

## Typical SITL Run

Start PX4 SITL:

```bash
cd ~/PX4-Hnuter/PX4-Autopilot-Hnuter
make px4_sitl gz_hnuter
```

Run the stable PX4 offboard controller:

```bash
cd ~/px4_ws_ros2
python3 hnuter_external_controller.py
```

Run the direct actuator debug controller:

```bash
cd ~/px4_ws_ros2
python3 hnuter_external_direct_controller_debug.py
```

In the debug controller, press `o` to allow takeoff after the ground tilt self-test.
Press `2` after takeoff to fly the closed 3D Lissajous trajectory. Its default
frequency ratio is `2:3:1`, with `1.0 m`, `0.75 m`, and `0.35 m` axis
amplitudes over 24 seconds. The altitude profile starts and finishes with zero
vertical speed. Override it with `HNUTER_LISSAJOUS_AMP_X_M`,
`HNUTER_LISSAJOUS_AMP_Y_M`, `HNUTER_LISSAJOUS_AMP_Z_M`, and
`HNUTER_LISSAJOUS_PERIOD_S`.

Run the experimental DRCDA direct controller:

```bash
cd ~/px4_ws_ros2
python3 hnuter_external_direct_drcda.py
```

This entry point defaults to
`config/identified_delay_damped_xy.json`; set `HNUTER_TUNING_FILE` explicitly
to test another tuning set.

In this DRCDA controller, LT/RT adjust the currently selected manual attitude
axis. The initial axis is roll; each rising-edge press of `RB` toggles between
pitch and roll without resetting the angle already commanded on the other axis.
Override the default Xbox/XInput RB button index `5` with
`HNUTER_PAD_RB_BUTTON`.

For the actuator-dynamics SITL plant, load the damped lateral-position tuning
that was validated with hover and the 3D Lissajous trajectory:

```bash
cd ~/PX4-Hnuter/PX4-Autopilot-Hnuter-delay
HEADLESS=1 make px4_sitl gz_hnuter
```

In a second terminal, run the DRCDA controller:

```bash
cd ~/px4_ws_ros2
HNUTER_TUNING_FILE=$PWD/config/identified_delay_damped_xy.json \
HNUTER_DRCDA_VARIANT=full \
HNUTER_DRCDA_SERVO_MODEL=identified \
python3 hnuter_external_direct_drcda.py
```

The DRCDA controller keeps position targets and integrator state in NED, but
rotates horizontal position and velocity errors into the yaw-only body frame
before applying gains. Body X uses `Kp=0.8`, `Kd=1.4`, and `Ki=0.02`; the
slower delayed body Y path uses `Kp=0.8`, `Kd=2.4`, and `Ki=0.02`. Feedback is
rotated back into NED before the desired wrench is formed. These body-frame
gains and the body-Y trim belong only to
`hnuter_external_direct_drcda.py`; the original direct debug controller keeps
its NED position loop.

Manual horizontal control uses independent body-X/Y speed limits of `0.5/0.4
m/s`, with a `0.4 m` maximum position lead and body-X/Y controller
acceleration limits of `2.0/1.8 m/s^2`. The gamepad uses a `0.12` deadzone,
`0.5` expo, independent body-X/Y command filters of `0.20/0.45 s`, and
command acceleration limits of `1.0/0.55 m/s^2`. The lower body-Y reversal
rate prevents a full-stick direction change from outrunning the slower
lateral tilt path. All of these values can be changed live in the tuning JSON.

The controller uses the primary/secondary servo static gain and directional
rate limits. Pure command dead time and the separate first-order command lag
are disabled. Gazebo joint physics and a near-critically-damped joint PID
represent the remaining servo response. The current `gz_hnuter` model must
contain the corresponding plant-side servo dynamics for a meaningful
comparison.
When intentionally testing against an ideal instantaneous-servo SITL model, use:

```bash
HNUTER_DRCDA_SERVO_MODEL=ideal python3 hnuter_external_direct_drcda.py
```

The Cuniato paper stages can be selected with
`HNUTER_DRCDA_VARIANT=ada`, `nda`, or `pda`. They implement the wrench-error
augmentation, asymmetric actuator-rate normalization with global saturation,
speed-dependent motor acceleration limits, and exact sampled first-order
actuator inversion. `HNUTER_DRCDA_VARIANT=ftr_drcda` selects the separate
180 ms move-blocked finite-time-reachability allocator.

The default `full` mode uses the paper PDA for hover and translational
trajectories. During the automatic large-attitude trajectory it transfers the
estimated actuator state to the finite-time reachable allocator, then transfers
state back to PDA when the trajectory ends. This hybrid remains useful for the
slower, rate-limited tilt response during large-attitude motion; pure paper PDA
remains available for an apples-to-apples paper-stage comparison.

Select the original differential-allocation baseline with
`HNUTER_DRCDA_VARIANT=basic_da`. The `no_delay` name remains as a compatibility
alias and is equivalent to the zero-delay default. Reachability ablations remain
available as `no_horizon`, `no_physical_rate`,
`no_command_slew`, `no_reachability_gate`, and `no_multirate`. Each variant
writes diagnostics under its own
`hnuter_logs/external_control/ablation/VARIANT/` directory. Run the
ROS-independent model and allocation checks with:

```bash
python3 -m unittest -v test_hnuter_drcda.py
```

Compare an original-direct log with a DRCDA log and generate 3D and
time-series plots under `hnuter_logs/comparison/`:

```bash
python3 plot_lissajous_comparison.py \
  --baseline hnuter_logs/external_control/hnuter_direct_debug_RUN.csv \
  --drcda hnuter_logs/external_control/hnuter_drcda_debug_RUN.csv
```

Compare lateral oscillation before and after tuning, including the trajectory
and the final 30 seconds of post-trajectory hover:

```bash
python3 plot_xy_tuning_comparison.py \
  --baseline hnuter_logs/external_control/ablation/full/BEFORE.csv \
  --tuned hnuter_logs/external_control/ablation/full/TUNED.csv
```

Generate the multi-run ablation plots and report with:

```bash
python3 plot_drcda_ablation.py \
  --run basic_da=hnuter_logs/external_control/ablation/basic_da/RUN.csv \
  --run full=hnuter_logs/external_control/ablation/full/RUN.csv \
  --run no_delay=hnuter_logs/external_control/ablation/no_delay/RUN.csv \
  --run no_horizon=hnuter_logs/external_control/ablation/no_horizon/RUN.csv \
  --run no_rate_limits=hnuter_logs/external_control/ablation/no_rate_limits/RUN.csv
```

Run the repeatable actuator-dynamics SITL matrix. The default set covers the
24 s 3D Lissajous trajectory, the 8 s aggressive maneuver, the validated
45/60 degree attitude hold, and the 90/150 degree envelope probe:

```bash
cd ~/px4_ws_ros2
python3 run_drcda_multiscenario_experiments.py
python3 analyze_drcda_multiscenario_experiments.py
```

The paper revalidation report and its figures are generated from the selected
final runs with:

```bash
python3 analyze_cuniato_paper_revalidation.py
```

The consolidated output is under
`hnuter_logs/cuniato_paper_revalidation_20260730/`.

The 60/90 degree boundary probe is intentionally excluded from the default
matrix because the current controller loses stability near 50-60 degrees of
roll. Re-run that single probe explicitly with:

```bash
python3 run_drcda_multiscenario_experiments.py \
  --scenarios large_attitude_60_90 \
  --variants full
```

Run the firmware-controller setpoint gamepad controller:

```bash
cd ~/px4_ws_ros2
python3 hnuter_external_setpoint_gamepad.py
```

Press `o` to request Offboard, Arm, and takeoff. The default gamepad mapping is:
left stick X for yaw rate, left stick Y for vertical speed, right stick X/Y
for horizontal velocity, `A/B` for negative/positive roll steps, and `X/Y`
for negative/positive pitch steps. The terminal prints every ABXY attitude-step
event and the resulting roll/pitch target, which makes it clear whether the
gamepad was read correctly. By default DDS discovery is restricted to localhost
to avoid another PX4/ROS 2 instance on the LAN writing into the same topics;
set `HNUTER_ALLOW_REMOTE_DDS=1` only when remote DDS discovery is intentional.
If the horizontal stick direction looks wrong, run with `HNUTER_PAD_DEBUG=1` to
print raw joystick axes, then adjust `HNUTER_PAD_AXIS_XY_X` and
`HNUTER_PAD_AXIS_XY_Y`.

## Direct Debug Notes

By default, direct actuator mode publishes only `actuator_motors` and `actuator_servos` so PX4's internal allocator does not overwrite the external actuator commands.

Set `HNUTER_DIRECT_PUBLISH_ALLOCATOR_SETPOINTS=1` only when intentionally comparing against the allocator path.

Generated logs, plots, build products, and Python caches are intentionally ignored.
Hnuter controller logs are written under `hnuter_logs/` by default. Set
`HNUTER_LOG_DIR=/path/to/logs` before starting a script to store them elsewhere.

## Attitude Tuning Dashboards

The Matplotlib dashboard is intended for local SITL tuning:

```bash
source ~/PX4-Autopilot-Hnuter/px4-venv/bin/activate
cd ~/px4_ws_ros2
python3 hnuter_attitude_tuning_dashboard.py
```

For real-aircraft tuning on a companion computer, use the lightweight LAN web
dashboard. It uses only the Python standard library in addition to the existing
ROS 2, `px4_msgs`, and `pymavlink` environment:

```bash
source ~/PX4-Autopilot-Hnuter/px4-venv/bin/activate
cd ~/px4_ws_ros2
python3 hnuter_attitude_tuning_web.py --host 0.0.0.0 --port 8765
```

Open `http://COMPANION_IP:8765` from a browser on the same trusted LAN. To
require a token, set `HNUTER_WEB_TOKEN` before starting the service and append
the printed `?token=...` value to the browser URL.

The browser receives attitude, angular-rate, local NED position, NED velocity,
tracking-error, torque, and motor-command plots at 15 Hz by default. Each
three-axis plot can be filtered to a single axis from its title-bar selector,
while CSV data is recorded at 25 Hz under `hnuter_logs/tuning/`. The parameter panel discovers the PX4 parameter
catalog over MAVLink and, by default, shows every parameter whose name starts
with `HNTR_`; use `--param-prefix HNTR_,CA_` to include more prefixes or
`--param-prefix all` to show every PX4 parameter. Each row's Reset button
returns the browser controls to the last value read back from PX4; it does not
write a parameter. `Save to PX4` remains a separate confirmed operation for
parameters changed by other tools.
