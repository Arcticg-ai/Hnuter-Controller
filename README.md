# Hnuter Controller

ROS 2 offboard controllers for the Hnuter PX4/Gazebo setup.

## Main Files

- `hnuter_external_controller.py`: PX4 position-offboard controller with hover and trajectory modes.
- `hnuter_external_controller_px4_position.py`: preserved PX4 position-control baseline.
- `hnuter_external_direct_controller_debug.py`: direct actuator debug controller for checking motor/tilt command paths.
- `hnuter_external_direct_controller_hardware.py`: standalone RC-driven hardware direct controller. It does not import another local controller module and leaves Arm/Offboard authority with PX4 and the transmitter.
- `hnuter_external_direct_drcda.py`: delay-aware, reachability-constrained differential allocator for direct actuator control.
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

## Hardware Direct Controller

Run the standalone real-aircraft entry point with:

```bash
cd ~/px4_ws_ros2
python3 hnuter_external_direct_controller_hardware.py
```

The file contains its own controller helpers, RC parser, logging paths, and
Offboard task-restart state machine. Its only Python runtime dependencies are
the ROS 2/PX4 environment, `px4_msgs`, `std_msgs`, and NumPy.

The node continuously publishes the Offboard proof-of-life but never sends
Arm, Disarm, or mode-change commands. Use the transmitter to Arm and enable
Offboard. Control starts only after PX4 reports both states active, and the
measured position becomes the initial target without an automatic climb.

Pitch commands body-forward speed, Roll commands body-lateral speed, centered
Throttle commands vertical speed, and Yaw commands yaw rate. Only RC-sourced
`manual_control_setpoint` data is accepted, with `rc_channels` as a fallback.
If RC input times out, manual rates return to zero and the current target is
held. Keys `1`, `2`, and `3` queue the rectangle, 3D Lissajous, and attitude
tasks. Switching Offboard off and back on while a task is active restarts that
task from the current position. Keyboard `o` is disabled.

Before the first powered test, verify stick signs and the Offboard exit switch
without propellers. Direction signs can be adjusted with
`HNUTER_RC_PITCH_SIGN`, `HNUTER_RC_ROLL_SIGN`,
`HNUTER_RC_THROTTLE_SIGN`, and `HNUTER_RC_YAW_SIGN`.

Run the experimental DRCDA direct controller:

```bash
cd ~/px4_ws_ros2
python3 hnuter_external_direct_drcda.py
```

In this DRCDA controller, LT/RT adjust the currently selected manual attitude
axis. The initial axis is roll; each rising-edge press of `RB` toggles between
pitch and roll without resetting the angle already commanded on the other axis.
Override the default Xbox/XInput RB button index `5` with
`HNUTER_PAD_RB_BUTTON`.

For the identified-delay SITL plant, load the damped lateral-position tuning
that was validated with hover and the 3D Lissajous trajectory:

```bash
HNUTER_TUNING_FILE=$PWD/config/identified_delay_damped_xy.json \
HNUTER_DRCDA_VARIANT=full \
python3 hnuter_external_direct_drcda.py
```

The position gains use NED axis order. The slower North actuator path therefore
uses `Kp=1.6`, `Kd=3.2`, and `Ki=0.35`; the East path uses `Kp=2.0`,
`Kd=2.8`, and `Ki=0.2`. The small horizontal integral gains remove static
offset, while the increased derivative gains damp the identified actuator lag.

DRCDA defaults to the identified primary/secondary servo gain, pure delay,
first-order lag, and directional rate limits. The current `gz_hnuter` model must
contain the corresponding plant-side servo dynamics for a meaningful comparison.
When intentionally testing against an ideal instantaneous-servo SITL model, use:

```bash
HNUTER_DRCDA_SERVO_MODEL=ideal python3 hnuter_external_direct_drcda.py
```

For the no-delay `main` firmware model, use the separately validated tuning
file. It also overrides the DRCDA horizon and optimization weights, and is
reloaded while the controller is running whenever the JSON file changes:

```bash
HNUTER_DRCDA_SERVO_MODEL=ideal \
HNUTER_DRCDA_VARIANT=full \
HNUTER_TUNING_FILE=$PWD/config/no_delay_drcda_tuning.json \
python3 hnuter_external_direct_drcda.py
```

The direct and DRCDA controllers compensate PX4 estimator quaternion resets by
synchronizing the yaw references. DRCDA additionally resets its desired-wrench
derivative history on that event, avoiding a false allocation transient while
preserving the actuator state estimate.

The solver uses a 180 ms move-blocked prediction horizon at 100 Hz by default.
Select the basic differential-allocation baseline or one DRCDA ablation with
`HNUTER_DRCDA_VARIANT=basic_da`, `no_delay`, `no_horizon`, or
`no_rate_limits`; the default is `full`. Each variant writes diagnostics under
its own `hnuter_logs/external_control/ablation/VARIANT/` directory, including
predicted actuator state, wrench residual, and solve time. Run the
ROS-independent model and allocation checks with:

```bash
python3 -m unittest -v tests.test_hnuter_drcda
```

Compare an original-direct log with a DRCDA log and generate 3D and
time-series plots under `hnuter_logs/comparison/`:

```bash
python3 tools/plotting/plot_lissajous_comparison.py \
  --baseline hnuter_logs/external_control/hnuter_direct_debug_RUN.csv \
  --drcda hnuter_logs/external_control/hnuter_drcda_debug_RUN.csv
```

Compare lateral oscillation before and after tuning, including the trajectory
and the final 30 seconds of post-trajectory hover:

```bash
python3 tools/plotting/plot_xy_tuning_comparison.py \
  --baseline hnuter_logs/external_control/ablation/full/BEFORE.csv \
  --tuned hnuter_logs/external_control/ablation/full/TUNED.csv
```

Generate the multi-run ablation plots and report with:

```bash
python3 tools/plotting/plot_drcda_ablation.py \
  --run basic_da=hnuter_logs/external_control/ablation/basic_da/RUN.csv \
  --run full=hnuter_logs/external_control/ablation/full/RUN.csv \
  --run no_delay=hnuter_logs/external_control/ablation/no_delay/RUN.csv \
  --run no_horizon=hnuter_logs/external_control/ablation/no_horizon/RUN.csv \
  --run no_rate_limits=hnuter_logs/external_control/ablation/no_rate_limits/RUN.csv
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
python3 tools/tuning/hnuter_attitude_tuning_dashboard.py
```

For real-aircraft tuning on a companion computer, use the lightweight LAN web
dashboard. It uses only the Python standard library in addition to the existing
ROS 2, `px4_msgs`, and `pymavlink` environment:

```bash
source ~/PX4-Autopilot-Hnuter/px4-venv/bin/activate
cd ~/px4_ws_ros2
python3 tools/tuning/hnuter_attitude_tuning_web.py --host 0.0.0.0 --port 8765
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
