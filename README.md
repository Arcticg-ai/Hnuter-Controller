# Hnuter Controller

ROS 2 offboard controllers for the Hnuter PX4/Gazebo setup.

## Main Files

- `hnuter_external_controller.py`: PX4 position-offboard controller with hover and trajectory modes.
- `hnuter_external_controller_px4_position.py`: preserved PX4 position-control baseline.
- `hnuter_external_direct_controller_debug.py`: direct actuator debug controller for checking motor/tilt command paths.
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

Run the experimental DRCDA direct controller:

```bash
cd ~/px4_ws_ros2
python3 hnuter_external_direct_drcda.py
```

DRCDA defaults to the identified primary/secondary servo gain, pure delay,
first-order lag, and directional rate limits. The current `gz_hnuter` model must
contain the corresponding plant-side servo dynamics for a meaningful comparison.
When intentionally testing against an ideal instantaneous-servo SITL model, use:

```bash
HNUTER_DRCDA_SERVO_MODEL=ideal python3 hnuter_external_direct_drcda.py
```

The solver uses a 180 ms move-blocked prediction horizon at 100 Hz by default.
Its diagnostics, including predicted actuator state, wrench residual, and solve
time, are written under `hnuter_logs/external_control/`. Run the ROS-independent
model and allocation checks with:

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
