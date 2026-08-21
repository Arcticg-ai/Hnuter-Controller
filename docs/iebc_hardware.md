# Hnuter IEBC hardware Offboard gateway

`hnuter_external_controller_px4_position_iebc_hardware.py` inserts the closed-loop IEBC reference
filter into the `hardware` branch's validated PX4 position-Offboard controller.
It inherits `hnuter_external_controller_px4_position_hardware.HnuterController`;
there is no copied Arm/Offboard gate, PX4 state conversion, RC handling or
PX4 setpoint publisher. The IEBC filter itself is embedded in this file, so a
real-aircraft deployment never imports or executes the Gazebo simulation code.

## Boundary and authority

- The transmitter and PX4 own Arm, Disarm, Offboard selection and failsafes.
- The IEBC node never publishes `VehicleCommand` or direct actuator topics.
- PX4 remains the low-level position/velocity/acceleration controller.
- The upstream task publishes only a nominal reference; IEBC publishes the
  filtered reference through the inherited `/fmu/in/trajectory_setpoint` path.
- The default hardware IEBC path reconstructs propulsive force from PX4's
  post-allocation motor and servo commands. This is a command/model estimate,
  not measured thrust, RPM or servo position.
- An external ENU actuator-force estimator can be selected explicitly when
  real feedback becomes available. The Gazebo-only proxy remains rejected.

Position Offboard and direct thrust/torque Offboard are mutually different PX4
control paths. This gateway therefore does **not** send a second direct force
command to PX4. The force topic is measured/estimated actuator force used by
the energy certificate; acceleration in `TrajectorySetpoint` is the nominal
feedforward term consumed by the PX4 position-control path.

## Topics

| Direction | Topic | Type | Frame/meaning |
|---|---|---|---|
| Input | `/hnuter/iebc/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | Absolute NED position, velocity, acceleration and yaw |
| Input | `/fmu/out/actuator_motors` | `px4_msgs/ActuatorMotors` | Post-allocation normalized motor commands; default force-model input |
| Input | `/fmu/out/actuator_servos` | `px4_msgs/ActuatorServos` | Post-allocation normalized tilt-servo commands; default force-model input |
| Input | `/hnuter/iebc/in/actuator_wrench` | `geometry_msgs/WrenchStamped` | Optional external actuator-force estimate in ENU `map`/`world`; not contact force |
| Input | `/hnuter/iebc/in/recovery` | `std_msgs/Bool` | Rising `true` edge means the physical load was released |
| Input | `/hnuter/iebc/in/reset` | `std_msgs/Empty` | Reset IEBC storage/reference state |
| Output | `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | Inherited 20 Hz proof-of-life |
| Output | `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | IEBC-filtered PX4 reference |
| Output | `/hnuter/iebc/out/status` | `std_msgs/String` | JSON health, energy, barrier and stale-input state |

The node already subscribes, through the inherited hardware controller, to
PX4 local position, attitude, vehicle status/control mode and RC topics.

## Nominal-reference modes

`HNUTER_IEBC_NOMINAL_SOURCE=rc_task` (default) keeps manual RC flight after
entering Offboard and adds an AUX-triggered push task:

1. Start the node, enter Offboard with the task switch low, and fly manually
   to the task area using the existing hardware controller.
2. The node must observe a fresh low task-switch value before it arms the task
   trigger. A switch already high while entering Offboard cannot auto-start.
3. Raising the switch latches the measured position, measured yaw and the
   aircraft's horizontal forward axis. It then ramps the nominal reference
   forward with configured speed/acceleration limits while IEBC filters it.
4. Lowering the switch at any time stops further forward-reference growth,
   acceleration-limits the reversal, and returns along the latched forward axis
   toward the position at which the switch was raised.
5. Manual RC control is restored after position and velocity remain inside the
   return tolerances. The switch must be observed low again before another run.

The task switch defaults to PX4 logical `AUX4`. This leaves `AUX1/AUX2` for
the validated manual Roll/Pitch attitude inputs and avoids the firmware's
existing `AUX3` attitude-level switch. Map the desired physical receiver
channel in PX4 and verify it before installing propellers:

```text
param show RC_MAP_AUX4
param set RC_MAP_AUX4 <receiver-channel-number>
param save
```

`HNUTER_IEBC_NOMINAL_SOURCE=topic` accepts the private
`TrajectorySetpoint` input. This is the reusable composition mode: a door,
surface, trajectory or experiment task can be a separate ROS 2 node.

`HNUTER_IEBC_NOMINAL_SOURCE=baseline` keeps the existing RC/keyboard reference
generator and inserts IEBC directly before the inherited PX4 publisher. It is
useful for regression checks against the validated hardware controller.

## Required configuration

IEBC is disabled unless explicitly enabled. Before a quantitative hardware
experiment, configure values certified for the actual vehicle:

```bash
export HNUTER_IEBC_ENABLE=1
export HNUTER_IEBC_WRENCH_SOURCE=external
export HNUTER_IEBC_ACTUATOR_SOURCE=px4_outputs
export HNUTER_IEBC_MASS_KG=...
export HNUTER_IEBC_LAMBDA_BAR_KG=...
export HNUTER_IEBC_E_MAX_J=...
export HNUTER_IEBC_KC_NPM=...
export HNUTER_IEBC_DC_NSPM=...
export HNUTER_IEBC_AXIS_X=...
export HNUTER_IEBC_AXIS_Y=...
export HNUTER_IEBC_AXIS_Z=...
```

Relevant interface gates:

```bash
export HNUTER_IEBC_COMMAND_TIMEOUT_S=0.30
export HNUTER_IEBC_WRENCH_TIMEOUT_S=0.20
export HNUTER_IEBC_INITIAL_COMMAND_RADIUS_M=0.75
export HNUTER_IEBC_REQUIRE_WRENCH_FRAME=1
```

`HNUTER_IEBC_WRENCH_SOURCE=external` is the internal IEBC-core setting: it
means that the node supplies a complete ENU actuator force rather than using
the Gazebo proxy. `HNUTER_IEBC_ACTUATOR_SOURCE` selects how the hardware node
obtains that force:

- `px4_outputs` (default): subscribe to `/fmu/out/actuator_motors` and
  `/fmu/out/actuator_servos`, invert the firmware's Hnuter motor mapping, apply
  the primary/secondary tilt geometry, and rotate body-FLU force to world ENU.
- `external_wrench`: consume `/hnuter/iebc/in/actuator_wrench`. Use this only
  with a separate estimator that publishes actuator force in an explicit ENU
  `map`/`world` frame.

The command model defaults mirror the current hardware firmware parameters:

```bash
export HNUTER_IEBC_ACT_MASS_KG=4.5
export HNUTER_IEBC_ACT_MOT_HOV=0.50
export HNUTER_IEBC_ACT_MOT_EXPO=0.50
export HNUTER_IEBC_ACT_MAX_ARM_T_N=170.96
export HNUTER_IEBC_ACT_MAX_TAIL_T_N=85.48
export HNUTER_IEBC_ACT_S1_MAX_DEG=180
export HNUTER_IEBC_ACT_S2_SERVO_MAX_DEG=180
export HNUTER_IEBC_ACT_S2_GEAR=2.0
```

These values must match the parameters actually saved on the flight
controller. Persisted PX4 parameters override firmware defaults. In
particular, `HNTR_MOT_HOV`, `HNTR_MOT_EXPO`, `HNTR_MAX_ARM_T`,
`HNTR_MAX_TAIL_T`, the tilt angle ranges and `HNTR_S2_GEAR` must be recorded
with each hardware run.

RC push-task settings:

```bash
export HNUTER_IEBC_NOMINAL_SOURCE=rc_task
export HNUTER_IEBC_TASK_RC_FUNCTION=11    # RcChannels.FUNCTION_AUX_4
export HNUTER_IEBC_TASK_SWITCH_HIGH=0.50
export HNUTER_IEBC_TASK_SWITCH_LOW=0.00
export HNUTER_IEBC_TASK_SWITCH_TIMEOUT_S=0.50
export HNUTER_IEBC_TASK_PUSH_SPEED_MPS=0.05
export HNUTER_IEBC_TASK_PUSH_ACCEL_MPS2=0.15
export HNUTER_IEBC_TASK_MAX_PUSH_M=3.0
export HNUTER_IEBC_TASK_RETURN_SPEED_MPS=0.25
export HNUTER_IEBC_TASK_RETURN_ACCEL_MPS2=0.35
export HNUTER_IEBC_TASK_RETURN_POS_TOL_M=0.12
export HNUTER_IEBC_TASK_RETURN_VEL_TOL_MPS=0.08
```

The RC push task refuses to start unless IEBC is enabled, configured, and
receiving a fresh selected actuator-force input. In default `px4_outputs`
mode, both motor and servo output topics must be fresh. A stale task switch
during `PUSH` is treated as a cancel and starts `RETURN`; stale actuator input
holds the current position instead of failing open.

The upstream task must first publish a nominal position close to the measured
vehicle position. A new topic command farther than the configured initial
radius is rejected. When IEBC is enabled, stale wrench or stale command data
latches a zero-velocity hold rather than passing the nominal command through.

## Build and run

```bash
cd /home/hnuter/px4_ws_ros2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs --symlink-install \
  --allow-overriding px4_msgs
source install/local_setup.bash

python3 hnuter_external_controller_px4_position_iebc_hardware.py
```

Start the node while disarmed, confirm PX4 topics and the JSON status topic,
then use the transmitter to enter Armed + Offboard. First validation is
propellers-off, followed by restrained/tethered testing; Gazebo results do not
certify the hardware force model or energy bounds. The default command model
also cannot detect a stalled motor, missed servo angle, ESC delay, or thrust
calibration error; use conservative uncertainty margins until sensor-backed
estimation has been validated.
