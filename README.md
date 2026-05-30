# Hnuter Controller

ROS 2 offboard controllers for the Hnuter PX4/Gazebo setup.

## Main Files

- `hnuter_external_controller.py`: PX4 position-offboard controller with hover and trajectory modes.
- `hnuter_external_controller_px4_position.py`: preserved PX4 position-control baseline.
- `hnuter_external_direct_controller_debug.py`: direct actuator debug controller for checking motor/tilt command paths.

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

## Direct Debug Notes

By default, direct actuator mode publishes only `actuator_motors` and `actuator_servos` so PX4's internal allocator does not overwrite the external actuator commands.

Set `HNUTER_DIRECT_PUBLISH_ALLOCATOR_SETPOINTS=1` only when intentionally comparing against the allocator path.

Generated logs, plots, build products, and Python caches are intentionally ignored.

