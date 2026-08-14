# Pico 6-DOF robot ROS 2 driver

Python ROS 2 driver for the robot firmware in `pico_6dof-robot_firmware`. It
uses the firmware's 115200-baud, newline-delimited ASCII protocol and converts
the firmware's degree values to the radians required by ROS.

The ROS interface follows the supported joint/gripper subset of
[`ur_impedance_driver_ros2`](https://github.com/edgarwelteKIT/ur_impedance_driver_ros2).
Cartesian pose, twist, impedance, and force/torque interfaces are deliberately
not exposed because the Pico firmware has no kinematics or force/torque data.

## Build and run

```bash
cd ~/ros2_ws/src
git clone <this-repository-url> ros2_6dof-robot_driver
python3 -m pip install pyserial
cd ..
colcon build --packages-select pico_6dof_robot_driver
source install/setup.bash
ros2 launch pico_6dof_robot_driver driver.launch.py serial_port:=/dev/ttyACM0
```

To start the driver, robot model in RViz, and the example six-joint goal GUI:

```bash
ros2 launch pico_6dof_robot_driver driver_rviz.launch.py
```

Set the J1--J6 sliders in degrees and press **Send target** to publish the
complete configuration to the driver. Disable either optional window with
`launch_rviz:=false` or `launch_gui:=false`.

The user running the node must have access to the serial device (commonly the
`dialout` group on Linux). Set `auto_connect:=false` to start disconnected and
then call `ros2 service call /connect std_srvs/srv/Trigger {}`.
On connection the driver selects the firmware's `AUTO` mode; set
`set_auto_mode_on_connect:=false` only if another client owns mode selection.

## ROS interfaces

Published:

- `joint_states` (`sensor_msgs/msg/JointState`), six measured arm positions
  plus the last commanded `gripper_joint` position for RViz. Arm velocities
  are finite differences in radians/second; gripper velocity is reported as
  zero because the firmware has no gripper feedback.
- `control_state` (`std_msgs/msg/String`), JSON connection/telemetry status.
- `gripper_position` (`std_msgs/msg/Float64`), last commanded normalized value;
  the firmware does not report gripper feedback.

Commands:

- `forward_position_controller/commands`
  (`std_msgs/msg/Float64MultiArray`), exactly six target positions in radians.
- `scaled_joint_trajectory_controller/follow_joint_trajectory`
  (`control_msgs/action/FollowJointTrajectory`). Joint names can be reordered,
  but must match the configured six names. Waypoints are sent to the firmware
  with a segment speed; the firmware performs the low-level position control.
- `gripper_position_cmd` (`std_msgs/msg/Float64`), normalized `0.0` open to
  `1.0` closed.
- `robotiq_2f_urcap_adapter/gripper_command` action and
  `robotiq_2f_urcap_adapter/gripper_command_topic` for compatibility with the
  MoveIt configuration. Its position uses the URDF `gripper_joint` convention:
  0 rad closed and pi rad open.

Services (`std_srvs/srv/Trigger`): `connect`, `disconnect`, `stop`, `home_j1`,
`save_zeros`, and `load_zeros`. `stop` replaces the active target with the
latest measured pose; it requires recent telemetry.

Example direct command:

```bash
ros2 topic pub --once /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0, -0.5, 0.2, 0.0, -1.0, 0.0]}"
```

By default, each received position target is logged with a sequence number,
values, selected velocity, latest telemetry age and target delta, exact
firmware command, and serial-write completion. Set
`debug_position_commands:=false` to suppress the detailed transmission logs;
receipt and rejection messages remain enabled.

With debugging enabled, non-telemetry firmware replies are shown as
`Firmware RX`. A successful target is acknowledged by current firmware as
`VEL_CONFIG is set`. The logged `encoder_status` should contain valid values
for every encoder; the firmware disables joint control if any encoder is
invalid.

## Firmware command buffer

The driver targets the updated firmware with a 128-byte
`Communication::BUFFER_SIZE`. Every six-axis position or trajectory waypoint is
sent as one atomic `VEL_CONFIG(...)` command. The 126-byte payload limit reserves
space for the newline and terminating NUL.

## Safety

Start with the robot unloaded, a low `default_velocity`, and small moves. The
firmware applies its own J2/J3, J4, J5, and J6 limits, but this driver cannot
detect collisions and does not implement a safety-rated emergency stop.
