# Pico 6-DOF robot ROS 2 driver

Python ROS 2 driver for the robot firmware in `pico_6dof-robot_firmware`. It
uses the firmware's 115200-baud, newline-delimited ASCII protocol and converts
the firmware's degree values to the radians required by ROS.

The ROS interface follows the supported joint/gripper subset of
[`ur_impedance_driver_ros2`](https://github.com/edgarwelteKIT/ur_impedance_driver_ros2).
Cartesian pose, twist, impedance, and force/torque interfaces are deliberately
not exposed because the Pico firmware has no kinematics or force/torque data.

## Getting started / usage

This stack targets ROS 2 Humble and consists of these repositories:

- [Pico firmware](https://github.com/techniccontroller/pico_6dof-robot_firmware)
- [ROS 2 driver (this repository)](https://github.com/techniccontroller/ros2_6dof_robot_driver)
- [Robot description](https://github.com/techniccontroller/ros2_6dof_robot_description)
- [MoveIt configuration](https://github.com/techniccontroller/ros2_6dof_robot_moveit_config)

Flash the firmware to the Pico, then clone the three ROS packages into one
workspace and install their dependencies:

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/techniccontroller/ros2_6dof_robot_driver.git
git clone https://github.com/techniccontroller/ros2_6dof_robot_description.git
git clone https://github.com/techniccontroller/ros2_6dof_robot_moveit_config.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to ros2_6dof_robot_moveit_config
source install/setup.bash
```

First verify planning and execution with MoveIt's mock hardware:

```bash
ros2 launch ros2_6dof_robot_moveit_config demo.launch.py
```

To open the MoveIt interface and control the physical robot:

```bash
ros2 launch ros2_6dof_robot_moveit_config real_robot.launch.py \
  serial_port:=/dev/ttyACM0
```

This single launch starts the driver, robot-control UI, robot state publisher,
`move_group`, and two RViz windows: one state-only view and one MoveIt
MotionPlanning view. Do not start `driver.launch.py` separately. In the
MotionPlanning window, select the `manipulator` planning group, set a goal with
the interactive marker, choose **Plan**, inspect the trajectory, and then
choose **Execute**. The `gripper` group also provides the named states `open`
and `close`.

To run the driver, robot-control UI, and a state-only RViz view without MoveIt:

```bash
ros2 launch pico_6dof_robot_driver driver.launch.py \
  serial_port:=/dev/ttyACM0
```

The older combined launch name remains available as an equivalent wrapper:

```bash
ros2 launch pico_6dof_robot_driver driver_rviz.launch.py
```

`driver.launch.py` always starts the Tkinter control UI and starts the basic
RViz visualization by default. Use `launch_rviz:=false` when running headless.
The MoveIt real-robot launch keeps this basic view and adds a second
MotionPlanning RViz window. Tkinter is part of the standard Python/Linux
desktop stack and adds no Python package dependency. The UI provides:

- J1--J6 target sliders alongside live measured joint positions.
- ROS 2 controls for connect/disconnect, hold, J1 homing, encoder-zero storage,
  and the gripper.
- Recording of the latest measured six-joint position, commanded gripper
  state, and per-step delay.
- Smooth, synchronized trajectory playback through the same
  `FollowJointTrajectory` action used by MoveIt.
- Reordering, replacing, and deleting recorded steps.
- Saving and loading human-readable, versioned JSON sequence files.

To teach a sequence, move the robot and gripper to a pose, enter how long it
should hold after reaching that step, and press **Record measured position**.
Repeat for the remaining poses and press **Execute sequence**. Each new step
stores J1--J6 and the normalized gripper state (`0` open, `1` closed). Choose a
trajectory speed in degrees/second before playback. The UI creates a dense,
smooth trajectory from the current measured pose through every recorded pose;
all six arm joints share the same interpolation progress and the recorded
gripper command is sent at each nominal arrival time. **Stop playback + hold**
cancels the trajectory and calls the driver's `stop` service. This is a
convenience hold, not a safety-rated emergency stop.

To run only the GUI after sourcing the workspace:

```bash
ros2 run pico_6dof_robot_driver pico_6dof_joint_goal_gui
```

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
