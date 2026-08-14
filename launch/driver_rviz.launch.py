"""Launch the Pico robot driver together with its model in RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start the driver and visualize its joint states."""
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    auto_connect = LaunchConfiguration("auto_connect")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_gui = LaunchConfiguration("launch_gui")
    debug_position_commands = LaunchConfiguration("debug_position_commands")

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("pico_6dof_robot_driver"),
                    "launch",
                    "driver.launch.py",
                ]
            )
        ),
        launch_arguments={
            "serial_port": serial_port,
            "baudrate": baudrate,
            "auto_connect": auto_connect,
            "debug_position_commands": debug_position_commands,
        }.items(),
    )

    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_6dof_robot_description"),
                    "launch",
                    "display.launch.py",
                ]
            )
        ),
        launch_arguments={
            # The hardware driver publishes /joint_states; do not start a
            # second publisher from the description package.
            "use_joint_state_publisher": "false",
            "use_gui": "false",
            "use_rviz": launch_rviz,
        }.items(),
    )

    joint_goal_gui = Node(
        package="pico_6dof_robot_driver",
        executable="pico_6dof_joint_goal_gui",
        name="pico_6dof_joint_goal_gui",
        output="screen",
        condition=IfCondition(launch_gui),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("auto_connect", default_value="true"),
            DeclareLaunchArgument("debug_position_commands", default_value="true"),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Start RViz with the robot model.",
            ),
            DeclareLaunchArgument(
                "launch_gui",
                default_value="true",
                description="Start the example joint-goal slider GUI.",
            ),
            driver_launch,
            visualization_launch,
            joint_goal_gui,
        ]
    )
