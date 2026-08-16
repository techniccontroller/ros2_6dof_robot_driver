"""Compatibility wrapper for the driver launch with state visualization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start the driver and visualize its joint states."""
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    auto_connect = LaunchConfiguration("auto_connect")
    launch_rviz = LaunchConfiguration("launch_rviz")
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
            "launch_rviz": launch_rviz,
            "debug_position_commands": debug_position_commands,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("auto_connect", default_value="true"),
            DeclareLaunchArgument(
                "debug_position_commands", default_value="true"
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Start RViz with the robot model.",
            ),
            driver_launch,
        ]
    )
