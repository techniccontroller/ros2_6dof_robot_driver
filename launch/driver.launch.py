from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    auto_connect = LaunchConfiguration("auto_connect")
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("auto_connect", default_value="true"),
        Node(
            package="pico_6dof_robot_driver",
            executable="pico_6dof_driver",
            name="pico_6dof_driver",
            output="screen",
            emulate_tty=True,
            parameters=[
                PathJoinSubstitution([FindPackageShare("pico_6dof_robot_driver"), "config", "driver.yaml"]),
                {
                    "serial_port": serial_port,
                    "baudrate": ParameterValue(baudrate, value_type=int),
                    "auto_connect": ParameterValue(auto_connect, value_type=bool),
                },
            ],
        ),
    ])
