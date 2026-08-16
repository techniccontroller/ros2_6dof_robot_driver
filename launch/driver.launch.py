from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start the hardware driver, control UI, and state visualization."""
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    auto_connect = LaunchConfiguration("auto_connect")
    launch_rviz = LaunchConfiguration("launch_rviz")
    debug_position_commands = LaunchConfiguration("debug_position_commands")

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
            "use_joint_state_publisher": "false",
            "use_gui": "false",
            "use_rviz": "true",
            "rviz_node_name": "robot_state_rviz",
        }.items(),
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("auto_connect", default_value="true"),
        DeclareLaunchArgument("debug_position_commands", default_value="true"),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Start RViz to visualize the measured robot state.",
        ),
        Node(
            package="pico_6dof_robot_driver",
            executable="pico_6dof_driver",
            name="pico_6dof_driver",
            output="screen",
            emulate_tty=True,
            parameters=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("pico_6dof_robot_driver"),
                        "config",
                        "driver.yaml",
                    ]
                ),
                {
                    "serial_port": serial_port,
                    "baudrate": ParameterValue(baudrate, value_type=int),
                    "auto_connect": ParameterValue(
                        auto_connect, value_type=bool
                    ),
                    "debug_position_commands": ParameterValue(
                        debug_position_commands, value_type=bool
                    ),
                },
            ],
        ),
        Node(
            package="pico_6dof_robot_driver",
            executable="pico_6dof_joint_goal_gui",
            name="pico_6dof_joint_goal_gui",
            output="screen",
        ),
        visualization_launch,
    ])
