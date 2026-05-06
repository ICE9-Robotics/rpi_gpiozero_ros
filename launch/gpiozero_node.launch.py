"""Launch the gpiozero ROS 2 node with configurable parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create launch description for gpiozero node."""
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rpi_gpiozero_ros"), "config", "gpiozero_params.yaml"]
        ),
        description="Path to YAML parameter file for gpiozero_node.",
    )

    node = Node(
        package="rpi_gpiozero_ros",
        executable="gpiozero_node",
        name="gpiozero_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_file_arg, node])
