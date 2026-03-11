from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("enpm690_hw3_phase1"))
    default_config = str(package_share / "config" / "bridge.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("bridge_config", default_value=default_config),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                output="screen",
                parameters=[{"config_file": LaunchConfiguration("bridge_config")}],
            ),
        ]
    )
