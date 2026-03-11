from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("enpm690_hw3_phase1"))
    default_model = str(package_share / "models" / "turtlebot3_burger_like" / "model.sdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="tb3_phase1"),
            DeclareLaunchArgument("model", default_value=default_model),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.08"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-name",
                    LaunchConfiguration("robot_name"),
                    "-allow_renaming",
                    "false",
                    "-x",
                    LaunchConfiguration("x"),
                    "-y",
                    LaunchConfiguration("y"),
                    "-z",
                    LaunchConfiguration("z"),
                    "-Y",
                    LaunchConfiguration("yaw"),
                    "-file",
                    LaunchConfiguration("model"),
                ],
            ),
        ]
    )
