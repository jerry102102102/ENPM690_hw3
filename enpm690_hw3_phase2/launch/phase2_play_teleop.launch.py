from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    phase1_share = Path(get_package_share_directory("enpm690_hw3_phase1"))
    phase2_share = Path(get_package_share_directory("enpm690_hw3_phase2"))
    params = str(phase2_share / "config" / "phase2_params.yaml")
    rviz_config = str(phase2_share / "rviz" / "phase2_pacman.rviz")
    default_world = str(phase1_share / "worlds" / "phase1_obstacles.sdf")

    phase1_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(phase1_share / "launch" / "phase1_bringup.launch.py")),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "rviz_config": rviz_config,
            "robot_name": "tb3_phase2",
            "x": "0.0",
            "y": "0.0",
            "z": "0.08",
            "yaw": "0.0",
            "use_rviz": "true",
            "rviz_use_sim_time": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            phase1_stack,
            Node(
                package="enpm690_hw3_phase2",
                executable="pacman_game_manager",
                output="screen",
                parameters=[params, {"use_sim_time": False, "mode": "teleop_play"}],
            ),
            Node(
                package="enpm690_hw3_phase2",
                executable="marker_publisher",
                output="screen",
                parameters=[params, {"use_sim_time": False}],
            ),
        ]
    )
