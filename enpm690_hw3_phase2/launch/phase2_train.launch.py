from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    phase1_share = Path(get_package_share_directory("enpm690_hw3_phase1"))
    phase2_share = Path(get_package_share_directory("enpm690_hw3_phase2"))
    params = str(phase2_share / "config" / "phase2_params.yaml")

    phase1_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(phase1_share / "launch" / "phase1_bringup.launch.py")),
        launch_arguments={
            "world": str(phase1_share / "worlds" / "phase1_obstacles.sdf"),
            "rviz_config": str(phase2_share / "rviz" / "phase2_pacman.rviz"),
            "robot_name": "tb3_phase2_train",
            "x": "0.0",
            "y": "0.0",
            "z": "0.08",
            "yaw": "0.0",
            "use_rviz": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            phase1_stack,
            Node(
                package="enpm690_hw3_phase2",
                executable="pacman_game_manager",
                output="screen",
                parameters=[params, {"use_sim_time": True, "mode": "train_skeleton"}],
            ),
        ]
    )
