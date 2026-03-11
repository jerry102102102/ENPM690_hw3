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
            "robot_name": "tb3_phase2_auto",
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
            DeclareLaunchArgument("forward_speed", default_value="0.28"),
            DeclareLaunchArgument("turn_gain", default_value="1.7"),
            DeclareLaunchArgument("ghost_avoid_gain", default_value="1.6"),
            phase1_stack,
            Node(
                package="enpm690_hw3_phase2",
                executable="pacman_game_manager",
                output="screen",
                parameters=[params, {"use_sim_time": False, "mode": "auto_play"}],
            ),
            Node(
                package="enpm690_hw3_phase2",
                executable="pacman_auto_controller",
                output="screen",
                parameters=[
                    params,
                    {
                        "use_sim_time": False,
                        "forward_speed": LaunchConfiguration("forward_speed"),
                        "turn_gain": LaunchConfiguration("turn_gain"),
                        "ghost_avoid_gain": LaunchConfiguration("ghost_avoid_gain"),
                    },
                ],
            ),
            Node(
                package="enpm690_hw3_phase2",
                executable="marker_publisher",
                output="screen",
                parameters=[params, {"use_sim_time": False}],
            ),
        ]
    )
