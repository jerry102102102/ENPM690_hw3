from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    phase1_share = Path(get_package_share_directory("enpm690_hw3_phase1"))
    phase2_share = Path(get_package_share_directory("enpm690_hw3_phase2"))
    ros_gz_sim_share = Path(get_package_share_directory("ros_gz_sim"))
    robot_description = (phase1_share / "urdf" / "turtlebot3_burger_like.urdf").read_text()

    world = str(phase2_share / "worlds" / "phase2_ocean_arena.sdf")
    params = str(phase2_share / "config" / "phase2_params.yaml")
    rviz_config = str(phase2_share / "rviz" / "phase2.rviz")

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(phase1_share / "launch" / "spawn_robot.launch.py")),
        launch_arguments={"robot_name": "tb3_phase2", "x": "0.0", "y": "0.0", "z": "0.08", "yaw": "0.0"}.items(),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                [
                    str(phase2_share / "models"),
                    ":",
                    str(phase2_share / "worlds"),
                    ":",
                    str(phase1_share / "models"),
                    ":",
                    str(phase1_share / "worlds"),
                    ":",
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(ros_gz_sim_share / "launch" / "gz_sim.launch.py")),
                launch_arguments={"gz_args": [f"-r -v 4 {world}"]}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(phase1_share / "launch" / "bridge.launch.py")),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            ),
            Node(
                package="enpm690_hw3_phase1",
                executable="odom_tf_broadcaster",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="enpm690_hw3_phase1",
                executable="teleop_command_logger",
                output="screen",
                parameters=[{"cmd_topic": "/cmd_vel_input", "use_sim_time": True}],
            ),
            Node(
                package="enpm690_hw3_phase2",
                executable="game_manager",
                output="screen",
                parameters=[
                    params,
                    {
                        "mode": "teleop_play",
                        "auto_reset": False,
                        "episode_duration": 9999.0,
                        "use_sim_time": True,
                    },
                ],
            ),
            Node(
                package="enpm690_hw3_phase2",
                executable="gazebo_fish_sync",
                name="gazebo_fish_sync",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="enpm690_hw3_phase2",
                executable="marker_publisher",
                output="screen",
                parameters=[params, {"use_sim_time": True}],
            ),
            TimerAction(period=3.0, actions=[spawn]),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": False}],
            ),
        ]
    )
