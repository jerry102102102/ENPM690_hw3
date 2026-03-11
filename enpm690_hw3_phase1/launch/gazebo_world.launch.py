from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("enpm690_hw3_phase1"))
    ros_gz_sim_share = Path(get_package_share_directory("ros_gz_sim"))

    default_world = str(package_share / "worlds" / "phase1_obstacles.sdf")
    model_path = str(package_share / "models")
    world_path = str(package_share / "worlds")

    resource_path = [
        model_path,
        ":",
        world_path,
        ":",
        EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(ros_gz_sim_share / "launch" / "gz_sim.launch.py")),
                launch_arguments={"gz_args": ["-r -v 4 ", LaunchConfiguration("world")]}.items(),
            ),
        ]
    )
