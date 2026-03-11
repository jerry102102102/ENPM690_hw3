from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    phase2_share = Path(get_package_share_directory("enpm690_hw3_phase2"))
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(phase2_share / "launch" / "phase2_play_auto.launch.py"))
            )
        ]
    )
