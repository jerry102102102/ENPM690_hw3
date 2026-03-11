from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="enpm690_hw3_phase1",
                executable="keyboard_teleop",
                output="screen",
                emulate_tty=True,
                parameters=[{"use_sim_time": True}],
            )
        ]
    )
