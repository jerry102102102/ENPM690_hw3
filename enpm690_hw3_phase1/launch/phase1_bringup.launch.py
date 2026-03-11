from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("enpm690_hw3_phase1"))
    launch_dir = package_share / "launch"
    robot_description = (package_share / "urdf" / "turtlebot3_burger_like.urdf").read_text()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "gazebo_world.launch.py")),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "bridge.launch.py")),
        launch_arguments={"bridge_config": LaunchConfiguration("bridge_config")}.items(),
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "spawn_robot.launch.py")),
        launch_arguments={
            "robot_name": LaunchConfiguration("robot_name"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "yaw": LaunchConfiguration("yaw"),
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "rviz.launch.py")),
        launch_arguments={"rviz_config": LaunchConfiguration("rviz_config")}.items(),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    odom_tf = Node(
        package="enpm690_hw3_phase1",
        executable="odom_tf_broadcaster",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    teleop_logger = Node(
        package="enpm690_hw3_phase1",
        executable="teleop_command_logger",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=str(package_share / "worlds" / "phase1_obstacles.sdf"),
            ),
            DeclareLaunchArgument(
                "bridge_config",
                default_value=str(package_share / "config" / "bridge.yaml"),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(package_share / "rviz" / "phase1.rviz"),
            ),
            DeclareLaunchArgument("robot_name", default_value="tb3_phase1"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.08"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            gazebo,
            bridge,
            robot_state_publisher,
            odom_tf,
            teleop_logger,
            TimerAction(period=3.0, actions=[spawn]),
            rviz,
        ]
    )
