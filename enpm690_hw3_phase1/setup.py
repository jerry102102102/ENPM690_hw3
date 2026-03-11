from glob import glob
from pathlib import Path

from setuptools import setup


package_name = "enpm690_hw3_phase1"
share_dir = Path("share") / package_name


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (str(share_dir), ["package.xml"]),
        (str(share_dir / "launch"), glob("launch/*.launch.py")),
        (str(share_dir / "worlds"), glob("worlds/*.sdf")),
        (str(share_dir / "config"), glob("config/*.yaml")),
        (str(share_dir / "rviz"), glob("rviz/*.rviz")),
        (str(share_dir / "urdf"), glob("urdf/*.urdf")),
        (str(share_dir / "models" / "turtlebot3_burger_like"), glob("models/turtlebot3_burger_like/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ENPM690 Student",
    maintainer_email="student@example.com",
    description="Phase 1 Gazebo Harmonic + ROS 2 Jazzy simulation package for ENPM690 Homework 3.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_command_logger = enpm690_hw3_phase1.teleop_command_logger:main",
            "odom_tf_broadcaster = enpm690_hw3_phase1.odom_tf_broadcaster:main",
            "keyboard_teleop = enpm690_hw3_phase1.keyboard_teleop:main",
        ],
    },
)
