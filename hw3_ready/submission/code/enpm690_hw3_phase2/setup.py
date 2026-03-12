from glob import glob
from pathlib import Path

from setuptools import setup


package_name = "enpm690_hw3_phase2"
share_dir = Path("share") / package_name


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (str(share_dir), ["package.xml", "README_phase2.md"]),
        (str(share_dir / "launch"), glob("launch/*.launch.py")),
        (str(share_dir / "config"), glob("config/*.yaml")),
        (str(share_dir / "worlds"), glob("worlds/*.sdf")),
        (str(share_dir / "rviz"), glob("rviz/*.rviz")),
        (str(share_dir / "models" / "pellet_simple"), glob("models/pellet_simple/*")),
        (str(share_dir / "models" / "ghost_simple"), glob("models/ghost_simple/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ENPM690 Student",
    maintainer_email="student@example.com",
    description="Phase 2 Pac-Man style game built on top of the Phase 1 Gazebo and ROS 2 stack.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pacman_matplotlib_sim = enpm690_hw3_phase2.matplotlib_pacman_sim:main",
            "pacman_game_manager = enpm690_hw3_phase2.pacman_game_manager:main",
            "marker_publisher = enpm690_hw3_phase2.marker_publisher:main",
            "pacman_auto_controller = enpm690_hw3_phase2.pacman_auto_controller:main",
            "gazebo_pacman_sync = enpm690_hw3_phase2.gazebo_pacman_sync:main",
        ],
    },
)
