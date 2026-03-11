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
        (str(share_dir / "models" / "tuna_simple"), glob("models/tuna_simple/*")),
        (str(share_dir / "models" / "sardine_simple"), glob("models/sardine_simple/*")),
        (str(share_dir / "models" / "seaweed_simple"), glob("models/seaweed_simple/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ENPM690 Student",
    maintainer_email="student@example.com",
    description="Phase 2 shark hunt scenario, rule-based baseline, and Gymnasium environment.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "game_manager = enpm690_hw3_phase2.game_manager:main",
            "gazebo_fish_sync = enpm690_hw3_phase2.gazebo_fish_sync:main",
            "marker_publisher = enpm690_hw3_phase2.marker_publisher:main",
            "shark_auto_controller = enpm690_hw3_phase2.shark_auto_controller:main",
            "train_ppo = enpm690_hw3_phase2.train_ppo:main",
            "eval_policy = enpm690_hw3_phase2.eval_policy:main",
        ],
    },
)
