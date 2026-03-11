# ENPM690 Homework 3

This repository contains the Phase 1 simulation stack for ENPM690 Homework 3.

The implementation targets:

- ROS 2 Jazzy
- Gazebo Harmonic
- `ros_gz_sim` and `ros_gz_bridge`
- RViz2
- Python ROS 2 nodes with `rclpy`

## Workspace layout

The ROS 2 package lives at:

- `enpm690_hw3_phase1/`

Key package assets:

- `launch/`: Gazebo, robot spawn, bridge, RViz, and top-level bringup launch files
- `worlds/`: deterministic obstacle world for LiDAR validation
- `models/`: TurtleBot3-style differential drive robot model used by Gazebo
- `urdf/`: RViz and TF-friendly robot description
- `config/`: structured `ros_gz_bridge` YAML mapping
- `rviz/`: ready-to-use RViz display profile
- `enpm690_hw3_phase1/`: Python nodes for keyboard teleop, teleop logging, and odom-to-TF

## Phase 1 features

- Gazebo Harmonic launches with a custom obstacle world.
- A TurtleBot3-like differential-drive robot is spawned at a known pose.
- ROS-to-Gazebo command velocity bridging is handled from YAML.
- LiDAR, odometry, and clock are bridged back into ROS 2.
- `robot_state_publisher` provides the robot model and static TF.
- A Python TF broadcaster converts `/odom` into the `odom -> base_footprint` transform for RViz.
- A Python teleop logger prints readable command semantics in real time.
- A Python keyboard teleop node publishes `/cmd_vel` without relying on Gazebo Classic tooling.
- RViz2 is preconfigured for the robot model, TF tree, LaserScan, and odometry.

## Runtime dependencies

Install the Phase 1 runtime dependencies on a Linux machine with ROS 2 Jazzy:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2
```

If your Jazzy installation does not already include the standard message packages and TF tools, install the desktop metapackage:

```bash
sudo apt install ros-jazzy-desktop
```

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select enpm690_hw3_phase1
source install/setup.bash
```

## Phase 1 run procedure

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch enpm690_hw3_phase1 phase1_bringup.launch.py
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run enpm690_hw3_phase1 keyboard_teleop
```

Notes:

- The main launch starts Gazebo Harmonic, the robot spawn flow, the bridge, `robot_state_publisher`, the odom-to-TF helper, the teleop logger, and RViz2.
- The keyboard teleop node is run separately so it can own terminal stdin reliably.
- A convenience launch wrapper also exists at `ros2 launch enpm690_hw3_phase1 teleop_keyboard.launch.py`, but direct `ros2 run` is the safer choice for keyboard capture during demos.
- The robot spawn pose defaults to `x=0.0`, `y=0.0`, `z=0.08`, `yaw=0.0`.

## Phase 1 topic interface

Primary ROS-side topics:

- `/cmd_vel`
- `/scan`
- `/odom`
- `/tf`
- `/tf_static`
- `/clock`

## Phase 1 validation checklist

After bringup:

```bash
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /cmd_vel
ros2 run tf2_tools view_frames
```

Manual validation steps:

1. Confirm the robot appears in Gazebo and RViz2.
2. Drive forward toward the obstacle directly in front of the spawn pose.
3. Turn left and right and watch the scan arc reshape in RViz2.
4. Confirm `/scan` and `/odom` update continuously.
5. Confirm the teleop logger prints readable events such as `forward`, `turn left`, and `stop`.

## World description

The obstacle world is intentionally simple and deterministic:

- bounded rectangular arena
- large open start area near the origin
- obstacle directly ahead of the spawn pose
- obstacle cluster on the left
- obstacle cluster on the right
- a few farther obstacles to make scan geometry obvious while driving

This is a debugging world for teleop and LiDAR verification, not a final autonomy benchmark map.

## Phase 2 Python environment policy

Phase 2 mixes ROS 2 Python packages with ML dependencies such as `gymnasium` and `stable-baselines3`.
This repo treats play/demo nodes and training/evaluation scripts differently on purpose:

- ROS runtime nodes continue to use `ros2 launch` and `ros2 run`.
- ML scripts use `python -m enpm690_hw3_phase2.train_ppo` and `python -m enpm690_hw3_phase2.eval_policy`.

Why:

- Ubuntu 24.04 and Python 3.12 enforce PEP 668 protections, so installing ML packages into the system Python is not the right workflow.
- ROS 2 documentation recommends using a virtual environment for extra Python packages.
- In binary-installed ROS 2 setups, `ros2 run` console entry scripts may still be generated with `/usr/bin/python3`, which bypasses the active project `.venv`.

Recommended shell setup for every new terminal:

```bash
source /Users/jerrychuang/Desktop/2026_spring/robot_learning/ENPM690_hw3/scripts/dev_env.sh
```

That script activates `.venv`, sources `/opt/ros/jazzy`, and sources the workspace `install/setup.*`.
After that, the official Phase 2 ML commands are:

```bash
python -m enpm690_hw3_phase2.train_ppo --launch-stack --timesteps 20000 --output-dir artifacts/phase2_ppo
python -m enpm690_hw3_phase2.eval_policy --launch-stack --headless --model artifacts/phase2_ppo/ppo_shark_hunt.zip --episodes 3
```
