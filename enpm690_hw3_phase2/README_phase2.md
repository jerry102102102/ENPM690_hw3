# ENPM690 Homework 3 Phase 2

Phase 2 now uses a pure `matplotlib` 2D simulator as the main path.
Gazebo is no longer the recommended Phase 2 workflow.

## Main idea

- Pac-Man = robot
- pellets = fixed points
- ghost = one simple waypoint patrol agent
- visualization = `matplotlib`
- teleop and auto both run inside the same 2D simulator

## Requirements

Install matplotlib if needed:

```bash
sudo apt install python3-matplotlib
```

## Run teleop

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run enpm690_hw3_phase2 pacman_matplotlib_sim -- --mode teleop
```

Controls:

- `W` / `Up`: forward
- `S` / `Down`: backward
- `A` / `Left`: turn left
- `D` / `Right`: turn right
- `R`: reset

## Run autonomous baseline

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run enpm690_hw3_phase2 pacman_matplotlib_sim -- --mode auto
```

The auto mode uses:

- nearest-pellet target selection
- simple simulated LiDAR ray distances to walls
- a simple ghost avoidance term

## Rules

- each pellet gives `+1`
- touching the ghost ends the run
- collecting all pellets is a victory
- time limit is `30s`

## Notes

- Phase 1 remains unchanged and still uses the real ROS 2 + Gazebo stack
- old Gazebo-heavy Phase 2 files remain only as legacy reference
