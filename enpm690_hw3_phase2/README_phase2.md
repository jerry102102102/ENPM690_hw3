# ENPM690 Homework 3 Phase 2

Phase 2 provides a Pac-Man-style game flow on top of the Phase 1 ROS 2 + Gazebo stack, plus a deterministic `matplotlib` demo path used by submission artifact generation.

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

- deterministic LiDAR segmentation into obstacle risk + candidate pursuit direction
- deterministic next-target heading selection over active pellets
- obstacle-aware pursuit with ghost avoidance

## Rules

- each pellet gives `+1`
- touching the ghost ends the run
- collecting all pellets is a victory
- time limit is `30s`

## Notes

- Phase 1 remains unchanged and still uses the real ROS 2 + Gazebo stack
- `scripts/regenerate_submission.sh` generates the required HW3 submission artifacts without launching Gazebo
