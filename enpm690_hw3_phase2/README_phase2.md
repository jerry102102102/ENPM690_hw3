# ENPM690 Homework 3 Phase 2

Phase 2 is now a minimal Pac-Man-style game built directly on top of the working Phase 1 stack.

## Design goals

- reuse the existing Phase 1 Gazebo + RViz + ROS 2 stack
- keep keyboard teleop unchanged
- keep LiDAR, odometry, TF, and `/cmd_vel` flow simple
- use RViz markers for game visualization
- avoid complex fish / shark / RL orchestration in the main path

## Game rules

- the TurtleBot3-like robot is Pac-Man
- pellets are fixed 2D points
- one ghost moves on a simple waypoint loop
- touching a pellet gives `+1`
- touching the ghost ends the game
- time limit is `30s`
- collecting all pellets is a victory

## Runtime topics

- `/phase2/score`
- `/phase2/time_remaining`
- `/phase2/game_state_json`
- `/phase2/pellet_state_json`
- `/phase2/ghost_state_json`
- `/phase2/markers`

## Teleop play

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch enpm690_hw3_phase2 phase2_play_teleop.launch.py
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run enpm690_hw3_phase1 keyboard_teleop
```

This launch reuses the full Phase 1 bringup and adds:

- `pacman_game_manager`
- `marker_publisher`

## Autonomous play

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch enpm690_hw3_phase2 phase2_play_auto.launch.py
```

The baseline controller:

- picks the nearest active pellet
- uses LiDAR sector values for obstacle avoidance
- adds a simple ghost avoidance term when the ghost is close

Useful tuning arguments:

```bash
ros2 launch enpm690_hw3_phase2 phase2_play_auto.launch.py \
  forward_speed:=0.28 \
  turn_gain:=1.7 \
  ghost_avoid_gain:=1.6
```

## Train skeleton

`phase2_train.launch.py` is only a lightweight scaffold now.
It is not the main deliverable.

## Package structure

- `launch/`: teleop, auto, and train-skeleton launches
- `config/phase2_params.yaml`: game and controller parameters
- `rviz/phase2_pacman.rviz`: RViz profile with markers
- `enpm690_hw3_phase2/pacman_game_manager.py`: game rules
- `enpm690_hw3_phase2/pellet_manager.py`: fixed pellets
- `enpm690_hw3_phase2/ghost_manager.py`: simple ghost waypoint motion
- `enpm690_hw3_phase2/pacman_auto_controller.py`: baseline autonomy
- `enpm690_hw3_phase2/marker_publisher.py`: RViz markers and HUD

## Legacy

Older shark / fish / RL-heavy files remain only as legacy reference.
They are not used by the active launch files or console entry points.
