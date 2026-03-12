# ENPM690 Homework 3

Current HW3 repository focused on:

- Phase 1 ROS 2 + Gazebo stack (`enpm690_hw3_phase1`)
- Phase 2 Pac-Man-style play/auto stack (`enpm690_hw3_phase2`)
- Submission artifact regeneration pipeline (`scripts/`, `submission/`)

## Regenerate submission artifacts

From repo root:

```bash
./scripts/regenerate_submission.sh
```

Generated outputs:

- `submission/demo_video.mp4`
- `submission/teleop_input_log.txt`
- `submission/demo_metrics.md`

## Build ROS packages (optional)

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select enpm690_hw3_phase1 enpm690_hw3_phase2
source install/setup.bash
```

## Run Phase 1

```bash
ros2 launch enpm690_hw3_phase1 phase1_bringup.launch.py
ros2 run enpm690_hw3_phase1 keyboard_teleop
```

## Run Phase 2

Teleop mode:

```bash
ros2 launch enpm690_hw3_phase2 phase2_play_teleop.launch.py
ros2 run enpm690_hw3_phase1 keyboard_teleop
```

Autonomous mode:

```bash
ros2 launch enpm690_hw3_phase2 phase2_play_auto.launch.py
```

## Submission docs

- `submission/README.md`
- `submission/report.md`
