# ENPM690 HW3 Report

## Objective

Demonstrate teleoperation logging, autonomous pellet collection, and parameterized controller behavior for Homework 3.

## Demonstration Coverage

The video includes:

- teleop control sequence with on-screen command feedback
- exported teleop log (`teleop_input_log.txt`)
- autonomous run with LiDAR-based targeting and obstacle response
- five parameter-group comparison summarized in `demo_metrics.md`

## Autonomous Method

The autonomous controller uses segmented LiDAR ranges and target-heading pursuit. Main tuning terms are:

- `max_linear` for forward speed
- `heading_gain` for steering sensitivity

Playback is rendered at 4x simulation speed to shorten demonstration runtime while preserving controller behavior.

## Parameter Sweep Summary

Five parameter groups are evaluated and logged with:

- completion flag
- score
- collisions
- close-calls
- completion time

All groups complete pellet collection, with faster groups reducing completion time.

## Reproducibility

From `hw3_ready/submission/`:

```bash
./run_demo.sh
```

This regenerates `demo_video.mp4`, `teleop_input_log.txt`, and `demo_metrics.md` using code in `code/`.

## Submission Stack

The submitted code path is intentionally matplotlib-only for artifact generation and excludes ROS/Gazebo launch assets and runtime nodes.
