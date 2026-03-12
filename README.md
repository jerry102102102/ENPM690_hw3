# ENPM690 HW3 Submission Package

This repository is intentionally reduced for direct submission.

## Submission Folder

Use only `hw3_ready/`, which contains:

- `enpm690_hw3_phase2/`
- `scripts/regenerate_submission.sh`
- `scripts/generate_submission_artifacts.py`
- `submission/` (generated artifacts)

## Regenerate Artifacts

```bash
cd hw3_ready
./scripts/regenerate_submission.sh
```

Artifacts are regenerated in:

- `hw3_ready/submission/demo_video.mp4`
- `hw3_ready/submission/teleop_input_log.txt`
- `hw3_ready/submission/demo_metrics.md`

## Optional ROS Run (Phase 2)

```bash
source /opt/ros/jazzy/setup.bash
cd hw3_ready
colcon build --packages-select enpm690_hw3_phase2
source install/setup.bash
ros2 launch enpm690_hw3_phase2 phase2_play_auto.launch.py
```
