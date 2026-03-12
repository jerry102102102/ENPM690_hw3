# ENPM690 Homework 3 Phase 2 (Submission Subset)

This package is intentionally pruned for submission artifact regeneration only.

## Scope

- Keeps only the utility modules imported by `code/scripts/generate_submission_artifacts.py`
- Removes ROS/Gazebo launch, world, model, RViz, and runtime node code
- Supports deterministic video, teleop log, and metrics generation in `hw3_ready/submission/`

## Kept Python Modules

- `enpm690_hw3_phase2/constants.py`
- `enpm690_hw3_phase2/geometry_utils.py`

## Regeneration Entry Point

Run from `hw3_ready/submission/`:

```bash
./run_demo.sh
```
