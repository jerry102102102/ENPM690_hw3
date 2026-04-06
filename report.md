# ENPM690 HW3 Report

## Objective

Demonstrate three required deliverables in a reproducible instructor-facing bundle:

1. teleoperation logging with visible command feedback
2. autonomous pellet collection with LiDAR-based obstacle response
3. controller-parameter comparison summarized in metrics output

## Included demo sections

The generated video contains:

- a teleop segment with command overlays and exported input log
- an autonomous run that collects all pellets while reacting to obstacles
- a side-by-side controller comparison illustrating parameter effects

## Autonomous method

The autonomous controller uses segmented LiDAR readings, nearest-target pursuit, and wall-risk balancing.
Two exposed control terms drive most behavior:

- `max_linear` controls forward speed
- `heading_gain` controls steering aggressiveness

The autonomous section is rendered at 4x playback speed so the artifact remains short while preserving the same simulated controller behavior.

## Parameter sweep summary

Five controller profiles are evaluated and exported in `demo_metrics.md`.
Each profile records:

- completion flag
- score
- collisions
- close-calls
- completion time

All five profiles complete pellet collection in the generated results.

## Reproducibility

From the repository root:

```bash
uv sync
uv run ./run_demo.sh
```

This regenerates `demo_video.mp4`, `demo_metrics.md`, and `teleop_input_log.txt` at the repository root.

## Packaging note

This submission bundle is intentionally minimal.
All code needed to regenerate the artifacts is under `code/`, and no Docker workflow is required.
