# ENPM690 HW3 Short Report

## Objective

Deliver a shippable HW3 submission package with:

- teleop demo including input logging and visual feedback,
- autonomous behavior demo with sensor display,
- controller parameter impact comparison,
- reproducible one-command artifact generation.

## Engineering Approach

To reduce setup burden and avoid Gazebo dependency for this package, the demo pipeline uses a deterministic 2D simulator rendered with `matplotlib` and encoded with `ffmpeg`.

Key design points:

- fixed world bounds and obstacle geometry,
- fixed pellet/goal layout for reproducibility,
- deterministic teleop command schedule,
- deterministic autonomous controller logic with tunable safety/speed gains,
- scripted video rendering so artifacts can be regenerated exactly.

## Controller Summary

The autonomous controller is deterministic and follows a fixed four-stage loop:

- segment LiDAR into obstacle-risk sectors and candidate pursuit direction signals,
- select next target heading from active pellets plus LiDAR candidate signal,
- apply obstacle-aware pursuit (target term + risk turn + front-clearance gating),
- stop and mark completion when all pellets are collected.

Control outputs:

- linear velocity scales with pursuit-heading alignment and obstacle clearance,
- angular velocity follows fused pursuit heading plus obstacle-risk correction.

## Parameter Comparison

Two tunings are compared side-by-side:

- **Cautious**
  - larger safety distance,
  - lower max speed,
  - generally fewer close obstacle encounters.
- **Aggressive**
  - smaller safety distance,
  - higher max speed,
  - faster collection tendency but greater near-obstacle risk.

The generated `demo_metrics.md` captures run-level values (score, collisions, close-call frame counts, completion flag) for quick reference.

## Reproducibility

One command regenerates the full package artifacts:

```bash
./scripts/regenerate_submission.sh
```

Outputs are created under `./submission/`:

- `demo_video.mp4`
- `teleop_input_log.txt`
- `demo_metrics.md`

## Notes

The existing ROS 2 Phase 1 and Phase 2 code remains in the repository and can still be run in a full ROS environment. The submission artifact path intentionally prioritizes deterministic, lightweight generation for reliability and grading portability.
