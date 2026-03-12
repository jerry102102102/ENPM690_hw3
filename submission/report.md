# ENPM690 HW3 Short Report

## Objective

Update the autonomous demo to finish much faster while preserving full pellet collection, expose tunable control parameters, and document quantitative results across five experiment groups.

## Teleop Segment (Rubric Coverage)

The teleop segment is preserved in the demo video and remains the first section:

- real-time command schedule (`W`, `A`, `D`, `SPACE`),
- live on-screen input monitor,
- exported `teleop_input_log.txt` for reproducible command trace.

## Autonomous Speedup Strategy

Autonomous completion speed was improved with two explicit tuning levers:

- `max_linear` (speed term),
- `heading_gain` (steering sensitivity term).

For demo playback, autonomous simulation is rendered at **4x playback speed** (four simulation control steps per rendered frame) so behavior is preserved while demo duration is much shorter.

Measured baseline impact:

- baseline autonomous completion: **43.40 s** (reference playback),
- accelerated autonomous completion: **10.85 s**,
- net speedup: **4.00x**.

## 5-Group Parameter Sweep Summary

Five parameter groups were executed and logged in `demo_metrics.md` with:

- completion flag,
- score,
- collisions,
- close-calls,
- completion time.

Observed trend:

- higher `max_linear` reduces completion time,
- higher `heading_gain` improves turn responsiveness and can reduce close-calls in this map,
- all five groups achieved full pellet collection (`completion_flag=1`, `score=10`).

## Reproducibility

Regenerate artifacts with:

```bash
./scripts/regenerate_submission.sh
```

Generated files:

- `submission/demo_video.mp4`
- `submission/teleop_input_log.txt`
- `submission/demo_metrics.md`

## Notes

ROS 2 Phase 1/2 packages remain intact. Autonomous runtime tunables are exposed in `phase2_play_auto.launch.py` (`forward_speed`, `turn_gain`, `target_signal_gain`) and in `phase2_params.yaml`.
