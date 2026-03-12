# Demo Metrics

## Scalar Metrics

- auto_close_call_frames: 6.00
- auto_collisions: 0.00
- auto_completion_flag: 1.00
- auto_completion_time_s: 10.85
- auto_pellets_remaining: 0.00
- auto_score: 10.00
- auto_speedup_factor: 4.00
- baseline_reference_completion_s: 43.40
- compare_aggressive_close_frames: 5.00
- compare_aggressive_collisions: 0.00
- compare_aggressive_score: 4.00
- compare_cautious_close_frames: 0.00
- compare_cautious_collisions: 0.00
- compare_cautious_score: 2.00
- teleop_collisions: 24.00
- teleop_pellets_remaining: 10.00
- teleop_score: 0.00

## 5-Group Autonomous Parameter Sweep

| Group | max_linear | heading_gain | completion_flag | score | collisions | close_calls | completion_time_s |
|---|---:|---:|---:|---:|---:|---:|---:|
| G1_SAFE | 1.00 | 1.10 | 1 | 10 | 0 | 5 | 12.95 |
| G2_BASE | 1.20 | 1.30 | 1 | 10 | 0 | 6 | 10.85 |
| G3_BAL | 1.40 | 1.30 | 1 | 10 | 0 | 4 | 9.68 |
| G4_QUICK | 1.50 | 1.60 | 1 | 10 | 0 | 4 | 8.83 |
| G5_FAST | 1.60 | 1.80 | 1 | 10 | 0 | 5 | 8.30 |

## Parameter Impact Analysis

- Autonomous playback acceleration set to 4x; baseline completion moved from 43.40s to 10.85s (4.00x faster).
- Highest-speed completion in the sweep: G5_FAST at 8.30s with 0 collisions.
- Safest setting by close-calls/collisions: G4_QUICK with 4 close-calls and 0 collisions.
- All five experiment groups reached full pellet collection (completion_flag=1).
