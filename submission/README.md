# ENPM690 HW3 Submission Package

This submission uses a deterministic lightweight simulation pipeline (no Gazebo required) to generate the required HW3 demo artifacts.

## Contents

- `demo_video.mp4`: Required demonstration video.
- `teleop_input_log.txt`: Real-time teleop input log used in Segment 1.
- `demo_metrics.md`: Summary metrics from the generated runs.
- `run_demo.sh`: Submission-local one-command artifact generator.
- `report.md`: Short technical report.

## What the Demo Video Shows

1. **Teleop with real-time input logging + visual feedback**
- Scripted keyboard-like commands (`W`, `A`, `D`, `SPACE`) are applied.
- Input events are logged live on-screen and also exported to `teleop_input_log.txt`.
- The robot trajectory, simulated LiDAR beams, and environment feedback are visualized.

2. **Autonomous behavior with sensor display**
- A deterministic LiDAR-segmentation controller runs autonomously to clear pellets.
- LiDAR bins are shown in real time with a marked safety-distance threshold.
- Overlay shows current target plus collected/remaining counts.

3. **Tunable parameter impact comparison**
- Two autonomous tunings run side-by-side:
  - Cautious: larger safety margin, lower max speed.
  - Aggressive: smaller safety margin, higher max speed.
- Visual and metric differences are shown in the same scene.

## Regenerate Artifacts

From repo root:

```bash
./scripts/regenerate_submission.sh
```

Equivalent (from `submission/`):

```bash
./run_demo.sh
```

## Runtime Requirements

- Python 3.10+
- `matplotlib`
- `ffmpeg`

Install on Ubuntu if needed:

```bash
sudo apt update
sudo apt install -y python3-matplotlib ffmpeg
```

## Run Existing ROS Packages (Optional)

The repository still contains ROS 2 Phase 1/2 packages under:

- `enpm690_hw3_phase1/`
- `enpm690_hw3_phase2/`

For this submission package, the required deliverables are generated through the lightweight simulator pipeline above for faster and more deterministic reproduction.
