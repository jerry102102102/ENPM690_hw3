# ENPM690 HW3 Submission

This folder is the complete hand-in package.

## Included Files

- `demo_video.mp4` - required demonstration video
- `report.md` - short technical report
- `run_demo.sh` - one-command artifact regeneration
- `teleop_input_log.txt` - teleop command log
- `demo_metrics.md` - autonomous experiment summary
- `code/` - self-contained source bundle (`phase2` package + artifact scripts)

## Regenerate Artifacts

Run from this folder:

```bash
./run_demo.sh
```

Generated files:

- `demo_video.mp4`
- `teleop_input_log.txt`
- `demo_metrics.md`

## Runtime Requirements

- Python 3.10+
- `matplotlib`
- `ffmpeg`

Ubuntu install:

```bash
sudo apt update
sudo apt install -y python3-matplotlib ffmpeg
```

## Code Layout

- `code/enpm690_hw3_phase2/` - phase2 package and launch/config/model files
- `code/scripts/generate_submission_artifacts.py` - deterministic demo generator
- `code/scripts/regenerate_submission.sh` - helper script used by `run_demo.sh`
