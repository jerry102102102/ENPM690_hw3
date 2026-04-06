# ENPM690 HW3 Demo Bundle

This repository root is the submission bundle.

It contains the exact instructor-facing materials needed to review and regenerate the demo artifacts:

- `README.md`
- `report.md`
- `run_demo.sh`
- `demo_video.mp4`
- `demo_metrics.md`
- `teleop_input_log.txt`
- `code/`

## Runtime

- Docker is **not** required.
- Python dependencies are managed with `uv`.
- `ffmpeg` must be installed and available on `PATH`.

## Regenerate the demo

From the repository root:

```bash
uv sync
uv run ./run_demo.sh
```

This overwrites and regenerates the root-level artifacts:

- `demo_video.mp4`
- `demo_metrics.md`
- `teleop_input_log.txt`

## What to review

- `report.md` — short technical writeup
- `demo_video.mp4` — demo video
- `demo_metrics.md` — scalar metrics and 5-profile sweep summary
- `teleop_input_log.txt` — teleop command log

## Code layout

All implementation content required to regenerate the demo is inside `code/`:

```text
code/
├── enpm690_hw3_phase2/
└── scripts/
```

## Minimal rerun command

```bash
uv run ./run_demo.sh
```
