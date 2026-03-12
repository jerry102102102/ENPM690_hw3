#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CODE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SUBMISSION_DIR="$(cd "$CODE_ROOT/.." && pwd)"

cd "$CODE_ROOT"
python3 scripts/generate_submission_artifacts.py --output-dir "$SUBMISSION_DIR" --fps 10 --auto-speedup 4

echo "Generated artifacts in: $SUBMISSION_DIR"
