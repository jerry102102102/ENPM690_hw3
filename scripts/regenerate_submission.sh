#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

mkdir -p submission
python3 scripts/generate_submission_artifacts.py --output-dir submission --fps 10

echo "Generated artifacts in: $REPO_ROOT/submission"
