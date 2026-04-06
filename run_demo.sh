#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

python code/scripts/generate_demo_artifacts.py --output-dir "$REPO_ROOT" --fps 10 --auto-speedup 4

echo "Generated artifacts in: $REPO_ROOT"
