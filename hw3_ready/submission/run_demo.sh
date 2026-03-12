#!/usr/bin/env bash
set -euo pipefail

SUBMISSION_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SUBMISSION_DIR"

./code/scripts/regenerate_submission.sh
