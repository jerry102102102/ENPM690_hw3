#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ ! -f "${REPO_ROOT}/.venv/bin/activate" ]]; then
  echo "Missing virtual environment at ${REPO_ROOT}/.venv" >&2
  echo "Create it first with: uv venv .venv --python /usr/bin/python3" >&2
  return 1 2>/dev/null || exit 1
fi

if [[ -f /opt/ros/jazzy/setup.zsh && -n "${ZSH_VERSION:-}" ]]; then
  source /opt/ros/jazzy/setup.zsh
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
else
  echo "ROS 2 Jazzy setup script not found under /opt/ros/jazzy" >&2
  return 1 2>/dev/null || exit 1
fi

if [[ -f "${REPO_ROOT}/install/setup.zsh" && -n "${ZSH_VERSION:-}" ]]; then
  source "${REPO_ROOT}/install/setup.zsh"
elif [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  source "${REPO_ROOT}/install/setup.bash"
else
  echo "Workspace install setup script not found. Run colcon build first." >&2
  return 1 2>/dev/null || exit 1
fi

source "${REPO_ROOT}/.venv/bin/activate"

export ENPM690_HW3_REPO_ROOT="${REPO_ROOT}"

echo "Activated ENPM690 HW3 dev environment"
echo "REPO_ROOT=${REPO_ROOT}"
echo "PYTHON=$(command -v python)"
python -c "import sys; print(f'sys.executable={sys.executable}'); print(f'sys.version={sys.version}')"
