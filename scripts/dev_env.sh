_enpm690_hw3_dev_env() {
  set -euo pipefail

  local this_file=""
  if [[ -n "${ZSH_VERSION:-}" ]]; then
    this_file="${(%):-%N}"
  elif [[ -n "${BASH_VERSION:-}" ]]; then
    this_file="${BASH_SOURCE[0]}"
  else
    this_file="$0"
  fi

  local script_dir
  script_dir="$(cd "$(dirname "${this_file}")" && pwd)"
  local repo_root
  repo_root="$(cd "${script_dir}/.." && pwd)"

  if [[ ! -f "${repo_root}/.venv/bin/activate" ]]; then
    echo "Missing virtual environment at ${repo_root}/.venv" >&2
    echo "Create it first with: uv venv .venv --python /usr/bin/python3" >&2
    return 1
  fi

  if [[ -f /opt/ros/jazzy/setup.zsh && -n "${ZSH_VERSION:-}" ]]; then
    source /opt/ros/jazzy/setup.zsh
  elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
  else
    echo "ROS 2 Jazzy setup script not found under /opt/ros/jazzy" >&2
    return 1
  fi

  if [[ -f "${repo_root}/install/setup.zsh" && -n "${ZSH_VERSION:-}" ]]; then
    source "${repo_root}/install/setup.zsh"
  elif [[ -f "${repo_root}/install/setup.bash" ]]; then
    source "${repo_root}/install/setup.bash"
  else
    echo "Workspace install setup script not found. Run colcon build first." >&2
    return 1
  fi

  source "${repo_root}/.venv/bin/activate"

  export ENPM690_HW3_REPO_ROOT="${repo_root}"

  echo "Activated ENPM690 HW3 dev environment"
  echo "REPO_ROOT=${repo_root}"
  echo "PYTHON=$(command -v python)"
  python -c "import sys; print(f'sys.executable={sys.executable}'); print(f'sys.version={sys.version}')"
}

_enpm690_hw3_dev_env "$@"
unset -f _enpm690_hw3_dev_env
