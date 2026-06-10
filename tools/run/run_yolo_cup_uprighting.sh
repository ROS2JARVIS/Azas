#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

source_if_exists() {
  local setup_file="$1"
  if [[ -f "$setup_file" ]]; then
    set +u
    # shellcheck source=/dev/null
    source "$setup_file"
    set -u
    echo "[Azas] sourced: $setup_file"
  fi
}

source_if_exists /opt/ros/humble/setup.bash

# Local robot PC workspaces used by this project. They are intentionally kept
# outside the Azas repository, but the motion-enabled uprighting flow needs
# their package prefixes when preview_only=false.
source_if_exists /home/ssu/ws_moveit/install/setup.bash
source_if_exists /home/ssu/ros2_ws/install/setup.bash

if [[ -f "$ROOT/install/setup.bash" ]]; then
  set +u
  # shellcheck source=/dev/null
  source "$ROOT/install/setup.bash"
  set -u
elif [[ -f "$ROOT/install/local_setup.bash" ]]; then
  set +u
  # shellcheck source=/dev/null
  source "$ROOT/install/local_setup.bash"
  set -u
else
  cat >&2 <<MSG
[Azas] Missing workspace install setup.
Build first:
  cd $ROOT
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install
MSG
  exit 1
fi

mkdir -p /tmp/azas_ros_logs
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"

exec ros2 launch azas_bringup yolo_cup_uprighting.launch.py "$@"
