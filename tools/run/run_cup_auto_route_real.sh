#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source_if_exists() {
  local path="$1"
  if [[ -f "${path}" ]]; then
    # shellcheck disable=SC1090
    source "${path}"
  fi
}

set +u
source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/ssu/ws_moveit/install/setup.bash
source_if_exists /home/ssu/ros2_ws/install/setup.bash
source_if_exists "${ROOT_DIR}/install/setup.bash"
set -u

exec python3 "${ROOT_DIR}/tools/run/cup_auto_route_real.py" "$@"
