#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
REPORT="${REPORT:-/tmp/azas_team_pc_bootstrap_report.txt}"
ROS_SETUP="/opt/ros/humble/setup.bash"
INSTALL_SETUP="$ROOT/install/local_setup.bash"
REQUIRED_BRANCH="${REQUIRED_BRANCH:-develop}"

cd "$ROOT"
: >"$REPORT"

log() {
  echo "$*" | tee -a "$REPORT"
}

run() {
  log ""
  log "[RUN] $*"
  "$@" 2>&1 | tee -a "$REPORT"
}

check_file() {
  local path="$1"
  if [[ -e "$path" ]]; then
    log "[OK] $path"
  else
    log "[FAIL] missing: $path"
    return 1
  fi
}

check_cmd() {
  local cmd="$1"
  if command -v "$cmd" >/dev/null 2>&1; then
    log "[OK] command: $cmd ($(command -v "$cmd"))"
  else
    log "[FAIL] missing command: $cmd"
    return 1
  fi
}

check_ros_pkg() {
  local pkg="$1"
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    log "[OK] ROS package: $pkg -> $(ros2 pkg prefix "$pkg")"
  else
    log "[FAIL] missing ROS package: $pkg"
    return 1
  fi
}

log "[Azas team bootstrap] workspace=$ROOT"
log "[Azas team bootstrap] report=$REPORT"

branch="$(git branch --show-current 2>/dev/null || true)"
log "[INFO] git branch=${branch:-<unknown>}"
if [[ -n "$REQUIRED_BRANCH" && "$branch" != "$REQUIRED_BRANCH" ]]; then
  log "[FAIL] expected branch '$REQUIRED_BRANCH' but current branch is '${branch:-<unknown>}'"
  log "       Fix with: git switch $REQUIRED_BRANCH && git pull --ff-only origin $REQUIRED_BRANCH"
  exit 1
fi

run git fetch origin --prune
run git status --short --branch

if [[ -n "$(git status --porcelain)" ]]; then
  log "[FAIL] worktree has local changes. Commit/stash them before bootstrapping another PC."
  exit 1
fi

run git pull --ff-only origin "$REQUIRED_BRANCH"

check_file "$ROS_SETUP"
check_cmd colcon

if ! command -v rosdep >/dev/null 2>&1; then
  log "[FAIL] missing command: rosdep"
  log "       Install example: sudo apt install -y python3-rosdep && sudo rosdep init || true && rosdep update"
  exit 1
fi
log "[OK] command: rosdep ($(command -v rosdep))"

# shellcheck source=/opt/ros/humble/setup.bash
source "$ROS_SETUP"

log ""
log "[Azas team bootstrap] installing rosdep dependencies"
rosdep install --from-paths src --ignore-src -r -y 2>&1 | tee -a "$REPORT"

log ""
log "[Azas team bootstrap] building workspace"
colcon build --symlink-install 2>&1 | tee -a "$REPORT"

check_file "$INSTALL_SETUP"
# shellcheck source=/dev/null
source "$INSTALL_SETUP"

log ""
log "[Azas team bootstrap] verifying required packages"
missing=0
for pkg in \
  azas_bringup \
  azas_dispenser \
  azas_gripper \
  azas_interfaces \
  azas_motion \
  azas_perception \
  azas_task_manager \
  azas_voice \
  dsr_bringup2 \
  dsr_msgs2 \
  dsr_moveit_config_m0609 \
  realsense2_camera; do
  check_ros_pkg "$pkg" || missing=1
done

log ""
log "[Azas team bootstrap] verifying panel entrypoint"
check_file "$ROOT/tools/run/open_robot_pipeline_control_panel.sh" || missing=1
check_file "$ROOT/tools/run/robot_pipeline_control_server.py" || missing=1
check_file "$ROOT/docs/robot_pipeline_control.html" || missing=1

log ""
log "[Azas team bootstrap] verifying YOLO model link"
if [[ ! -f "$ROOT/local_models/best.pt" ]]; then
  if [[ -f "/home/ssu/Downloads/best.pt" ]]; then
    "$ROOT/tools/setup/link_yolo_model.sh" "/home/ssu/Downloads/best.pt" 2>&1 | tee -a "$REPORT"
  else
    log "[FAIL] missing YOLO model: $ROOT/local_models/best.pt"
    log "       Put best.pt on this PC, then run:"
    log "       bash tools/setup/link_yolo_model.sh /path/to/best.pt"
    missing=1
  fi
else
  log "[OK] YOLO model: $ROOT/local_models/best.pt"
fi

if [[ "$missing" != "0" ]]; then
  log ""
  log "[FAIL] bootstrap completed but required packages/files are missing."
  log "       See report: $REPORT"
  exit 1
fi

log ""
log "[PASS] team PC bootstrap completed."
log "Next:"
log "  cd $ROOT"
log "  bash tools/run/open_robot_pipeline_control_panel.sh"
log ""
log "Robot connection fields must still match that PC/network:"
log "  ROBOT_HOST=<actual robot IP>"
log "  RT_HOST=<that PC IP on robot subnet, or leave blank if panel infers it>"
log "  SERVICE_PREFIX=dsr01"
