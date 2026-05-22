#!/usr/bin/env bash
set -euo pipefail

cd /home/ssu/Azas

if [[ ! -f /home/ssu/Azas/install/local_setup.bash ]]; then
  cat >&2 <<'MSG'
[Azas] /home/ssu/Azas/install/local_setup.bash가 없습니다.
먼저 이 PC에서 빌드하세요:
  cd /home/ssu/Azas
  bash tools/setup/bootstrap_local_workspace.sh
MSG
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
source /home/ssu/Azas/install/local_setup.bash
set -u

exec python3 tools/run/robot_pipeline_control_server.py
