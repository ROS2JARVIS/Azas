#!/usr/bin/env bash
set -euo pipefail

AZAS_ROOT="/home/ssu/Azas"
CONFIG_PATH="${CONFIG_PATH:-${AZAS_ROOT}/install/azas_bringup/share/azas_bringup/config/measured_dispenser_collision.yaml}"

cd "${AZAS_ROOT}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
source "${AZAS_ROOT}/install/local_setup.bash"
set -u

if [[ ! -f "${CONFIG_PATH}" ]]; then
  CONFIG_PATH="${AZAS_ROOT}/src/azas_bringup/config/measured_dispenser_collision.yaml"
fi

exec ros2 run azas_motion measured_dispenser_collision_scene_node \
  --ros-args \
  -p config_path:="${CONFIG_PATH}" \
  -p publish_period_sec:=2.0
