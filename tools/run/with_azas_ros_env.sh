#!/usr/bin/env bash
# Run a ROS command with the Azas field defaults.
# This avoids asking operators to remember ROS_DOMAIN_ID / DDS env exports.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

set +u
source /opt/ros/humble/setup.bash

if [[ -f /home/ssu/ws_moveit/install/setup.bash ]]; then
  source /home/ssu/ws_moveit/install/setup.bash
fi
if [[ -f /home/ssu/ros2_ws/install/setup.bash ]]; then
  source /home/ssu/ros2_ws/install/setup.bash
fi
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  source "${ROOT_DIR}/install/setup.bash"
fi
set -u

export ROS_DOMAIN_ID="${AZAS_ROS_DOMAIN_ID:-9}"
export ROS_LOCALHOST_ONLY="${AZAS_ROS_LOCALHOST_ONLY:-${ROS_LOCALHOST_ONLY:-1}}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/azas_mpl_config}"

exec "$@"
