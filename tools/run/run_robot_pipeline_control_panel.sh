#!/usr/bin/env bash
set -euo pipefail

cd /home/ssu/Azas

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
source /home/ssu/Azas/install/local_setup.bash
set -u

exec python3 tools/run/robot_pipeline_control_server.py
