#!/usr/bin/env bash
set -euo pipefail

# Quick check after run_robot_dryrun.sh is running.
# Success condition for a motion-facing cup: /azas/cup_detection status starts
# with detected:upright. Lid detections are perception evidence only and do not
# satisfy the cup pose contract used by /jarvis/tumbler_dispenser/tumbler_pose.

ROS2_DAEMON_FLAG="${ROS2_DAEMON_FLAG:---no-daemon}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
set -u

topic_list() {
  if [[ -n "${ROS2_DAEMON_FLAG}" ]]; then
    ros2 topic list "${ROS2_DAEMON_FLAG}"
  else
    ros2 topic list
  fi
}

echo "[Azas] Camera topics:"
topic_list | grep -E "camera|depth|color|rgb|info" || true

echo
echo "[Azas] Robot/perception topics:"
topic_list | grep -E "azas|jarvis|tumbler|cup|rg2" || true

echo
echo "[Azas] One cup detection message:"
if [[ -n "${ROS2_DAEMON_FLAG}" ]]; then
  echo_cmd=(ros2 topic echo --once /azas/cup_detection "${ROS2_DAEMON_FLAG}")
else
  echo_cmd=(ros2 topic echo --once /azas/cup_detection)
fi
timeout 10 "${echo_cmd[@]}" || {
  echo "[Azas] No /azas/cup_detection message within 10 seconds."
  exit 1
}

echo
echo "[Azas] Motion-facing cup contract: status must start with detected:upright."
echo "[Azas] detected:lid is lid evidence only; it is not a valid cup pose for motion."
echo "[Azas] If status is no_tumbler_detection or rejected:*, move an upright tumbler body to the camera center and rerun this script."
