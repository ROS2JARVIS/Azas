#!/usr/bin/env bash
set -euo pipefail

# No-motion TF pipeline helper for camera_color_optical_frame -> base_link.
# It verifies ROS can write logs under /tmp and that Humble's
# static_transform_publisher supports the named-argument interface.

ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros2_logs}"

mkdir -p "${ROS_LOG_DIR}"
touch "${ROS_LOG_DIR}/test_write"
export ROS_LOG_DIR

set +u
source /opt/ros/humble/setup.bash
if [[ -f /home/ssu/Azas/install/setup.bash ]]; then
  source /home/ssu/Azas/install/setup.bash
fi
set -u

echo "[Azas] ROS_LOG_DIR=${ROS_LOG_DIR}"
echo "[Azas] Checking static_transform_publisher named-argument support."
ros2 run tf2_ros static_transform_publisher --help >/tmp/azas_static_tf_help.txt
grep -q -- "--frame-id" /tmp/azas_static_tf_help.txt
grep -q -- "--child-frame-id" /tmp/azas_static_tf_help.txt
grep -q -- "--roll" /tmp/azas_static_tf_help.txt
grep -q -- "--pitch" /tmp/azas_static_tf_help.txt
grep -q -- "--yaw" /tmp/azas_static_tf_help.txt
echo "[PASS] static_transform_publisher help supports named arguments"

cat <<'EOF'

Next no-motion runtime checks:

Terminal 1:
  mkdir -p /tmp/ros2_logs
  touch /tmp/ros2_logs/test_write
  export ROS_LOG_DIR=/tmp/ros2_logs
  source /opt/ros/humble/setup.bash
  ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
    model:=m0609 mode:=virtual host:=127.0.0.1 port:=12345

Terminal 2:
  mkdir -p /tmp/ros2_logs
  touch /tmp/ros2_logs/test_write
  export ROS_LOG_DIR=/tmp/ros2_logs
  source /opt/ros/humble/setup.bash
  source /home/ssu/Azas/install/setup.bash
  ros2 launch azas_bringup yolo_to_floor_place.launch.py \
    publish_camera_base_tf:=true \
    source_frame:=camera_color_optical_frame \
    target_class_names:=cup,tumbler,bottle \
    selection_policy:=largest_bbox \
    depth_window_size:=7 \
    min_depth_m:=0.15 \
    max_depth_m:=2.0 \
    debug_pose_logging:=true \
    camera_base_tf_x:=0.0 \
    camera_base_tf_y:=0.0 \
    camera_base_tf_z:=0.0 \
    camera_base_tf_roll:=0.0 \
    camera_base_tf_pitch:=0.0 \
    camera_base_tf_yaw:=0.0 \
    allow_demo_tumbler_position_fallback:=false

Terminal 3:
  mkdir -p /tmp/ros2_logs
  touch /tmp/ros2_logs/test_write
  export ROS_LOG_DIR=/tmp/ros2_logs
  source /opt/ros/humble/setup.bash
  ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
  ros2 topic echo /jarvis/tumbler_dispenser/tumbler_pose

Do not use placeholder static TF values for real robot execution.
EOF
