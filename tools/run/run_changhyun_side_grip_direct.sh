#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/ssu/Azas}"
SERVICE_PREFIX="${SERVICE_PREFIX:-dsr01}"
DISPLAY="${DISPLAY:-:0}"
XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-9}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

cd "${ROOT}"

set +u
source /opt/ros/humble/setup.bash
if [[ -f /home/ssu/ws_moveit/install/setup.bash ]]; then
  source /home/ssu/ws_moveit/install/setup.bash
fi
if [[ -f /home/ssu/ros2_ws/install/setup.bash ]]; then
  source /home/ssu/ros2_ws/install/setup.bash
fi
if [[ -f "${ROOT}/install/setup.bash" ]]; then
  source "${ROOT}/install/setup.bash"
else
  source "${ROOT}/install/local_setup.bash"
fi
source "${ROOT}/install/dsr_practice/share/dsr_practice/package.bash"
set -u

export DISPLAY XAUTHORITY ROS_DOMAIN_ID ROS_LOCALHOST_ONLY
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"
export PYTHONPATH="${ROOT}/tools/run/python_compat:${PYTHONPATH:-}"
export PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0
mkdir -p "${ROS_LOG_DIR}"

echo "[Azas] START Changhyun side-grip direct tmux command"
echo "[Azas] OpenCV window: confirm cup, then press p. Quit with q/Esc."
echo "[Azas] service_prefix=${SERVICE_PREFIX} DISPLAY=${DISPLAY} XAUTHORITY=${XAUTHORITY}"
echo "[Azas] ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "[Azas] start_joint_state_relay=${START_JOINT_STATE_RELAY:-false}"
echo "[Azas] moving to side-grip camera scan pose before starting YOLO"

trap 'jobs -pr | xargs -r kill >/dev/null 2>&1 || true' EXIT

python3 "${ROOT}/tools/run/direct_movej_joints.py" \
  --service-prefix "${SERVICE_PREFIX}" \
  --j1 3.0 --j2 -12.7 --j3 44.0 --j4 -9.0 --j5 133.0 --j6 90.0 \
  --velocity 20 --acceleration 20 \
  --j5-min-deg -150 --j5-max-deg 150 \
  --timeout-sec 60 --motion-timeout-sec 120 \
  --execute --confirm ENABLE_DIRECT_MOVEJ

echo "[Azas] side-grip camera scan pose reached; starting YOLO/OpenCV node"

ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
  --frame-id world --child-frame-id base_link &

ros2 run azas_perception hand_eye_static_tf_node \
  --ros-args -p compose_timeout_sec:=30.0 -p allow_direct_fallback:=false &

# Publish only the attached RG2/link_6 collision object before launching the
# picker. Keep the launch-side include disabled because it also starts an
# auxiliary robot_state_publisher and can stall MoveItPy initialization in the
# field tmux workflow.
ros2 run azas_motion link6_gripper_collision_node &

if [[ "${START_JOINT_STATE_RELAY:-false}" == "true" ]]; then
  (
    sleep 5
    python3 "${ROOT}/src/dsr_practice/dsr_practice/joint_state_relay.py" \
      --ros-args -r __node:=azas_joint_state_relay \
      -p input_topic:=/"${SERVICE_PREFIX}"/joint_states \
      -p output_topic:=/joint_states
  ) &
fi

ros2 launch dsr_practice yolo_cup_pick_node.launch.py \
  model_path:="${ROOT}/local_models/best.pt" \
  conf:=0.35 imgsz:=640 device:=cpu target_class:=cup \
  auto_pick:=false auto_pick_interval:=8.0 exit_after_pick:=false \
  depth_patch_radius:=7 min_depth_valid_ratio:=0.03 min_depth_m:=0.15 max_depth_m:=1.20 \
  redetect_on_approach:=false redetect_settle_sec:=0.5 \
  grasp_mode:=side side_far_stage_enabled:=false side_approach_offset:=0.18 \
  side_short_stage_backoff_m:=0.08 side_grasp_stop_backoff_m:=0.04 side_close_underreach_m:=0.03 \
  side_low_retry_lift_m:=0.0 side_low_retry_attempts:=0 \
  side_linear_approach_enabled:=true side_final_slide_enabled:=false \
  side_fixed_grasp_z_enabled:=true side_fixed_grasp_z:=0.07 side_project_bbox_center_to_fixed_z:=true \
  side_candidate_plan_check_enabled:=true pre_pick_joint1_clearance_deg:=12.0 \
  side_move_to_initial_center_before_close:=false verify_motion:=false \
  skip_initial_home_move:=true move_to_camera_home:=false move_joint_home_before_camera_home:=false camera_home_mode:=joint min_motion_z:=0.07 \
  workspace_xy_clamp_enabled:=false return_home_after_task:=false return_to_camera_home_after_attempt:=true \
  workspace_collision_scene_enabled:=false table_collision_enabled:=true table_surface_z:=0.0 table_thickness:=0.04 \
  table_size_x:=1.10 table_size_y:=0.65 table_center_x:=0.29 table_center_y:=0.0 table_collision_expand_to_workspace_walls:=true \
  workspace_boundary_collision_enabled:=true dispenser_collision_enabled:=true dispenser_collision_publish_objects:=true \
  dispenser_collision_publish_markers:=true link6_gripper_collision_enabled:=false \
  dispenser_collision_config_path:="${ROOT}/src/azas_bringup/config/measured_dispenser_collision.yaml" \
  moveit_controller_name:=/"${SERVICE_PREFIX}"/dsr_moveit_controller start_joint_state_relay:=false
