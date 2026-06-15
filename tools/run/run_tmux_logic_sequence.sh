#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/ssu/Azas"
LOG_DIR="${ROOT}/log/tmux_logic"
SERVICE_PREFIX="${SERVICE_PREFIX:-dsr01}"
DISPLAY="${DISPLAY:-:0}"
XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
ROS_LOCALHOST_ONLY="${TMUX_LOGIC_ROS_LOCALHOST_ONLY:-1}"
LID_TCP_GRASP_OFFSET_Z_M="${LID_TCP_GRASP_OFFSET_Z_M:--0.032}"
LID_MIN_GRASP_Z_M="${LID_MIN_GRASP_Z_M:-0.020}"

mkdir -p "${LOG_DIR}" /tmp/azas_ros_logs
cd "${ROOT}"

set +u
source /opt/ros/humble/setup.bash
if [[ -f /home/ssu/ws_moveit/install/setup.bash ]]; then source /home/ssu/ws_moveit/install/setup.bash; fi
if [[ -f /home/ssu/ros2_ws/install/setup.bash ]]; then source /home/ssu/ros2_ws/install/setup.bash; fi
if [[ -f "${ROOT}/install/setup.bash" ]]; then source "${ROOT}/install/setup.bash"; else source "${ROOT}/install/local_setup.bash"; fi
set -u

export ROS_LOG_DIR=/tmp/azas_ros_logs
export PYTHONPATH="${ROOT}/tools/run/python_compat:${PYTHONPATH:-}"
export DISPLAY XAUTHORITY ROS_LOCALHOST_ONLY

support_pids=()

cleanup_support() {
  for pid in "${support_pids[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -TERM "${pid}" >/dev/null 2>&1 || true
    fi
  done
}
trap cleanup_support EXIT

log_msg() {
  printf '\n[%(%Y-%m-%d %H:%M:%S)T] %s\n' -1 "$*"
}

wait_service_call() {
  local service="$1"
  local type="$2"
  local label="$3"
  local timeout="${4:-8.0}"
  local attempt=1
  while true; do
    log_msg "waiting: ${label} (${service}) attempt=${attempt}"
    if timeout 12s python3 tools/run/ros_call_empty_service.py "${service}" "${type}" --timeout "${timeout}"; then
      return 0
    fi
    sleep 2
    attempt=$((attempt + 1))
  done
}

wait_topic_once() {
  local topic="$1"
  local label="$2"
  local attempt=1
  while true; do
    log_msg "waiting topic: ${label} (${topic}) attempt=${attempt}"
    if timeout 4s ros2 topic echo "${topic}" --once >/tmp/azas_topic_wait.log 2>&1; then
      cat /tmp/azas_topic_wait.log | head -n 12
      return 0
    fi
    tail -n 8 /tmp/azas_topic_wait.log || true
    sleep 2
    attempt=$((attempt + 1))
  done
}

ensure_robot_ready() {
  wait_service_call "/${SERVICE_PREFIX}/system/get_robot_state" "dsr_msgs2/srv/GetRobotState" "Doosan robot_state"
  wait_service_call "/${SERVICE_PREFIX}/motion/check_motion" "dsr_msgs2/srv/CheckMotion" "Doosan check_motion"
  log_msg "robot ready gate passed"
}

ensure_gripper() {
  if timeout 3s ros2 service type /jarvis/rg2/set_width >/dev/null 2>&1; then
    log_msg "RG2 services already visible"
    return 0
  fi
  log_msg "starting RG2 bridge"
  (
    set +u
    source /opt/ros/humble/setup.bash
    source "${ROOT}/install/setup.bash"
    set -u
    source "${ROOT}/install/azas_gripper/share/azas_gripper/package.bash"
    ros2 launch "${ROOT}/install/azas_gripper/share/azas_gripper/launch/rg2_trigger.launch.py" \
      ip:=192.168.1.1 port:=502 connect:=true open_width:=1100 close_width:=0 force:=300 settle_seconds:=0.6
  ) >"${LOG_DIR}/gripper.log" 2>&1 &
  support_pids+=("$!")
  until timeout 3s ros2 service type /jarvis/rg2/set_width >/dev/null 2>&1; do
    tail -n 12 "${LOG_DIR}/gripper.log" || true
    sleep 2
  done
  log_msg "RG2 services ready"
}

ensure_camera() {
  if timeout 3s ros2 topic echo /camera/camera/aligned_depth_to_color/image_raw --once >/dev/null 2>&1; then
    log_msg "RealSense aligned depth already visible"
    return 0
  fi
  log_msg "starting RealSense camera with initial_reset"
  ros2 launch realsense2_camera rs_launch.py \
    camera_name:=camera \
    initial_reset:=true reconnect_timeout:=5.0 \
    enable_color:=true enable_depth:=true align_depth.enable:=true \
    rgb_camera.color_profile:=640x480x30 \
    depth_module.depth_profile:=640x480x30 \
    >"${LOG_DIR}/camera.log" 2>&1 &
  support_pids+=("$!")
  wait_topic_once /camera/camera/color/image_raw "RealSense color"
  wait_topic_once /camera/camera/aligned_depth_to_color/image_raw "RealSense aligned depth"
  wait_topic_once /camera/camera/color/camera_info "RealSense camera info"
  log_msg "RealSense camera topics ready"
}

ensure_collision_scene() {
  log_msg "starting collision/TF support stack"
  (
    ros2 launch azas_bringup workspace_collision_scene.launch.py \
      publish_collision_objects:=true \
      table_collision_enabled:=true \
      workspace_boundary_collision_enabled:=true \
      table_collision_expand_to_workspace_walls:=true \
      dispenser_collision_enabled:=true \
      dispenser_collision_publish_objects:=true \
      dispenser_collision_publish_markers:=true &
    ros2 launch azas_bringup rg2_link6_tcp.launch.py publish_gripper_collision:=false &
    # The RG2 mesh is now in the MoveIt URDF; purge the legacy attached box.
    timeout 12s ros2 run azas_motion link6_gripper_collision_node \
      --ros-args -p operation:=remove -p publish_once:=true -p publish_markers:=false || true
    ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id world --child-frame-id base_link &
    ros2 run azas_perception hand_eye_static_tf_node --ros-args -p compose_timeout_sec:=30.0 -p allow_direct_fallback:=false &
    python3 -m azas_motion.tumbler_collision_scene_node --ros-args -p action:=publish_detected -p object_id:=detected_tumbler -p use_lidded_height:=true
  ) >"${LOG_DIR}/collision_scene.log" 2>&1 &
  support_pids+=("$!")
  sleep 5
  tail -n 40 "${LOG_DIR}/collision_scene.log" || true
}

run_side_grip() {
  log_msg "START 창현 side-grip. OpenCV 창에서 컵 확인 후 p를 누르세요. 종료는 q/Esc."
  source "${ROOT}/install/dsr_practice/share/dsr_practice/package.bash"
  (
    trap 'jobs -pr | xargs -r kill >/dev/null 2>&1 || true' EXIT
    ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id world --child-frame-id base_link &
    ros2 run azas_perception hand_eye_static_tf_node --ros-args -p compose_timeout_sec:=30.0 -p allow_direct_fallback:=false &
    (sleep 5; python3 "${ROOT}/src/dsr_practice/dsr_practice/joint_state_relay.py" --ros-args -r __node:=azas_joint_state_relay -p input_topic:=/${SERVICE_PREFIX}/joint_states -p output_topic:=/joint_states) &
    ros2 launch "${ROOT}/src/dsr_practice/launch/yolo_cup_pick_node.launch.py" \
      model_path:="${ROOT}/local_models/best.pt" \
      conf:=0.35 imgsz:=640 device:=cpu target_class:=cup \
      auto_pick:=false auto_pick_interval:=8.0 exit_after_pick:=false \
      depth_patch_radius:=7 min_depth_valid_ratio:=0.03 min_depth_m:=0.15 max_depth_m:=1.20 \
      redetect_on_approach:=false redetect_settle_sec:=0.5 \
      grasp_mode:=side side_far_stage_enabled:=false side_approach_offset:=0.18 \
      side_short_stage_backoff_m:=0.08 side_grasp_stop_backoff_m:=0.04 side_close_underreach_m:=0.03 \
      side_target_x_offset_m:="${SIDE_TARGET_X_OFFSET_M:--0.020}" \
      side_target_joint6_inset_m:="${SIDE_TARGET_JOINT6_INSET_M:-0.070}" \
      side_target_joint6_inset_sign:="${SIDE_TARGET_JOINT6_INSET_SIGN:-1.0}" \
      side_low_retry_lift_m:=0.0 side_low_retry_attempts:=0 \
      side_linear_approach_enabled:=true side_final_slide_enabled:=false \
      side_fixed_grasp_z_enabled:=false side_grasp_z_offset:=0.05 side_project_bbox_center_to_fixed_z:=false \
      side_candidate_plan_check_enabled:=true pre_pick_joint1_clearance_deg:=12.0 \
      side_move_to_initial_center_before_close:=false verify_motion:=false \
      move_to_camera_home:=true move_joint_home_before_camera_home:=false camera_home_mode:=joint min_motion_z:=0.10 \
      workspace_xy_clamp_enabled:=false return_home_after_task:=false return_to_camera_home_after_attempt:=true \
      workspace_collision_scene_enabled:=true table_collision_enabled:=true table_surface_z:=0.0 table_thickness:=0.04 \
      table_size_x:=1.10 table_size_y:=0.65 table_center_x:=0.29 table_center_y:=0.0 table_collision_expand_to_workspace_walls:=true \
      workspace_boundary_collision_enabled:=true dispenser_collision_enabled:=true dispenser_collision_publish_objects:=true \
      dispenser_collision_publish_markers:=true link6_gripper_collision_enabled:=false \
      dispenser_collision_config_path:="${ROOT}/src/azas_bringup/config/measured_dispenser_collision.yaml" \
      moveit_controller_name:=/${SERVICE_PREFIX}/dsr_moveit_controller start_joint_state_relay:=false
  )
}

run_cup_uprighting() {
  log_msg "START 소명 cup_uprighting. OpenCV 창에서 누운 컵 확인 후 p를 누르세요. 종료는 q/Esc."
  export AZAS_CUP_UPRIGHTING_MODEL_PATH="${ROOT}/src/azas_perception/config/yolo_cup_uprighting_best.pt"
  ros2 launch "${ROOT}/src/azas_cup_uprighting/launch/yolo_cup_uprighting.launch.py" \
    model_path:="${AZAS_CUP_UPRIGHTING_MODEL_PATH}" \
    service_prefix:="${SERVICE_PREFIX}" \
    enable_hardware:=true hardware_confirm:=ENABLE_REAL_ROBOT_MOTION \
    run_yolo:=true auto_pick:=false publish_hand_eye_tf:=true
}

run_lid_grip_close() {
  log_msg "START 강개발자 lid_grip_close. ArUco는 기본 DICT_6X6_250 id0, fallback DICT_4X4_50 id14."
  ros2 launch azas_bringup lid_sticker_grip_planning.launch.py \
    model_path:="${ROOT}/local_models/best.pt" \
    marker_type:=aruco require_lid_detection:=false \
    allow_aruco_only_after_grip_request:=false aruco_only_after_grip_request_sec:=20.0 \
    aruco_dictionary:=DICT_6X6_250 aruco_marker_id:=0 aruco_fallback_markers:=DICT_4X4_50:14 aruco_marker_length_m:=0.03 \
    use_aruco_axis_for_orientation:=true aruco_finger_axis_quarter_turns:=0 \
    use_lid_pose_yaw_for_pick:=true lid_pose_yaw_axis:=y lid_pose_yaw_offset_deg:=0.0 lid_pose_yaw_equivalence_deg:=180.0 \
    visual_refine_before_grasp:=true visual_refine_sample_count:=5 visual_refine_timeout_sec:=3.0 visual_refine_max_yaw_std_deg:=3.0 \
    visual_refine_max_position_std_m:=0.005 visual_refine_apply_xy:=true visual_refine_apply_yaw:=true visual_refine_fallback_to_initial_plan:=true \
    enable_hardware:=true hardware_confirm:=ENABLE_REAL_ROBOT_MOTION allow_service_control_without_moveit:=true service_prefix:=/${SERVICE_PREFIX} \
    rx:=108.41 ry:=-176.32 rz:=175.98 offset_axis:=base_z surface_offset_m:=0.0 \
    tcp_grasp_offset_x_m:=0.0 tcp_grasp_offset_y_m:=0.0 tcp_grasp_offset_z_m:="${LID_TCP_GRASP_OFFSET_Z_M}" min_grasp_z_m:="${LID_MIN_GRASP_Z_M}" \
    approach_offset_m:=0.08 lift_offset_m:=0.10 settle_seconds_before_grasp:=0.5 hold_seconds_after_grasp:=3.0 \
    line_velocity:=30.0 line_acceleration:=10.0 move_timeout_sec:=90.0 \
    enable_gripper_service_calls:=true gripper_set_service:=/jarvis/rg2/set_width \
    gripper_preopen_width_m:=0.110 gripper_grasp_width_m:=0.020 gripper_force_n:=16.0 \
    continue_after_gripper_grasp_failure:=true gripper_grasp_failure_wait_sec:=2.0 \
    enable_lid_twist_after_grasp:=true \
    lid_twist_target_x_m:=0.422959106 lid_twist_target_y_m:=0.223224869 lid_twist_target_z_m:=0.166827988 \
    lid_twist_rx:=73.901489 lid_twist_ry:=-178.542740 lid_twist_rz:=117.385612 \
    lid_twist_transfer_clearance_m:=0.12 lid_twist_transfer_max_z_m:=0.60 \
    lid_twist_use_force_control:=false lid_twist_force_rotation_mode:=j6 \
    lid_twist_preseat_periodic_before_turn:=true \
    lid_twist_preseat_periodic_x_amp_mm:=0.0 lid_twist_preseat_periodic_y_amp_mm:=0.0 lid_twist_preseat_periodic_z_amp_mm:=1.0 \
    lid_twist_preseat_periodic_rx_amp_deg:=0.0 lid_twist_preseat_periodic_ry_amp_deg:=0.0 lid_twist_preseat_periodic_rz_amp_deg:=10.0 \
    lid_twist_preseat_periodic_period_sec:=3.6 lid_twist_preseat_periodic_acc_time_sec:=1.0 lid_twist_preseat_periodic_repeat:=2 \
    lid_twist_preseat_periodic_ref:=tool lid_twist_rz_delta_deg:=300.0 lid_twist_turn_step_deg:=50.0 \
    lid_twist_release_lift_m:=0.03 lid_twist_min_z_m:=0.140 lid_twist_max_z_m:=0.220 \
    lid_twist_transfer_velocity:=25.0 lid_twist_press_velocity:=5.0 lid_twist_turn_velocity:=30.0 lid_twist_acceleration:=15.0 \
    lid_twist_hold_seconds_before_turn:=0.0 lid_twist_hold_seconds_after_turn:=0.5
}

run_with_retry() {
  local name="$1"
  shift
  while true; do
    log_msg "running ${name}"
    if "$@"; then
      log_msg "${name} exited cleanly"
      break
    fi
    log_msg "${name} failed. Press Enter to retry, type s then Enter to skip, or q then Enter to stop."
    read -r answer
    case "${answer}" in
      s|S) break ;;
      q|Q) exit 1 ;;
    esac
  done
}

log_msg "tmux logic sequence started. Connect the robot in another tmux pane if it is not connected yet."
ensure_robot_ready
ensure_gripper
ensure_camera
ensure_collision_scene
run_with_retry "창현 side-grip" run_side_grip
ensure_robot_ready
ensure_camera
run_with_retry "소명 cup_uprighting" run_cup_uprighting
ensure_robot_ready
ensure_camera
ensure_gripper
run_with_retry "강개발자 lid_grip_close" run_lid_grip_close
log_msg "logic sequence complete"
