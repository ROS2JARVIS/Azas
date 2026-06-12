#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/ssu/Azas}"
SERVICE_PREFIX="${SERVICE_PREFIX:-dsr01}"
DISPLAY="${DISPLAY:-:0}"
XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
MODEL_PATH="${MODEL_PATH:-${ROOT}/local_models/best.pt}"
ARUCO_DICTIONARY="${ARUCO_DICTIONARY:-DICT_4X4_50}"
ARUCO_MARKER_ID="${ARUCO_MARKER_ID:-14}"
ARUCO_FALLBACK_MARKERS="${ARUCO_FALLBACK_MARKERS:-}"
ARUCO_MARKER_LENGTH_M="${ARUCO_MARKER_LENGTH_M:-0.03}"
ROS_DOMAIN_ID="${LID_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-15}}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
MOVE_TO_LID_VIEW_POSE="${MOVE_TO_LID_VIEW_POSE:-false}"

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
set -u

export DISPLAY XAUTHORITY ROS_DOMAIN_ID ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"
export PYTHONPATH="${ROOT}/src/azas_motion:${ROOT}/tools/run/python_compat:${PYTHONPATH:-}"
mkdir -p "${ROS_LOG_DIR}" "${ROOT}/log/tmux_logic"

if [[ "${SERVICE_PREFIX}" != /* ]]; then
  SERVICE_PREFIX="/${SERVICE_PREFIX}"
fi

echo "[Azas] START Kang lid_grip_close direct command"
echo "[Azas] OpenCV window: confirm lid ArUco, then press p. Quit with q/Esc."
echo "[Azas] service_prefix=${SERVICE_PREFIX} DISPLAY=${DISPLAY} XAUTHORITY=${XAUTHORITY}"
echo "[Azas] ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY} FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS}"
echo "[Azas] aruco=${ARUCO_DICTIONARY}:${ARUCO_MARKER_ID} fallback=${ARUCO_FALLBACK_MARKERS} length_m=${ARUCO_MARKER_LENGTH_M}"
echo "[Azas] note: use_j6_yaw_for_pick/pick_j6_* are not supported by this Azas launch; using supported ArUco-axis orientation parameters."

if [[ ! -f "${MODEL_PATH}" ]]; then
  echo "[Azas][WARN] model_path not found: ${MODEL_PATH}"
fi

if [[ "${MOVE_TO_LID_VIEW_POSE}" == "true" ]]; then
  echo "[Azas] moving to lid camera view pose before ArUco detection"
  python3 "${ROOT}/tools/run/direct_movej_joints.py" \
    --service-prefix "${SERVICE_PREFIX}" \
    --j1 3.0 --j2 -20.0 --j3 52.0 --j4 -9.0 --j5 125.0 --j6 90.0 \
    --velocity 10 --acceleration 10 \
    --j5-min-deg -150 --j5-max-deg 150 --timeout-sec 60 --motion-timeout-sec 120 \
    --execute --confirm ENABLE_DIRECT_MOVEJ
fi

ros2 pkg executables azas_perception | grep -q '^azas_perception lid_sticker_detector_node$' || {
  echo "[Azas][FAIL] missing azas_perception lid_sticker_detector_node" >&2
  exit 2
}
ros2 pkg executables azas_motion | grep -q '^azas_motion lid_grip_planner_node$' || {
  echo "[Azas][FAIL] missing azas_motion lid_grip_planner_node" >&2
  exit 3
}

launch_args=(
  azas_bringup lid_sticker_grip_planning.launch.py
  model_path:="${MODEL_PATH}" \
  marker_type:=aruco require_lid_detection:=true \
  allow_aruco_only_after_grip_request:=true aruco_only_after_grip_request_sec:=20.0 \
  aruco_dictionary:="${ARUCO_DICTIONARY}" aruco_marker_id:="${ARUCO_MARKER_ID}" \
  aruco_marker_length_m:="${ARUCO_MARKER_LENGTH_M}" \
  use_aruco_axis_for_orientation:=true aruco_finger_axis_quarter_turns:=0 \
  use_lid_pose_yaw_for_pick:=false lid_pose_yaw_axis:=y lid_pose_yaw_offset_deg:=0.0 lid_pose_yaw_equivalence_deg:=360.0 \
  visual_refine_before_grasp:=true visual_refine_sample_count:=5 visual_refine_timeout_sec:=3.0 visual_refine_max_yaw_std_deg:=5.0 \
  visual_refine_max_position_std_m:=0.005 visual_refine_apply_xy:=true visual_refine_apply_yaw:=true visual_refine_fallback_to_initial_plan:=true \
  enable_hardware:=true hardware_confirm:=ENABLE_REAL_ROBOT_MOTION allow_service_control_without_moveit:=true service_prefix:="${SERVICE_PREFIX}" \
  approach_lid_with_movej:=false approach_movej_velocity:=20.0 approach_movej_acceleration:=20.0 \
  lid_overhead_approach_enabled:=false lid_overhead_min_z_m:=0.260 \
  rx:=108.41 ry:=-176.32 rz:=175.98 offset_axis:=base_z surface_offset_m:=0.0 \
  tcp_grasp_offset_x_m:=0.0 tcp_grasp_offset_y_m:=0.0 tcp_grasp_offset_z_m:=-0.040 min_grasp_z_m:=0.025 \
  approach_offset_m:=0.08 min_approach_z_m:=0.0 lift_offset_m:=0.10 settle_seconds_before_grasp:=0.5 hold_seconds_after_grasp:=3.0 \
  line_velocity:=30.0 line_acceleration:=10.0 move_timeout_sec:=90.0 \
  enable_gripper_service_calls:=true gripper_set_service:=/jarvis/rg2/set_width \
  gripper_preopen_width_m:=0.110 gripper_grasp_width_m:=0.020 gripper_force_n:=12.0 \
  continue_after_gripper_grasp_failure:=true gripper_grasp_failure_wait_sec:=2.0 \
  enable_lid_twist_after_grasp:=true \
  lid_twist_target_x_m:=0.422959106 lid_twist_target_y_m:=0.223224869 lid_twist_target_z_m:=0.166827988 \
  lid_twist_rx:=73.901489 lid_twist_ry:=-178.542740 lid_twist_rz:=117.385612 \
  lid_twist_transfer_clearance_m:=0.20 lid_twist_transfer_max_z_m:=0.60 \
  lid_twist_use_force_control:=false lid_twist_use_force_spiral:=true lid_twist_force_rotation_mode:=j6 \
  lid_twist_down_force_n:=2.0 lid_twist_force_ref:=base lid_twist_force_service_timeout_sec:=20.0 \
  lid_twist_force_settle_seconds:=0.2 lid_twist_force_release_time:=0.2 \
  lid_twist_preseat_periodic_before_turn:=true \
  lid_twist_preseat_periodic_x_amp_mm:=0.0 lid_twist_preseat_periodic_y_amp_mm:=0.0 lid_twist_preseat_periodic_z_amp_mm:=1.0 \
  lid_twist_preseat_periodic_rx_amp_deg:=0.0 lid_twist_preseat_periodic_ry_amp_deg:=0.0 lid_twist_preseat_periodic_rz_amp_deg:=10.0 \
  lid_twist_preseat_periodic_period_sec:=3.6 lid_twist_preseat_periodic_acc_time_sec:=1.0 lid_twist_preseat_periodic_repeat:=2 \
  lid_twist_preseat_periodic_ref:=tool lid_twist_rz_delta_deg:=360.0 lid_twist_turn_step_deg:=60.0 \
  lid_twist_release_lift_m:=0.03 lid_twist_min_z_m:=0.140 lid_twist_max_z_m:=0.260 \
  lid_twist_transfer_velocity:=25.0 lid_twist_press_velocity:=10.0 lid_twist_turn_velocity:=40.0 lid_twist_acceleration:=15.0 \
  lid_twist_hold_seconds_before_turn:=0.2 lid_twist_hold_seconds_after_turn:=0.5 \
  lid_twist_compliance_x_stiffness:=3000.0 lid_twist_compliance_y_stiffness:=3000.0 lid_twist_compliance_z_stiffness:=300.0 \
  lid_twist_compliance_rx_stiffness:=200.0 lid_twist_compliance_ry_stiffness:=200.0 lid_twist_compliance_rz_stiffness:=200.0
)

if [[ -n "${ARUCO_FALLBACK_MARKERS}" ]]; then
  launch_args+=(aruco_fallback_markers:="${ARUCO_FALLBACK_MARKERS}")
fi

ros2 launch "${launch_args[@]}"
