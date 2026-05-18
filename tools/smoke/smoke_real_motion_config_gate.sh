#!/usr/bin/env bash
set -euo pipefail

# Verify check_real_motion_config.sh fails placeholders and passes measured-like
# temporary fixtures. This sends no ROS commands and does not touch hardware.

TMP_DIR="${TMP_DIR:-/tmp/azas_real_motion_config_gate}"
mkdir -p "${TMP_DIR}"

PLACEHOLDER_CAL="${TMP_DIR}/placeholder_calibration.yaml"
PLACEHOLDER_SAFE="${TMP_DIR}/placeholder_safety.yaml"
VALID_CAL="${TMP_DIR}/valid_calibration.yaml"
VALID_SAFE="${TMP_DIR}/valid_safety.yaml"

cat >"${PLACEHOLDER_CAL}" <<'EOF'
frames:
  base_frame: base_link
  camera_frame: null
hand_eye:
  parent_frame: base_link
  child_frame: null
outlet:
  clearance_m: null
dispenser_outlets:
  "1":
    outlet_pose_xyz_m: null
EOF

cat >"${PLACEHOLDER_SAFE}" <<'EOF'
motion:
  max_velocity_scale: 0.1
  max_acceleration_scale: 0.1
  workspace_bounds_m: null
  min_z_m: null
gripper:
  default_width_m: null
  default_force_n: null
failure_behavior:
  on_detection_failure: abort_without_motion
  on_tf_failure: abort_without_motion
  on_plan_failure: stop_before_execution
EOF

cat >"${VALID_CAL}" <<'EOF'
frames:
  base_frame: base_link
  camera_frame: camera_color_optical_frame
  ee_link: link_6
  planning_group: manipulator
  gripper_tcp: gripper_tcp
  cup_mouth_center: cup_mouth_center
hand_eye:
  parent_frame: base_link
  child_frame: camera_color_optical_frame
  xyz_m: [0.10, 0.00, 0.50]
  rpy_rad: [0.0, 0.0, 0.0]
cup_offsets:
  default:
    tcp_to_cup_mouth_m: [0.0, 0.0, 0.12]
    status: measured
outlet:
  outlet_id: dispenser_outlet
  parent_frame: base_link
  pose_xyz_m: [0.43, 0.08, 0.392]
  pose_rpy_rad: [0.0, 0.0, 0.0]
  clearance_m: 0.03
dispenser_outlets:
  "1":
    outlet_pose_xyz_m: [0.43, 0.18, 0.392]
    outlet_pose_rpy_rad: [0.0, 0.0, 0.0]
    press_pose_xyz_m: [0.51, 0.18, 0.367]
    press_pose_rpy_rad: [0.0, 0.0, 0.0]
    clearance_m: 0.03
  "2":
    outlet_pose_xyz_m: [0.43, 0.08, 0.392]
    outlet_pose_rpy_rad: [0.0, 0.0, 0.0]
    press_pose_xyz_m: [0.51, 0.08, 0.367]
    press_pose_rpy_rad: [0.0, 0.0, 0.0]
    clearance_m: 0.03
  "3":
    outlet_pose_xyz_m: [0.43, -0.02, 0.392]
    outlet_pose_rpy_rad: [0.0, 0.0, 0.0]
    press_pose_xyz_m: [0.51, -0.02, 0.367]
    press_pose_rpy_rad: [0.0, 0.0, 0.0]
    clearance_m: 0.03
  "4":
    outlet_pose_xyz_m: [0.43, -0.12, 0.392]
    outlet_pose_rpy_rad: [0.0, 0.0, 0.0]
    press_pose_xyz_m: [0.51, -0.12, 0.367]
    press_pose_rpy_rad: [0.0, 0.0, 0.0]
    clearance_m: 0.03
EOF

cat >"${VALID_SAFE}" <<'EOF'
motion:
  max_velocity_scale: 0.1
  max_acceleration_scale: 0.1
  workspace_bounds_m: {x_min: 0.0, x_max: 0.8, y_min: -0.35, y_max: 0.35, z_min: 0.0, z_max: 0.8}
  min_z_m: 0.02
gripper:
  default_width_m: 0.05
  default_force_n: 20.0
  timeout_s: 5.0
failure_behavior:
  on_detection_failure: abort_without_motion
  on_tf_failure: abort_without_motion
  on_plan_failure: stop_before_execution
EOF

if CALIBRATION_FILE="${PLACEHOLDER_CAL}" SAFETY_FILE="${PLACEHOLDER_SAFE}" \
  /home/ssu/Azas/tools/checks/check_real_motion_config.sh >/tmp/azas_config_placeholder.out 2>&1; then
  echo "[FAIL] placeholder config unexpectedly passed"
  sed -n '1,120p' /tmp/azas_config_placeholder.out
  exit 1
fi
echo "[PASS] placeholder config is blocked"

CALIBRATION_FILE="${VALID_CAL}" SAFETY_FILE="${VALID_SAFE}" \
  /home/ssu/Azas/tools/checks/check_real_motion_config.sh >/tmp/azas_config_valid.out 2>&1
echo "[PASS] measured-like fixture config passes"
