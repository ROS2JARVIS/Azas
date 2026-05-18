#!/usr/bin/env bash
set -euo pipefail

# Reproducible non-hardware verifier for the Azas open-source robot-control stack.
# It builds/tests nothing destructive and sends no motion or gripper commands.

REPORT="${REPORT:-/tmp/azas_control_readiness_report.txt}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CHECKS_DIR="${ROOT_DIR}/tools/checks"
RUN_DIR="${ROOT_DIR}/tools/run"
SMOKE_DIR="${ROOT_DIR}/tools/smoke"
STRICT_OPTIONAL="${STRICT_OPTIONAL:-false}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"

rm -f "${REPORT}"
mkdir -p "${ROS_LOG_DIR}"

section() {
  local title="$1"
  {
    echo
    echo "## ${title}"
  } | tee -a "${REPORT}"
}

run_step() {
  local label="$1"
  shift
  section "${label}"
  if "$@" 2>&1 | tee -a "${REPORT}"; then
    echo "[RESULT] ${label}: PASS" | tee -a "${REPORT}"
  else
    echo "[RESULT] ${label}: FAIL" | tee -a "${REPORT}"
    return 1
  fi
}

python_syntax_check() {
  python3 - "$@" <<'PY'
import pathlib
import sys

failed = False
for raw_path in sys.argv[1:]:
    path = pathlib.Path(raw_path)
    try:
        source = path.read_text(encoding="utf-8")
        compile(source, str(path), "exec")
    except Exception as exc:
        failed = True
        print(f"[FAIL] {path}: {exc}")
    else:
        print(f"[OK] {path}")

raise SystemExit(1 if failed else 0)
PY
}

{
  echo "# Azas Control Readiness Report"
  date -Is
  echo
  echo "Scope: non-hardware verifier. No robot motion or gripper commands are sent."
  echo "Strict optional dependencies: ${STRICT_OPTIONAL}"
} | tee -a "${REPORT}"

run_step "Script syntax" bash -lc '
  for script in "$@"; do
    bash -n "${script}"
  done
' bash \
  "${CHECKS_DIR}/check_oss_stack.sh" \
  "${SMOKE_DIR}/smoke_control_path.sh" \
  "${CHECKS_DIR}/check_live_hardware_gates.sh" \
  "${CHECKS_DIR}/check_connection_stage.sh" \
  "${CHECKS_DIR}/check_real_motion_config.sh" \
  "${CHECKS_DIR}/explain_real_robot_blockers.sh" \
  "${RUN_DIR}/run_doosan_virtual_m0609.sh" \
  "${RUN_DIR}/run_doosan_real_no_motion_m0609.sh" \
  "${RUN_DIR}/run_connected_robot_control.sh" \
  "${RUN_DIR}/run_robot_dryrun.sh" \
  "${RUN_DIR}/run_robot_real.sh" \
  "${RUN_DIR}/run_connected_cup_pick_real.sh" \
  "${RUN_DIR}/run_cup_to_dispenser_press_real.sh" \
  "${RUN_DIR}/run_rule_based_dispenser_then_shake_real.sh" \
  "${RUN_DIR}/run_rule_based_dispenser_then_shake_sim.sh" \
  "${RUN_DIR}/run_rule_based_shake_real.sh" \
  "${SMOKE_DIR}/smoke_fake_hardware_path.sh" \
  "${SMOKE_DIR}/smoke_cup_to_dispenser_press_path.sh" \
  "${SMOKE_DIR}/smoke_tumbler_shake_sequence.sh" \
  "${CHECKS_DIR}/check_depth_projection_sample.sh" \
  "${CHECKS_DIR}/check_detection_stability.sh" \
  "${SMOKE_DIR}/smoke_cocktail_dryrun_sequence.sh" \
  "${RUN_DIR}/field_no_motion_report.sh" \
  "${CHECKS_DIR}/robot_connection_acceptance.sh" \
  "${SMOKE_DIR}/smoke_real_motion_entrypoint_gates.sh" \
  "${SMOKE_DIR}/smoke_real_motion_config_gate.sh"

run_step "Python syntax" python_syntax_check \
  "${SMOKE_DIR}/fake_hardware_services.py" \
  "/home/ssu/ros2_ws/src/Azas/jarvis/m0609_shake_joint_state_node.py" \
  "/home/ssu/ros2_ws/src/Azas/jarvis/tumbler_shake_sequence_node.py" \
  "/home/ssu/ros2_ws/src/Azas/jarvis/dispense_lid_sequence_node.py" \
  "/home/ssu/ros2_ws/src/Azas/jarvis/shake_visualizer_node.py" \
  "${CHECKS_DIR}/check_static_cup_lid_dataset.py" \
  "${CHECKS_DIR}/check_fixed_dispenser_geometry.py" \
  "${CHECKS_DIR}/check_measured_dispenser_geometry.py" \
  "${CHECKS_DIR}/check_cocktail_workflow_plan.py" \
  "${CHECKS_DIR}/check_depth_projection_sample.py" \
  "${CHECKS_DIR}/check_detection_stability.py" \
  "${SMOKE_DIR}/smoke_cocktail_dryrun_sequence.py" \
  "${ROOT_DIR}/src/azas_task_manager/azas_task_manager/cocktail_workflow_plan.py" \
  "${ROOT_DIR}/src/azas_task_manager/azas_task_manager/cocktail_dryrun_sequence_node.py"

run_step "OSS stack availability" env STRICT_OPTIONAL="${STRICT_OPTIONAL}" "${CHECKS_DIR}/check_oss_stack.sh"

run_step "Static cup/lid photo dataset gate" "${CHECKS_DIR}/check_static_cup_lid_dataset.py"

run_step "Fixed dispenser geometry gate" "${CHECKS_DIR}/check_fixed_dispenser_geometry.py"

run_step "Non-hardware control smoke" "${SMOKE_DIR}/smoke_control_path.sh"

run_step "Fake hardware-armed smoke" env SERVICE_PREFIX=azas_verify_floor "${SMOKE_DIR}/smoke_fake_hardware_path.sh"

run_step "Dispenser press fake hardware smoke" env SERVICE_PREFIX=azas_verify_press "${SMOKE_DIR}/smoke_cup_to_dispenser_press_path.sh"

run_step "Rule-based high-shake fake hardware smoke" env SERVICE_PREFIX=azas_verify_shake "${SMOKE_DIR}/smoke_tumbler_shake_sequence.sh"

run_step "Cocktail dry-run sequence smoke" "${SMOKE_DIR}/smoke_cocktail_dryrun_sequence.sh"

run_step "Full cocktail workflow plan gate" "${CHECKS_DIR}/check_cocktail_workflow_plan.py"

run_step "Real-motion entrypoint fail-closed smoke" "${SMOKE_DIR}/smoke_real_motion_entrypoint_gates.sh"

run_step "Real-motion config gate smoke" "${SMOKE_DIR}/smoke_real_motion_config_gate.sh"

run_step "Doosan virtual launch args" bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/ssu/ros2_ws/install/setup.bash
  timeout 20s ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py --show-args
'

run_step "Doosan real no-motion launch args" env SHOW_ARGS_ONLY=true "${RUN_DIR}/run_doosan_real_no_motion_m0609.sh"

{
  echo
  echo "## Completion Boundary"
  echo "PASS here means the local non-hardware stack is wired."
  echo "It does not prove real camera detection, hand-eye calibration, RG2 actuation, MoveIt feasibility for measured poses, e-stop behavior, or real Doosan motion."
  echo "Run ${CHECKS_DIR}/check_live_hardware_gates.sh after live bringup for field evidence."
  echo
  echo "Report: ${REPORT}"
} | tee -a "${REPORT}"
