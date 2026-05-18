#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_FILE="${LOG_FILE:-/tmp/azas_observe_pose_planning_only.log}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros2_logs}"
export ROS_LOG_DIR
mkdir -p "${ROS_LOG_DIR}"

PLANNING_GROUP="${PLANNING_GROUP:-manipulator}"
EE_LINK="${EE_LINK:-tool0}"
BASE_FRAME="${BASE_FRAME:-base_link}"
OBSERVE_FRAME="${OBSERVE_FRAME:-base_link}"
OBSERVE_X="${OBSERVE_X:-0.35}"
OBSERVE_Y="${OBSERVE_Y:--0.25}"
OBSERVE_Z="${OBSERVE_Z:-0.45}"
OBSERVE_QX="${OBSERVE_QX:-0.0}"
OBSERVE_QY="${OBSERVE_QY:-0.0}"
OBSERVE_QZ="${OBSERVE_QZ:-0.0}"
OBSERVE_QW="${OBSERVE_QW:-1.0}"
PLANNING_TIMEOUT_SEC="${PLANNING_TIMEOUT_SEC:-5.0}"

echo "[INFO] Azas OBSERVE_CUP_POSE planning-only check"
echo "[INFO] planning_group=${PLANNING_GROUP} ee_link=${EE_LINK} base_frame=${BASE_FRAME}"
echo "[INFO] observe pose: frame=${OBSERVE_FRAME} xyz=(${OBSERVE_X}, ${OBSERVE_Y}, ${OBSERVE_Z}) quat=(${OBSERVE_QX}, ${OBSERVE_QY}, ${OBSERVE_QZ}, ${OBSERVE_QW})"
echo "[INFO] No MoveIt execute, Doosan motion, or RG2 command is allowed by this script."

cd "${ROOT_DIR}"

if grep -R "exec""ute(" -n src/azas_motion src/azas_task_manager; then
  echo "[FAIL] Found MoveIt execution call in Azas motion/task-manager observe check scope."
  exit 1
fi
echo "[OK] Static check: no MoveIt execution call found in Azas motion/task-manager observe check scope."

set +u
source /opt/ros/humble/setup.bash
if [ -f "${ROOT_DIR}/install/setup.bash" ]; then
  source "${ROOT_DIR}/install/setup.bash"
fi
if [ -f /home/ssu/ros2_ws/install/setup.bash ]; then
  source /home/ssu/ros2_ws/install/setup.bash
fi
set -u

if ! python3 -c "from moveit.planning import MoveItPy" >/dev/null 2>&1; then
  echo "[FAIL] MoveItPy import failed. Source ROS/Azas/MoveIt workspaces before planning."
  exit 1
fi
echo "[OK] MoveItPy import succeeded."

if ! timeout 5s ros2 node list --no-daemon 2>/dev/null | grep -q "/move_group"; then
  echo "[WARN] /move_group is not visible. Planning request may fail until MoveIt is launched."
fi

echo "[INFO] Running alignment_executor_node in observe-pose planning-only mode."
set +e
timeout 20s ros2 run azas_motion alignment_executor_node --ros-args \
  -p enable_planning_only:=true \
  -p allow_execute:=false \
  -p planning_group:="${PLANNING_GROUP}" \
  -p ee_link:="${EE_LINK}" \
  -p base_frame:="${BASE_FRAME}" \
  -p use_fake_side_grasp_plan:=false \
  -p use_observe_pose_plan:=true \
  -p observe_frame:="${OBSERVE_FRAME}" \
  -p observe_pose_x:="${OBSERVE_X}" \
  -p observe_pose_y:="${OBSERVE_Y}" \
  -p observe_pose_z:="${OBSERVE_Z}" \
  -p observe_qx:="${OBSERVE_QX}" \
  -p observe_qy:="${OBSERVE_QY}" \
  -p observe_qz:="${OBSERVE_QZ}" \
  -p observe_qw:="${OBSERVE_QW}" \
  -p planning_timeout_sec:="${PLANNING_TIMEOUT_SEC}" \
  2>&1 | tee "${LOG_FILE}"
run_status=${PIPESTATUS[0]}
set -e

if [ "${run_status}" -ne 0 ] && [ "${run_status}" -ne 124 ]; then
  echo "[FAIL] alignment_executor_node exited with status ${run_status}. See ${LOG_FILE}."
  exit "${run_status}"
fi

if grep -q '"label": "observe".*"status": "planned"' "${LOG_FILE}"; then
  echo "[PASS] OBSERVE_CUP_POSE planning-only request succeeded."
  exit 0
fi

if grep -q '"label": "observe".*"status": "failed"' "${LOG_FILE}"; then
  echo "[FAIL] OBSERVE_CUP_POSE planning-only request failed. See ${LOG_FILE}."
  exit 1
fi

if grep -q '"event": "moveitpy_init".*"status": "failed"' "${LOG_FILE}"; then
  echo "[FAIL] MoveItPy initialization failed before OBSERVE_CUP_POSE planning. See ${LOG_FILE}."
  exit 1
fi

echo "[FAIL] OBSERVE_CUP_POSE planning result was not observed. See ${LOG_FILE}."
exit 1
