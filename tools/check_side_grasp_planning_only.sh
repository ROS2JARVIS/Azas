#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="${ROOT_DIR:-/home/ssu/Azas}"
NODE_FILE="${ROOT_DIR}/src/azas_motion/azas_motion/alignment_executor_node.py"
SWEEP_TOOL="${ROOT_DIR}/tools/pick/sweep_side_grasp_planning_candidates.py"

echo "[Azas] Side grasp planning-only readiness check"
echo "[INFO] If this repo was rebuilt recently, run:"
echo "       source /opt/ros/humble/setup.bash"
echo "       source ${ROOT_DIR}/install/setup.bash"

if ! grep -q 'declare_parameter("allow_execute", False)' "${NODE_FILE}"; then
  echo "[FAIL] alignment_executor_node.py does not default allow_execute to false"
  exit 1
fi

EXECUTE_PATTERN='exec''ute('
if grep -R "${EXECUTE_PATTERN}" -n "${ROOT_DIR}/src/azas_motion" "${ROOT_DIR}/src/azas_task_manager" >/tmp/azas_side_grasp_execute_grep.txt; then
  echo "[FAIL] Potential execute call found in motion/task-manager boundary"
  sed -n '1,120p' /tmp/azas_side_grasp_execute_grep.txt
  exit 1
fi

if [[ ! -f "${SWEEP_TOOL}" ]]; then
  echo "[FAIL] Missing side-grasp candidate sweep tool: ${SWEEP_TOOL}"
  exit 1
fi

if ! grep -q "MoveItPy plan() only" "${SWEEP_TOOL}"; then
  echo "[FAIL] Sweep tool does not state the planning-only boundary"
  exit 1
fi

python3 - <<'PY'
import sys

sys.path.insert(0, "/home/ssu/Azas/src/azas_motion")
from azas_motion.alignment import SideGraspConfig, compute_side_grasp_plan
from geometry_msgs.msg import Pose

pose = Pose()
pose.position.x = 0.32
pose.position.y = -0.22
pose.position.z = 0.05
pose.orientation.w = 1.0
plan = compute_side_grasp_plan(
    pose,
    SideGraspConfig(
        orientation_source="parameter",
        side_grasp_qx=0.0,
        side_grasp_qy=0.0,
        side_grasp_qz=0.0,
        side_grasp_qw=1.0,
    ),
)
print(
    "[OK] Fake side-grasp planning targets computed: "
    f"approach=({plan.approach_pose.position.x:.3f},"
    f"{plan.approach_pose.position.y:.3f},{plan.approach_pose.position.z:.3f}) "
    f"grasp=({plan.grasp_pose.position.x:.3f},"
    f"{plan.grasp_pose.position.y:.3f},{plan.grasp_pose.position.z:.3f}) "
    f"lift=({plan.lift_pose.position.x:.3f},"
    f"{plan.lift_pose.position.y:.3f},{plan.lift_pose.position.z:.3f})"
)

try:
    from moveit.planning import MoveItPy  # noqa: F401
except Exception as exc:
    print(f"[WARN] MoveItPy import failed; planning-only node will fail closed: {exc}")
else:
    print("[OK] MoveItPy import available")
PY

PLANNING_GROUP="${PLANNING_GROUP:-}"
EE_LINK="${EE_LINK:-}"
if [[ -z "${PLANNING_GROUP}" || -z "${EE_LINK}" ]]; then
  echo "[WARN] PLANNING_GROUP or EE_LINK is empty; planning-only runtime must fail closed until verified values are provided"
else
  echo "[OK] Planning params supplied: planning_group=${PLANNING_GROUP}, ee_link=${EE_LINK}"
fi

echo "[OK] Static planning-only boundary check complete; no execute call was made"
