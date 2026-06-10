#!/usr/bin/env bash
set -euo pipefail

# Course-material execution path (12~13차시):
# 1) Doosan MoveIt bringup exactly like 25장, no namespace by default
# 2) dsr_practice/mp_basic.launch.py exactly like 26장
# 3) RViz robot motion comes from MoveItPy robot.execute() -> controller -> /joint_states
# No custom /joint_states publisher. No /display_planned_path usage.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="${LOG_DIR:-${ROOT_DIR}/log/manual}"
MODE="${MODE:-virtual}"
HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-12345}"
MODEL="${MODEL:-m0609}"
COLOR="${COLOR:-white}"
RT_HOST="${RT_HOST:-192.168.137.50}"
START_DELAY_SEC="${START_DELAY_SEC:-18}"
JOINT_WAIT_SEC="${JOINT_WAIT_SEC:-45}"
mkdir -p "${LOG_DIR}"

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT
PIDS=()

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/ws_moveit/install/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  source "${ROOT_DIR}/install/setup.bash"
fi
set -u

# Refuse if fake visual joint publishers are still present. The course path must
# be controller-backed /joint_states, not a hand-written animation node.
if pgrep -af 'm0609_shake_joint_state_node|side_grasp_ik_preview_node' >/dev/null; then
  echo '[Azas] Refusing: fake RViz joint publisher is still running.' >&2
  pgrep -af 'm0609_shake_joint_state_node|side_grasp_ik_preview_node' >&2 || true
  exit 1
fi

ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
  mode:="${MODE}" \
  model:="${MODEL}" \
  host:="${HOST}" \
  port:="${PORT}" \
  color:="${COLOR}" \
  rt_host:="${RT_HOST}" \
  >"${LOG_DIR}/course_moveit_bringup.log" 2>&1 &
PIDS+=("$!")

sleep "${START_DELAY_SEC}"

joint_deadline=$((SECONDS + JOINT_WAIT_SEC))
while (( SECONDS < joint_deadline )); do
  if timeout 3 ros2 topic echo /joint_states --once >/tmp/azas_course_joint_state.txt 2>/dev/null; then
    if grep -q '^header:' /tmp/azas_course_joint_state.txt; then
      break
    fi
  fi
  sleep 1
done
if ! grep -q '^header:' /tmp/azas_course_joint_state.txt 2>/dev/null; then
  echo '[Azas] No fresh /joint_states. Course MoveItPy cannot run.' >&2
  tail -80 "${LOG_DIR}/course_moveit_bringup.log" >&2 || true
  exit 1
fi

ros2 launch dsr_practice mp_basic.launch.py \
  >"${LOG_DIR}/course_mp_basic.log" 2>&1

echo '[Azas] Course mp_basic finished: MoveItPy plan -> robot.execute -> /joint_states -> RViz robot motion.'
echo "[Azas] Logs: ${LOG_DIR}/course_moveit_bringup.log ${LOG_DIR}/course_mp_basic.log"
wait
