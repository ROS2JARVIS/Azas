#!/usr/bin/env bash
# 음성 주문(confirmed recipe)을 받아 전체 자동 칵테일 파이프라인을 실행한다:
# 컵 분류/픽 -> 디스펜서 레시피 -> 컵홀더 -> 뚜껑 체결 -> 쉐이킹.
# 2026-06-13 수동 4-터미널 구성으로 검증된 라우터 명령을 그대로 고정한 래퍼.
# 사용: RECIPE_COLORS="yellow:2,blue:1" bash run_voice_auto_cup_flow.sh
#   또는 bash run_voice_auto_cup_flow.sh "yellow:2,blue:1"
set -euo pipefail

RECIPE_COLORS="${1:-${RECIPE_COLORS:-}}"
if [[ -z "${RECIPE_COLORS}" ]]; then
  echo "[voice_flow] RECIPE_COLORS is required (e.g. \"yellow:2,blue:1\")" >&2
  exit 2
fi
if ! [[ "${RECIPE_COLORS}" =~ ^(red|yellow|green|blue):[0-9]+(,(red|yellow|green|blue):[0-9]+)*$ ]]; then
  echo "[voice_flow] invalid RECIPE_COLORS: ${RECIPE_COLORS}" >&2
  exit 2
fi

SERVICE_PREFIX="${SERVICE_PREFIX:-dsr01}"
MOTION_SERVICE_PREFIX="${MOTION_SERVICE_PREFIX:-${SERVICE_PREFIX}}"
AUTO_FLOW_RESUME_MODE="${AUTO_FLOW_RESUME_MODE:-normal}"
AUTO_FLOW_RESUME_STATE_FILE="${AUTO_FLOW_RESUME_STATE_FILE:-/home/ssu/Azas/outputs/auto_cup_flow_resume.json}"
AUTO_FLOW_RESUME_EVENTS_FILE="${AUTO_FLOW_RESUME_EVENTS_FILE:-/home/ssu/Azas/outputs/auto_cup_flow_events.jsonl}"
AUTO_FLOW_DISPENSER_RESUME_STATE_FILE="${AUTO_FLOW_DISPENSER_RESUME_STATE_FILE:-/home/ssu/Azas/outputs/measured_dispenser_recipe_resume.json}"
ROUTER_CONFIRM="${ROUTER_CONFIRM:-}"
if [[ "${ROUTER_CONFIRM}" != "ENABLE_AUTO_CUP_ROUTER" ]]; then
  echo "[voice_flow] BLOCKED: set ROUTER_CONFIRM=ENABLE_AUTO_CUP_ROUTER to run real motion." >&2
  exit 3
fi

cd /home/ssu/Azas
set +u
source /opt/ros/humble/setup.bash
[[ -f /home/ssu/ws_moveit/install/setup.bash ]] && source /home/ssu/ws_moveit/install/setup.bash
[[ -f /home/ssu/ros2_ws/install/setup.bash ]] && source /home/ssu/ros2_ws/install/setup.bash
source /home/ssu/Azas/install/setup.bash
set -u

# 검증된 단일 DDS 구성: 모든 스택 터미널과 동일해야 service discovery가 안정적이다.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-9}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

echo "[voice_flow] starting full auto cup flow: recipe_colors=${RECIPE_COLORS} resume_mode=${AUTO_FLOW_RESUME_MODE}"
mkdir -p "${ROS_LOG_DIR:-/tmp/azas_ros_logs}"
VOICE_FLOW_LOG="${ROS_LOG_DIR:-/tmp/azas_ros_logs}/voice_auto_cup_flow_$(date +%Y%m%d_%H%M%S).log"

set +e
ros2 launch azas_bringup auto_cup_flow_router.launch.py \
  enable_real_motion:=true \
  router_confirm:=ENABLE_AUTO_CUP_ROUTER \
  cup_holder_place_x_offset_m:=0.010 \
  service_prefix:="${SERVICE_PREFIX}" \
  motion_service_prefix:="${MOTION_SERVICE_PREFIX}" \
  moveit_controller_name:=/${SERVICE_PREFIX}/dsr_moveit_controller \
  controller_action_name:=/${SERVICE_PREFIX}/dsr_moveit_controller/follow_joint_trajectory \
  classifier_path:=/home/ssu/Azas/cup_classifier_best.pth \
  classifier_arch:=resnet18 \
  route_hold_sec:=2.0 \
  route_stable_required_samples:=5 \
  route_stable_min_sec:=0.8 \
  recipe_colors:="${RECIPE_COLORS}" \
  resume_mode:="${AUTO_FLOW_RESUME_MODE}" \
  resume_state_file:="${AUTO_FLOW_RESUME_STATE_FILE}" \
  resume_events_file:="${AUTO_FLOW_RESUME_EVENTS_FILE}" \
  dispenser_resume_state_file:="${AUTO_FLOW_DISPENSER_RESUME_STATE_FILE}" 2>&1 | tee "${VOICE_FLOW_LOG}"
pipeline_status=("${PIPESTATUS[@]}")
set -e
launch_rc="${pipeline_status[0]}"

if grep -Eq "\[auto_cup_flow_router-[0-9]+\]: process has died|auto_cup_flow_router.*exit code 1|lid_shake: process exited with code [1-9]" "${VOICE_FLOW_LOG}"; then
  echo "[voice_flow] auto cup flow failed; see ${VOICE_FLOW_LOG}" >&2
  exit 1
fi
exit "${launch_rc}"
