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

echo "[voice_flow] starting full auto cup flow: recipe_colors=${RECIPE_COLORS}"
exec ros2 launch azas_bringup auto_cup_flow_router.launch.py \
  enable_real_motion:=true \
  router_confirm:=ENABLE_AUTO_CUP_ROUTER \
  service_prefix:="${SERVICE_PREFIX}" \
  moveit_controller_name:=/${SERVICE_PREFIX}/dsr_moveit_controller \
  controller_action_name:=/${SERVICE_PREFIX}/dsr_moveit_controller/follow_joint_trajectory \
  classifier_path:=/home/ssu/Azas/cup_classifier_best.pth \
  classifier_arch:=resnet18 \
  route_hold_sec:=2.0 \
  route_stable_required_samples:=5 \
  route_stable_min_sec:=0.8 \
  recipe_colors:="${RECIPE_COLORS}"
