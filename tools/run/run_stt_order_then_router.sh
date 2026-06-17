#!/usr/bin/env bash
set -euo pipefail

# STT/키오스크 주문 -> 자동 칵테일 파이프라인.
# /azas/voice/confirmed_recipe_decision 주문을 기다렸다가(listen_stt_recipe.py가
# outputs/latest_recipe.json 저장) auto_cup_flow_router를 실행한다.
# recipe_colors를 일부러 비워 두므로 레시피 단계는 방금 저장된 주문 내용을 사용한다.
#
# Usage:
#   bash tools/run/run_stt_order_then_router.sh              # 주문 1건 처리 후 종료
#   LOOP=true bash tools/run/run_stt_order_then_router.sh    # 주문 올 때마다 반복 처리
#   ORDER_TIMEOUT_SEC=3600 ...                               # 주문 대기 한도(기본 86400초)
#
# 사전 조건(이 스크립트가 띄우지 않음):
#   - tmux 로봇 스택: bash tools/run/start_azas_tmux_stack.sh
#   - kiosk/voice 데모: bash tools/run/run_kiosk_voice_demo.sh

ROOT="${ROOT:-/home/ssu/Azas}"
SERVICE_PREFIX="${SERVICE_PREFIX:-dsr01}"
ORDER_TIMEOUT_SEC="${ORDER_TIMEOUT_SEC:-86400}"
LOOP="${LOOP:-false}"
CLASSIFIER_PATH="${CLASSIFIER_PATH:-${ROOT}/cup_classifier_best.pth}"

cd "${ROOT}"

set +u
source /opt/ros/humble/setup.bash
[[ -f /home/ssu/ws_moveit/install/setup.bash ]] && source /home/ssu/ws_moveit/install/setup.bash
[[ -f /home/ssu/ros2_ws/install/setup.bash ]] && source /home/ssu/ros2_ws/install/setup.bash
source "${ROOT}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-9}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"
mkdir -p "${ROS_LOG_DIR}"

run_one_order() {
  echo "[Azas] STT 주문 대기 중... (/azas/voice/confirmed_recipe_decision, timeout=${ORDER_TIMEOUT_SEC}s)"
  if ! python3 "${ROOT}/tools/run/listen_stt_recipe.py" --timeout "${ORDER_TIMEOUT_SEC}"; then
    echo "[Azas] 주문 수신 실패/타임아웃" >&2
    return 1
  fi
  echo "[Azas] 주문 수신 -> auto_cup_flow_router 시작 (recipe=outputs/latest_recipe.json)"
  # recipe_colors는 비워 둔다: 채우면 latest_recipe.json(방금 받은 주문)이 무시된다.
  ros2 launch azas_bringup auto_cup_flow_router.launch.py \
    enable_real_motion:=true \
    router_confirm:=ENABLE_AUTO_CUP_ROUTER \
    service_prefix:="${SERVICE_PREFIX}" \
    moveit_controller_name:="/${SERVICE_PREFIX}/dsr_moveit_controller" \
    controller_action_name:="/${SERVICE_PREFIX}/dsr_moveit_controller/follow_joint_trajectory" \
    classifier_path:="${CLASSIFIER_PATH}" \
    classifier_arch:=resnet18 \
    route_hold_sec:=2.0 \
    route_stable_required_samples:=5 \
    route_stable_min_sec:=2.0
}

if [[ "${LOOP}" == "true" ]]; then
  echo "[Azas] LOOP 모드: 주문이 올 때마다 파이프라인을 반복 실행합니다. 중지: Ctrl-C"
  while true; do
    if ! run_one_order; then
      echo "[Azas] 이번 주문 처리 실패; 5초 후 다음 주문 대기" >&2
      sleep 5
    fi
  done
else
  run_one_order
fi
