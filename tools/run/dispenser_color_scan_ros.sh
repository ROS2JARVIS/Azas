#!/usr/bin/env bash
# 디스펜서 색상 스캔 (ROS 모드).
# 로봇이 color_scan_pose (joints [0,10,32,0,100,90]°)에 있으면
# 카메라 화면의 색상 핸들을 직접 검출해 왼쪽→오른쪽을 1→4번으로 저장합니다.
# TF 투영은 visible-handle 검출 실패 시 보조 경로로만 사용됩니다.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

source_setup() {
    local setup_file="$1"
    if [ ! -f "$setup_file" ]; then
        return 0
    fi
    # Colcon setup files may read optional environment variables while this
    # wrapper runs with nounset enabled.
    set +u
    source "$setup_file"
    set -u
}

source_setup /opt/ros/humble/setup.bash
source_setup "$ROOT/install/local_setup.bash"

python3 "$ROOT/tools/perception/dispenser_color_scan.py" --ros \
    --settle-sec "${AZAS_COLOR_SCAN_SETTLE_SEC:-1.5}" \
    --sample-frames "${AZAS_COLOR_SCAN_SAMPLE_FRAMES:-5}" \
    --debug-image "$ROOT/outputs/dispenser_color_scan_debug.jpg" \
    --output "$ROOT/outputs/dispenser_color_map.json"
