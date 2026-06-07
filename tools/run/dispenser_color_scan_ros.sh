#!/usr/bin/env bash
# 디스펜서 색상 스캔 (ROS 모드).
# 로봇이 color_scan_pose (joints [0,10,32,0,100,90]°)에 있어야 합니다.
# 카메라, TF, 로봇 드라이버가 실행 중이어야 합니다.
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
    --output "$ROOT/outputs/dispenser_color_map.json"
