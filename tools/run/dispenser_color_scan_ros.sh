#!/usr/bin/env bash
# 디스펜서 색상 스캔 (ROS 모드).
# 로봇이 color_scan_pose (joints [3,-12.7,44,-9,133,90]°)에 있어야 합니다.
# 카메라, TF, 로봇 드라이버가 실행 중이어야 합니다.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
source "$ROOT/install/local_setup.bash" 2>/dev/null || true
python3 "$ROOT/tools/perception/dispenser_color_scan.py" --ros \
    --output "$ROOT/outputs/dispenser_color_map.json"
