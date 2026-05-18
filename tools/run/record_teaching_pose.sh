#!/usr/bin/env bash
set -euo pipefail

# 현재 두산 M0609 TCP 위치를 읽어 teaching record 형식으로 출력합니다.
# 로봇을 목표 위치(티칭 포즈)에 놓은 뒤 이 스크립트를 실행하세요.
# 이 스크립트는 로봇을 움직이지 않습니다.
#
# 사용법:
#   bash tools/run/record_teaching_pose.sh
#   bash tools/run/record_teaching_pose.sh dispenser_1_press
#
# 전제조건:
#   - 두산 드라이버 실행 중 (run_doosan_real_no_motion_m0609.sh)
#   - 로봇이 기록할 위치에 있어야 함
#   - calibration.yaml용 m/rad 변환 출력은 단위 확인 후에만 허용

LABEL="${1:-teaching_pose}"
ROBOT_NAME="${ROBOT_NAME:-dsr01}"
SERVICE="${SERVICE:-/${ROBOT_NAME}/aux_control/get_current_posx}"
CONFIRM_DOOSAN_POSX_UNITS="${CONFIRM_DOOSAN_POSX_UNITS:-}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash 2>/dev/null || true
source /home/ssu/Azas/install/setup.bash 2>/dev/null || true
set -u

echo "[Azas] 현재 TCP 위치 읽기: label=${LABEL}"
echo "[Azas] 서비스: ${SERVICE}"
echo "[Azas] 로봇을 목표 위치에 놓고 Enter를 누르세요..."
read -r

echo "[Azas] 위치 읽는 중..."
RAW=$(ros2 service call "${SERVICE}" dsr_msgs2/srv/GetCurrentPosx "{ref: 0}" 2>&1)

VALUES=$(RAW_TEXT="${RAW}" python3 - <<'PY'
import os
import re
import sys

text = os.environ.get("RAW_TEXT", "")
match = re.search(r"data[:=]\s*\[([^\]]+)\]", text, re.S)
if match:
    values = re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", match.group(1))
else:
    values = []
    in_data = False
    for line in text.splitlines():
        stripped = line.strip()
        if re.match(r"data:\s*$", stripped):
            in_data = True
            continue
        if in_data:
            item = re.match(r"-\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)", stripped)
            if item:
                values.append(item.group(1))
                if len(values) >= 7:
                    break
                continue
            if stripped and not stripped.startswith("-"):
                break

if len(values) < 6:
    sys.exit(1)
print(" ".join(values[:7]))
PY
) || {
    echo "[FAIL] 위치 읽기 실패. 두산 드라이버가 실행 중인지 확인하세요."
    echo "드라이버 실행: ROBOT_HOST=192.168.1.101 DOOSAN_NO_MOTION_CONFIRM=CONNECT_DOOSAN_NO_MOTION bash tools/run/run_doosan_real_no_motion_m0609.sh"
    echo ""
    echo "원본 출력:"
    echo "${RAW}" | head -20
    exit 1
}

read -r X_RAW Y_RAW Z_RAW RX_RAW RY_RAW RZ_RAW SOL_RAW <<<"${VALUES}"

echo ""
echo "========================================="
echo "[${LABEL}] raw Doosan GetCurrentPosx"
echo "========================================="
echo "    raw_posx_mm_deg: [${X_RAW}, ${Y_RAW}, ${Z_RAW}, ${RX_RAW}, ${RY_RAW}, ${RZ_RAW}]"
if [[ -n "${SOL_RAW:-}" ]]; then
    echo "    solution_space: ${SOL_RAW}"
fi
echo "========================================="
echo ""

if [[ "${CONFIRM_DOOSAN_POSX_UNITS}" != "MM_DEG" ]]; then
    echo "[STOP] calibration.yaml 입력값은 출력하지 않았습니다."
    echo "[STOP] 로컬 GetCurrentPosx 인터페이스는 단위 메타데이터를 제공하지 않습니다."
    echo "[STOP] Doosan posx가 mm/degree임을 실측 또는 공식 문서로 확인한 뒤에만 아래처럼 재실행하세요:"
    echo "  CONFIRM_DOOSAN_POSX_UNITS=MM_DEG bash tools/run/record_teaching_pose.sh ${LABEL}"
    exit 1
fi

CONVERTED=$(python3 - "${X_RAW}" "${Y_RAW}" "${Z_RAW}" "${RX_RAW}" "${RY_RAW}" "${RZ_RAW}" <<'PY'
import math
import sys

x, y, z, rx, ry, rz = (float(value) for value in sys.argv[1:7])
print(
    f"{x / 1000.0:.6f} {y / 1000.0:.6f} {z / 1000.0:.6f} "
    f"{math.radians(rx):.6f} {math.radians(ry):.6f} {math.radians(rz):.6f}"
)
PY
)
read -r X_M Y_M Z_M RX_RAD RY_RAD RZ_RAD <<<"${CONVERTED}"

echo "========================================="
echo "[${LABEL}] calibration.yaml 입력 형식"
echo "========================================="
echo "    pose_xyz_m: [${X_M}, ${Y_M}, ${Z_M}]"
echo "    pose_rpy_rad: [${RX_RAD}, ${RY_RAD}, ${RZ_RAD}]"
echo "========================================="
echo ""
echo "[Azas] 위 값은 CONFIRM_DOOSAN_POSX_UNITS=MM_DEG 확인 하에서만 출력되었습니다."
