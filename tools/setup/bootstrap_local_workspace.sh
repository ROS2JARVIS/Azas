#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
INSTALL_SETUP="$ROOT/install/local_setup.bash"

cd "$ROOT"

echo "[Azas bootstrap] workspace: $ROOT"

if [[ ! -f "$ROS_SETUP" ]]; then
  cat >&2 <<MSG
[Azas bootstrap] ROS Humble setup 파일을 찾지 못했습니다: $ROS_SETUP
먼저 이 PC에 ROS 2 Humble을 설치해야 합니다.
MSG
  exit 1
fi

# shellcheck source=/opt/ros/humble/setup.bash
source "$ROS_SETUP"

if ! command -v colcon >/dev/null 2>&1; then
  cat >&2 <<'MSG'
[Azas bootstrap] colcon 명령을 찾지 못했습니다.
설치 예시:
  sudo apt update
  sudo apt install -y python3-colcon-common-extensions
MSG
  exit 1
fi

if command -v rosdep >/dev/null 2>&1; then
  echo "[Azas bootstrap] rosdep 의존성 확인/설치"
  rosdep install --from-paths src --ignore-src -r -y
else
  cat >&2 <<'MSG'
[Azas bootstrap] rosdep 명령을 찾지 못했습니다. rosdep 단계는 건너뜁니다.
의존성 오류가 나면 설치 예시:
  sudo apt install -y python3-rosdep
  sudo rosdep init  # 이미 되어 있으면 생략
  rosdep update
MSG
fi

echo "[Azas bootstrap] colcon build --symlink-install"
colcon build --symlink-install

if [[ ! -f "$INSTALL_SETUP" ]]; then
  cat >&2 <<MSG
[Azas bootstrap] 빌드는 끝났지만 install setup 파일이 없습니다: $INSTALL_SETUP
위 colcon 로그의 에러를 확인하세요.
MSG
  exit 1
fi

# shellcheck source=/dev/null
source "$INSTALL_SETUP"

cat <<MSG
[Azas bootstrap] 완료.

이 터미널에서 바로 패널 실행:
  bash tools/run/open_robot_pipeline_control_panel.sh

새 터미널을 열었다면 먼저:
  cd $ROOT
  source /opt/ros/humble/setup.bash
  source install/local_setup.bash
  bash tools/run/open_robot_pipeline_control_panel.sh
MSG
