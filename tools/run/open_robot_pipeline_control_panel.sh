#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
URL="${AZAS_PANEL_URL:-http://127.0.0.1:8765/}"
LOG_DIR="$ROOT/log/panel"
LOG_FILE="$LOG_DIR/robot_pipeline_control_panel.log"
PID_FILE="/tmp/azas-panel-8765.pid"
COMMAND_DIR="${AZAS_PANEL_COMMAND_DIR:-$HOME/.local/bin}"
COMMAND_PATH="$COMMAND_DIR/azas-panel"

mkdir -p "$LOG_DIR"

install_command_symlink() {
  if [[ "${AZAS_PANEL_SKIP_COMMAND_INSTALL:-0}" == "1" ]]; then
    return 0
  fi
  if [[ -L "$COMMAND_PATH" && "$(readlink "$COMMAND_PATH")" == "$SCRIPT_DIR/open_robot_pipeline_control_panel.sh" ]]; then
    return 0
  fi
  if [[ -e "$COMMAND_PATH" && ! -L "$COMMAND_PATH" ]]; then
    echo "[Azas] azas-panel 명령 설치 생략: 이미 파일이 있습니다: $COMMAND_PATH" >&2
    return 0
  fi
  if mkdir -p "$COMMAND_DIR" 2>/dev/null && ln -sfn "$SCRIPT_DIR/open_robot_pipeline_control_panel.sh" "$COMMAND_PATH" 2>/dev/null; then
    echo "[Azas] azas-panel 명령 준비됨: $COMMAND_PATH"
    return 0
  fi
  echo "[Azas] azas-panel symlink를 만들지 못했습니다. 레포 명령으로 계속 실행합니다." >&2
}

panel_ready() {
  curl -fsS --max-time 1 "$URL" >/dev/null 2>&1
}

start_panel_server() {
  cd "$ROOT"
  setsid env AZAS_ROOT="$ROOT" bash -lc '
    cd "$AZAS_ROOT"
    source /opt/ros/humble/setup.bash
    source "$AZAS_ROOT/install/local_setup.bash"
    exec python3 tools/run/robot_pipeline_control_server.py
  ' >> "$LOG_FILE" 2>&1 < /dev/null &
  echo "$!" > "$PID_FILE"
}

open_browser() {
  if command -v xdg-open >/dev/null 2>&1; then
    nohup xdg-open "$URL" >/dev/null 2>&1 &
  elif command -v gio >/dev/null 2>&1; then
    nohup gio open "$URL" >/dev/null 2>&1 &
  elif command -v google-chrome >/dev/null 2>&1; then
    nohup google-chrome "$URL" >/dev/null 2>&1 &
  elif command -v firefox >/dev/null 2>&1; then
    nohup firefox "$URL" >/dev/null 2>&1 &
  else
    echo "브라우저 자동 실행 명령을 찾지 못했습니다. 직접 여세요: $URL" >&2
    return 1
  fi
}

install_command_symlink

if ! panel_ready; then
  echo "[Azas] 패널 서버 시작 중..."
  start_panel_server
  for _ in $(seq 1 40); do
    if panel_ready; then
      break
    fi
    sleep 0.25
  done
fi

if ! panel_ready; then
  echo "[Azas] 패널 서버가 아직 준비되지 않았습니다." >&2
  echo "[Azas] 로그: $LOG_FILE" >&2
  exit 1
fi

open_browser

echo "[Azas] 브라우저 열기: $URL"
echo "[Azas] 다음부터는 터미널에서: azas-panel"
