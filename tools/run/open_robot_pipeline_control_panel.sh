#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/ssu/Azas"
URL="${AZAS_PANEL_URL:-http://127.0.0.1:8765/}"
LOG_DIR="$ROOT/log/panel"
LOG_FILE="$LOG_DIR/robot_pipeline_control_panel.log"
PID_FILE="/tmp/azas-panel-8765.pid"

mkdir -p "$LOG_DIR"

panel_ready() {
  curl -fsS --max-time 1 "$URL" >/dev/null 2>&1
}

start_panel_server() {
  cd "$ROOT"
  setsid bash -lc '
    cd /home/ssu/Azas
    source /opt/ros/humble/setup.bash
    source /home/ssu/Azas/install/local_setup.bash
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
