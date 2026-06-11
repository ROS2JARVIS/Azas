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
PANEL_ROS_DOMAIN_ID="${AZAS_PANEL_ROS_DOMAIN_ID:-9}"
SERVER_SCRIPT="$ROOT/tools/run/robot_pipeline_control_server.py"
RESTART_SERVER=1

case "${1:-}" in
  --restart|restart)
    RESTART_SERVER=1
    ;;
  --reuse|reuse)
    RESTART_SERVER=0
    ;;
  -h|--help)
    cat <<MSG
Usage: azas-panel [--restart|--reuse]

Restarts the Azas robot pipeline panel server, then opens the browser.
This is the default so changed HTML/API code is always applied when azas-panel is used.

Options:
  --restart   Restart the panel server before opening the browser. This is the default.
  --reuse     Reuse a running panel server if it is already ready.
MSG
    exit 0
    ;;
  "")
    ;;
  *)
    echo "[Azas] unknown option: $1" >&2
    echo "Usage: azas-panel [--restart|--reuse]" >&2
    exit 2
    ;;
esac

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

server_pid() {
  if [[ -f "$PID_FILE" ]]; then
    local pid
    pid="$(cat "$PID_FILE" 2>/dev/null || true)"
    if [[ "$pid" =~ ^[0-9]+$ ]] && ps -p "$pid" -o args= 2>/dev/null | grep -q "robot_pipeline_control_server.py"; then
      echo "$pid"
      return 0
    fi
  fi
  pgrep -f "python3 .*tools/run/robot_pipeline_control_server.py|python3 tools/run/robot_pipeline_control_server.py" | head -n 1
}

server_needs_restart() {
  local pid="$1"
  [[ "$RESTART_SERVER" == "1" ]] && return 0
  [[ -z "$pid" ]] && return 1
  [[ ! -f "$SERVER_SCRIPT" ]] && return 1
  local etimes now started script_mtime
  etimes="$(ps -p "$pid" -o etimes= 2>/dev/null | tr -d ' ' || true)"
  [[ ! "$etimes" =~ ^[0-9]+$ ]] && return 1
  now="$(date +%s)"
  started=$((now - etimes))
  script_mtime="$(stat -c %Y "$SERVER_SCRIPT" 2>/dev/null || echo 0)"
  [[ "$script_mtime" -gt "$started" ]]
}

stop_panel_server() {
  local pid="${1:-}"
  if [[ -n "$pid" ]]; then
    echo "[Azas] 기존 패널 서버 종료: pid=$pid"
    kill "$pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      if ! ps -p "$pid" >/dev/null 2>&1; then
        break
      fi
      sleep 0.1
    done
    if ps -p "$pid" >/dev/null 2>&1; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  fi
  rm -f "$PID_FILE"
}

ensure_workspace_built() {
  if [[ ! -f "/opt/ros/humble/setup.bash" ]]; then
    cat >&2 <<'MSG'
[Azas] /opt/ros/humble/setup.bash가 없습니다.
이 PC에 ROS 2 Humble 설치가 먼저 필요합니다.
MSG
    exit 1
  fi
  if [[ ! -f "$ROOT/install/local_setup.bash" ]]; then
    cat >&2 <<MSG
[Azas] install/local_setup.bash가 없습니다. 이 PC에서 아직 빌드되지 않았습니다.
먼저 실행하세요:
  cd $ROOT
  bash tools/setup/bootstrap_local_workspace.sh

수동으로 하려면:
  source /opt/ros/humble/setup.bash
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --symlink-install
  source install/local_setup.bash
MSG
    exit 1
  fi
}

start_panel_server() {
  cd "$ROOT"
  setsid env AZAS_ROOT="$ROOT" ROS_DOMAIN_ID="$PANEL_ROS_DOMAIN_ID" ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" bash -lc '
    cd "$AZAS_ROOT"
    source /opt/ros/humble/setup.bash
    source /home/ssu/ros2_ws/install/setup.bash
    source "$AZAS_ROOT/install/local_setup.bash"
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-9}"
    export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
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
ensure_workspace_built

PID="$(server_pid || true)"
if [[ -n "$PID" ]] && server_needs_restart "$PID"; then
  if [[ "$RESTART_SERVER" == "1" ]]; then
    echo "[Azas] 패널 서버를 새로 초기화합니다."
  else
    echo "[Azas] 패널 서버 코드 변경 감지: 새 코드로 자동 재시작합니다."
  fi
  stop_panel_server "$PID"
fi

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
echo "[Azas] ROS_DOMAIN_ID: $PANEL_ROS_DOMAIN_ID"
echo "[Azas] 다음부터는 터미널에서: azas-panel"
