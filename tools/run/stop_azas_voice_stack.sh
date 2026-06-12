#!/usr/bin/env bash
# azas-voice tmux 세션(로봇/그리퍼/카메라/음성스택)을 정리한다.
set -euo pipefail

SESSION="${SESSION:-azas-voice}"

if ! tmux has-session -t "${SESSION}" >/dev/null 2>&1; then
  echo "[Azas] no '${SESSION}' session running."
  exit 0
fi

tmux list-panes -s -t "${SESSION}" -F '#{pane_id}' | while read -r pane; do
  [[ -n "${pane}" ]] && tmux send-keys -t "${pane}" C-c >/dev/null 2>&1 || true
done

for _ in {1..25}; do
  pgrep -f 'run_doosan_real_m0609|rg2_trigger.launch.py|rs_launch.py camera_name:=camera|azas_voice.launch.py|auto_cup_flow_router' >/dev/null 2>&1 || break
  sleep 0.2
done

tmux kill-session -t "${SESSION}" >/dev/null 2>&1 || true
echo "[Azas] '${SESSION}' session stopped."
