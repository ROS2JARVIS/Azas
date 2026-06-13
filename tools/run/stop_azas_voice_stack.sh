#!/usr/bin/env bash
# azas-voice tmux 세션(로봇/그리퍼/카메라/음성스택)과 수동으로 띄운
# azas_voice 노드를 정리한다.
set -euo pipefail

SESSION="${SESSION:-azas-voice}"
GRACE_SEC="${GRACE_SEC:-3}"

VOICE_PATTERN='azas_voice\.launch\.py|/azas_voice/(recipe_mapper_node|llm_recipe_mapper_node|conversation_manager_node|voice_pipeline_executor_node|voice_dispenser_executor_node|tts_node|voice_screen_node|stt_node)'
PROTECT_PATTERN='codex|oh-my-codex|omx|stop_azas_voice_stack\.sh|grep -E'

collect_voice_pids() {
  ps -eo pid=,stat=,args= | grep -E "${VOICE_PATTERN}" | grep -Ev "${PROTECT_PATTERN}" \
    | while read -r pid stat args; do
        [[ "${stat}" == Z* ]] && continue
        echo "${pid}"
      done
}

if tmux has-session -t "${SESSION}" >/dev/null 2>&1; then
  tmux list-panes -s -t "${SESSION}" -F '#{pane_id}' | while read -r pane; do
    [[ -n "${pane}" ]] && tmux send-keys -t "${pane}" C-c >/dev/null 2>&1 || true
  done

  for _ in {1..25}; do
    pgrep -f 'run_doosan_real_m0609|rg2_trigger.launch.py|rs_launch.py camera_name:=camera|azas_voice.launch.py|auto_cup_flow_router' >/dev/null 2>&1 || break
    sleep 0.2
  done

  tmux kill-session -t "${SESSION}" >/dev/null 2>&1 || true
  echo "[Azas] '${SESSION}' session stopped."
else
  echo "[Azas] no '${SESSION}' session running."
fi

mapfile -t voice_pids < <(collect_voice_pids)
if [[ "${#voice_pids[@]}" -gt 0 ]]; then
  echo "[Azas] stopping ${#voice_pids[@]} azas_voice processes"
  kill -TERM "${voice_pids[@]}" 2>/dev/null || true
  deadline=$((SECONDS + GRACE_SEC))
  while [[ ${SECONDS} -lt ${deadline} ]]; do
    mapfile -t voice_pids < <(collect_voice_pids)
    [[ "${#voice_pids[@]}" -eq 0 ]] && break
    sleep 0.2
  done
  mapfile -t voice_pids < <(collect_voice_pids)
  if [[ "${#voice_pids[@]}" -gt 0 ]]; then
    echo "[Azas] force-killing ${#voice_pids[@]} lingering azas_voice processes"
    kill -KILL "${voice_pids[@]}" 2>/dev/null || true
  fi
else
  echo "[Azas] no stray azas_voice processes found."
fi
