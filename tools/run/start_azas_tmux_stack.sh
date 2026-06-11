#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/ssu/Azas}"
SESSION="${SESSION:-azas-logic}"
ROBOT_HOST="${ROBOT_HOST:-192.168.1.100}"
ROBOT_NAME="${ROBOT_NAME:-dsr01}"
RT_HOST="${RT_HOST:-0.0.0.0}"
RG2_IP="${RG2_IP:-192.168.1.1}"
RG2_PORT="${RG2_PORT:-502}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-9}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
FORCE_RESTART="${FORCE_RESTART:-false}"
CLEAN_FASTDDS_SHM="${CLEAN_FASTDDS_SHM:-false}"

cd "${ROOT}"
mkdir -p "${ROOT}/log/tmux_logic" /tmp/azas_ros_logs

if tmux has-session -t "${SESSION}" >/dev/null 2>&1; then
  if tmux list-windows -t "${SESSION}" -F '#{window_name}' | grep -qx 'side_grip' \
    && pgrep -f '[y]olo_cup_pick_node' >/dev/null 2>&1 \
    && [[ "${FORCE_RESTART}" != "true" ]]; then
    echo "[Azas] BLOCKED: ${SESSION}:side_grip is running. Stop side_grip first or set FORCE_RESTART=true." >&2
    exit 3
  fi

  tmux list-panes -t "${SESSION}" -F '#{pane_id}' | while read -r pane; do
    [[ -n "${pane}" ]] && tmux send-keys -t "${pane}" C-c >/dev/null 2>&1 || true
  done
  for _ in {1..20}; do
    if ! tmux has-session -t "${SESSION}" >/dev/null 2>&1; then
      break
    fi
    if ! pgrep -f "tmux.*${SESSION}|run_doosan_real_m0609|rg2_trigger.launch.py|rs_launch.py camera_name:=camera|joint_state_relay.py" >/dev/null 2>&1; then
      break
    fi
    sleep 0.2
  done
  tmux kill-session -t "${SESSION}" >/dev/null 2>&1 || true
fi

# Stop only the ROS CLI graph daemon. Robot/camera processes are cleaned by the tmux session above.
while read -r pid cmd; do
  [[ -z "${pid:-}" ]] && continue
  if [[ "${cmd}" == *"ros2cli.daemon.daemonize"* ]]; then
    kill "${pid}" >/dev/null 2>&1 || true
  fi
done < <(ps -eo pid=,cmd=)

if [[ "${CLEAN_FASTDDS_SHM}" == "true" ]]; then
  if pgrep -f 'ros2|realsense2_camera_node|dsr_controller2|move_group|rviz2|rg2_gripper_node|yolo_cup_pick_node' >/dev/null 2>&1; then
    echo "[Azas] SKIP FastDDS SHM cleanup: ROS processes are still running." >&2
  else
    rm -f /dev/shm/fastrtps_* >/dev/null 2>&1 || true
    echo "[Azas] cleaned /dev/shm/fastrtps_*"
  fi
fi

common_env="export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}; export FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS}; export ROS_LOG_DIR=/tmp/azas_ros_logs"
robot_cmd="cd ${ROOT}; mkdir -p log/tmux_logic /tmp/azas_ros_logs; ${common_env}; export ROBOT_HOST=${ROBOT_HOST}; export ROBOT_NAME=${ROBOT_NAME}; export RT_HOST=${RT_HOST}; export DOOSAN_REAL_MOTION_CONFIRM=ENABLE_DOOSAN_REAL_MOTION_BRINGUP; bash ${ROOT}/tools/run/run_doosan_real_m0609.sh 2>&1 | tee ${ROOT}/log/tmux_logic/robot-\$(date +%Y%m%d-%H%M%S).log"
gripper_cmd="cd ${ROOT}; ${common_env}; source /opt/ros/humble/setup.bash; source ${ROOT}/install/setup.bash; ros2 launch ${ROOT}/install/azas_gripper/share/azas_gripper/launch/rg2_trigger.launch.py ip:=${RG2_IP} port:=${RG2_PORT} connect:=true open_width:=1100 close_width:=0 force:=300 settle_seconds:=0.6 2>&1 | tee ${ROOT}/log/tmux_logic/gripper-\$(date +%Y%m%d-%H%M%S).log"
camera_cmd="cd ${ROOT}; ${common_env}; source /opt/ros/humble/setup.bash; source ${ROOT}/install/setup.bash; ros2 launch realsense2_camera rs_launch.py camera_name:=camera initial_reset:=true reconnect_timeout:=5.0 enable_color:=true enable_depth:=true align_depth.enable:=true rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30 2>&1 | tee ${ROOT}/log/tmux_logic/camera-\$(date +%Y%m%d-%H%M%S).log"
relay_cmd="cd ${ROOT}; ${common_env}; source /opt/ros/humble/setup.bash; source ${ROOT}/install/setup.bash; python3 ${ROOT}/src/dsr_practice/dsr_practice/joint_state_relay.py --ros-args -r __node:=azas_joint_state_relay -p input_topic:=/${ROBOT_NAME}/joint_states -p output_topic:=/joint_states 2>&1 | tee ${ROOT}/log/tmux_logic/joint_relay-\$(date +%Y%m%d-%H%M%S).log"

tmux new-session -d -s "${SESSION}" -n robot "${robot_cmd}"
sleep 10
tmux new-window -t "${SESSION}" -n gripper "${gripper_cmd}"
sleep 3
tmux new-window -t "${SESSION}" -n camera "${camera_cmd}"
sleep 8
tmux new-window -t "${SESSION}" -n joint_relay "${relay_cmd}"

echo "[Azas] tmux stack started: ${SESSION}"
echo "[Azas] attach outside tmux: tmux attach -t ${SESSION}"
echo "[Azas] switch inside tmux: tmux switch-client -t ${SESSION}"
tmux list-windows -t "${SESSION}"
