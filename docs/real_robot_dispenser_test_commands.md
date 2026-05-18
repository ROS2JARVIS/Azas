# 실제 로봇 디스펜서 테스트 명령어 모음

이 문서는 비전공자가 터미널에 순서대로 입력해서 실제 Doosan M0609 로봇 연결, 상태 확인, RViz 확인, side grip, 디스펜서 앞 이동 테스트를 진행할 수 있게 정리한 절차입니다.

중요 원칙:

- Azas 프로젝트 작업은 `/home/ssu/Azas` 안에서만 합니다.
- 이 절차에서는 외부 ROS 워크스페이스를 소싱하지 않습니다.
- 실제 로봇이 움직입니다. 비상정지, 안전복구, 주변 장애물 제거를 먼저 확인하세요.
- 컵 좌표는 사람이 임의로 정하지 않습니다. 실제 컵 위치는 나중에 비전 토픽 `/jarvis/tumbler_dispenser/tumbler_pose`에서 받아야 합니다.
- 지금 문서는 “컵을 이미 side grip으로 잡았다고 가정한 디스펜서 접근 테스트”입니다.

---

## 0. 터미널 기본 준비

새 터미널을 열 때마다 아래를 먼저 입력합니다.

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash
```

현재 ROS 패키지가 Azas 안에서 잡히는지 확인합니다.

```bash
ros2 pkg prefix dsr_bringup2
ros2 pkg prefix dsr_controller2
ros2 pkg prefix dsr_moveit_config_m0609
ros2 pkg prefix azas_bringup
```

정상 예시는 전부 `/home/ssu/Azas/install/...`로 나와야 합니다.

---

## 1. 코드 빌드

실제 로봇 테스트 전에는 변경된 안전 코드가 반영되도록 빌드합니다.

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
colcon build --packages-select azas_motion --symlink-install
source /home/ssu/Azas/install/local_setup.bash
```

오늘 추가한 안전장치:

- trajectory 실행 전 joint velocity를 검사합니다.
- 기본 제한은 `max_commanded_joint_velocity_deg_s:=120.0`입니다.
- 제한을 넘는 trajectory는 실제 로봇으로 보내기 전에 거부합니다.

---

## 2. 기존 실행 정리

이미 실행 중인 테스트가 있으면 새 테스트가 막히거나 로봇 명령이 꼬일 수 있습니다.

실행 중인 관련 프로세스 확인:

```bash
ps
pgrep -af "doosan_moveit|dsr_bringup2|ros2_control_node|move_group|rviz2|robot_state_publisher"
```

테스트 스크립트 lock 파일 때문에 막힐 때:

```bash
rm -f /tmp/azas_side_grip_to_dispenser.lock
rm -f /tmp/azas_side_grip_to_floor.lock
```

정말 이전 테스트를 끝내야 할 때만 아래를 사용합니다.

```bash
pkill -f doosan_moveit_grasped_tumbler_to_dispenser_node || true
pkill -f rviz2 || true
pkill -f move_group || true
```

---

## 3. 실제 로봇 연결

로봇과 PC 네트워크가 연결된 상태에서 실행합니다.

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
  mode:=real model:=m0609 host:=192.168.1.100 port:=12345 rt_host:=192.168.1.101
```

이 터미널은 계속 켜둡니다. `Ctrl+C`를 누르면 로봇 연결이 종료됩니다.

---

## 4. 연결 확인

새 터미널을 열고 아래를 실행합니다.

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash
```

실제 모드인지 확인:

```bash
ros2 param get /virtual_node mode
ros2 param get /virtual_node host
ros2 param get /virtual_node rt_host
ros2 param get /virtual_node model
```

정상 기준:

- `mode`는 `real`
- `host`는 `192.168.1.100`
- `rt_host`는 `192.168.1.101`
- `model`은 `m0609`

현재 로봇 위치 확인:

```bash
ros2 service call /aux_control/get_current_posx dsr_msgs2/srv/GetCurrentPosx '{}'
```

현재 관절값 확인:

```bash
ros2 service call /aux_control/get_current_posj dsr_msgs2/srv/GetCurrentPosj '{}'
```

알람 확인:

```bash
ros2 service call /system/get_last_alarm dsr_msgs2/srv/GetLastAlarm '{}'
```

정상 기준:

```text
level=0
success=True
```

`level=3` 같은 값이 나오면 실제 이동을 중단하고 로봇 티치펜던트에서 안전복구 후 다시 확인합니다.

---

## 5. 디스펜서 장애물 정보 실행

디스펜서 collision box를 MoveIt planning scene에 올립니다.

새 터미널:

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

tools/run/run_measured_dispenser_collision_scene.sh
```

이 터미널도 계속 켜둡니다.

장애물 토픽 확인:

```bash
ros2 topic info /collision_object
```

정상 기준:

- Publisher가 1개 이상 있어야 합니다.

---

## 6. RViz 실행

RViz로 로봇과 planning scene을 봅니다.

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

ros2 launch azas_bringup doosan_moveit_rviz_only.launch.py model:=m0609
```

RViz가 안 뜨고 `could not connect to display`가 나오면 화면이 없는 터미널에서 실행한 것입니다. 로봇 PC의 GUI 터미널에서 실행하세요.

---

## 7. 실제 이동 전 plan-only 테스트

아래 명령은 실제 로봇을 움직이지 않고 계획만 확인합니다.

오늘 테스트한 side grip 관절값:

```text
joint_1 = 119
joint_2 = -41
joint_3 = -120
joint_4 = 32
joint_5 = -103
joint_6 = -137
```

side grip 자세 계획만 확인:

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

ros2 run azas_motion doosan_moveit_grasped_tumbler_to_dispenser_node --ros-args \
  -p start_delay_sec:=0.5 \
  -p shutdown_on_complete:=true \
  -p execute_motion:=false \
  -p assume_already_at_side_grip:=false \
  -p task_mode:=side_grip_hold \
  -p enable_demo_obstacle:=false \
  -p enable_obstacle_detour:=false \
  -p planning_timeout_sec:=8.0 \
  -p max_velocity_scaling_factor:=0.002 \
  -p max_acceleration_scaling_factor:=0.002 \
  -p max_single_segment_joint_motion_deg:=220.0 \
  -p max_commanded_joint_velocity_deg_s:=120.0 \
  -p joint_1_deg:=119.0 \
  -p joint_2_deg:=-41.0 \
  -p joint_3_deg:=-120.0 \
  -p joint_4_deg:=32.0 \
  -p joint_5_deg:=-103.0 \
  -p joint_6_deg:=-137.0
```

정상 기준:

- `Planning assumed_side_grasp`
- 실패 로그 없이 종료

---

## 8. 실제 side grip 자세로 천천히 이동

주의: 이 명령은 실제 로봇이 움직입니다.

오늘 실제 테스트에서 한 번 멈춘 원인:

```text
joint 6 desired velocity=-1921.979 deg/s
limit=225.000 deg/s
```

그래서 아래 명령은 매우 낮은 속도와 velocity guard를 같이 사용합니다.

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

ros2 run azas_motion doosan_moveit_grasped_tumbler_to_dispenser_node --ros-args \
  -p start_delay_sec:=0.5 \
  -p shutdown_on_complete:=true \
  -p execute_motion:=true \
  -p assume_already_at_side_grip:=false \
  -p task_mode:=side_grip_hold \
  -p enable_demo_obstacle:=false \
  -p enable_obstacle_detour:=false \
  -p planning_timeout_sec:=8.0 \
  -p max_velocity_scaling_factor:=0.002 \
  -p max_acceleration_scaling_factor:=0.002 \
  -p max_single_segment_joint_motion_deg:=220.0 \
  -p max_commanded_joint_velocity_deg_s:=120.0 \
  -p controller_action_wait_sec:=180.0 \
  -p joint_1_deg:=119.0 \
  -p joint_2_deg:=-41.0 \
  -p joint_3_deg:=-120.0 \
  -p joint_4_deg:=32.0 \
  -p joint_5_deg:=-103.0 \
  -p joint_6_deg:=-137.0
```

성공 기준:

```text
Controller reached assumed_side_grasp
DONE: robot is holding the assumed side-grip posture.
```

실패 기준:

- `refusing ... commanded velocity`
- `controller execution failed`
- 로봇 알람 발생

실패하면 바로 아래를 확인합니다.

```bash
ros2 service call /system/get_last_alarm dsr_msgs2/srv/GetLastAlarm '{}'
ros2 service call /aux_control/get_current_posx dsr_msgs2/srv/GetCurrentPosx '{}'
```

---

## 9. side grip 상태에서 디스펜서 앞 위치로 계획만 확인

먼저 실제 이동 없이 확인합니다.

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

ros2 run azas_motion doosan_moveit_grasped_tumbler_to_dispenser_node --ros-args \
  -p start_delay_sec:=0.5 \
  -p shutdown_on_complete:=true \
  -p execute_motion:=false \
  -p assume_already_at_side_grip:=true \
  -p task_mode:=dispenser_front \
  -p selected_dispenser_id:=1 \
  -p enable_demo_obstacle:=false \
  -p enable_obstacle_detour:=false \
  -p planning_timeout_sec:=8.0 \
  -p max_velocity_scaling_factor:=0.002 \
  -p max_acceleration_scaling_factor:=0.002 \
  -p max_commanded_joint_velocity_deg_s:=120.0 \
  -p allow_dispenser_orientation_fallback:=false \
  -p dispenser_outlet_positions:="[0.490,-0.100,0.350]"
```

이 명령의 의미:

- `assume_already_at_side_grip:=true`: 로봇이 이미 컵을 side grip으로 잡았다고 가정합니다.
- `allow_dispenser_orientation_fallback:=false`: 컵 각도를 억지로 바꾸는 fallback을 막습니다.
- `dispenser_outlet_positions`: 임시 목표 위치입니다. 실제 최종 시스템에서는 비전/캘리브레이션 기반 좌표를 써야 합니다.

오늘 확인된 점:

- 너무 낮은 z에서는 디스펜서 collision과 충돌합니다.
- 기존 link_6 기준으로 `z=0.250m` 근처는 planning collision이 났습니다.
- 실제 컵을 들면 link_6와 컵 중심 사이 offset 때문에 다시 측정해야 합니다.

---

## 10. side grip 상태에서 디스펜서 앞 위치로 실제 이동

주의: 9번 plan-only가 성공한 뒤에만 실행합니다.

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

ros2 run azas_motion doosan_moveit_grasped_tumbler_to_dispenser_node --ros-args \
  -p start_delay_sec:=0.5 \
  -p shutdown_on_complete:=true \
  -p execute_motion:=true \
  -p assume_already_at_side_grip:=true \
  -p task_mode:=dispenser_front \
  -p selected_dispenser_id:=1 \
  -p enable_demo_obstacle:=false \
  -p enable_obstacle_detour:=false \
  -p planning_timeout_sec:=8.0 \
  -p max_velocity_scaling_factor:=0.002 \
  -p max_acceleration_scaling_factor:=0.002 \
  -p max_commanded_joint_velocity_deg_s:=120.0 \
  -p controller_action_wait_sec:=180.0 \
  -p allow_dispenser_orientation_fallback:=false \
  -p dispenser_outlet_positions:="[0.490,-0.100,0.350]"
```

성공 기준:

```text
Controller reached ...
DONE: grasped tumbler moved to dispenser front.
```

---

## 11. 더 낮은 z로 내려가며 한계 찾기

한 번에 낮게 가지 말고 5cm 또는 2cm씩 낮춥니다.

예시 순서:

```text
z=0.350
z=0.320
z=0.300
z=0.280
z=0.260
```

명령어에서 이 부분만 바꿉니다.

```bash
-p dispenser_outlet_positions:="[0.490,-0.100,0.350]"
```

예를 들어 z를 0.320으로 바꿀 때:

```bash
-p dispenser_outlet_positions:="[0.490,-0.100,0.320]"
```

오늘 실제 확인된 한계:

- link_6 기준으로 `x=0.555, y=-0.100` 근처에서 `z=0.300`은 성공했습니다.
- `z=0.250`은 collision planning 실패가 났습니다.
- 따라서 실제 컵 side grip 상태에서는 `z=0.350`부터 천천히 낮춰야 합니다.

---

## 12. 디스펜서 4개 순차 테스트

실제 이동 전에는 먼저 RViz 또는 plan-only로 확인합니다.

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

TASK_MODE=dispenser_sequence \
DISPENSER_SEQUENCE_IDS="[1,2,3,4]" \
MAX_VELOCITY_SCALING_FACTOR=0.002 \
MAX_ACCELERATION_SCALING_FACTOR=0.002 \
MAX_SINGLE_SEGMENT_JOINT_MOTION_DEG=220.0 \
CONTROLLER_ACTION_WAIT_SEC=180.0 \
tools/run/run_doosan_moveit_side_grip_to_all_dispensers.sh
```

주의:

- 이 스크립트는 내부에서 실제 실행을 포함합니다.
- 실제 로봇에서는 먼저 개별 디스펜서 1개가 안정적으로 되는지 확인한 뒤 사용하세요.

---

## 13. 문제 발생 시 확인 순서

로봇이 멈췄을 때:

```bash
ros2 service call /system/get_last_alarm dsr_msgs2/srv/GetLastAlarm '{}'
ros2 service call /aux_control/get_current_posx dsr_msgs2/srv/GetCurrentPosx '{}'
ros2 topic echo /joint_states --once
```

오늘 발생한 알람:

```text
level=3
group=2
index=1908
joint 6 desired velocity exceeded limit
```

이 알람이 나오면:

1. 같은 명령을 바로 재실행하지 않습니다.
2. 티치펜던트에서 안전복구합니다.
3. `get_last_alarm`이 `level=0`인지 다시 확인합니다.
4. 속도를 더 낮추거나 trajectory guard가 적용된 최신 빌드인지 확인합니다.

---

## 14. 자주 나오는 에러

### `controller action server not ready`

컨트롤러가 아직 준비되지 않았거나 로봇 launch가 제대로 안 된 상태입니다.

확인:

```bash
ros2 action list | grep follow_joint_trajectory
ros2 control list_controllers
```

정상적으로는 `/dsr_moveit_controller/follow_joint_trajectory`가 보여야 합니다.

### `another side-grip-to-dispenser validation is already running`

이전 테스트 lock이 남아 있습니다.

```bash
rm -f /tmp/azas_side_grip_to_dispenser.lock
```

그래도 안 되면 프로세스 확인:

```bash
pgrep -af doosan_moveit_grasped_tumbler_to_dispenser_node
```

### RViz가 안 보임

```text
qt.qpa.xcb: could not connect to display
```

GUI 화면이 없는 터미널에서 실행한 것입니다. 로봇 PC의 화면이 있는 터미널에서 RViz를 실행하세요.

---

## 15. 오늘까지의 결론

현재 확인된 사실:

- 실제 로봇 연결은 `/home/ssu/Azas/install` 기준으로 가능했습니다.
- 장애물 collision scene도 `/collision_object`로 올라갔습니다.
- 디스펜서 body/head collision 정보는 반영되어 있습니다.
- link_6 기준으로 디스펜서 앞 낮은 z 접근은 collision 한계가 있습니다.
- side grip 자세 이동 중 joint 6 velocity 초과 알람이 발생했습니다.

다음 테스트의 안전한 순서:

1. 알람 `level=0` 확인
2. velocity guard가 들어간 `azas_motion` 빌드
3. `execute_motion:=false`로 side grip plan-only 확인
4. `execute_motion:=true`로 side grip까지만 이동
5. 그 상태에서 z=0.350부터 디스펜서 접근 plan-only
6. 성공한 z만 실제 이동
