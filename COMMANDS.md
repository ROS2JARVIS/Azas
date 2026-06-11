# Azas 명령어 빠른 참조

> Doosan M0609 + OnRobot RG2 + RealSense 칵테일 로봇  
> 현장 투입 전 이 순서대로 진행하세요.
>
> 원칙: Azas 프로젝트 작업은 `/home/ssu/Azas`와 `/home/ssu/Azas/install`만 사용합니다.
> 외부 ROS 워크스페이스를 소싱하는 예전 절차는 폐기했습니다.

---

## 목차

1. [환경 설정](#1-환경-설정)
2. [빌드 · 테스트](#2-빌드--테스트)
3. [시뮬레이션 (가상 로봇)](#3-시뮬레이션-가상-로봇)
4. [비-하드웨어 점검](#4-비-하드웨어-점검)
5. [카메라 연결](#5-카메라-연결)
6. [드라이런 (Dry-run)](#6-드라이런-dry-run)
7. [실제 로봇 운용](#7-실제-로봇-운용)
8. [스모크 테스트](#8-스모크-테스트)
9. [TF · 토픽 디버그](#9-tf--토픽-디버그)
10. [그라스프 프레임 수집](#10-그라스프-프레임-수집)
11. [전원 차단 복구](#11-전원-차단-복구)

---

## 1. 환경 설정

처음 받은 PC에서는 먼저 로컬 `install/`을 생성합니다.

```bash
cd /home/ssu/Azas
git switch develop
git pull origin develop
bash tools/setup/bootstrap_local_workspace.sh
```

매 터미널 시작 시 필요한 기본 소싱:

```bash
# ROS 2 Humble 소싱
source /opt/ros/humble/setup.bash

# Azas 워크스페이스 소싱 (bootstrap 또는 colcon build 후)
source /home/ssu/Azas/install/local_setup.bash
```

> **팁**: `~/.bashrc`에 `source /opt/ros/humble/setup.bash` 추가하면 편합니다.
> `install/`은 Git에 올리지 않습니다. 각 PC에서 `bootstrap_local_workspace.sh` 또는 `colcon build --symlink-install`로 재생성합니다.

---

## 2. 빌드 · 테스트

```bash
cd /home/ssu/Azas

# 처음 PC 또는 install 누락 시 권장
bash tools/setup/bootstrap_local_workspace.sh

# 전체 빌드
colcon build --symlink-install

# 특정 패키지만 빌드
colcon build --symlink-install --packages-select azas_perception azas_interfaces

# 전체 테스트
colcon test
colcon test-result --verbose

# 특정 패키지 테스트
colcon test --packages-select azas_voice
```

---

## 제어 패널

```bash
cd /home/ssu/Azas

# 패널 서버를 새로 초기화하고, 브라우저에서 http://127.0.0.1:8765/ 를 엽니다.
# 첫 실행 때 ~/.local/bin/azas-panel symlink도 자동으로 준비합니다.
bash tools/run/open_robot_pipeline_control_panel.sh
```

첫 실행 뒤에는 어느 디렉터리에서든 짧은 명령으로 열 수 있습니다.

```bash
azas-panel
```

이미 떠 있는 서버를 그대로 재사용해서 열 때:

```bash
azas-panel --reuse
```

패널 서버만 종료할 때:

```bash
pgrep -af "robot_pipeline_control_server.py"
kill -TERM <PID>
```

---

## 3. 시뮬레이션 (가상 로봇)

```bash
# 가상 Doosan M0609 시작
bash tools/run/run_doosan_virtual_m0609.sh

# MoveIt + RViz 수동 실행
source /home/ssu/Azas/install/local_setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
  model:=m0609 mode:=virtual host:=127.0.0.1 port:=12345
```

### 칵테일 디스펜서 전체 사이클 RViz preview

컵을 디스펜서 앞에 놓고 → 그리퍼를 완전히 열고 → 안전하게 위로 올라간 뒤
→ 빈 그리퍼를 닫고 → 측정된 프레스 조인트에서 펌프질 → 컵을 다시 잡는
통합 사이클을 RViz에서 먼저 봅니다.

```bash
cd /home/ssu/Azas
bash tools/run/stop_cocktail_motion_preview.sh
bash tools/run/show_cocktail_motion_preview.sh 1x1
```

`show_cocktail_motion_preview.sh`의 기본 RViz는 교안의 MoveIt RViz
(`RVIZ_MODE=bringup`)라서 주황색 로봇/Trajectory/PlanningScene 화면으로 보입니다.
하얀 RobotModel 중심의 디버그 화면이 필요할 때만 `RVIZ_MODE=clean`을 붙입니다.

동일한 동작을 환경변수로 직접 실행하려면:

```bash
cd /home/ssu/Azas
RECIPE_DISPENSER_IDS=1x1 \
DISPENSER_COLLISION_OBJECTS=1 \
KEEP_ALIVE_AFTER_DONE=1 \
RESET_EXISTING_VIRTUAL_PREVIEW=1 \
REPLACE_EXISTING_RVIZ=1 \
bash tools/run/run_cocktail_collision_rviz_preview.sh
```

예: 1번 디스펜서 2회 프레스

```bash
RECIPE_DISPENSER_IDS=1x2 \
DISPENSER_COLLISION_OBJECTS=1 \
KEEP_ALIVE_AFTER_DONE=1 \
bash tools/run/run_cocktail_collision_rviz_preview.sh
```

RViz preview가 떠 있는 상태에서 실제 로봇 one-click 스크립트를 실행하면
virtual/emulator 세션과 실제 세션이 섞이지 않도록 거부합니다.

preview를 닫고 실제 로봇 실행으로 전환하려면:

```bash
bash tools/run/stop_cocktail_motion_preview.sh
```

이 정리 스크립트는 preview shell뿐 아니라 `dsr_bringup2_moveit.launch.py mode:=virtual`,
`run_emulator`, `DRCF M0609`, 관련 RViz까지 확인합니다. 남은 virtual/emulator가
있으면 실제 one-click은 계속 거부됩니다.

### 측정 조인트 티칭값 기반 RViz preview

`calibration.yaml`의 `cup_pre_place_joints_deg`, `cup_place_joints_deg`,
`press_pre_joints_deg`, `press_contact_joints_deg`를 읽어서 `/joint_states`만
publish합니다. 실제 로봇 서비스, MoveJoint, MoveLine, 그리퍼 서비스는 호출하지
않습니다.

RViz까지 한 번에 띄우려면 이 wrapper를 씁니다.

```bash
source /opt/ros/humble/setup.bash
cd /home/ssu/Azas
source install/setup.bash
bash tools/run/show_measured_recipe_joint_preview_rviz.sh --dispenser-ids 1 --loop
```

이 wrapper는 기본적으로 창현 side-grip 로직과 같은 source tree의
`src/azas_bringup/config/safety.yaml`,
`src/azas_bringup/config/measured_dispenser_collision.yaml`을 명시적으로 로드합니다.
`side_grip_table`, `side_grip_workspace_*` 안전영역, measured dispenser collision,
full collision YAML marker를 RViz에 띄웁니다. 깜빡임이 있으면 남은 preview 노드를
정리하고 다시 실행합니다.

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source install/setup.bash
pkill -f 'preview_measured_dispenser_recipe_rviz.py|publish_measured_recipe_joint_rviz_preview.py|joint_state_relay.py|dispenser_sequence_preview_node|workspace_collision_scene_node|measured_dispenser_collision_scene_node|collision_scene_rviz_publisher|link6_gripper_collision_node|__node:=m0609_robot_state_publisher' || true
pkill -x rviz2 || true

DISPLAY=:20.0 \
SHOW_WORKSPACE_SAFETY=true \
SHOW_MEASURED_DISPENSER_COLLISION=true \
SHOW_FULL_COLLISION_SCENE=true \
PUBLISH_WORKSPACE_COLLISION_OBJECTS=true \
PUBLISH_DISPENSER_COLLISION_OBJECTS=true \
JOINT_VELOCITY_DEG_S=40.0 \
HOLD_SECONDS=1.0 \
PUBLISH_RATE=60.0 \
bash tools/run/show_measured_recipe_joint_preview_rviz.sh \
  --dispenser-ids 1 \
  --no-press-reset-before-press \
  --press-use-recorded-pre-joints \
  --press-depth-m 0.040 \
  --press-extra-depth-m 0.0 \
  --safe-lift-joint-fallback \
  --press-lock-contact-joints 6 \
  --loop
```

첫 번째 터미널에서 RViz demo를 실행합니다.

```bash
source /opt/ros/humble/setup.bash
cd /home/ssu/Azas
source install/setup.bash
ros2 launch dsr_moveit_config_m0609 demo.launch.py
```

다른 터미널에서 측정 조인트 preview publisher를 실행합니다. 이 Python 명령만으로는
RViz 창을 띄우지 않습니다.

```bash
source /opt/ros/humble/setup.bash
cd /home/ssu/Azas
source install/setup.bash
python3 tools/run/preview_measured_dispenser_recipe_rviz.py --dispenser-ids 1 --loop
```

예: 1번 디스펜서를 2회, 3번 디스펜서를 1회 순서로 표시

```bash
python3 tools/run/preview_measured_dispenser_recipe_rviz.py --dispenser-ids 1x2,3
```

### 색상 스캔 자세 RViz preview

디스펜서 색상 구분 전에 쓰는 카메라 보기 관절 자세
`[0, 10, 32, 0, 100, 90]°`를 실제 로봇 명령 없이 RViz에서 표시합니다.

```bash
cd /home/ssu/Azas
bash tools/run/show_color_scan_pose_rviz.sh
```

검증용으로 RViz 창 없이 `/joint_states`만 확인하려면:

```bash
USE_RVIZ=false bash tools/run/show_color_scan_pose_rviz.sh
```

현재 상태가 실제 실행 가능한지 확인하려면:

```bash
bash tools/run/check_one_click_cocktail_ready.sh
```

---

## 4. 비-하드웨어 점검

```bash
# OSS 스택 전체 점검 (패키지·런치·의존성)
bash tools/checks/check_oss_stack.sh

# 제어 준비도 종합 점검
bash tools/checks/verify_control_readiness.sh

# 실제 모션 차단 요인 설명
bash tools/checks/explain_real_robot_blockers.sh

# 실제 모션 설정 점검
bash tools/checks/check_real_motion_config.sh

# 연결 단계 결정 (다음에 무엇을 연결해야 하는지)
bash tools/checks/check_connection_stage.sh

```

---

## 5. 카메라 연결

```bash
# RealSense D435i 시작
ros2 launch realsense2_camera rs_align_depth_launch.py

# 카메라 토픽 확인
ros2 topic list | grep camera
ros2 topic echo --once /camera/camera/color/camera_info

# 깊이 인코딩 확인
ros2 topic echo --once /camera/camera/aligned_depth_to_color/image_raw | grep encoding

# 카메라 TF 확인
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

# YOLO 탐지 확인
bash tools/checks/check_robot_detection.sh
```

모션으로 이어지는 컵 탐지는 `/azas/cup_detection` status가 `detected:upright`로 시작해야 합니다. `detected:lid`는 lid 확인용이며 `/jarvis/tumbler_dispenser/tumbler_pose`의 컵 pose 계약을 만족하지 않습니다.

컵 높이 기반 upright/lying threshold를 잡을 때는 작업대를 비운 뒤 30프레임 baseline을 먼저 저장합니다.

legacy `move_camera_home()` 관측 자세만 분리해서 이동하려면 먼저 dry-run으로 target을 확인합니다.

```bash
tools/run/move_to_legacy_camera_home.py --dry-run
```

실제 이동은 로봇 주변 안전 확인 후 명시 confirm과 함께 실행합니다.

```bash
tools/run/move_to_legacy_camera_home.py \
  --enable-real-motion \
  --confirm I_UNDERSTAND_THIS_WILL_MOVE_THE_ROBOT
```

컵 탐지가 `unknown`, `no_tumbler_detection`, `invalid_depth` 등으로 실패하면 HOME으로
복귀한 뒤 다시 OBSERVE 관측 자세로 이동시켜 재관측합니다. 기본 재시도는 2회이며,
재확인 결과가 `upright`면 side-grab으로, `lying`이면 lying-upright flow로 넘깁니다.
먼저 dry-run으로 확인합니다.

```bash
tools/run/recover_home_then_observe_on_cup_failure.py \
  --dry-run \
  --max-retries 2
```

실제 복귀 이동은 로봇 주변 안전 확인 후 명시 confirm과 함께 실행합니다.

```bash
tools/run/recover_home_then_observe_on_cup_failure.py \
  --enable-real-motion \
  --confirm I_UNDERSTAND_THIS_WILL_MOVE_THE_ROBOT \
  --max-retries 2
```

복귀 helper의 종료 의미는 `0=upright/side-grab 가능`, `10=lying/세우기 flow로 전달`,
`20=unknown 지속/operator 확인 필요`입니다.

```bash
ros2 launch azas_bringup yolo_perception.launch.py \
  capture_empty_table_baseline:=true \
  baseline_frame_count:=30 \
  empty_table_baseline_path:=/tmp/azas_empty_table_depth_baseline.npy
```

로그에 `Empty-table depth baseline ready`가 뜬 뒤 컵 샘플을 놓고 `/azas/cup_detection`의 `table_height_m`, `height_median`, `height_p90`, `height_max`, `height_valid_ratio`, `bbox`, `orientation`을 기록합니다. 세부 절차는 [docs/cup_height_orientation_calibration.md](docs/cup_height_orientation_calibration.md)를 따릅니다.

---

## 6. 드라이런 (Dry-run)

드라이런은 실제 모션 없이 전체 파이프라인을 검증합니다.

```bash
# 카메라 기반 드라이런 전체 파이프라인
bash tools/run/run_robot_dryrun.sh

# 칵테일 드라이런 시퀀스
ros2 launch azas_bringup cocktail_dryrun.launch.py

# 현장 비-모션 종합 보고서
bash tools/run/field_no_motion_report.sh
```

---

## 7. 실제 로봇 운용

> **경고**: 실제 로봇 연결 전 반드시 아래 게이트를 통과해야 합니다.

### 7-0. 실로봇 테스트 사다리

한 번에 full cocktail을 실행하지 말고 아래 staged ladder로 실패 지점을 좁힙니다.

```bash
# 현재 차단 원인 설명
STAGE=status tools/run/run_real_robot_test_ladder.sh

# strict live gate까지 통과해야 실제 one-shot 테스트 가능
STAGE=live-gate RUN_LID_STABILITY=true RUN_CUP_STABILITY=true \
  tools/run/run_real_robot_test_ladder.sh

# 마지막 supervised one-shot 실제 pick
STAGE=pick-real CONFIRM=I_UNDERSTAND_THIS_WILL_MOVE_THE_ROBOT \
  tools/run/run_real_robot_test_ladder.sh
```

상세 절차 → `docs/real_robot_test_ladder.md`

### 7-1. 로봇 네트워크 연결

```bash
# 로봇 서브넷 IP 임시 추가 (기본 서브넷)
sudo ip addr add 192.168.137.50/24 dev enp128s31f6
ping 192.168.137.100

# 또는 대체 서브넷
sudo ip addr add 192.168.127.50/24 dev enp128s31f6
ping 192.168.127.100
```

### 7-2. 실제 로봇 MoveIt 연결

```bash
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
  mode:=real model:=m0609 host:=192.168.1.100 port:=12345 rt_host:=192.168.1.101
```

### 7-3. RG2 그리퍼 서비스 시작

```bash
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash
ros2 launch jarvis rg2_trigger.launch.py ip:=192.168.1.1
```

비-모션 check/smoke 명령은 RG2 서비스의 존재와 타입을 확인할 수 있지만, 실제 `/jarvis/rg2/open` 또는 `/jarvis/rg2/close` 동작 검증이 아닙니다. 실제 그리퍼 actuation 증거는 별도 현장 절차로 기록해야 합니다.

### 7-4. 하드웨어 게이트 통과 확인

```bash
# 비-모션 하드웨어 게이트 점검
bash tools/checks/check_live_hardware_gates.sh

# 엄격 모드 (모든 경고 포함) + 게이트 스탬프 발급
STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed \
  bash tools/checks/check_live_hardware_gates.sh
```

### 7-5. 실제 모션 실행

```bash
# 게이트 통과 후에만 실행 가능
bash tools/run/run_robot_real.sh

# 연결 로봇 제어 (통합)
bash tools/run/run_connected_robot_control.sh
```

`run_robot_real.sh`는 strict gate stamp와 측정 config를 다시 확인한 뒤에도, operator 확인 전 `detected:upright` cup pose와 실제 camera-derived tumbler pose를 요구합니다.

### 7-6. 실제 로봇 디스펜서 통합 사이클 one-click

RViz preview로 동작을 확인한 뒤, 실제 로봇 연결부터 통합 디스펜서 사이클까지
한 번에 실행합니다. 실행 전 virtual/RViz preview 세션은 종료되어 있어야 합니다.
패널에서는 `실제 실행 준비확인` 버튼으로 상태를 보고, `실제 one-click` 버튼으로
preview 정리 후 동일한 통합 실행을 시작할 수 있습니다.

```bash
cd /home/ssu/Azas

# preview/emulator가 남아 있으면 먼저 정리
bash tools/run/stop_cocktail_motion_preview.sh

# 현재 real/RG2 서비스 상태 확인
TCP_CHECK_SEC=1 TCP_HARD_BLOCK=1 RECIPE_DISPENSER_IDS=1x1 \
bash tools/run/check_one_click_cocktail_ready.sh || true

REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE \
RECIPE_DISPENSER_IDS=1x1 \
ROBOT_HOST=192.168.1.100 \
bash tools/run/run_cocktail_now_real.sh
```

`run_cocktail_now_real.sh`는 내부에서도 preview 정리를 한 번 더 수행합니다. 따라서
운영 명령은 위 한 줄로 충분하지만, RViz preview에서 바로 넘어오는 경우에는 정리 로그가
`Preview stop complete`인지 확인하고 진행합니다.

예: 1번 디스펜서 2회 프레스

```bash
REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE \
ROBOT_HOST=192.168.1.100 \
bash tools/run/run_cocktail_now_real.sh 1x2
```

이 스크립트는 `/dsr01` 아래 실제 Doosan motion 서비스, `/jarvis/rg2/set_width`
그리퍼 서비스, 측정 디스펜서 collision publisher를 준비한 뒤
`run_measured_dispenser_recipe_sequence.py --execute --confirm`으로
컵놓기→프레스→다시잡기 사이클을 실행합니다. 컵 좌표는 직접 입력하지 않고
기존 비전/pose 파이프라인과 측정 디스펜서 pose만 사용합니다.
`calibration.yaml`에 `press_contact_joints_deg`가 있는 디스펜서는
해당 측정 조인트를 프레스 접촉 자세의 기준으로 사용하고, 설정된 Cartesian
pre-pose를 먼저 강제로 타지 않습니다. 접촉 조인트 도달 후 live TCP를 읽어
그 위치의 Z만 올리고/내리며 `RECIPE_DISPENSER_IDS=1x2` 같은 반복 프레스를 수행합니다.
실제 Doosan bringup 전에 `check_one_click_cocktail_config.sh`가 먼저 실행되어
해당 레시피의 front-hold pose와 press contact joint가 모두 있는지 확인합니다.
motion service가 아직 없으면 `check_one_click_cocktail_ready.sh`와
`run_one_click_cocktail_real.sh`가 `ROBOT_HOST:12345` TCP 연결을 먼저 확인합니다.
`[WARN] Doosan TCP not reachable now` 또는 연결 timeout 진단이 나오면 프레스 로직으로
진입하지 못한 상태이므로 로봇 컨트롤러 IP/네트워크/펜던트 상태를 먼저 복구해야 합니다.
`run_cocktail_now_real.sh`는 실제 실행 모드에서 이 TCP 불가 상태를 hard-block으로
처리하고, `DRY_RUN=1`일 때만 명령 경로 확인을 위해 계속 진행합니다.

정상 종료 시 콘솔과 `log/manual/one_click_real_integrated_recipe.log`에
`[PASS] measured dispenser recipe sequence completed`가 남고,
마지막에 `get_current_posj`/`get_current_posx` 샘플을 출력합니다.
실패 시에는 실패 stage와 통합 로그 tail을 바로 출력합니다.
실행 후 로그만 다시 판정하려면:

```bash
bash tools/run/check_one_click_cocktail_result.sh
```

로그 tail, 관련 프로세스, 결과 판정을 한 번에 모으려면:

```bash
bash tools/run/report_cocktail_now_status.sh
```

프레스 전 안전 상승 높이, 누르는 깊이, RG2 대기시간은 환경변수로 조절할 수 있습니다.

```bash
REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE \
RECIPE_DISPENSER_IDS=1x2 \
PRESS_PRE_LIFT_M=0.35 \
PRESS_TRANSIT_HEIGHT_M=0.30 \
PRESS_DEPTH_M=0.07 \
RG2_OPEN_SETTLE_SECONDS=6.0 \
ROBOT_HOST=192.168.1.100 \
bash tools/run/run_one_click_cocktail_real.sh
```

## 8. 스모크 테스트

하드웨어 없이 실행 가능한 자동화 테스트입니다.

```bash
# 픽앤얼라인 액션 비-모션 스모크
bash tools/smoke/smoke_pick_and_align_no_motion.sh

# 제어 경로 엔드투엔드 스모크
bash tools/smoke/smoke_control_path.sh

# 실제 모션 없이 one-click 칵테일 경로/패널 명령 생성 검증
bash tools/smoke/smoke_one_click_cocktail_no_motion.sh

# 가짜 하드웨어 서비스 스모크
bash tools/smoke/smoke_fake_hardware_path.sh

# 칵테일 드라이런 시퀀스 스모크
bash tools/smoke/smoke_cocktail_dryrun_sequence.sh

# 실제 모션 진입점 게이트 스모크
bash tools/smoke/smoke_real_motion_entrypoint_gates.sh

# 가짜 하드웨어 서비스 수동 시작 (별도 터미널)
python3 tools/smoke/fake_hardware_services.py
```

---

## 9. TF · 토픽 디버그

```bash
# TF 파이프라인 점검
bash tools/checks/check_tf_pipeline.sh

# TF 트리 시각화
ros2 run tf2_tools view_frames

# TF 에코
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

# 탐지 포즈 확인
ros2 topic echo /jarvis/tumbler_dispenser/tumbler_pose

# 활성 토픽 목록
ros2 topic list | grep -E "tf|tumbler|camera|yolo"

# 깊이 투영 샘플 점검
bash tools/checks/check_depth_projection_sample.sh
```

---

## 10. 그라스프 프레임 수집

```bash
# 기본 수집 (rgb + depth + camera_info)
python3 tools/perception/export_grasp_frame.py \
  --output /tmp/azas_grasp_frame \
  --rgb-topic /camera/camera/color/image_raw \
  --depth-topic /camera/camera/aligned_depth_to_color/image_raw \
  --camera-info-topic /camera/camera/color/camera_info \
  --timeout-sec 10

# 탐지 bbox 대기 후 수집
python3 tools/perception/export_grasp_frame.py \
  --output /tmp/azas_grasp_frame \
  --rgb-topic /camera/camera/color/image_raw \
  --depth-topic /camera/camera/aligned_depth_to_color/image_raw \
  --camera-info-topic /camera/camera/color/camera_info \
  --wait-for-bbox \
  --timeout-sec 10
```

---

## 11. 전원 차단 복구

```bash
# 복구 절차 문서
cat docs/recovery_after_poweroff.md
```

---

## 현장 투입 순서 요약

```
① 환경 소싱
   source /opt/ros/humble/setup.bash && source install/setup.bash

② 가상 Doosan 시작
   bash tools/run/run_doosan_virtual_m0609.sh

③ 비-모션 전체 점검
   bash tools/checks/verify_control_readiness.sh

④ 엄격 게이트 스탬프 발급
   STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed \
     bash tools/checks/check_live_hardware_gates.sh

⑤ `detected:upright` 컵 pose 확인 후 실제 로봇 모션 실행
   bash tools/run/run_robot_real.sh
```

---

## 자주 쓰는 ros2 명령어

```bash
# 서비스 목록
ros2 service list | grep -E "gripper|motion|azas"

# 액션 목록
ros2 action list

# 노드 목록
ros2 node list

# 패키지 실행
ros2 run azas_perception yolo_tumbler_detector_node

# 런치
ros2 launch azas_bringup mvp_bringup.launch.py
ros2 launch azas_voice azas_voice.launch.py

# 파라미터 조회
ros2 param list /azas_pick_and_align_action_server
```

---

> 문서 최종 업데이트: 2026-05-13  
> 문의: GitHub Issues → `[docs]` 라벨로 등록
