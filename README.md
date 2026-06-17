# Azas

> Doosan M0609, OnRobot RG2, RealSense D435i, ROS 2 Humble 기반의 지능형 칵테일 제조 로봇 프로젝트입니다.<br>
> RGB-D 비전으로 컵과 뚜껑을 인식하고, 로봇 매니퓰레이터가 컵 파지, 디스펜서 출수, 뚜껑 체결, 쉐이킹, 사람 손바닥 핸드오버까지 수행하는 통합 서비스 로봇 시스템을 목표로 합니다.

---

## 프로젝트 한 줄 요약

Azas는 카메라가 본 컵과 사람 손의 위치를 ROS 2 토픽으로 구조화하고, 측정된 캘리브레이션 값과 안전 게이트를 통과한 경로만 사용해서 Doosan M0609 로봇과 RG2 그리퍼를 제어하는 칵테일 로봇 워크스페이스입니다.

LLM 또는 음성 인식은 사용자 의도와 레시피 선택까지만 담당합니다. 로봇 좌표, 궤적, 충돌 판단, 캘리브레이션 값은 비전, 실측 설정, MoveIt/Doosan 제어 계층에서만 다룹니다.

---

## 이 프로젝트가 다루는 기술

이 저장소는 단순 웹/앱 프로젝트가 아니라 실제 하드웨어와 연결되는 로보틱스 통합 프로젝트입니다. 핵심 기술 영역은 다음과 같습니다.

| 영역 | 내용 |
|------|------|
| 로봇 매니퓰레이션 | Doosan M0609 6축 로봇의 MoveJoint/MoveLine, MoveItPy 기반 경로 계획, RViz/Gazebo preview |
| 엔드이펙터 제어 | OnRobot RG2 그리퍼 open/close/set_width 서비스, Modbus/TCP 기반 현장 연결 |
| RGB-D 비전 | RealSense D435i color/depth 입력, YOLO 탐지, depth projection, TF2 base frame 변환 |
| 컵/뚜껑 인식 | 텀블러 upright/side 상태 분기, 컵/뚜껑 YOLO, 빨간 스티커/ArUco 기반 뚜껑 pose 후보 |
| 레시피 실행 | 음성/키오스크 주문을 symbolic recipe로 변환하고 색상 디스펜서 sequence로 실행 |
| 디스펜서 조작 | 실측된 front-hold/press pose를 사용해 컵 이송, 펌프 press, 컵홀더 재배치 |
| 쉐이킹 | 뚜껑 체결 후 컵홀더 재파지, 관절/직선 motion 기반 rule-based shake |
| 사람 핸드오버 | MediaPipe 손바닥 검출, stable palm 후보, XY 선이동 후 Z 하강, force contact 감지 후 release |
| 안전/운영 | strict live gate, no-motion checks, 측정 YAML, 로그/패널, 단계형 현장 runbook |

<p align="center">
<img width="2032" height="1286" alt="Image" src="https://github.com/user-attachments/assets/be781226-fe22-4f49-a8ac-4f7d8a4639c4" />
</p>
<p align="center">
<img width="1362" height="623" alt="Image" src="https://github.com/user-attachments/assets/d31949f5-1e2c-4e6e-9765-9aed3c464237" />
</p>
---

## 주요 태스크

현재 워크스페이스가 다루는 태스크는 아래 흐름으로 나뉩니다.

| 태스크 | 설명 | 대표 파일/진입점 |
|--------|------|------------------|
| 환경 준비 | ROS 2 Humble, Doosan/MoveIt, RealSense, 로컬 모델, install workspace 준비 | `tools/setup/`, `COMMANDS.md` |
| 로봇/카메라 연결 | Doosan, RG2, RealSense, joint relay, TF, ROS_DOMAIN 설정 | `tools/run/start_azas_tmux_stack.sh`, `tools/run/run_connected_robot_control.sh` |
| 컵 탐지 | RealSense RGB-D에서 YOLO로 컵 bbox를 찾고 depth로 3D 후보 생성 | `azas_perception/yolo_tumbler_detector_node.py` |
| 컵 pose 변환 | camera frame 후보를 `base_link` 기준 pose로 변환 | `azas_perception/cup_detection_pose_bridge_node.py` |
| 컵 상태 분기 | 직립/누운 컵 판단 후 side grip 또는 upright 경로 선택 | `auto_cup_flow_router.py`, `azas_cup_uprighting/` |
| 컵 파지 | RG2 open, side-grip 접근, soft close, lift | `tools/run/run_connected_cup_pick_real.sh`, `tools/run/pick_from_measured_dispenser_front_hold.py` |
| 색상 스캔 | 디스펜서 색상/스티커 매핑을 `outputs/dispenser_color_map.json`으로 기록 | `tools/run/run_color_scan_stage.sh` |
| 레시피 선택 | STT/키오스크 입력을 symbolic recipe와 dispenser sequence로 변환 | `azas_voice/`, `azas_kiosk/` |
| 디스펜서 출수 | 컵을 출수구 아래로 이동하고 측정된 press pose에서 펌프질 | `tools/run/run_color_recipe_sequence.py` |
| 컵홀더 배치 | 출수 후 컵을 홀더에 놓고 다음 작업을 위해 pose 정렬 | `tools/run/place_side_grip_cup_in_holder.py` |
| 뚜껑 체결 | 뚜껑 인식, RG2 뚜껑 파지, 컵 위치 이동, J6 twist 체결 | `tools/run/run_kang_lid_grip_close_direct.sh` |
| 쉐이킹 | 체결된 컵 재파지 후 rule-based shake 실행 | `tools/run/run_lid_close_then_shake_chain.sh`, `tools/run/run_rule_based_shake_real.sh` |
| 사람 전달 | 손바닥 검출 후 force-monitored descent로 접촉 확인, 그리퍼 release, 후퇴 | `tools/run/auto_handover_on_palm.py`, `tools/run/handover_cup_to_palm.py` |
| 운영 패널 | 현장 버튼형 실행, 로그 확인, 명령 override, readiness 확인 | `tools/run/open_robot_pipeline_control_panel.sh`, `docs/robot_pipeline_control.html` |
| 검증/스모크 | 패키지, 토픽, 서비스, fake hardware, readiness, RViz preview 확인 | `tools/checks/`, `tools/smoke/` |

---

## 전체 자동 플로우

현재 통합 플로우는 `azas_task_manager.auto_cup_flow_router`가 큰 순서를 관리합니다.

```text
주문 입력
  -> 색상 스캔 / 레시피 준비
  -> 관측 pose 이동
  -> RG2 full-open
  -> 컵 탐지 및 상태 분기
  -> 컵 side-grip 또는 upright 경로
  -> 디스펜서 색상 sequence 실행
  -> 컵홀더 배치
  -> 뚜껑 체결
  -> 컵홀더 재파지
  -> 쉐이킹
  -> 손바닥 감지 기반 핸드오버
```

각 stage는 resume state와 로그를 남깁니다. 실패 시 어느 단계에서 멈췄는지 확인할 수 있도록 `/tmp/azas_ros_logs`, `/tmp/azas_router_logs`, `outputs/` 아래에 실행 산출물이 저장됩니다.

<p align="center">
<img width="2048" height="519" alt="Image" src="https://github.com/user-attachments/assets/6dcbee5a-5c50-43ac-bf6c-70bf0d444814" />
</p>

---

## 핵심 파이프라인

### 1. 컵 인식에서 파지까지

```text
RealSense D435i
  -> /camera/camera/color/image_raw
  -> /camera/camera/aligned_depth_to_color/image_raw
  -> yolo_tumbler_detector_node
  -> /azas/cup_detection
  -> cup_detection_pose_bridge_node
  -> TF2: camera_color_optical_frame -> base_link
  -> /jarvis/tumbler_dispenser/tumbler_pose
  -> side-grip / cup-uprighting / PickAndAlign 경로
```

컵 좌표는 반드시 이 파이프라인 또는 실측된 calibration/config 파일에서만 옵니다. README, 코드, 스크립트에 임의 좌표를 새로 만들지 않습니다.

### 2. 주문에서 디스펜서 출수까지

```text
STT 또는 키오스크
  -> recipe_mapper / conversation_manager
  -> outputs/latest_recipe.json
  -> color map: outputs/dispenser_color_map.json
  -> run_color_recipe_sequence.py
  -> measured dispenser front-hold / press pose
  -> Doosan MoveLine + RG2 service
```

레시피는 색상과 펌프 횟수의 symbolic plan입니다. 디스펜서 좌표와 press 깊이는 `calibration.yaml` 및 측정 스크립트 결과를 사용합니다.

### 3. 뚜껑 체결과 쉐이킹

```text
YOLO/ArUco/sticker 기반 lid pose 후보
  -> RG2로 뚜껑 파지
  -> 컵/컵홀더 측정 pose로 이동
  -> J6 twist로 뚜껑 체결
  -> 컵홀더에서 side-grip 재파지
  -> rule-based shake
```

뚜껑과 쉐이킹 경로는 실제 로봇 pose, RG2 상태, 관절 한계, 작업공간 경계를 확인한 뒤 실행되도록 구성되어 있습니다.

### 4. 사람 손바닥 핸드오버

```text
RealSense color/depth
  -> MediaPipe hand detection
  -> /azas/human_hand_detection
  -> stable palm 후보
  -> 측정된 XY 위치로 먼저 이동
  -> coarse Z descent
  -> fine Z descent + tool force 확인
  -> 외력 접촉 감지
  -> RG2 full-open
  -> retreat
```

핸드오버는 사람 근처 동작이므로 카메라 검출만으로 무조건 release하지 않습니다. stable palm, Z 하강 중 힘 변화, 접촉 확인, 후퇴 단계가 별도로 분리되어 있습니다.

---

## 기술 스택

| 분류 | 사용 기술 |
|------|-----------|
| OS/ROS | Ubuntu, ROS 2 Humble, `rclpy`, `ament_python`, `ament_cmake` |
| 로봇 | Doosan M0609, `dsr_msgs2`, `dsr_bringup2`, `dsr_moveit_config_m0609` |
| 모션 계획 | MoveIt 2, MoveItPy, RViz2, PlanningScene, collision object |
| 시뮬레이션/프리뷰 | RViz preview, Gazebo Classic/ros2_control 경로, `ros_gz_sim` 보조 경로 |
| 그리퍼 | OnRobot RG2, ROS Trigger 서비스, `azas_interfaces/SetGripper`, `python3-pymodbus` |
| 카메라 | Intel RealSense D435i, `realsense2_camera`, aligned depth, `sensor_msgs/Image` |
| 비전/ML | Ultralytics YOLO, OpenCV, `cv_bridge`, NumPy, SciPy, MediaPipe hand tracking |
| 좌표계 | TF2, `geometry_msgs/PoseStamped`, `PointStamped`, camera optical frame, `base_link` |
| 음성/대화 | STT/TTS node, symbolic recipe mapper, YAML recipe catalog |
| UI | 로컬 HTML/JS 패널, Python `http.server`, kiosk web UI |
| 운영/검증 | Bash run scripts, `tools/checks`, `tools/smoke`, staged real robot ladder |
| 설정/데이터 | YAML calibration/safety, local YOLO model, JSON resume/output state |

---

## ROS 패키지 구성

| 패키지 | 역할 |
|--------|------|
| `azas_interfaces` | 공용 메시지, 서비스, 액션 정의 (`CupDetection`, `PickAndAlign`, `SetGripper`) |
| `azas_description` | M0609 + RG2 xacro/URDF overlay |
| `azas_bringup` | 시스템 launch, calibration/safety YAML, RViz 설정 |
| `azas_perception` | YOLO 컵/뚜껑 탐지, depth projection, TF bridge, hand-eye static TF |
| `azas_cup_uprighting` | 누운 컵 직립화 및 YOLO 기반 pick/place 실험 경로 |
| `azas_motion` | MoveItPy 모션, collision scene, dispenser/shake/side-grip preview 노드 |
| `azas_gripper` | RG2 서비스 경계와 open/close/set_width adapter |
| `azas_dispenser` | 디스펜서 press task, press-ready pose 탐색, MoveIt/Doosan press 노드 |
| `azas_task_manager` | 컵 자동 플로우 라우터, PickAndAlign action, resume state |
| `azas_voice` | STT, TTS, conversation manager, recipe mapper, voice pipeline executor |
| `azas_kiosk` | 로컬 키오스크 UI와 symbolic cocktail order bridge |
| `azas_calibration` | hand-eye, RealSense, calibration loader 및 legacy calibration 도구 |
| `dsr_practice` | 교안/legacy 실험 노드와 launch 파일 보관 |

---

## 주요 토픽, 서비스, 액션

| 이름 | 타입 | 역할 |
|------|------|------|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RealSense color 입력 |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | color 정렬 depth 입력 |
| `/azas/cup_detection` | `azas_interfaces/msg/CupDetection` | YOLO + depth 컵 탐지 결과 |
| `/jarvis/tumbler_dispenser/tumbler_pose` | `geometry_msgs/PoseStamped` | `base_link` 기준 컵 pose |
| `/azas/human_hand_detection` | `geometry_msgs/PointStamped` | 손바닥 후보 위치 |
| `/azas/human_hand_detection/status` | JSON string | 손 감지 상태, 안정성, 디버그 정보 |
| `/azas/human_hand_detection/overlay` | `sensor_msgs/Image` | 손 감지 overlay 영상 |
| `/stt_result` | `std_msgs/String` | 음성 인식 결과 입력 |
| `/jarvis/rg2/open` | `std_srvs/srv/Trigger` | RG2 open |
| `/jarvis/rg2/close` | `std_srvs/srv/Trigger` | RG2 close |
| `/jarvis/rg2/set_width` | `azas_interfaces/srv/SetGripper` | RG2 width/force command |
| `/azas/pick_and_align` | `azas_interfaces/action/PickAndAlign` | 컵 pick and align action |
| `/dsr01/motion/move_joint` | `dsr_msgs2/srv/MoveJoint` | Doosan joint motion |
| `/dsr01/motion/move_line` | `dsr_msgs2/srv/MoveLine` | Doosan Cartesian line motion |
| `/dsr01/aux_control/get_tool_force` | Doosan service | 핸드오버 접촉 force 확인 |

---

## 빠른 시작

처음 받은 PC에서는 `install/`을 Git에서 받지 않고 로컬에서 생성합니다.

```bash
cd /home/ssu/Azas
git status --short
bash tools/setup/bootstrap_local_workspace.sh
```

YOLO 모델은 각 PC에서 repo-local 경로로 연결합니다. `best.pt` 파일 자체는 Git에 커밋하지 않습니다.

```bash
bash tools/setup/link_yolo_model.sh /path/to/best.pt
```

패널 실행:

```bash
bash tools/run/open_robot_pipeline_control_panel.sh
```

수동 빌드가 필요할 때만 아래 순서를 사용합니다.

```bash
source /opt/ros/humble/setup.bash
cd /home/ssu/Azas
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/local_setup.bash
```

전체 명령어는 [COMMANDS.md](COMMANDS.md), 협업 규칙은 [CONTRIBUTING.md](CONTRIBUTING.md), 파일 맵은 [docs/repository_file_map.md](docs/repository_file_map.md)를 먼저 봅니다.

---

## 대표 실행 경로

| 목적 | 명령 |
|------|------|
| 통합 제어 패널 | `bash tools/run/open_robot_pipeline_control_panel.sh` |
| 가상 Doosan M0609 | `bash tools/run/run_doosan_virtual_m0609.sh` |
| 연결 상태 점검 | `bash tools/run/check_one_click_cocktail_ready.sh` |
| RealSense 준비 확인 | `bash tools/checks/check_realsense_camera_ready.sh` |
| 컵 탐지 확인 | `bash tools/checks/check_robot_detection.sh` |
| TF 파이프라인 확인 | `bash tools/checks/check_tf_pipeline.sh` |
| 제어 준비도 확인 | `bash tools/checks/verify_control_readiness.sh` |
| one-click 칵테일 | `bash tools/run/run_one_click_cocktail_real.sh` |
| 음성 주문 자동 플로우 | `bash tools/run/run_voice_auto_cup_flow.sh` |
| 색상 레시피 sequence | `python3 tools/run/run_color_recipe_sequence.py --execute --confirm` |
| 뚜껑 체결 후 쉐이킹 | `bash tools/run/run_lid_close_then_shake_chain.sh` |
| 손바닥 감지 | `bash tools/run/run_human_hand_detection.sh` |
| 자동 핸드오버 | `python3 tools/run/auto_handover_on_palm.py` |

실제 로봇을 움직이는 명령은 각 스크립트의 확인 플래그와 현장 gate를 요구합니다. 자세한 단계형 절차는 [docs/real_robot_test_ladder.md](docs/real_robot_test_ladder.md)와 [docs/field_control_runbook.md](docs/field_control_runbook.md)를 따릅니다.

---

## 안전 정책

1. 컵, 디스펜서, 뚜껑, 손 위치는 임의 생성하지 않습니다. 카메라 파이프라인, 실측 YAML, 현장 티칭 결과만 사용합니다.
2. `calibration.yaml`에서 `null` 또는 `확인 필요`인 값은 실측 전 수정하지 않습니다.
3. 실제 모션은 `--enable-real-motion`, 확인 문구, strict gate, 로봇 상태 확인을 통과한 경로만 사용합니다.
4. MoveIt 계획 실패를 Doosan 직접 명령으로 몰래 폴백하지 않습니다.
5. `tools/checks/`는 기본적으로 서비스 존재, 타입, 토픽, 설정을 확인하는 검증 계층입니다. 실제 motion/RG2 actuation과 분리합니다.
6. 사람 핸드오버는 손 검출만으로 release하지 않고, 안정 손바닥 후보와 tool force 접촉 조건을 함께 확인합니다.
7. `build/`, `install/`, `log/`, `.omx/`, `.agents/`, `.codex/`, 모델 weight, 런타임 output은 커밋하지 않습니다.

---

## 모델과 데이터

| 경로 | 설명 |
|------|------|
| `local_models/best.pt` | 현장 YOLO 모델 symlink 또는 로컬 파일 위치 |
| `src/azas_perception/config/yolo_cup_uprighting_best.pt` | cup-uprighting 전용 모델 |
| `src/azas_bringup/config/calibration.yaml` | 실측 pose, hand-eye, 디스펜서, 컵홀더, 안전 관련 값 |
| `src/azas_bringup/config/safety.yaml` | 작업공간, 속도, 충돌, gate 관련 설정 |
| `outputs/latest_recipe.json` | 음성/키오스크에서 선택된 최신 recipe |
| `outputs/dispenser_color_map.json` | 색상 스캔 결과 |
| `outputs/measured_dispenser_recipe_resume.json` | 디스펜서 sequence resume state |

모델 weight와 runtime output은 환경별 산출물이므로 Git에 올리지 않습니다.

---

## 문서 지도

| 문서 | 내용 |
|------|------|
| [COMMANDS.md](COMMANDS.md) | 전체 명령어 빠른 참조 |
| [docs/repository_file_map.md](docs/repository_file_map.md) | 파일/폴더 역할과 운영/검증/실험 구분 |
| [docs/collaboration_edit_map.md](docs/collaboration_edit_map.md) | 목적별 수정 위치 |
| [docs/real_robot_test_ladder.md](docs/real_robot_test_ladder.md) | 실제 로봇 단계형 테스트 절차 |
| [docs/field_control_runbook.md](docs/field_control_runbook.md) | 현장 터미널별 운용 절차 |
| [docs/safety_checklist.md](docs/safety_checklist.md) | 실제 로봇 안전 체크리스트 |
| [docs/tf_debug_checklist.md](docs/tf_debug_checklist.md) | TF/카메라/컵 pose 디버그 |
| [docs/lid_gripper_pipeline.md](docs/lid_gripper_pipeline.md) | 뚜껑 인식/체결 파이프라인 |
| [docs/post_shake_human_handover_plan.md](docs/post_shake_human_handover_plan.md) | 쉐이킹 후 사람 전달 계획 |
| [docs/full_cocktail_workflow_plan.md](docs/full_cocktail_workflow_plan.md) | 전체 칵테일 워크플로우 |
| [docs/recovery_after_poweroff.md](docs/recovery_after_poweroff.md) | 전원 차단 후 복구 절차 |

---

## 개발 원칙

- 새 기능은 `src/` 패키지에 구현하고, 현장 실행은 `tools/run/`, 검증은 `tools/checks/` 또는 `tools/smoke/`에 둡니다.
- 좌표와 보정값은 코드 상수보다 calibration/config 파일, 측정 스크립트, perception topic을 우선합니다.
- 하드웨어에 영향을 주는 변경은 안전 가정, 속도 제한, 실패 동작, 검증 절차를 함께 문서화합니다.
- 패널 버튼이나 one-click 스크립트는 여러 하드웨어 경로를 동시에 섞지 않도록 fail-closed 방식으로 작성합니다.
- 신규 협업자는 [docs/onboarding/01-quickstart-kor.md](docs/onboarding/01-quickstart-kor.md)와 [docs/onboarding/02-role-map-kor.md](docs/onboarding/02-role-map-kor.md)를 먼저 읽습니다.
