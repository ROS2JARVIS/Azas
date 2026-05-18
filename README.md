# Azas

> **Doosan M0609 + OnRobot RG2 + RealSense D435i 기반 칵테일 로봇 (ROS 2 Humble)**  
> 7인 협업 프로젝트 | MVP-1: 텀블러 감지 → RG2 픽업 → 디스펜서 정렬

---

## 빠른 시작

```bash
source /opt/ros/humble/setup.bash
cd /home/ssu/Azas
colcon build --symlink-install
source install/setup.bash
```

전체 명령어 → **[COMMANDS.md](COMMANDS.md)** | 협업 가이드 → **[CONTRIBUTING.md](CONTRIBUTING.md)**

처음 읽는 개발자는 먼저 **[docs/repository_file_map.md](docs/repository_file_map.md)** 를 보세요.
각 폴더·파일의 역할, 운영/검증/실험 구분, 실제 로봇이 움직이지 않는 이유가 정리되어 있습니다.

실제 로봇 테스트는 **[docs/real_robot_test_ladder.md](docs/real_robot_test_ladder.md)** 의 staged ladder를 따릅니다.

---

## 처음 온 팀원은 여기부터

| 목적 | 문서 |
|------|------|
| 처음 빌드하고 드라이런까지 확인 | `docs/onboarding/01-quickstart-kor.md` |
| 패키지별 담당 영역 이해 | `docs/onboarding/02-role-map-kor.md` |
| `tools/` 명령어 분류 이해 | `docs/reference/command-taxonomy.md` |

룰베이스 담당자는 먼저 `src/azas_voice/azas_voice/command_parser.py`,
`src/azas_voice/azas_voice/recipe_catalog.py`,
`src/azas_voice/config/recipes.yaml`을 봅니다.

---

## MVP-1 목표

```
랜덤 위치 텀블러 감지
  → RG2 사이드 그라스프
  → cup_mouth_center를 dispenser_outlet 아래로 정렬
```

STT/LLM 은 사용자 의도·레시피 선택만 담당합니다.  
**로봇 좌표, 궤적, 충돌 판단, 캘리브레이션 값을 절대 생성하지 않습니다.**

전원 차단 복구 절차: `docs/recovery_after_poweroff.md`

---

## 패키지 구성

| 패키지 | 역할 |
|--------|------|
| `azas_interfaces` | 공용 메시지, 서비스, `PickAndAlign` 액션 정의 |
| `azas_perception` | YOLO 탐지, 깊이 투영, 컵 자세 브릿지 |
| `azas_calibration` | 카메라-베이스 TF, 디스펜서·컵 오프셋 캘리브레이션 |
| `azas_motion` | MoveItPy 모션 플래닝 (현재 no_motion 기본) |
| `azas_gripper` | RG2 서비스 경계 (플레이스홀더) |
| `azas_task_manager` | `/azas/pick_and_align` 액션 서버, 칵테일 시퀀스 |
| `azas_bringup` | 시스템 런치 파일, YAML 설정 |
| `azas_voice` | STT → 레시피 매핑 (`/stt_result` 입력) |

---

## 탐지 파이프라인

```
YOLO 모델 (best.pt)
  → yolo_tumbler_detector_node      직립 텀블러만 통과 (bbox 비율 >= 1.2)
  → /azas/cup_detection
  → cup_detection_pose_bridge_node  TF2로 base_link 변환
  → /jarvis/tumbler_dispenser/tumbler_pose
  → PickAndAlignActionServer         그라스프 계획 및 실행
```

컵 좌표는 반드시 이 파이프라인에서 옵니다. 하드코딩하거나 AI 에이전트에게 요청하지 마세요.

---

## 픽앤얼라인 실행 순서

```
HOME
→ OBSERVE_CUP_POSE   (j1=0, j2=25, j3=65, j4=0, j5=135, j6=0)
→ DETECT_CUP
→ COMPUTE_SIDE_GRASP
→ PLAN_SIDE_GRASP
→ GRIPPER_OPEN
→ MOVE_APPROACH
→ MOVE_GRASP
→ GRIPPER_CLOSE
→ LIFT
→ DONE
```

기본값은 `execution_mode=no_motion` — 실제 모션 명령 없이 계획만 수행합니다.

---

## 하드웨어 값 정책

`src/azas_bringup/config/calibration.yaml` 에서 `null` 또는 `확인 필요` 로 표시된 항목은  
**실측 완료 전 절대 수정하지 않습니다.**

미확정 항목: `EE_LINK`, `GROUP_NAME`, 카메라 토픽/프레임, 핸드-아이 변환,  
`dispenser_outlet` 자세, RG2 명령 단위/범위, TCP 오프셋, 컵 치수, 작업 공간 경계

---

## 현장 투입 순서

```bash
# ① 가상 Doosan 시작
bash tools/run/run_doosan_virtual_m0609.sh

# ② 드라이런 검증
bash tools/run/run_robot_dryrun.sh

# ③ 엄격 게이트 통과 (실제 모션 전 필수)
STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed \
  bash tools/checks/check_live_hardware_gates.sh

# ④ 실제 로봇 모션
bash tools/run/run_robot_real.sh

# 컵 pick만 먼저 검증하는 단일 진입점
bash tools/run/run_connected_cup_pick_real.sh

# ⑤ 컵을 선택 출수구 아래에 놓고 디스펜서 프레스
bash tools/run/run_cup_to_dispenser_press_real.sh
```

---

## 그리퍼 서비스 현황

| 서비스 | 타입 | 상태 |
|--------|------|------|
| `/jarvis/rg2/open` | `std_srvs/Trigger` | 드라이런: 가짜 서비스 / 실제: 미검증 |
| `/jarvis/rg2/close` | `std_srvs/Trigger` | 드라이런: 가짜 서비스 / 실제: 미검증 |
| `/jarvis/rg2/set_width` | `azas_interfaces/SetGripper` | 드라이런만 가능, 실제 RG2 미연결 |
| `/azas/gripper/open_close` | `azas_interfaces/SetGripper` | Azas 내부 플레이스홀더, 실제 RG2 아님 |

---

## YOLO 탐지 상세

- 대상 클래스: `cup`, `tumbler`, `bottle`
- 선택 정책: bbox 면적이 가장 큰 객체
- 직립 판단: `bbox_height / bbox_width >= 1.2` → 직립, `< 0.8` → 거부
- 깊이: 중심 7×7 픽셀 중앙값
- 깊이 스케일: `16UC1`/`mono16` → 0.001 m/mm, `32FC1` → 1.0 m
- 거부 조건: 0, NaN, inf, 0.15 m 미만, 2.0 m 초과

---

## TF 디버그

```bash
# 가상 Doosan 시작
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
  model:=m0609 mode:=virtual host:=127.0.0.1 port:=12345

# TF 확인
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 run tf2_tools view_frames
ros2 topic echo /jarvis/tumbler_dispenser/tumbler_pose
```

TF 체크리스트: `docs/tf_debug_checklist.md`

---

## 비-하드웨어 점검

```bash
bash tools/checks/check_oss_stack.sh           # 패키지·런치·의존성 전체 점검
bash tools/checks/verify_control_readiness.sh  # 제어 준비도 종합

bash tools/smoke/smoke_pick_and_align_no_motion.sh  # 액션 스모크
bash tools/smoke/smoke_control_path.sh              # 제어 경로 엔드투엔드
bash tools/smoke/smoke_fake_hardware_path.sh        # 가짜 하드웨어 스모크
bash tools/smoke/smoke_cup_to_dispenser_press_path.sh # 가짜 디스펜서 프레스
bash tools/smoke/smoke_cocktail_dryrun_sequence.sh  # 칵테일 시퀀스 스모크
```

---

## 참고 문서

| 문서 | 내용 |
|------|------|
| `COMMANDS.md` | 전체 명령어 빠른 참조 |
| `CONTRIBUTING.md` | 브랜치 전략, 역할 분담, 커밋 컨벤션 |
| `AGENTS.md` | AI 에이전트 규칙 (컵 좌표 파이프라인 포함) |
| `docs/safety_checklist.md` | 실제 로봇 운용 안전 체크리스트 |
| `docs/field_control_runbook.md` | 현장 터미널별 절차 |
| `docs/simulation_and_connection_plan.md` | 시뮬·카메라·로봇 연결 판단 기준 |
| `docs/tf_debug_checklist.md` | 카메라-베이스 TF 디버그 체크리스트 |
| `docs/full_cocktail_workflow_plan.md` | 전체 칵테일 워크플로우 마일스톤 |
| `docs/recovery_after_poweroff.md` | 전원 차단 후 복구 절차 |
| `docs/onboarding/01-quickstart-kor.md` | 신규 협업자 빠른 시작 |
| `docs/onboarding/02-role-map-kor.md` | 역할별 패키지 맵 |
| `docs/reference/command-taxonomy.md` | 명령어와 도구 분류 기준 |
