# 역할별 패키지 맵

이 문서는 협업자가 자기 담당 영역과 건드리면 안 되는 경계를 빠르게 찾기 위한 지도입니다.

## 두 개의 워크스페이스 구조

Azas 프로젝트는 두 개의 ROS 2 워크스페이스를 함께 사용합니다.

```
/home/ssu/Azas/          ← Azas 메인 패키지 (이 레포)
  src/azas_*/            ← 비전, 모션, 태스크, 음성 등

/home/ssu/ros2_ws/       ← 외부 패키지 (별도 레포)
  src/Azas/jarvis/       ← 실제 RG2 그리퍼 + 디스펜서 제어
  src/doosan-robot2/     ← 두산 M0609 ROS 2 드라이버
```

매 터미널에서 두 워크스페이스를 모두 소싱해야 합니다:

```bash
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
```

## 패키지별 책임

### Azas 레포 (`/home/ssu/Azas/src/`)

| 패키지 | 담당 영역 | 주요 파일/경계 | 상태 |
|--------|----------|----------------|------|
| `azas_interfaces` | 공용 메시지, 서비스, 액션 계약 | `msg/`, `srv/`, `action/PickAndAlign.action` | 구현됨 |
| `azas_voice` | STT 텍스트를 상징적 레시피 결정으로 변환 | `command_parser.py`, `recipe_catalog.py`, `recipe_mapper_node.py` | 구현됨 |
| `azas_task_manager` | 레시피 결정과 컵 탐지를 태스크 단계로 조합 | `cocktail_dryrun_sequence_node.py`, `pick_and_align_action_server.py` | 구현됨 (no-motion) |
| `azas_perception` | 직립 컵 탐지, 깊이 투영, base_link 기준 컵 자세 발행 | `yolo_tumbler_detector_node.py`, `cup_detection_pose_bridge_node.py` | 구현됨 (`detected:upright`만 pose 발행) |
| `azas_motion` | 그라스프 계획 계산 | `alignment.py`, `alignment_executor_node.py` | 계획만, 실행 없음 |
| `azas_gripper` | **내부 placeholder** — 실제 RG2 아님 | `rg2_gripper_node.py` | **미연결** (아래 jarvis 사용) |
| `azas_calibration` | 실측 캘리브레이션 값 로드/저장 경계 | `calibration_loader_node.py`, `calibration.yaml` | 실측 대기 중 |
| `azas_bringup` | 런치 파일과 시스템 설정 조합 | `launch/`, `config/` | 구현됨 |

### ros2_ws (`/home/ssu/ros2_ws/src/`)

| 패키지 | 담당 영역 | 서비스/토픽 | 상태 |
|--------|----------|------------|------|
| `jarvis` (rg2_trigger_node) | **실제 RG2 그리퍼** Modbus 제어 | `/jarvis/rg2/open`, `/jarvis/rg2/close` | 구현됨, IP 연결 필요 |
| `jarvis` (tumbler_floor_place_node) | 컵을 디스펜서 아래로 이동 | — | 구현됨, 캘리브레이션 필요 |
| `dsr_bringup2` | 두산 M0609 MoveIt 드라이버 | `/dsr01/motion/move_line` 등 | 구현됨, 로봇 IP 필요 |

## RG2 실제 연결 경로

```
supervised real-runner scripts / jarvis floor-place path
  ↓  explicit real-motion confirmation + strict gates 통과 시
/jarvis/rg2/open   ← jarvis/rg2_trigger_node  ← Modbus 192.168.1.1
/jarvis/rg2/close
```

`azas_gripper/rg2_gripper_node`는 내부 플레이스홀더로, 실제 RG2를 제어하지 않습니다.
`pick_and_align_action_server`의 `execution_mode=no_motion`은 `enable_gripper_service_calls=true`가 들어와도 실제 RG2 서비스를 호출하지 않고 실패해야 합니다.
no-motion 스모크와 readiness/check 명령은 RG2 서비스의 존재나 타입을 볼 수는 있지만, `/jarvis/rg2/open` 또는 `/jarvis/rg2/close`를 호출했다는 뜻이 아니며 실제 그리퍼 동작 증거도 아닙니다.

## 컵 탐지 계약

모션으로 이어지는 컵 포즈는 `/azas/cup_detection`의 status가 `detected:upright`로 시작할 때만 `/jarvis/tumbler_dispenser/tumbler_pose`로 변환됩니다. `detected:lid`, `rejected:*`, 애매한 탐지는 perception 상태 확인에는 쓸 수 있지만 컵 pose 계약을 만족하지 않습니다.

## 룰베이스 담당 범위

룰베이스 담당자는 기본적으로 `azas_voice` 안에서 작업합니다.

안전한 수정 범위:

- 한국어 명령 alias 추가
- 레시피 ID alias 추가
- 색상/디스펜서 ID alias 추가
- 취소/확인 단어 보강
- `test_command_parser.py` 테스트 추가

주의가 필요한 수정 범위:

- `/azas/voice/recipe_decision` JSON 필드 변경
- `cocktail_dryrun_sequence_node.py`의 blocked 조건 변경
- `cocktail_workflow_plan.py` 단계 순서 변경

금지 범위:

- 컵 좌표 생성
- 로봇 TCP 좌표 생성
- 캘리브레이션 값 생성
- 모션 게이트 우회

## 룰베이스 데이터 흐름

```text
/stt_result
  -> azas_voice.recipe_mapper_node
  -> azas_voice.command_parser.parse_recipe_command()
  -> /azas/voice/recipe_decision
  -> azas_task_manager.cocktail_dryrun_sequence_node
  -> /azas/cocktail/task_plan
  -> /azas/cocktail/status
```

룰베이스는 “무슨 칵테일을 만들지”와 “어떤 디스펜서를 사용할지”까지만 결정합니다. 실제 컵 위치, 로봇 자세, 그리퍼 동작은 다른 패키지의 안전 경계를 통과해야 합니다.

## PR 분리 기준

| 변경 내용 | 권장 PR 범위 |
|----------|--------------|
| 한국어 명령/레시피 alias | `azas_voice` 단독 |
| 태스크 단계 추가 | `azas_task_manager` 단독 |
| 컵 탐지/pose 발행 변경 | `azas_perception` 단독 |
| 모션 계획 변경 | `azas_motion` 단독, 안전 리뷰 필요 |
| 캘리브레이션 값 변경 | 실측 근거 문서 포함, 안전 리뷰 필요 |
| 런치/설정 변경 | `azas_bringup` 단독 또는 관련 패키지와 분리 |

## 리뷰 때 확인할 질문

- 이 변경이 좌표나 캘리브레이션 값을 새로 만들고 있지 않은가?
- 룰베이스 출력은 여전히 상징 정보만 담고 있는가?
- 실패할 때 blocked 또는 명시적 오류로 멈추는가?
- 하드웨어 영향이 있다면 안전 게이트와 검증 절차가 문서화됐는가?
