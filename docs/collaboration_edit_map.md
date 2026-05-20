# 협업 수정 위치 맵

팀원이 “어디를 고쳐야 하지?”에서 막히지 않도록, 변경 목적별 수정 위치를 고정합니다.

## 기본 원칙

- `src/`는 ROS 2 패키지 소스입니다. 노드, 메시지, launch, config처럼 실제 기능을 바꿀 때 수정합니다.
- `tools/`는 사람이 실행하는 보조 명령입니다. 패널, 점검, 스모크, 현장 실행 스크립트를 바꿀 때 수정합니다.
- `docs/`는 절차와 판단 근거입니다. 팀원이 따라 할 명령, 안전 절차, 운영 기준을 바꿀 때 수정합니다.
- `build/`, `install/`, `log/`는 각 PC에서 생기는 산출물입니다. 수정하거나 커밋하지 않습니다.

## 목적별 수정 위치

| 하고 싶은 일 | 주로 수정할 곳 | 같이 확인할 곳 | PR 범위 |
| --- | --- | --- | --- |
| 한국어 명령/레시피 단어 추가 | `src/azas_voice/azas_voice/command_parser.py`, `src/azas_voice/config/recipes.yaml` | `src/azas_voice/test/test_command_parser.py` | voice 단독 |
| 레시피 단계/작업 순서 변경 | `src/azas_task_manager/azas_task_manager/cocktail_workflow_plan.py` | `tools/smoke/smoke_cocktail_dryrun_sequence.sh` | task_manager 단독 |
| 컵 탐지 YOLO 로직 변경 | `src/azas_perception/azas_perception/yolo_tumbler_detector_node.py` | `tools/checks/check_robot_detection.sh` | perception 단독 |
| 컵 pose 토픽/TF 변환 변경 | `src/azas_perception/azas_perception/cup_detection_pose_bridge_node.py` | `tools/checks/check_tf_pipeline.sh` | perception 단독 |
| YOLO 모델 경로/정책 통일 | `src/azas_bringup/config/`, `src/azas_bringup/launch/yolo_*.launch.py` | `docs/`, `COMMANDS.md` | 별도 perception/config PR |
| 모션 계획 알고리즘 변경 | `src/azas_motion/` | `tools/checks/check_side_grasp_planning_only.sh` | motion 단독, 안전 리뷰 |
| 실제 Doosan/RG2 실행 경로 변경 | `tools/run/`, 관련 ROS 노드 | `docs/safety_checklist.md`, `docs/field_control_runbook.md` | integration 또는 motion/gripper 분리 |
| 제어 패널 버튼/HTML 변경 | `tools/run/robot_pipeline_control_server.py`, `docs/robot_pipeline_control.html` | `tools/checks/check_panel_*.py` | panel/tooling 단독 |
| 빌드/처음 실행 문제 해결 | `tools/setup/`, `README.md`, `COMMANDS.md` | `docs/onboarding/01-quickstart-kor.md` | tooling/docs 단독 |
| 문서만 수정 | `README.md`, `COMMANDS.md`, `docs/` | 해당 명령이 실제 존재하는지 확인 | docs 단독 |

## `src/`와 `tools/` 차이

### `src/`를 고치는 경우

ROS graph 안에서 돌아가는 기능을 바꿀 때입니다.

예:

- 토픽 publish/subscribe 변경
- 메시지/서비스/action 계약 변경
- launch 파라미터 변경
- perception, task, motion, gripper 노드 동작 변경

### `tools/`를 고치는 경우

사람이 터미널이나 패널에서 실행하는 절차를 바꿀 때입니다.

예:

- 패널 실행 스크립트
- 실제 로봇 연결 스크립트
- smoke/check 명령
- fake hardware 테스트
- 직접교시 기록 helper

`tools/`가 ROS 노드를 실행할 수는 있지만, 주 역할은 “운영 진입점”입니다.

## 패키지별 책임

| 패키지 | 책임 | 건드리면 안 되는 것 |
| --- | --- | --- |
| `azas_interfaces` | 메시지/서비스/action 계약 | 이유 없이 필드 변경 금지. 바꾸면 관련 패키지 전체 수정 필요 |
| `azas_voice` | 사용자 말 → 레시피/디스펜서 같은 상징 정보 | 좌표, 자세, 궤적 생성 금지 |
| `azas_perception` | 카메라/YOLO/depth/TF로 컵 pose 생성 | 사람이 입력한 컵 좌표 사용 금지 |
| `azas_task_manager` | 레시피와 컵 pose를 작업 단계로 묶음 | 안전 gate 우회 금지 |
| `azas_motion` | 계획/정렬/충돌 고려 | 실측 없는 좌표/캘리브레이션 생성 금지 |
| `azas_gripper` | 그리퍼 경계 | 실제 RG2 경로와 placeholder 혼동 금지 |
| `azas_bringup` | launch/config 조합 | 실측 전 `null`/`확인 필요` 임의 수정 금지 |
| `azas_calibration` | 캘리브레이션 로드/저장 경계 | LLM/추정값으로 캘리브레이션 작성 금지 |

## PR을 섞지 않는 기준

하나의 PR에는 되도록 한 종류만 넣습니다.

- perception 변경 PR에 패널 HTML 정리 섞지 않기
- motion 변경 PR에 YOLO 모델 정책 섞지 않기
- docs PR에 실제 로봇 실행 로직 섞지 않기
- bootstrap/build PR에 디스펜서 자세 변경 섞지 않기

섞어야 한다면 PR 설명에 “왜 한 PR이어야 하는지”를 적습니다.

## 팀원이 자주 헷갈리는 질문

### install은 어디에 두나요?

직접 만들지 않습니다. `/home/ssu/Azas`에서 빌드하면 자동 생성됩니다.

```bash
cd /home/ssu/Azas
bash tools/setup/bootstrap_local_workspace.sh
```

생성 위치:

```text
/home/ssu/Azas/install/
```

### 컵 좌표가 필요하면 어디를 수정하나요?

좌표를 직접 쓰지 않습니다. 아래 topic을 구독합니다.

```text
/jarvis/tumbler_dispenser/tumbler_pose
```

### 패널 버튼을 추가하려면 어디를 보나요?

우선 여기입니다.

```text
tools/run/robot_pipeline_control_server.py
docs/robot_pipeline_control.html
```

### YOLO 모델을 통일하려면 어디서 시작하나요?

별도 PR로 진행합니다. 먼저 현재 launch/config에서 모델 경로가 어디에 흩어져 있는지 확인합니다.

```bash
grep -R "model_path\|best.pt\|YOLO" -n src tools docs README.md COMMANDS.md
```
