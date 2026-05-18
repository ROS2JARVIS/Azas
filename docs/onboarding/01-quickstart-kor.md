# 신규 협업자 빠른 시작

이 문서는 Azas 레포를 처음 보는 한국인 개발자가 어디부터 보면 되는지 정리한 입문 경로입니다.

## 프로젝트 한 줄 요약

Azas는 Doosan M0609, OnRobot RG2, RealSense D435i를 사용하는 ROS 2 Humble 기반 칵테일 로봇 프로젝트입니다.

현재 안전한 기본 목표는 실제 모션 없이 다음 흐름을 검증하는 것입니다.

```text
음성/STT 명령
  -> 룰베이스 레시피 결정
  -> 컵 탐지/자세 변환
  -> 픽앤얼라인 태스크 플랜
  -> 드라이런 또는 게이트 통과 후 실제 실행
```

## 처음 실행 순서

```bash
source /opt/ros/humble/setup.bash
cd /home/ssu/Azas
colcon build --symlink-install
source install/setup.bash
```

하드웨어 없이 먼저 확인할 명령은 아래입니다.

```bash
bash tools/checks/verify_control_readiness.sh
bash tools/smoke/smoke_cocktail_dryrun_sequence.sh
```

전체 명령은 루트의 `COMMANDS.md`를 봅니다.

## 룰베이스 담당자가 먼저 볼 파일

| 목적 | 파일 |
|------|------|
| 한국어 명령 파싱 규칙 | `src/azas_voice/azas_voice/command_parser.py` |
| 레시피/색상 별칭 데이터 | `src/azas_voice/azas_voice/recipe_catalog.py` |
| 레시피 설정 참고 | `src/azas_voice/config/recipes.yaml` |
| 룰베이스 노드 출력 | `src/azas_voice/azas_voice/recipe_mapper_node.py` |
| 룰베이스 결과를 태스크 플랜으로 변환 | `src/azas_task_manager/azas_task_manager/cocktail_dryrun_sequence_node.py` |
| 칵테일 단계 정의 | `src/azas_task_manager/azas_task_manager/cocktail_workflow_plan.py` |
| 룰베이스 테스트 | `src/azas_voice/test/test_command_parser.py` |

룰베이스 담당자의 출력 계약은 `/azas/voice/recipe_decision`입니다. 이 토픽은 레시피 ID와 디스펜서 ID 같은 상징 정보만 전달해야 합니다.

## 절대 하지 말아야 할 것

- 컵 좌표를 코드에 직접 적지 않습니다.
- 사람에게 컵 좌표를 물어보지 않습니다.
- LLM이나 룰베이스가 로봇 좌표, 궤적, 충돌 판단, 캘리브레이션 값을 만들지 않습니다.
- `src/azas_bringup/config/calibration.yaml`의 `null` 또는 `확인 필요` 값을 실측 없이 채우지 않습니다.
- 실제 모션 관련 코드는 안전 게이트와 실패 동작을 문서화하지 않은 채 수정하지 않습니다.

컵 자세는 항상 아래 파이프라인에서 옵니다.

```text
/azas/cup_detection
  -> cup_detection_pose_bridge_node
  -> /jarvis/tumbler_dispenser/tumbler_pose
```

## 개발 전 확인

```bash
git status --short
python3 -m py_compile \
  src/azas_voice/azas_voice/command_parser.py \
  src/azas_voice/azas_voice/recipe_catalog.py \
  src/azas_task_manager/azas_task_manager/cocktail_workflow_plan.py \
  src/azas_task_manager/azas_task_manager/cocktail_dryrun_sequence_node.py
```

`pytest`가 설치되어 있으면 아래 테스트를 추가로 실행합니다.

```bash
pytest src/azas_voice/test/test_command_parser.py
```
