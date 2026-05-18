# Real Robot Test Ladder

Updated: 2026-05-14

이 문서는 Azas를 실제 동작으로 가져가기 위한 현장 테스트 순서입니다. 핵심 원칙은 한 번에 full cocktail을 실행하지 않고, 실패 지점을 좁히는 작은 단계만 통과시키는 것입니다.

## 목표

1. 코드가 빌드/테스트되는지 확인한다.
2. 카메라, depth, CameraInfo, cup detection, TF, tumbler pose topic이 살아 있는지 확인한다.
3. Doosan/RG2 서비스가 존재하고 타입이 맞는지 확인한다.
4. `calibration.yaml`과 `safety.yaml`에 실측값이 들어가 gate를 통과하는지 확인한다.
5. observe-only, planning-only pick을 먼저 돌린다.
6. 마지막에만 supervised one-shot real pick을 실행한다.
7. pick/place가 통과한 뒤에만 cup-to-dispenser press entrypoint를 별도로 실행한다.

## 단일 진입점

```bash
cd /home/ssu/Azas
STAGE=status tools/run/run_real_robot_test_ladder.sh
```

사용 가능한 stage:

| Stage | 실제 모션 | 설명 |
| --- | --- | --- |
| `status` | 없음 | 현재 real robot 차단 사유를 설명한다. |
| `no-hardware` | 없음 | repository no-hardware verifier를 실행한다. |
| `field` | 없음 | field no-motion report를 실행한다. gate stamp를 쓰지 않는다. |
| `live-gate` | 없음 | strict live gate를 실행한다. 통과할 때만 `/tmp/azas_live_hardware_gates_passed`를 쓴다. |
| `observe-dry` | 없음 | supervised observe-only flow를 dry-run으로 실행한다. |
| `pick-dry` | 없음 | live pose 기반 one-shot pick planning flow를 dry-run으로 실행한다. |
| `pick-real` | 가능 | 명시 확인 문구와 fresh strict gate가 있을 때만 one-shot real pick을 실행한다. |

## 권장 순서

```bash
# 1. 지금 왜 막히는지 확인
STAGE=status tools/run/run_real_robot_test_ladder.sh

# 2. 하드웨어 없이 코드/스모크 확인
STAGE=no-hardware tools/run/run_real_robot_test_ladder.sh

# 3. 카메라/탐지/서비스/config 상태 보고서
STAGE=field RUN_LID_STABILITY=true RUN_CUP_STABILITY=true \
  tools/run/run_real_robot_test_ladder.sh

# 4. strict live gate. 통과해야 gate stamp가 생긴다.
STAGE=live-gate RUN_LID_STABILITY=true RUN_CUP_STABILITY=true \
  tools/run/run_real_robot_test_ladder.sh

# 5. 관측 자세까지만 dry-run
STAGE=observe-dry tools/run/run_real_robot_test_ladder.sh

# 6. pick 후보 계획까지만 dry-run
STAGE=pick-dry tools/run/run_real_robot_test_ladder.sh

# 7. 마지막 one-shot 실제 pick
STAGE=pick-real CONFIRM=I_UNDERSTAND_THIS_WILL_MOVE_THE_ROBOT \
  tools/run/run_real_robot_test_ladder.sh

# 8. 컵을 선택 출수구 아래에 놓고 디스펜서만 프레스
tools/run/run_cup_to_dispenser_press_real.sh
```

## `pick-real` 전 필수 조건

- `tools/checks/explain_real_robot_blockers.sh`가 blockers 없이 끝나야 한다.
- `STAGE=live-gate`가 fresh strict gate stamp를 써야 한다.
- `calibration.yaml`의 모든 `null`/`확인 필요` 값이 실측값으로 바뀌어야 한다.
- `check_measured_dispenser_geometry.py`가 통과해야 한다. 실측 outlet/press 좌표가 현재 launch geometry와 다르면 real motion config gate가 실패한다.
- `safety.yaml`의 workspace, min Z, RG2 width/force가 실측/운영 제한값으로 바뀌어야 한다.
- `/jarvis/tumbler_dispenser/tumbler_pose`는 fake publisher가 아니라 live camera detection과 measured TF에서 나와야 한다.
- `/execute_trajectory`, `/plan_kinematic_path`, `/move_group`, Doosan controller, RG2 services가 살아 있어야 한다.
- e-stop, 작업공간, 케이블, 카메라 마운트, 테이블, 컵, 디스펜서 충돌 위험을 현장에서 확인해야 한다.

## 왜 바로 full cocktail을 실행하지 않는가

현재 프로젝트는 full workflow plan과 dry-run/fake smoke를 갖고 있지만, 실제 로봇에서 검증되지 않은 영역이 남아 있습니다.

- hand-eye/base-camera TF 실측
- TCP와 cup mouth offset 실측
- 디스펜서별 outlet/press pose 실측
- RG2 width/force 단위와 실패 동작 검증
- MoveIt planning 결과와 실제 controller execution 검증
- collision scene/workspace bounds 검증

따라서 첫 실제 동작은 full cocktail이 아니라 supervised one-shot cup pick이어야 합니다. 이 단계가 통과하면 `run_cup_to_dispenser_press_real.sh`로 cup placement와 dispenser press를 별도 검증하고, 그 다음에 lid, shake, pour를 각각 같은 방식으로 독립 gate를 만들고 연결합니다.

## 오픈소스/논문 사용 기준

외부 오픈소스나 논문은 “실제 모션을 바로 가능하게 만드는 마법”이 아니라, 특정 빈칸을 채울 때만 도입합니다.

| 필요 | 후보 |
| --- | --- |
| hand-eye calibration | AprilTag, easy_handeye2, MoveIt calibration workflow |
| grasp pose 품질 향상 | GPD, AnyGrasp, GraspNet 계열 논문/구현 |
| collision/workspace 검증 | MoveIt PlanningScene, OctoMap, depth camera point cloud |
| perception robustness | YOLO dataset 재학습, static dataset gate, live stability gate |

도입 전에는 항상 `docs/grasp_detector_adapter.md`, `docs/anygrasp_integration.md`, `dependencies/*.repos`처럼 경계를 문서화하고, 실제 로봇 연결 전 no-motion adapter contract test를 먼저 만듭니다.
