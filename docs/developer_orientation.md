# Azas Developer Orientation

이 문서는 신규 개발자가 프로젝트에 들어왔을 때 먼저 봐야 하는 지도입니다.
목표는 모든 파일을 설명하는 것이 아니라, **어디를 수정하고 어떤 순서로 테스트해야 하는지**를 명확히 하는 것입니다.

## 저장소가 둘로 보이는 이유

현재 작업공간은 두 ROS 2 패키지 묶음이 같이 사용됩니다.

| 위치 | 역할 | 신규 개발자가 볼 우선순위 |
| --- | --- | --- |
| `/home/ssu/Azas` | 메인 Azas 프로젝트. voice, perception, task manager, bringup, safety gate, run/check/smoke 도구 | 1순위 |
| `/home/ssu/ros2_ws/src/Azas` | `jarvis` 패키지. RViz/Gazebo 장면, M0609 URDF 표시, 컵 이동/쉐이킹/디스펜서 동작 노드 | 2순위 |

운영 명령은 가능하면 `/home/ssu/Azas/tools/...`에서 실행합니다.  
`jarvis`의 launch/node는 기능 구현체이고, Azas의 `tools/run`이 사용자용 진입점입니다.

## 최상위 책임 경계

| 영역 | 위치 | 책임 | 금지/주의 |
| --- | --- | --- | --- |
| Voice/recipe | `src/azas_voice` | STT 텍스트를 레시피와 디스펜서 ID 순서로 변환 | 좌표/궤적 생성 금지 |
| Perception | `src/azas_perception` | 컵/뚜껑 탐지, depth projection, `base_link` pose bridge | 사람이 컵 좌표를 직접 입력하는 흐름 금지 |
| Task manager | `src/azas_task_manager` | 칵테일 단계 순서와 단계별 gate 정의 | 실제 모션 직접 실행 금지 |
| Bringup | `src/azas_bringup/launch` | 카메라, perception, gripper, robot-control launch 조립 | launch가 많으므로 아래 표의 권장 진입점 우선 |
| Motion helpers | `src/azas_motion`, `tools/pick` | 관측/정렬/픽 관련 실험 도구 | 실제 모션은 gate 없이 실행 금지 |
| Jarvis motion nodes | `/home/ssu/ros2_ws/src/Azas/jarvis` | 컵 이동, 디스펜서, 뚜껑, 쉐이킹 노드 | real/fake/sim 모드 구분 필수 |
| Operator tools | `tools/run` | 현장 실행 명령 | 신규 공개 명령은 여기 추가 |
| Checks | `tools/checks` | 비-모션 점검 | 로봇/RG2 실제 동작 금지 |
| Smoke | `tools/smoke` | fake hardware 자동 검증 | 실제 하드웨어 접촉 금지 |

## 왜 Python 파일이 많은가

2026-05-13 기준으로 `src`, `tools`, `/home/ssu/ros2_ws/src/Azas` 아래 Python 파일은 총 79개입니다. 많은 이유는 세 가지입니다.

1. ROS 2는 실행 단위를 node로 쪼갭니다. 각 node는 `setup.py`의 `console_scripts`에 등록되는 Python 파일로 존재합니다.
2. launch 파일도 Python입니다. `*.launch.py`는 일반 비즈니스 로직이 아니라 여러 node를 조립하는 실행 설정입니다.
3. 실제 로봇 프로젝트라서 run/check/smoke 도구를 분리했습니다. 실제 모션 전 검증 단계가 많기 때문에 `tools` 아래 Python과 shell이 늘어납니다.

현재 분포:

| 영역 | Python 수 | 성격 |
| --- | ---: | --- |
| `/home/ssu/ros2_ws/src/Azas` | 20 | jarvis motion node, launch, setup |
| `src/azas_bringup` | 10 | bringup launch 중심 |
| `src/azas_voice` | 10 | STT/recipe/LLM mapper, launch, test |
| `src/azas_perception` | 7 | YOLO, pose bridge, simulated detection, GPD adapter |
| `src/azas_task_manager` | 5 | workflow plan, dry-run node, action server |
| `src/azas_motion` | 4 | alignment helper와 executor |
| `src/azas_calibration`, `src/azas_gripper` | 각 3 | 작은 ROS node 패키지 |
| `tools/checks` | 7 | 비-모션 검증 |
| `tools/smoke` | 3 | fake hardware smoke 지원 Python |
| `tools/perception` | 3 | perception 데이터/라벨 보조 도구 |
| `tools/pick` | 2 | pick 실험/감독 실행 도구 |
| `tools/run` | 1 | 감독 관측 실행 도구 |

따라서 파일이 많은 것 자체는 ROS 2 로봇 프로젝트에서는 어느 정도 정상입니다. 다만 현재는 아래 두 가지가 정리 문제입니다.

- node/launch/tool이 한눈에 “운영용, 검증용, 실험용, 레거시”로 구분되지 않습니다.
- jarvis 쪽 motion node에 stage별 로직이 길게 들어 있어 새 개발자가 읽기 어렵습니다.

정리 방향은 Python 파일 수를 억지로 줄이는 것이 아니라, 공개 진입점과 내부 구현을 분리하는 것입니다.

- 운영자가 실행할 것은 `tools/run`에만 둡니다.
- 검증은 `tools/checks`와 `tools/smoke`로 나눕니다.
- ROS node는 계속 패키지 안에 두되 README/docstring과 launch 상태 태그를 붙입니다.
- 긴 motion node는 공통 service/gate helper와 stage primitive로 나눕니다.

## 현재 권장 파이프라인

사용자가 말한 전체 칵테일 순서는 설계상 아래와 같습니다.

```text
STT
-> recipe/dispenser IDs
-> cup/lid detection
-> cup pose in base_link
-> pick cup
-> align cup under dispenser
-> press dispenser
-> repeat dispensers
-> pick lid
-> close lid
-> shake
-> open/remove lid
-> pour into target cup
```

이 단계 정의는 `src/azas_task_manager/azas_task_manager/cocktail_workflow_plan.py`에 있습니다.

현재 구현 상태:

| 단계 | 상태 | 대표 파일 |
| --- | --- | --- |
| STT/recipe | 부분 구현, no-hardware smoke 있음 | `src/azas_voice`, `tools/smoke/smoke_voice_cocktail_no_hardware.sh` |
| cup/lid detection | 컵/뚜껑 탐지 파이프라인 있음, 현장 안정성 확인 필요 | `src/azas_perception`, `tools/checks/check_detection_stability.sh` |
| cup pose bridge | 구현됨 | `cup_detection_pose_bridge_node.py` |
| pick cup + dispenser pre-place | 구현됨, sim/fake/real 진입점 있음 | `tumbler_floor_place_node.py` |
| dispenser press | 부분 구현, real calibration 필요 | `dispenser_press_node.py`, `dispense_lid_sequence_node.py` |
| lid close | 부분 구현, fake 중심 | `dispense_lid_sequence_node.py` |
| shake | 구현됨, sim/fake/real 진입점 있음 | `tumbler_shake_sequence_node.py` |
| lid open/remove | 아직 부족 | 새 primitive 필요 |
| pour | 아직 부족 | target cup detection + pour primitive 필요 |

## 어떤 launch를 쓰는가

### 왜 `src/*/launch/*.py`가 있는가

이 구조는 ROS 2 Python 패키지 관례입니다. `setup.py`에서 launch 파일을 `share/<package>/launch`로 설치해야 `ros2 launch <package> <file>.launch.py`가 동작합니다.

현재 `/home/ssu/Azas/src`에서 launch 폴더가 있는 패키지는 두 개뿐입니다.

| 패키지 | launch 수 | 이유 |
| --- | ---: | --- |
| `azas_bringup` | 8 | 여러 패키지의 node/launch를 조립하는 상위 bringup 패키지이기 때문 |
| `azas_voice` | 1 | STT/recipe mapper를 한 번에 띄우는 voice 전용 launch |

검증 근거:

- `src/azas_bringup/setup.py`가 `glob("launch/*.launch.py")`를 `share/azas_bringup/launch`에 설치합니다.
- `src/azas_voice/setup.py`도 같은 방식으로 voice launch를 설치합니다.
- `/home/ssu/ros2_ws/src/Azas/setup.py`는 `jarvis` launch, rviz, urdf, model, world asset을 `share/jarvis/...`에 설치합니다.

따라서 `src` 아래에 launch `.py`가 있는 것 자체는 정상입니다. 문제는 launch가 많다는 사실이 아니라, `azas_bringup` 안에 권장/실험/레거시 launch가 섞여 있고 이름만 봐서는 상태를 알기 어렵다는 점입니다.

### Azas bringup launch

| launch | 상태 | 용도 |
| --- | --- | --- |
| `robot_connection_control.launch.py` | 권장 real bringup 내부용 | RealSense/RG2/perception/floor-place 제어 조립. 직접보다 `tools/run/run_robot_real.sh` 또는 real pipeline script 사용 |
| `yolo_to_floor_place.launch.py` | 권장 내부용 | YOLO -> pose bridge -> floor/pre-place control |
| `yolo_perception.launch.py` | 권장 perception 단독 | 카메라/YOLO 탐지만 볼 때 |
| `cocktail_dryrun.launch.py` | 권장 planning 검증 | 전체 칵테일 순서 no-motion 검증 |
| `hardware_free_demo.launch.py` | 데모/실험 | 하드웨어 없는 데모 |
| `simulated_cup_grasp_dryrun.launch.py` | 실험 | simulated cup detection 기반 dry-run |
| `mvp_bringup.launch.py` | 레거시/확인 필요 | 현재 권장 진입점 아님 |
| `gpd_grasp_adapter.launch.py` | 실험 | GPD grasp adapter 계약/실험 |

### Jarvis launch

| launch | 상태 | 용도 |
| --- | --- | --- |
| `tumbler_dispenser_then_shake_demo.launch.py` | 권장 sim | RViz에서 M0609 + 디스펜서 pre-place + 쉐이킹을 한 번에 표시 |
| `tumbler_dispenser_scene.launch.py` | 권장 sim 내부용 | RViz scene + M0609 URDF 표시 |
| `tumbler_floor_place.launch.py` | 권장 내부용 | 컵 pick/floor/pre-place control node |
| `tumbler_shake_sequence.launch.py` | 권장 내부용 | high lifted shake node |
| `tumbler_floor_place_demo.launch.py` | 단독 sim | 컵 이동만 볼 때 |
| `dispenser_press.launch.py` | 부분 구현 | 디스펜서 press 단독 테스트 |
| `dispense_lid_sequence.launch.py` | 부분 구현 | 디스펜서 press + lid close fake/dry-run |
| `rg2_trigger.launch.py` | real support | RG2 Trigger wrapper |
| `tumbler_dispenser_gazebo.launch.py` | visual preview | Gazebo asset preview, full physics pipeline 아님 |

## launch 정리 판정

아래는 현재 코드 참조와 역할 기준의 정리 후보입니다. 바로 삭제하지 말고, 한 번의 PR에서 `deprecated/experimental` 표기 후 다음 PR에서 제거합니다.

| launch | 판정 | 이유 | 권장 조치 |
| --- | --- | --- | --- |
| `mvp_bringup.launch.py` | 삭제 후보 | 오래된 MVP 골격입니다. 현재 컵 pose bridge, jarvis floor-place, real gate 흐름을 쓰지 않습니다. | `deprecated_mvp_bringup.launch.py`로 이름 변경 또는 제거 |
| `hardware_free_demo.launch.py` | 통합 후보 | `tumbler_floor_place_demo.launch.py`와 역할이 겹칩니다. 다만 voice/LLM/demo pose까지 묶는 차이가 있습니다. | RViz demo는 jarvis 쪽으로 통일하고, voice demo만 남길지 결정 |
| `simulated_cup_grasp_dryrun.launch.py` | 통합 후보 | simulated cup detection + floor-place dry-run입니다. fake/smoke 경로와 목적이 겹칩니다. | `tools/smoke` 또는 `tools/run` 진입점으로 흡수 |
| `gpd_grasp_adapter.launch.py` | 실험 보존 | GPD 외부 grasp adapter 경계입니다. 현재 메인 파이프라인은 아니지만 독립 실험 가치가 있습니다. | `experimental_gpd_grasp_adapter.launch.py`로 이름 명확화 |
| `tumbler_floor_place_demo.launch.py` | 유지 | 컵 이동 단독 RViz 확인용입니다. | 단독 stage demo로 유지 |
| `tumbler_dispenser_then_shake_demo.launch.py` | 유지 | 현재 사용자가 보는 통합 RViz demo의 권장 진입점입니다. | 대표 sim launch로 유지 |
| `tumbler_dispenser_gazebo.launch.py` | 실험 보존 | Gazebo visual preview일 뿐, 현재 full physics/control 검증은 아닙니다. | `experimental_` 표기 또는 README 경고 유지 |

정리 원칙:

- `tools/run`에서 직접 쓰는 launch는 보존합니다.
- smoke/check에서 검증하는 launch는 보존합니다.
- 같은 목적의 demo launch가 둘 이상이면 하나만 공개 진입점으로 남깁니다.
- 실제 로봇을 움직일 수 있는 launch는 직접 실행용으로 노출하지 말고 `tools/run` gate script 뒤에 둡니다.
- 삭제 전에는 `rg <launch-name>`로 참조를 확인하고, COMMANDS/README/docs 링크를 같이 수정합니다.

### 2026-05-13 실행성 감사 결과

기준:

- Python 파일: `python3 -m compileall -q src tools /home/ssu/ros2_ws/src/Azas`
- Shell 파일: `bash -n tools/**/*.sh`
- ROS launch: `ros2 launch <package> <launch>.launch.py --show-args`
- Shell 실행 비트: `find tools -type f -name '*.sh' ! -perm -111`

결과:

| 항목 | 결과 |
| --- | --- |
| Python 문법 | 통과 |
| Shell 문법 | 통과 |
| Azas bringup launch resolve | 통과 |
| Jarvis launch resolve | 통과 |
| 실행 비트 누락 | `tools/smoke/smoke_voice_cocktail_no_hardware.sh`만 발견, 실행 가능하도록 수정 |

따라서 현재 확인된 문제는 “아예 실행이 안 되는 파일”보다는 “중복 launch, 오래된 MVP launch, 실험 파일의 이름/위치 불명확성”입니다.

## 대표 명령만 먼저 사용

### 전체 no-motion 준비도

```bash
cd /home/ssu/Azas
bash tools/checks/verify_control_readiness.sh
```

### RViz 시뮬레이션

```bash
cd /home/ssu/Azas
tools/run/run_rule_based_dispenser_then_shake_sim.sh
```

### fake hardware 검증

```bash
cd /home/ssu/Azas
tools/smoke/smoke_fake_hardware_path.sh
tools/smoke/smoke_tumbler_shake_sequence.sh
```

### 실제 로봇 전 live gate

```bash
cd /home/ssu/Azas
STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed \
  tools/checks/check_live_hardware_gates.sh
```

### 실제 로봇 파이프라인

```bash
cd /home/ssu/Azas
SELECTED_DISPENSER_ID=2 \
  tools/run/run_rule_based_dispenser_then_shake_real.sh
```

## 기능 개발 절차

새 기능은 바로 full pipeline에 붙이지 않습니다. 아래 순서로 올립니다.

```text
1. stage 단독 구현
2. plan/no-motion 테스트
3. RViz 시각 테스트
4. fake hardware smoke
5. strict live gate 확인
6. 실제 로봇 저속/작은 범위 1회 테스트
7. full pipeline에 연결
```

예시: `PRESS_DISPENSER`

```text
dispenser_press_node 단독 dry-run
-> RViz에서 press pose 확인
-> fake MoveLine smoke
-> calibration.yaml에 measured press pose 확인
-> check_real_motion_config.sh
-> 실제 로봇 저속 1회
-> cocktail pipeline의 PRESS_DISPENSER에 연결
```

## 파일 추가 규칙

- 사용자가 직접 실행할 명령은 `tools/run/`에 둡니다.
- 하드웨어를 움직이지 않는 점검은 `tools/checks/`에 둡니다.
- fake service를 쓰는 자동 검증은 `tools/smoke/`에 둡니다.
- ROS node는 책임 패키지의 `src/<package>/<package>/` 또는 `jarvis/`에 둡니다.
- launch는 가능한 한 “node 조립”만 하고, 안전 gate는 `tools/run` script에서 처리합니다.
- real motion entrypoint는 반드시 motion hold, strict live gate, config gate, typed confirmation을 통과해야 합니다.

## 정리 필요 항목

아래는 아직 정리가 더 필요한 영역입니다.

| 항목 | 필요한 작업 |
| --- | --- |
| full cocktail real orchestrator | `MODE=plan/sim/fake/real`을 지원하는 단일 runner 또는 ROS state machine |
| lid open/remove | primitive와 테스트 작성 |
| pour | target cup detection, pour pose, spill safety rule 작성 |
| launch 정리 | 레거시/실험 launch 이름에 `experimental` 또는 문서 태그 부여 |
| Python 파일 가독성 | node별 README/docstring, stage interface, 공통 service/gate helper 추출 |
