# 패키지 경계와 정리 기준

이 문서는 현재 동작을 유지하면서 Azas 코드를 어떤 패키지에 두어야 하는지 정리한 기준입니다.
기존 실행점은 호환성을 위해 유지하되, 새 기능은 아래 소유 패키지에 추가합니다.

## 패키지 소유권

| 역할 | 담당 패키지 | 새 코드 기준 |
|------|-------------|--------------|
| 시스템 통합, launch, YAML 설정 | `azas_bringup` | launch 파일, RViz 설정, `config/*.yaml` |
| 공용 메시지, 서비스, 액션 | `azas_interfaces` | 노드가 공유하는 ROS 인터페이스 |
| 비전, 탐지, 3D 투영 | `azas_perception` | 카메라 입력, YOLO, depth projection, TF pose bridge |
| 카메라/베이스/디스펜서 캘리브레이션 | `azas_calibration` | 실측/검증 도구와 캘리브레이션 절차 |
| 로봇 팔 모션, MoveIt, planning scene | `azas_motion` | 궤적 계획, collision scene, preview/visualizer |
| RG2 그리퍼 제어 | `azas_gripper` | 그리퍼 서비스, 드라이버 wrapper |
| 디스펜서 장치/펌프 액추에이션 | `azas_dispenser` | 디스펜서 자체 동작, press 준비/실행 도구 |
| 작업 오케스트레이션 | `azas_task_manager` | pick-and-align 액션, 칵테일 순서 제어 |
| 음성, 레시피, 의도 해석 | `azas_voice` | STT, recipe mapping, intent 발행 |
| 과거 실험/호환 sandbox | `dsr_practice` | 제거 완료 대상. 새 production 기능 추가 금지 |

## 현재 canonical 실행 경로

| 목적 | 기준 실행점 |
|------|-------------|
| 컵 탐지 | `ros2 run azas_perception yolo_tumbler_detector_node` |
| 컵 pose 변환 | `ros2 run azas_perception cup_detection_pose_bridge_node` |
| workspace 벽/바닥 collision | `ros2 run azas_motion workspace_collision_scene_node` |
| 디스펜서 collision | `ros2 run azas_motion measured_dispenser_collision_scene_node` |
| 전체 scene collision 적용 | `ros2 launch azas_bringup workspace_collision_scene.launch.py` |
| 픽앤얼라인 액션 서버 | `ros2 run azas_task_manager pick_and_align_action_server` |
| 컵홀더 컵 뚜껑 체결 | `ros2 run azas_motion lid_screw_on_holder_node` |
| RG2 서비스 | `ros2 run azas_gripper rg2_gripper_node` |
| 음성/레시피 매핑 | `ros2 launch azas_voice azas_voice.launch.py` |

`workspace_collision_scene.launch.py`는 벽/바닥과 디스펜서 collision을 함께 적용하는 통합 launch입니다.
MoveIt의 RG2 collision은 URDF/Xacro 로봇 모델에 포함되어야 하므로 별도 scene node가 아니라 robot description 쪽에서 검증합니다.

## Legacy 분류 기준

이름에 `_legacy`가 붙은 실행점은 과거 `dsr_practice` 코드를 패키지별로 옮겨 둔 호환 진입점입니다.
새 기능을 붙이지 않고, 필요하면 같은 동작을 위 canonical 패키지에 다시 설계합니다.

| 위치 | 상태 | 이유 |
|------|------|------|
| `azas_motion/*_legacy.py` | 유지 | 과거 MoveIt 예제/현장 테스트 호환 |
| `azas_perception/*_legacy_node.py` | 유지 | 과거 카메라/클릭 기반 실험 호환 |
| `azas_voice/stt_*_legacy.py` | 유지 | 음성에서 직접 모션을 호출하던 과거 흐름 호환 |
| `azas_bringup/launch/*_legacy.launch.py` | 유지 | 기존 명령어 문서/현장 습관 보호 |
| `dsr_practice/*` | 제거 | 기존 구현은 소유 패키지로 이관 완료 |

## 정리 원칙

1. 컵 좌표는 사람이 입력하거나 코드에 하드코딩하지 않습니다. `/jarvis/tumbler_dispenser/tumbler_pose`를 사용합니다.
2. 모션, collision, 캘리브레이션 값은 LLM이 생성하지 않습니다. 실측값과 설정 파일만 반영합니다.
3. 실제 로봇을 움직일 수 있는 코드는 `azas_motion`, `azas_task_manager`, `azas_dispenser`, `azas_gripper` 중 하나에 둡니다.
4. `azas_voice`는 의도/레시피까지만 담당하고 직접 좌표/궤적을 만들지 않습니다.
5. `azas_perception`은 pose를 발행할 수 있지만 로봇을 움직이지 않습니다.
6. launch와 YAML은 `azas_bringup`에서 소유하고, 개별 기능 노드는 담당 패키지에서 소유합니다.
7. `dsr_practice`는 새 기능의 기준 위치로 사용하지 않습니다. 기존 구현은 소유 패키지에 있습니다.

## 다음 이관 후보

아래 항목은 기능을 유지한 채 이관할 수 있지만, launch 이름과 현장 명령어가 바뀔 수 있어 별도 검증 단위로 진행합니다.

| 후보 | 권장 방향 |
|------|----------|
| `dsr_practice/yolo_cup_pick_node.py` | 완료: `azas_motion.yolo_cup_pick_legacy_node` wrapper |
| `dsr_practice/gripper.py` | 완료: `azas_gripper.gripper_legacy` wrapper |
| `dsr_practice/Calibration_Tutorial/` | 완료: `azas_calibration` legacy 도구로 흡수 |
| `dsr_practice/realsense_data_collector.py` | 완료: `azas_perception.realsense_data_collector_legacy_node` wrapper |
| 중복 `src/dsr_practice/dsr_practice/dsr_practice/` | 완료: 중복 소스 트리 제거 |

이 문서의 기준으로 정리하면 기능은 유지하고, 새 코드가 다시 `dsr_practice`나 legacy 파일로 흘러들어가는 것을 막을 수 있습니다.
