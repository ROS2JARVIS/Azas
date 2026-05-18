# Azas Repository File Map

Updated: 2026-05-14

이 문서는 “이게 뭐예요?”가 나오지 않도록 파일의 역할과 실행 상태를 한 곳에 모은 지도입니다. 좌표, 캘리브레이션 값, 궤적은 여기서 새로 만들지 않습니다. 실제 로봇을 움직이는 판단은 측정값과 안전 게이트가 있어야 합니다.

## 상태 표기

| 상태 | 의미 |
| --- | --- |
| 운영 진입점 | 사람이 직접 실행해도 되는 대표 명령 또는 문서 |
| 내부 구현 | launch/script가 호출하는 ROS node 또는 helper |
| 검증 | no-motion, fake hardware, syntax, config, readiness 검사 |
| 실험 | 메인 경로가 아닌 후보/어댑터/데이터 보조 |
| 미완성 경계 | 인터페이스는 있으나 실제 하드웨어 동작이나 저장은 아직 안 됨 |
| 자산 | 모델, RViz, Gazebo, 설정 파일 |

## 실제 로봇이 손끝 하나 움직이지 않는 대표 원인

| 원인 | 직접 증거 | 의미 |
| --- | --- | --- |
| `PickAndAlign` 서버가 `execute_motion`을 실제 실행에 쓰지 않음 | `src/azas_task_manager/azas_task_manager/pick_and_align_action_server.py` | action은 no-motion 진단용이다. 실제 Doosan trajectory 실행이 없다. |
| `calibration.yaml`과 `safety.yaml`에 `null`/`확인 필요`가 남아 있음 | `src/azas_bringup/config/calibration.yaml`, `src/azas_bringup/config/safety.yaml` | 실측 전에는 real-motion config gate가 실패해야 정상이다. |
| 그리퍼 서비스 계약이 둘로 나뉨 | `/azas/gripper/open_close`, `/jarvis/rg2/open`, `/jarvis/rg2/close` | placeholder와 field/fake 경로가 다르다. real RG2 adapter 정리가 필요하다. |
| upright pose bridge/TF가 없으면 컵 포즈가 motion-facing topic으로 안 감 | `cup_detection_pose_bridge_node.py` | `detected:upright`와 base_link TF가 모두 필요하다. `detected:lid`나 `rejected:*`는 컵 pose가 아니다. |
| run/check/smoke가 대부분 안전하게 fail-closed 설계 | `tools/checks`, `tools/smoke`, `tools/run/run_robot_real.sh` | 로봇이 안 움직이는 것은 많은 경우 버그가 아니라 안전 차단이다. |

## 최상위 파일

| 파일 | 상태 | 설명 |
| --- | --- | --- |
| `README.md` | 운영 진입점 | 프로젝트 목표, 빠른 시작, 주요 파이프라인, 대표 명령 링크. |
| `COMMANDS.md` | 운영 진입점 | 빌드, 테스트, 시뮬레이션, 드라이런, 실제 로봇 절차 명령 모음. |
| `CONTRIBUTING.md` | 운영 진입점 | 협업 방식, PR/테스트 기준, 브랜치/커밋 규칙. |
| `AGENTS.md` | 운영 규칙 | AI 에이전트 안전 규칙. 컵 좌표를 직접 묻거나 만들지 말라는 최상위 규칙 포함. |
| `.gitignore` | 운영 규칙 | build/install/log, 로컬 agent 상태, 모델 가중치 등을 커밋에서 제외. |
| `.github/PULL_REQUEST_TEMPLATE.md` | GitHub | PR 체크리스트. 하드웨어 영향 여부를 명시하게 함. |
| `.github/ISSUE_TEMPLATE/task.yml` | GitHub | 구현 task 이슈 템플릿. 목적/범위/완료조건/테스트를 요구. |
| `.github/workflows/ci.yml` | GitHub 검증 | ROS 2 Humble에서 `colcon build`, `colcon test`, `colcon test-result` 실행. |

## `src/azas_interfaces`

| 파일 | 상태 | 설명 |
| --- | --- | --- |
| `CMakeLists.txt` | 빌드 | ROS message/service/action 생성 설정. |
| `package.xml` | 빌드 | 인터페이스 패키지 의존성 선언. |
| `action/PickAndAlign.action` | 내부 계약 | 컵 pick/alignment action 계약. `execute_motion`은 현재 서버에서 실제 모션을 켜지 않는다. |
| `msg/CupDetection.msg` | 내부 계약 | YOLO/depth 검출 결과와 후보 pose를 전달하는 메시지. |
| `msg/README.md` | 문서 | 메시지 설명. 현재 작업 전 사용자 변경이 있어 보존 중. |
| `srv/CalibrateOutlet.srv` | 미완성 경계 | 디스펜서 outlet 캘리브레이션 저장용 서비스 계약. |
| `srv/SaveCupOffset.srv` | 미완성 경계 | 컵/TCP offset 저장용 서비스 계약. |
| `srv/SetGripper.srv` | 내부 계약 | Azas 내부 gripper 명령 계약. 실제 RG2 검증 계약은 아직 별도. |

## `src/azas_perception`

| 파일 | 상태 | 설명 |
| --- | --- | --- |
| `azas_perception/yolo_tumbler_detector_node.py` | 내부 구현 | RealSense RGB-D와 YOLO로 컵 후보를 찾고 `/azas/cup_detection` 발행. bbox 비율로 upright만 `detected:upright ...` status를 받는다. |
| `azas_perception/depth_projection.py` | 내부 구현 | depth pixel과 CameraInfo intrinsics를 camera-frame 3D 좌표로 변환하는 순수 함수. |
| `azas_perception/cup_detection_pose_bridge_node.py` | 내부 구현 | `detected:upright`인 `/azas/cup_detection`만 TF2로 `base_link` 기준 `/jarvis/tumbler_dispenser/tumbler_pose`로 변환. TF 실패 또는 non-upright status면 publish 안 함. |
| `azas_perception/simulated_cup_detection_node.py` | 검증 | 하드웨어 없이 deterministic `CupDetection`을 발행하는 smoke용 노드. |
| `azas_perception/gpd_grasp_adapter_node.py` | 실험 | 외부 grasp detector 결과를 Azas 계약으로 맞추기 위한 adapter 후보. |
| `test/test_depth_and_detection_logic.py` | 검증 | depth projection, bbox orientation, detection selection policy 회귀 테스트. |
| `setup.py`, `setup.cfg`, `package.xml`, `resource/azas_perception` | 빌드 | Python ROS 패키지 등록, console script, pytest 수집 설정. |

## `src/azas_motion`

| 파일 | 상태 | 설명 |
| --- | --- | --- |
| `azas_motion/alignment.py` | 내부 구현 | no-motion pick/side-grasp/observe pose 후보 계산. 실제 trajectory 실행 아님. |
| `azas_motion/alignment_executor_node.py` | 내부 구현 | MoveItPy planning-only 경계. 기본은 실행 금지. |
| `azas_motion/dispenser_sequence_preview_node.py` | 실험/검증 | 디스펜서 시퀀스 preview helper. |
| `azas_motion/side_grasp_ik_preview_node.py` | 실험/검증 | side-grasp IK/계획 후보 preview. |
| `test/test_alignment.py` | 검증 | side-grasp offset, z bounds, observe quaternion normalization 테스트. |
| `setup.py`, `setup.cfg`, `package.xml`, `resource/azas_motion` | 빌드 | Python ROS 패키지 등록, MoveIt 관련 의존성, pytest 수집 설정. |

## `src/azas_task_manager`

| 파일 | 상태 | 설명 |
| --- | --- | --- |
| `azas_task_manager/pick_and_align_action_server.py` | 미완성 경계 | `/azas/pick_and_align` action server. 현재는 `no_motion`/`skeleton`만 지원하며 실제 Doosan/RG2 실행 없음. |
| `azas_task_manager/cocktail_workflow_plan.py` | 내부 구현 | 전체 칵테일 단계와 각 단계의 required input/gate를 데이터로 정의. |
| `azas_task_manager/cocktail_dryrun_sequence_node.py` | 검증 | fake detection/recipe로 전체 workflow plan을 no-motion으로 발행. |
| `setup.py`, `setup.cfg`, `package.xml`, `resource/azas_task_manager` | 빌드 | Python ROS 패키지 등록. |

## `src/azas_bringup`

| 파일 | 상태 | 설명 |
| --- | --- | --- |
| `config/calibration.yaml` | 자산/차단 | 실측 전 `null`/`확인 필요` 값을 유지하는 fail-closed 캘리브레이션 파일. |
| `config/safety.yaml` | 자산/차단 | 속도/가속/작업공간/RG2 안전 값. placeholder가 있으면 real motion 차단. |
| `config/perception.yaml` | 자산/차단 | 현장 perception 토픽/frame 확정용 placeholder. |
| `launch/mvp_bringup.launch.py` | 레거시/검증 | MVP 골격 launch. 현재 pose bridge 포함, real motion enable은 아님. |
| `launch/yolo_perception.launch.py` | 운영/검증 | YOLO perception 단독 실행. 기본 RealSense 토픽은 `/camera/camera/...`. |
| `launch/yolo_to_floor_place.launch.py` | 내부 구현 | YOLO-to-floor/pre-place 제어 경로 조립. |
| `launch/robot_connection_control.launch.py` | 내부 구현 | real connection control launch. 직접보다 `tools/run` 진입점 사용. |
| `launch/cocktail_dryrun.launch.py` | 검증 | 전체 칵테일 workflow no-motion dry-run. |
| `launch/hardware_free_demo.launch.py` | 실험 | 하드웨어 없는 데모 조립. |
| `launch/simulated_cup_grasp_dryrun.launch.py` | 실험/검증 | simulated cup detection 기반 dry-run. |
| `launch/gpd_grasp_adapter.launch.py` | 실험 | GPD adapter 실험 launch. |
| `rviz/azas_dispenser_sequence.rviz` | 자산 | RViz 화면 설정. |
| `setup.py`, `setup.cfg`, `package.xml`, `resource/azas_bringup` | 빌드 | launch/config/RViz 설치와 패키지 의존성. |

## `src/azas_calibration`

| 파일 | 상태 | 설명 |
| --- | --- | --- |
| `azas_calibration/calibration_loader_node.py` | 미완성 경계 | 캘리브레이션 저장 서비스 boundary. 현재 measured value를 저장하지 않고 실패 응답. |
| `setup.py`, `setup.cfg`, `package.xml`, `resource/azas_calibration` | 빌드 | calibration node 패키지 등록. |

## `src/azas_gripper`

| 파일 | 상태 | 설명 |
| --- | --- | --- |
| `azas_gripper/rg2_gripper_node.py` | 미완성 경계 | `/azas/gripper/open_close` placeholder. 실제 RG2를 움직이지 않음. |
| `setup.py`, `setup.cfg`, `package.xml`, `resource/azas_gripper` | 빌드 | gripper node 패키지 등록. |

## `src/azas_voice`

| 파일 | 상태 | 설명 |
| --- | --- | --- |
| `azas_voice/stt_node.py` | 내부 구현 | STT 결과를 ROS topic으로 발행. |
| `azas_voice/command_parser.py` | 내부 구현 | 한국어/간단 명령을 레시피와 디스펜서 ID로 변환. 좌표 생성 금지. |
| `azas_voice/recipe_catalog.py` | 내부 구현 | `recipes.yaml` 로드와 레시피 lookup. |
| `azas_voice/recipe_mapper_node.py` | 내부 구현 | rule-based recipe decision node. |
| `azas_voice/llm_recipe_mapper_node.py` | 내부 구현 | LLM 응답을 sanitize해서 좌표/궤적/캘리브레이션 출력 차단. |
| `config/recipes.yaml` | 자산 | 레시피와 고정 디스펜서 ID mapping. |
| `launch/azas_voice.launch.py` | 운영/검증 | voice pipeline launch. |
| `test/test_command_parser.py` | 검증 | rule-based parser 회귀 테스트. |
| `test/test_llm_recipe_mapper.py` | 검증 | LLM sanitizer가 좌표성 출력을 거부하는지 테스트. |
| `setup.py`, `setup.cfg`, `package.xml`, `resource/azas_voice` | 빌드 | voice 패키지 등록과 pytest 수집 설정. |

## `tools/run`

| 파일 | 상태 | 설명 |
| --- | --- | --- |
| `run_robot_dryrun.sh` | 운영 진입점 | 실제 모션 없이 Azas dry-run 파이프라인 실행. |
| `run_robot_real.sh` | 운영 진입점/차단 | strict live gate stamp와 explicit phrase 없이는 실제 모션 거부. |
| `run_connected_cup_pick_real.sh` | 운영 진입점/차단 | 로봇/카메라/RG2가 이미 연결된 상태에서 blocker 설명, strict live gate, dry pick, optional one-shot real pick을 순서대로 실행. |
| `run_cup_to_dispenser_press_real.sh` | 운영 진입점/차단 | live camera 감지 → 사이드그랩 → 선택 출수구 아래 컵 배치 → 디스펜서 프레스. strict gate와 측정 config 없이는 실행 거부. |
| `run_real_robot_test_ladder.sh` | 운영 진입점/차단 | status, no-hardware, field, live-gate, observe-dry, pick-dry, pick-real을 단계적으로 실행하는 실로봇 테스트 사다리. |
| `run_connected_robot_control.sh` | 운영 진입점/검증 | Doosan real no-motion bringup, dry-run, strict acceptance 후 gated real entrypoint로 연결. |
| `run_doosan_virtual_m0609.sh` | 운영 진입점 | Doosan virtual MoveIt launch helper. |
| `run_doosan_real_no_motion_m0609.sh` | 운영/차단 | 실제 로봇 연결 no-motion bringup. localhost/무확인 실행 거부. |
| `run_rule_based_dispenser_then_shake_sim.sh` | 운영 진입점 | rule-based dispenser/shake simulation. |
| `run_rule_based_dispenser_then_shake_real.sh` | 운영/차단 | 실제 dispenser-then-shake sequence. strict gate 후에만 진입. |
| `run_rule_based_shake_real.sh` | 운영/차단 | 실제 shake-only entrypoint. strict gate 필요. |
| `run_supervised_observe_pose.py` | 미완성 경계 | supervised observe pose gate. 현재 real observe motion은 구현되지 않았다고 거부. |
| `field_no_motion_report.sh` | 검증 | 현장 no-motion 상태 보고서 생성. |
| `README.md` | 문서 | run script 목록과 사용 기준. |

## `tools/checks`

| 파일 | 상태 | 설명 |
| --- | --- | --- |
| `verify_control_readiness.sh` | 검증 | syntax, OSS stack, smoke, workflow plan, real-motion gate smoke를 묶은 no-hardware verifier. |
| `check_oss_stack.sh` | 검증 | ROS/Doosan/MoveIt/RealSense/YOLO/STT 의존성 존재 확인. |
| `check_live_hardware_gates.sh` | 검증/차단 | camera, pose, Doosan/RG2 service type, config gate 확인. motion/RG2 호출 없음. |
| `check_real_motion_config.sh` | 검증/차단 | `calibration.yaml`, `safety.yaml` placeholder와 기본 안전값 검사. |
| `check_measured_dispenser_geometry.py` | 검증/차단 | 측정된 디스펜서 outlet/press 좌표가 현재 launch geometry와 다르면 real motion config gate를 실패시킴. |
| `check_connection_stage.sh` | 검증 | 현재 ROS graph/config 상태를 보고 다음 연결 단계 제안. |
| `check_depth_projection_sample.py`, `.sh` | 검증 | live depth와 CameraInfo에서 sample projection 확인. known-distance 검증은 별도 필요. |
| `check_detection_stability.py`, `.sh` | 검증 | `/azas/cup_detection` 안정성 sampling. |
| `check_robot_detection.sh` | 검증 | cup detection topic quick check. motion-facing 컵은 `detected:upright` status가 필요함을 안내한다. |
| `check_tf_pipeline.sh` | 검증 | TF pipeline 확인. placeholder TF는 real motion 금지. |
| `check_hand_eye_readiness.sh` | 검증 | camera topics, CameraInfo frame, base-camera TF evidence 확인. |
| `robot_connection_acceptance.sh` | 검증 | strict field no-motion report와 hand-eye readiness를 묶은 acceptance helper. |
| `check_cup_lid_sequence.sh` | 검증 | operator가 lid/cup을 놓고 안정성 확인하는 no-motion sequence. |
| `check_static_cup_lid_dataset.py` | 검증 | static dataset/label/prediction gate. depth/TF/robot 좌표는 검증하지 않음. |
| `check_fixed_dispenser_geometry.py` | 검증 | fixed dispenser geometry/config sanity. |
| `check_cocktail_workflow_plan.py` | 검증 | workflow plan에 calibration, press, shake gates가 있는지 확인. |
| `check_grasp_adapter_contract.py` | 검증 | grasp adapter contract 확인. |
| `check_cup_orientation_heuristic.py` | 검증 | bbox orientation heuristic 확인. |
| `check_observe_pose_planning_only.sh` | 검증 | observe pose가 planning-only boundary를 지키는지 확인. |
| `check_side_grasp_planning_only.sh` | 검증 | side grasp planning-only boundary 확인. |
| `explain_real_robot_blockers.sh` | 운영/진단 | 지금 왜 real robot이 차단되는지 설명. |
| `README.md` | 문서 | checks 목록. |

## `tools/smoke`

| 파일 | 상태 | 설명 |
| --- | --- | --- |
| `fake_hardware_services.py` | 검증 | fake Doosan/RG2 services. 실제 하드웨어를 움직이지 않음. |
| `smoke_control_path.sh` | 검증 | fake detection-to-control 경로 smoke. |
| `smoke_fake_hardware_path.sh` | 검증 | fake hardware service까지 포함한 smoke. |
| `smoke_cup_to_dispenser_press_path.sh` | 검증 | fake MoveLine 서비스로 디스펜서 press stage가 선택 출수구 좌표를 쓰는지 확인. |
| `smoke_pick_and_align_no_motion.sh` | 검증 | fake base_link pose로 `/azas/pick_and_align` no-motion action 검증. |
| `smoke_cocktail_dryrun_sequence.py`, `.sh` | 검증 | full cocktail dry-run sequence 검증. symbolic cup/lid presence만 주입하며 motion-facing `detected:upright` pose publish나 RG2 호출은 하지 않는다. |
| `smoke_voice_cocktail_no_hardware.py`, `.sh` | 검증 | voice/recipe no-hardware smoke. |
| `smoke_tumbler_shake_sequence.sh` | 검증 | fake high-shake sequence와 bad-case failure 검증. |
| `smoke_real_motion_entrypoint_gates.sh` | 검증/차단 | real entrypoint가 missing/non-strict/placeholder config에서 거부하는지 테스트. |
| `smoke_real_motion_config_gate.sh` | 검증/차단 | placeholder config fail과 measured-like fixture pass를 테스트. |
| `README.md` | 문서 | smoke 목록. |

## `tools/perception`, `tools/pick`, 모델/의존성

| 파일/폴더 | 상태 | 설명 |
| --- | --- | --- |
| `tools/perception/export_grasp_frame.py` | 실험/검증 | RGB-D/CameraInfo frame capture. |
| `tools/perception/auto_label_yolo_cups.py` | 실험 | YOLO cup dataset auto-label helper. |
| `tools/perception/depth_rule_cup_detector.py` | 실험 | depth-rule 기반 cup detector 후보. |
| `tools/pick/run_supervised_real_single_cup_pick.py` | 미완성/차단 | supervised one-shot pick gate. explicit gates 없이는 real motion 거부. |
| `tools/pick/sweep_side_grasp_planning_candidates.py` | 실험/검증 | side grasp planning candidate sweep. execution 없음. |
| `models/*` | 자산 | OBJ/USD/PNG preview assets. robot control truth가 아님. |
| `tools/gazebo_models/*` | 자산 | Gazebo preview model/world. full physics/control 검증 아님. |
| `dependencies/*` | 문서/의존성 | 외부 source repos, apt packages, optional Python packages 기록. |

## `docs`

| 파일 | 상태 | 설명 |
| --- | --- | --- |
| `developer_orientation.md` | 운영 문서 | 신규 개발자용 프로젝트 지도와 launch 정리 기준. |
| `repository_file_map.md` | 운영 문서 | 이 파일. 파일별 역할과 실제 모션 차단 이유. |
| `project_gap_audit.md` | 운영 문서 | 현재 구조적 약점과 보완/잔여 리스크. |
| `control_readiness_audit.md` | 운영 문서 | control-ready 상태와 미완료 하드웨어 증거. |
| `safety_checklist.md` | 운영 문서 | 실제 로봇 전 안전 체크리스트. |
| `field_control_runbook.md` | 운영 문서 | 현장 터미널별 실행 절차. |
| `field_execution_commands.md` | 운영 문서 | 현장 실행 명령 상세. |
| `real_motion_measurement_worksheet.md` | 운영 문서 | real motion 전에 채워야 할 모든 측정값. |
| `real_robot_test_ladder.md` | 운영 문서 | 실제 동작으로 가는 staged shell 테스트 절차. |
| `simulation_and_connection_plan.md` | 운영 문서 | non-hardware -> camera -> robot/RG2 no-motion -> real motion 연결 순서. |
| `tf_debug_checklist.md` | 검증 문서 | TF debug와 placeholder TF 금지 조건. |
| `frames.md` | 계약 문서 | frame 이름과 확인 필요 항목. |
| `full_cocktail_workflow_plan.md` | 설계 문서 | 전체 칵테일 단계와 구현/미구현 상태. |
| `cup_dataset_and_detection.md` | perception 문서 | dataset/detection 상태와 한계. |
| `grasp_detector_adapter.md` | 실험 문서 | grasp detector adapter contract. |
| `anygrasp_integration.md` | 실험 문서 | AnyGrasp 검토와 안전 gate. |
| `oss_robot_control_stack.md` | 설계 문서 | ROS/Doosan/MoveIt/RealSense/YOLO 등 OSS stack mapping. |
| `dsr_deeptree_integration.md` | 설계 문서 | DSR DeepTree 참고 패턴과 적용 경계. |
| `tumbler_dispenser_models.md` | 자산 문서 | dispenser/tumbler model 설명. |
| `recovery_after_poweroff.md` | 운영 문서 | 전원 차단 후 복구 절차. |
| `onboarding/01-quickstart-kor.md` | 운영 문서 | 신규 팀원 빠른 시작. |
| `onboarding/02-role-map-kor.md` | 운영 문서 | 역할별 패키지 지도. |
| `reference/command-taxonomy.md` | 운영 문서 | `tools/run`, `checks`, `smoke` 구분 기준. |

## 다음 정리 순서

1. `PickAndAlign.action.execute_motion`을 지금처럼 미래 필드로 둘지, no-motion action과 real-motion action을 분리할지 결정.
2. `/azas/gripper/open_close`와 `/jarvis/rg2/*` 중 real adapter 표준 계약을 하나로 정리.
3. `calibration_loader_node.py`가 measured value를 저장할지, config 파일은 수동 측정 워크시트만 받을지 결정.
4. `verify_control_readiness.sh`의 `/home/ssu/...` 전제와 Jarvis 경로를 CI 가능하게 정리.
5. 실제 모션 전에는 `docs/real_motion_measurement_worksheet.md`의 모든 항목을 실측으로 채우고 `check_real_motion_config.sh`를 통과시킨다.
