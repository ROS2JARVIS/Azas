# tools/checks/

비-모션 상태 점검 진입점입니다. 작은 래퍼를 계속 추가하지 말고 대표 점검에 통합하세요.

## 규칙

- 로봇 모션 명령 절대 금지
- RG2 open/close/set-width 명령 금지 (명시적 하드웨어 점검 파일 제외)
- 점검 실패 시 non-zero 종료 코드 반환
- 점검한 서비스/토픽/액션 이름을 출력

## 스크립트 목록

| 스크립트 | 설명 |
|----------|------|
| `check_oss_stack.sh` | OSS 스택 전체 점검 (패키지·런치·의존성) |
| `verify_control_readiness.sh` | 제어 준비도 종합 점검 |
| `check_live_hardware_gates.sh` | 하드웨어 게이트 점검 (드라이런 전) |
| `check_connection_stage.sh` | 다음 연결 단계 결정 |
| `check_real_motion_config.sh` | 실제 모션 설정 점검 |
| `check_robot_detection.sh` | 카메라 탐지 확인 |
| `check_tf_pipeline.sh` | TF 파이프라인 점검 |
| `check_observe_pose_planning_only.sh` | 관측 포즈 플래닝 점검 |
| `check_side_grasp_planning_only.sh` | 사이드 그라스프 플래닝 점검 |
| `check_hand_eye_readiness.sh` | 핸드-아이 캘리브레이션 준비도 |
| `check_depth_projection_sample.sh` | 깊이 투영 샘플 점검 |
| `check_detection_stability.sh` | 탐지 안정성 점검 |
| `check_cup_lid_sequence.sh` | 컵·뚜껑 시퀀스 점검 |
| `check_grasp_adapter_contract.py` | 그라스프 어댑터 계약 점검 |
| `check_fixed_dispenser_geometry.py` | 디스펜서 고정 기하 점검 |
| `check_cocktail_workflow_plan.py` | 칵테일 워크플로우 플랜 점검 |
| `check_cup_orientation_heuristic.py` | 컵 방향 휴리스틱 점검 |
| `check_static_cup_lid_dataset.py` | 정적 컵·뚜껑 데이터셋 점검 |
| `check_detection_stability.py` | 탐지 안정성 (Python) |
| `check_depth_projection_sample.py` | 깊이 투영 샘플 (Python) |
| `explain_real_robot_blockers.sh` | 실제 모션 차단 요인 설명 |
| `field_no_motion_report.sh` → (moved to run/) | 현장 비-모션 보고서 |
| `robot_connection_acceptance.sh` | 로봇 연결 수락 점검 |
