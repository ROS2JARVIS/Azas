# 명령어와 도구 분류 기준

이 문서는 `tools/` 아래 스크립트를 어떤 상황에서 써야 하는지 구분합니다.

## 디렉토리 기준

| 디렉토리 | 용도 | 실제 모션 |
|----------|------|----------|
| `tools/checks/` | 상태 점검, 준비도 확인, 안전 게이트 | 없음 |
| `tools/smoke/` | 가짜 하드웨어 또는 드라이런 회귀 테스트 | 없음 |
| `tools/run/` | 런치/실행/현장 오케스트레이션 | 스크립트별 다름 |
| `tools/pick/` | 컵 픽, 그라스프 계획 보조 도구 | 스크립트별 다름 |
| `tools/perception/` | 인식 데이터 수집, 라벨링, 프레임 추출 | 없음 |
| `tools/gazebo_models/` | Gazebo 프리뷰 모델 | 없음 |

## 어떤 명령을 써야 하나

| 상황 | 먼저 쓸 명령 |
|------|--------------|
| 레포가 빌드/검증 가능한지 확인 | `bash tools/checks/verify_control_readiness.sh` |
| 컵 탐지 파이프라인 확인 | `bash tools/checks/check_robot_detection.sh` |
| TF 파이프라인 확인 | `bash tools/checks/check_tf_pipeline.sh` |
| 룰베이스 칵테일 시퀀스 확인 | `bash tools/smoke/smoke_cocktail_dryrun_sequence.sh` |
| 픽앤얼라인 no-motion 확인 | `bash tools/smoke/smoke_pick_and_align_no_motion.sh` |
| 가상 Doosan 시작 | `bash tools/run/run_doosan_virtual_m0609.sh` |
| 카메라 기반 드라이런 | `bash tools/run/run_robot_dryrun.sh` |
| 실제 모션 전 게이트 | `STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed bash tools/checks/check_live_hardware_gates.sh` |
| 실제 로봇 실행 | `bash tools/run/run_robot_real.sh` |

## 이름 규칙

- `check_*.sh` 또는 `check_*.py`: 상태를 확인하고 실패 이유를 알려주는 점검 명령입니다.
- `smoke_*.sh` 또는 `smoke_*.py`: 하드웨어 없이 빠르게 회귀를 확인하는 자동 테스트입니다.
- `run_*.sh`: 런치 또는 실행을 시작하는 명령입니다.
- `*_report.sh`: 결과 보고서를 만드는 명령입니다. 실제 모션 명령과 섞어 쓰지 않습니다.

## 정리 원칙

1. 실제 로봇을 움직일 수 있는 명령은 이름과 문서에서 위험도를 명확히 표시합니다.
2. `checks/`는 기본적으로 비-모션이어야 합니다.
3. `checks/`에서 RG2/Doosan 서비스는 존재와 타입까지만 확인하고 open/close 또는 motion command를 보내지 않습니다.
4. `smoke/`는 가짜 하드웨어 또는 no-motion 경로만 사용해야 합니다.
5. motion-facing 컵 pose 예시는 `detected:upright` status를 사용합니다. `detected:lid`는 lid evidence이고 cup pose 계약이 아닙니다.
6. `run/`은 실행 진입점으로 유지하되, 오케스트레이션 성격이 강한 스크립트는 README에서 명확히 설명합니다.
7. 절대경로를 포함한 스크립트를 이동할 때는 `COMMANDS.md`, `README.md`, `docs/`, 다른 shell wrapper를 함께 갱신합니다.
8. 신규 `.sh`는 대표 명령으로 공개할 가치가 있을 때만 추가합니다. 단순 Python wrapper는 가능한 한 만들지 않습니다.

## 현재 2차 정리 후보

아래 항목은 바로 이동하면 문서 링크나 절대경로가 깨질 수 있으므로, 별도 PR에서 처리합니다.

| 후보 | 이유 |
|------|------|
| `tools/run/field_no_motion_report.sh` | 보고서 생성 성격이 강함 |
| `tools/run/run_connected_robot_control.sh` | 단일 실행보다 오케스트레이션 성격이 강함 |
| `tools/checks/check_depth_projection_sample.sh` | shell wrapper와 Python core 관계를 명확히 할 수 있음 |
| `tools/checks/check_detection_stability.sh` | shell wrapper와 Python core 관계를 명확히 할 수 있음 |
