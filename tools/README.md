# tools/

현장 운용 도구 모음입니다. 역할별 서브디렉토리로 정리되어 있습니다.

신규 개발자는 먼저 [docs/developer_orientation.md](../docs/developer_orientation.md)를 읽으세요.
이 문서가 `src/`, `tools/`, launch 파일의 책임과 현재 권장 진입점을 정리합니다.

## 먼저 외울 대표 명령

`tools/` 아래에는 내부 점검용 `.sh`가 많습니다. 신규 협업자는 전부 외우지 말고 아래 대표 명령만 먼저 사용하세요.

| 상황 | 명령 |
|------|------|
| 전체 비-하드웨어 준비도 확인 | `bash tools/checks/verify_control_readiness.sh` |
| 칵테일 룰베이스 드라이런 확인 | `bash tools/smoke/smoke_cocktail_dryrun_sequence.sh` |
| 카메라 기반 드라이런 | `bash tools/run/run_robot_dryrun.sh` |
| 실제 모션 전 게이트 | `STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed bash tools/checks/check_live_hardware_gates.sh` |
| 실제 모션 진입점 | `bash tools/run/run_robot_real.sh` |
| 디스펜서 이동 + 쉐이킹 RViz | `bash tools/run/run_rule_based_dispenser_then_shake_sim.sh` |
| 디스펜서 이동 + 쉐이킹 실제 로봇 | `bash tools/run/run_rule_based_dispenser_then_shake_real.sh` |

## 서브디렉토리 구조

| 디렉토리 | 설명 |
|----------|------|
| `checks/` | 비-모션 상태 점검 스크립트 (하드웨어 명령 없음) |
| `smoke/` | 가짜 하드웨어 기반 자동화 스모크 테스트 |
| `run/` | 현장 실행 스크립트 (드라이런 · 실제 모션 · 복구) |
| `pick/` | 컵 픽 · 사이드 그라스프 플래닝 도구 |
| `perception/` | 인식 데이터 수집 · 프레임 추출 도구 |
| `gazebo_models/` | Gazebo 프리뷰 모델 |

## 현장 투입 핵심 명령어

```bash
# 비-하드웨어 전체 점검
bash tools/checks/verify_control_readiness.sh

# 드라이런
bash tools/run/run_robot_dryrun.sh

# 엄격 게이트 (실제 모션 전 필수)
STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed \
  bash tools/checks/check_live_hardware_gates.sh

# 실제 모션
bash tools/run/run_robot_real.sh
```

> 전체 명령어 목록은 루트의 **[COMMANDS.md](../COMMANDS.md)** 를 참고하세요.

## 파일 추가 규칙

- 새 구현 파일은 반드시 적절한 서브디렉토리에 넣으세요.
- 루트에는 **안정적인 공개 진입점**만 허용합니다.
- 새 `.sh`를 추가하기 전에 기존 대표 명령 또는 Python 도구로 처리할 수 있는지 먼저 확인하세요.
- 단순 Python 실행 wrapper는 공개 진입점이 꼭 필요할 때만 추가하세요.
- 실제 모션 스크립트는 `--enable-real-motion` 플래그와 확인 문구가 필수입니다.
- 스모크 테스트는 실제 로봇 모션을 포함하지 않아야 합니다.

## 컵 픽 도구 진입점

```bash
# 감독 하에 실제 단일 컵 픽 (도움말)
python3 tools/pick/run_supervised_real_single_cup_pick.py --help

# 사이드 그라스프 후보 스윕 도움말
# 실제 컵 좌표는 비전 파이프라인에서 받아야 하며, 수동 좌표 예시는 문서에 두지 않습니다.
python3 tools/pick/sweep_side_grasp_planning_candidates.py --help
```
