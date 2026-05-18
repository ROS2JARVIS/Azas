# tools/run/

현장 실행 진입점입니다. 새 스크립트를 늘리기보다 아래 대표 명령에 옵션을 추가하세요.

## 규칙

- 실제 모션 스크립트는 `--enable-real-motion` 플래그와 확인 문구가 필수
- 실제 모션은 기본적으로 1회성 실행 (자동 반복 금지)
- MoveIt 플래닝 실패 시 Doosan 직접 명령으로 폴백 절대 금지
- 그리퍼 명령은 관측 모션과 분리하여 명시적으로 처리

## 현장 투입 순서

```
① run_doosan_virtual_m0609.sh   # 가상 로봇 시작
② run_robot_dryrun.sh            # 드라이런 검증
③ check_live_hardware_gates.sh   # 게이트 통과 (checks/ 참고)
④ run_real_robot_test_ladder.sh  # observe/pick 단계형 실로봇 테스트
⑤ run_connected_cup_pick_real.sh # live gate → dry pick → optional one-shot real pick
⑥ run_robot_real.sh              # full real entrypoint
⑦ run_cup_to_dispenser_press_real.sh # 컵을 출수구 아래에 놓고 디스펜서 프레스
```

## 스크립트 목록

| 스크립트 | 설명 |
|----------|------|
| `run_doosan_virtual_m0609.sh` | 가상 Doosan M0609 시작 |
| `run_doosan_real_no_motion_m0609.sh` | 실제 Doosan 비-모션 연결 |
| `run_robot_dryrun.sh` | 카메라 기반 드라이런 |
| `run_robot_real.sh` | 실제 로봇 모션 실행 |
| `run_connected_cup_pick_real.sh` | 로봇/카메라/RG2 연결 후 strict live gate, dry pick, optional one-shot real pick을 순서대로 실행 |
| `run_cup_to_dispenser_press_real.sh` | 카메라 감지 → 사이드그랩 → 선택 출수구 아래 컵 배치 → 디스펜서 프레스 |
| `run_real_robot_test_ladder.sh` | status → live-gate → dry-run → one-shot real pick 단계형 실로봇 테스트 |
| `run_rule_based_dispenser_then_shake_sim.sh` | RViz에서 디스펜서 pre-place 이동 후 high-shake 시뮬레이션 |
| `run_rule_based_dispenser_then_shake_real.sh` | 실제 로봇에서 디스펜서 pre-place 이동 후 high-shake 실행 |
| `run_rule_based_shake_real.sh` | 실제 로봇 high-shake 단독 실행 |
| `run_connected_robot_control.sh` | 연결 로봇 제어 통합 |
| `run_supervised_observe_pose.py` | 감독 하에 관측 포즈 이동 |
| `field_no_motion_report.sh` | 현장 비-모션 종합 보고서 |
