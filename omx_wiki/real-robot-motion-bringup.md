---
title: "Real Robot Motion Bringup"
tags: ["real-robot", "bringup", "motion", "safety", "github"]
created: 2026-05-15T08:28:36.609Z
updated: 2026-05-15T08:28:36.609Z
sources: []
links: []
category: session-log
confidence: medium
schemaVersion: 1
---

# Real Robot Motion Bringup

## 기준 레포

- 이제 기준 원격 저장소는 organization 레포 `https://github.com/ROS2JARVIS/Azas.git`이다.
- 로컬 `/home/ssu/Azas`의 `origin`은 `ROS2JARVIS/Azas`를 가리킨다.
- `main` 브랜치는 PR 승인 2명 이상, stale review dismiss, conversation resolution required, force push/deletion 금지 규칙을 사용한다.

## 담당 범위

- 이 작업 범위는 STT 이후 컵을 디스펜서 앞 위치로 옮기는 로직과 쉐이킹 로직이다.
- 컵 잡기 자체는 다른 담당자가 진행한다.
- 컵 위치는 사람이 입력하거나 하드코딩하지 않는다. `/jarvis/tumbler_dispenser/tumbler_pose` `PoseStamped` 토픽을 사용한다.

## 현재 Azas 실행 경로

- RViz/시뮬레이션 확인: `bash tools/run/run_rule_based_dispenser_then_shake_sim.sh`
- 실제 로봇 디스펜서 이동 후 쉐이킹: `bash tools/run/run_rule_based_dispenser_then_shake_real.sh`
- 실제 로봇 쉐이킹 단독: `bash tools/run/run_rule_based_shake_real.sh`
- 단계형 실로봇 테스트: `STAGE=<stage> bash tools/run/run_real_robot_test_ladder.sh`

## 실제 로봇 이동 전 필수 순서

1. `STAGE=status bash tools/run/run_real_robot_test_ladder.sh`
2. `STAGE=no-hardware bash tools/run/run_real_robot_test_ladder.sh`
3. `RUN_LID_STABILITY=true RUN_CUP_STABILITY=true STAGE=field bash tools/run/run_real_robot_test_ladder.sh`
4. `RUN_LID_STABILITY=true RUN_CUP_STABILITY=true STAGE=live-gate bash tools/run/run_real_robot_test_ladder.sh`
5. `STAGE=observe-dry bash tools/run/run_real_robot_test_ladder.sh`
6. `STAGE=pick-dry bash tools/run/run_real_robot_test_ladder.sh`
7. 실제 모션은 fresh live gate와 real-motion config gate가 모두 통과한 뒤에만 실행한다.

## 2026-05-15 현재 검증 결과

최근 점검에서 실제 로봇 모션은 아직 차단 상태다.

- `check_real_motion_config.sh` 실패: `calibration.yaml`, `safety.yaml`에 `null`/`확인 필요` 값이 남아 있음.
- 미측정 항목: camera frame, EE link, planning group, hand-eye transform, TCP-to-cup offset, dispenser outlet/press poses, clearance, workspace bounds, RG2 기본 폭/힘.
- `check_live_hardware_gates.sh` 실패: camera image/depth/camera_info 토픽 없음, Doosan `/motion/move_line`, `/motion/move_joint` 서비스 없음, RG2 `/jarvis/rg2/open`, `/jarvis/rg2/close` 서비스 없음.
- 따라서 로봇을 연결했다고 바로 모션을 실행할 수 있는 상태는 아니다.

## 다음 현장 목표

- RealSense, Doosan, RG2 드라이버를 띄워 live hardware gate를 통과시킨다.
- 실측으로만 calibration/safety 값을 채운다. LLM이 좌표나 캘리브레이션 값을 생성하지 않는다.
- config gate와 live gate 통과 후 dry-run을 먼저 실행한다.
- 실제 모션은 1회성, 낮은 속도/가속도 제한, 명시 confirmation 문구가 있는 엔트리포인트만 사용한다.
