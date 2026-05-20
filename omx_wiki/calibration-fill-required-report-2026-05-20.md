---
title: "Calibration Fill Required Report 2026-05-20"
tags: ["azas", "calibration", "real-robot", "measurement", "safety"]
created: 2026-05-20T15:54:58+09:00
updated: 2026-05-20T16:02:00+09:00
sources:
  - src/azas_bringup/config/calibration.yaml
  - src/azas_bringup/config/measured_dispenser_collision.yaml
  - docs/real_motion_measurement_worksheet.md
  - docs/control_readiness_audit.md
  - omx_wiki/azas-real-robot-handoff-2026-05-18-dispenser-shake-panel.md
links: []
category: real-robot-calibration
confidence: high
schemaVersion: 1
---

# Calibration Fill Required Report 2026-05-20

## 결론

`src/azas_bringup/config/calibration.yaml`은 실제 로봇 운용 전에 채워야 한다. 다만 현재 `null` 또는 `확인 필요`인 값은 LLM, 위키 추정, 예시 TF, 테스트 fixture 값으로 채우면 안 된다. 이 값들은 실측, hand-eye 캘리브레이션 결과, MoveIt/Doosan/RG2 실제 설정 증거로만 채운다.

현재 파일에서 실측 초안으로 볼 수 있는 항목은 `cup_holder` 블록뿐이다. 디스펜서 `front_hold_poses`는 별도 파일에 있고, `calibration.yaml`의 `dispenser_outlets` outlet/press pose를 대체하지 않는다.

## 측정했는데 calibration.yaml에 안 들어간 이유

현재 repo 구조상 측정값 저장 경로가 두 갈래로 나뉘어 있다.

1. 패널의 `teach_front_hold_1..4` 단계는 `tools/run/teach_measured_dispenser_front_hold.py --dispenser-id N --write`를 실행한다.
2. 이 스크립트의 기본 저장 대상은 `src/azas_bringup/config/measured_dispenser_collision.yaml`이다.
3. 스크립트 설명도 `base_link -> link_6` TF를 `front_hold_poses.dispenser_N`에 복사한다고 되어 있다.
4. 따라서 디스펜서 앞에서 컵을 들고 서는 front-hold 측정은 정상적으로 별도 파일에 들어갔지만, `calibration.yaml`의 `dispenser_outlets.<id>.outlet_pose_*`나 `press_pose_*`에는 쓰이지 않았다.
5. `src/azas_calibration/azas_calibration/calibration_loader_node.py`의 `/azas/calibration/set_dispenser_outlet`와 `/azas/calibration/save_cup_offset` 서비스는 현재 “boundary only” 상태이며 measured 값을 파일에 저장하지 않는다.

즉 누락 이유는 실측이 사라진 것이 아니라, 저장 도구가 `calibration.yaml`용 persist 구현이 아니었기 때문이다. 현재 확인한 소스본과 설치본의 `calibration.yaml`, `measured_dispenser_collision.yaml`은 서로 일치한다.

복구/정리 방향은 둘 중 하나다.

- 안전한 단기 방향: 측정된 front-hold는 계속 `measured_dispenser_collision.yaml`에서 쓰고, outlet/press pose는 별도 teaching 절차로 다시 기록해 `calibration.yaml`에 넣는다.
- 구조 개선 방향: `record_teaching_pose.sh` 또는 새 helper가 label별로 `calibration.yaml`의 `dispenser_outlets.<id>.outlet_pose_*`, `press_pose_*`, `cup_offsets.*`를 명시적으로 업데이트하도록 구현한다. 이때 front-hold `link_6` pose를 outlet/press pose로 자동 복사하지 않는다.

## 지금 채워야 하는 calibration.yaml 항목

| 필드 | 현재 상태 | 필요한 증거 |
| --- | --- | --- |
| `frames.camera_frame` | `null` | live `CameraInfo.header.frame_id` |
| `frames.ee_link` | `null` | 실제 M0609 MoveIt end-effector link |
| `frames.planning_group` | `null` | 실제 MoveIt planning group |
| `frames.gripper_tcp` | `gripper_tcp`, 확인 필요 | 실제 RG2 TCP frame 또는 Doosan TCP 설정 |
| `hand_eye.child_frame` | `null` | hand-eye에 사용한 camera frame |
| `hand_eye.xyz_m` | `null` | `base_link -> camera_frame` hand-eye 결과 |
| `hand_eye.rpy_rad` | `null` | 같은 transform의 RPY rad |
| `cup_offsets.default.tcp_to_cup_mouth_m` | `null` | 그립 후 TCP에서 컵 입구 중심까지 jig/dry-run 실측 |
| `outlet.pose_xyz_m` | `null` | 단일 기본 토출구 중심 실측값이 필요할 경우 |
| `outlet.pose_rpy_rad` | `null` | 위 outlet frame 방향 |
| `outlet.clearance_m` | `null` | 컵 rim/outlet 안전 간격 |
| `dispenser_outlets."1".."4".outlet_pose_xyz_m` | `null` | 각 디스펜서 토출구 중심, `base_link` 기준 |
| `dispenser_outlets."1".."4".outlet_pose_rpy_rad` | `null` | 각 토출구 정렬 방향 |
| `dispenser_outlets."1".."4".press_pose_xyz_m` | `null` | 각 디스펜서 누름/작동 pose, `base_link` 기준 |
| `dispenser_outlets."1".."4".press_pose_rpy_rad` | `null` | 각 press pose 방향 |
| `dispenser_outlets."1".."4".clearance_m` | `null` | 각 디스펜서별 rim/outlet 안전 간격 |

## 이미 파일에 있는 값

`calibration.yaml`의 `cup_holder` 블록에는 실측 초안이 들어 있다.

- `bottom_insert_center_pose_xyz_m`: `[0.429831, 0.220885, 0.017898]`
- `bottom_insert_center_pose_rpy_rad`: `[2.513735, -3.122200, 3.038293]`
- `top_center_estimated_xyz_m`: `[0.429831, 0.220885, 0.079500]`
- `side_grip_place.pre_place_pose_xyz_m`: `[0.425545, 0.251832, 0.217371]`
- `side_grip_place.place_final_pose_xyz_m`: `[0.425545, 0.251832, 0.117371]`
- `side_grip_place.retreat_pose_xyz_m`: `[0.415096, 0.189680, 0.113925]`
- `side_grip_place.status`: `measured_draft`

이 값은 컵홀더 배치용 측정 초안이다. 컵의 현재 위치를 의미하지 않는다.

## 별도 실측 파일에 있는 디스펜서 front-hold 값

`src/azas_bringup/config/measured_dispenser_collision.yaml`에는 direct teaching으로 기록한 `base_link -> link_6` front-hold pose가 있다.

- `front_hold_poses.dispenser_1.position_xyz_m`: `[0.437710, 0.041751, 0.077310]`
- `front_hold_poses.dispenser_2.position_xyz_m`: `[0.437119, 0.007800, 0.081750]`
- `front_hold_poses.dispenser_3.position_xyz_m`: `[0.436445, -0.028198, 0.088599]`
- `front_hold_poses.dispenser_4.position_xyz_m`: `[0.437010, -0.083524, 0.087128]`

주의: 이 값은 컵을 들고 디스펜서 앞에 서는 `link_6` 포즈다. `calibration.yaml`의 `dispenser_outlets.<id>.outlet_pose_*` 또는 `press_pose_*`에 그대로 복사하면 안 된다.

## 컵 좌표 정책

컵 좌표는 `calibration.yaml`에 고정값으로 쓰지 않는다. 실제 컵 위치는 비전 파이프라인이 `/jarvis/tumbler_dispenser/tumbler_pose`로 제공해야 한다.

필수 조건:

- `/azas/cup_detection` status가 `detected:upright`로 시작해야 한다.
- `cup_detection_pose_bridge_node`가 TF2로 `base_link` 기준 pose를 publish해야 한다.
- TF 실패, non-upright, stale detection이면 모션 pose를 publish하지 않는 것이 정상이다.

## 절대 채우면 안 되는 값

- `example_static_tf_do_not_use_on_real_robot` 값은 실제 hand-eye 값이 아니다.
- smoke test fixture의 디스펜서 좌표는 실제 캘리브레이션 값이 아니다.
- `measured_dispenser_collision.yaml`의 collision box estimate는 outlet/press pose가 아니다.
- 위키/LLM이 “그럴듯한” 좌표를 생성해서 `null`을 대체하면 안 된다.

## 현장 측정 순서

1. live camera topic과 `CameraInfo.header.frame_id` 확인.
2. MoveIt robot model에서 planning group과 end-effector link 확인.
3. RG2 TCP frame 또는 Doosan current TCP 설정 확인.
4. hand-eye 캘리브레이션으로 `base_link -> camera_frame` transform 기록.
5. 컵 그립 후 `tcp_to_cup_mouth_m` 측정.
6. 디스펜서 1-4 각각 outlet pose와 press pose를 teaching/calibration으로 기록.
7. 각 outlet clearance와 workspace/safety 값을 측정해 `safety.yaml`도 채운다.
8. strict gate를 통과하기 전까지 real motion은 차단한다.

## 검증 명령

```bash
bash tools/checks/check_hand_eye_readiness.sh
bash tools/checks/check_real_motion_config.sh
bash tools/checks/check_measured_dispenser_geometry.py
STRICT_LIVE_GATE=true RUN_LID_STABILITY=true RUN_CUP_STABILITY=true /home/ssu/Azas/tools/run/field_no_motion_report.sh
```

통과 기준은 `calibration.yaml`과 `safety.yaml`에 `null`/`확인 필요`가 남지 않고, live camera/TF/Doosan/RG2 증거가 함께 확인되는 것이다.
