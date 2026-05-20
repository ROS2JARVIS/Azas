---
title: "Measured Geometry Inventory 2026-05-20"
tags: ["azas", "geometry", "coordinates", "dispenser", "cup", "holder", "real-robot"]
created: 2026-05-20T16:12:00+09:00
updated: 2026-05-20T16:12:00+09:00
sources:
  - src/azas_bringup/config/calibration.yaml
  - src/azas_bringup/config/measured_dispenser_collision.yaml
  - docs/tumbler_dispenser_models.md
  - src/azas_motion/azas_motion/tumbler_collision_scene_node.py
  - tools/run/teach_measured_dispenser_front_hold.py
  - src/azas_calibration/azas_calibration/calibration_loader_node.py
category: real-robot-calibration
confidence: high
schemaVersion: 1
---

# Measured Geometry Inventory 2026-05-20

## 왜 한 번에 정리되지 않았나

좌표와 치수가 한 파일로 모이지 않은 이유는 저장 경로가 기능별로 쪼개져 있었기 때문이다.

- `calibration.yaml`: hand-eye, outlet, cup offset, cup-holder 같은 “최종 캘리브레이션 계약” 파일이다.
- `measured_dispenser_collision.yaml`: 직접교시로 얻은 디스펜서 앞 `base_link -> link_6` front-hold 포즈와 collision 추정값 파일이다.
- `docs/tumbler_dispenser_models.md`: 컵/디스펜서 물리 치수 문서다.
- `/jarvis/tumbler_dispenser/tumbler_pose`: 컵의 현재 위치를 담는 런타임 토픽이다. 고정 YAML 값이 아니다.

패널에서 측정한 `teach_front_hold_*` 값은 `calibration.yaml`이 아니라 `measured_dispenser_collision.yaml`에 저장되도록 구현되어 있다. `calibration_loader_node`는 아직 measured value를 YAML에 저장하지 않는 boundary-only 구현이다.

## 디스펜서 front-hold 좌표

파일: `src/azas_bringup/config/measured_dispenser_collision.yaml`

프레임/대상:

- `metadata.frame_id`: `base_link`
- `metadata.measured_target_frame`: `link_6`
- 의미: 컵을 들고 디스펜서 앞에 서는 direct-teaching pose
- 주의: 토출구 중심(outlet)이나 누름 위치(press pose)가 아니다.

| ID | position_xyz_m | quaternion_xyzw | rpy_deg |
| --- | --- | --- | --- |
| dispenser_1 | `[0.437710, 0.041751, 0.077310]` | `[0.458109, 0.520842, 0.538998, 0.477851]` | `[87.838, 0.225, 97.099]` |
| dispenser_2 | `[0.437119, 0.007800, 0.081750]` | `[0.500394, 0.482826, 0.508624, 0.507727]` | `[88.110, -1.074, 89.062]` |
| dispenser_3 | `[0.436445, -0.028198, 0.088599]` | `[0.526785, 0.467746, 0.481494, 0.521416]` | `[89.575, -1.118, 84.331]` |
| dispenser_4 | `[0.437010, -0.083524, 0.087128]` | `[0.509621, 0.485335, 0.481655, 0.522249]` | `[89.457, 0.917, 86.277]` |

## 디스펜서 collision/probe 참고값

같은 파일의 `raw_probe_poses`와 `estimated_collision_objects`에는 디스펜서 body/head/nozzle collision 추정값이 있다. 이 값은 planning scene 또는 RViz 확인용이며, `calibration.yaml`의 `dispenser_outlets.<id>.outlet_pose_*`에 넣을 값이 아니다.

## 아직 비어 있는 디스펜서 최종 캘리브레이션

파일: `src/azas_bringup/config/calibration.yaml`

아래는 아직 모두 `null`이다.

- `outlet.pose_xyz_m`
- `outlet.pose_rpy_rad`
- `outlet.clearance_m`
- `dispenser_outlets."1".."4".outlet_pose_xyz_m`
- `dispenser_outlets."1".."4".outlet_pose_rpy_rad`
- `dispenser_outlets."1".."4".press_pose_xyz_m`
- `dispenser_outlets."1".."4".press_pose_rpy_rad`
- `dispenser_outlets."1".."4".clearance_m`

이 값들은 front-hold `link_6` 포즈와 의미가 달라서 자동으로 채워지지 않았다.

## 컵 치수

출처: `docs/tumbler_dispenser_models.md`, `tumbler_collision_scene_node.py`

| 항목 | 값 | 상태 |
| --- | --- | --- |
| tumbler diameter | `0.075 m` | 문서/모델 기준 |
| tumbler radius | `0.0375 m` | 문서/모델 기준 |
| lidded height | `0.170 m` | 문서/모델 기준 |
| lidless body height | `0.140 m` | 문서/모델 기준 |
| collision radius margin | `0.006 m` | scene node 기본값 |
| collision height margin | `0.010 m` | scene node 기본값 |

주의: 일부 pick/alignment 기본값에는 `cup_radius_m=0.035` 같은 보수/레거시 기본값이 남아 있다. canonical physical model 값은 위의 `0.0375 m` radius다.

## 컵 현재 좌표

컵의 현재 좌표는 파일에 저장하지 않는다.

- 생산 토픽: `/jarvis/tumbler_dispenser/tumbler_pose`
- 기준 프레임: `base_link`
- 생산 조건: `/azas/cup_detection`이 `detected:upright`이고 TF 변환이 성공해야 한다.

즉 컵 위치는 매번 카메라/TF에서 나오는 런타임 값이다.

## TCP-to-cup offset

파일: `src/azas_bringup/config/calibration.yaml`

- `cup_offsets.default.tcp_to_cup_mouth_m`: `null`
- 상태: 아직 최종 YAML에 없음
- 필요한 증거: 그립 후 TCP에서 컵 입구 중심까지 jig/dry-run 실측

## 컵홀더 위치와 치수

파일: `src/azas_bringup/config/calibration.yaml`

프레임: `base_link`

| 항목 | 값 | 상태 |
| --- | --- | --- |
| `bottom_insert_center_pose_xyz_m` | `[0.429831, 0.220885, 0.017898]` | measured_draft |
| `bottom_insert_center_pose_rpy_rad` | `[2.513735, -3.122200, 3.038293]` | measured_draft |
| `top_center_estimated_xyz_m` | `[0.429831, 0.220885, 0.079500]` | estimated from holder height |
| `radius_m` | `0.045` | holder geometry |
| `height_m` | `0.062` | holder geometry |

## 컵홀더 side-grip 배치 포즈

파일: `src/azas_bringup/config/calibration.yaml`

| 단계 | xyz_m | rpy_rad | 상태 |
| --- | --- | --- | --- |
| pre_place | `[0.425545, 0.251832, 0.217371]` | `[1.600936, 1.622299, 2.132633]` | measured_draft |
| place_final | `[0.425545, 0.251832, 0.117371]` | `[1.600936, 1.622299, 2.132633]` | measured_draft |
| retreat | `[0.415096, 0.189680, 0.113925]` | `[1.608926, 1.585807, 2.146282]` | measured_draft |

기타:

- `approach_lift_m`: `0.100`
- `solution_space`: `2.0`
- `status`: `measured_draft`

## 남은 정리 작업

1. `calibration.yaml`에 넣을 outlet/press pose를 front-hold와 별개로 기록한다.
2. `cup_offsets.default.tcp_to_cup_mouth_m`을 실측한다.
3. hand-eye `base_link -> camera_frame` transform을 채운다.
4. `safety.yaml`의 workspace, min z, gripper width/force도 실측으로 채운다.
5. `calibration.yaml` 저장 helper를 만들면, 다음부터 측정값이 흩어지지 않는다.
