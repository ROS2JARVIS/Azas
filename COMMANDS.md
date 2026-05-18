# Azas 명령어 빠른 참조

> Doosan M0609 + OnRobot RG2 + RealSense 칵테일 로봇  
> 현장 투입 전 이 순서대로 진행하세요.

---

## 목차

1. [환경 설정](#1-환경-설정)
2. [빌드 · 테스트](#2-빌드--테스트)
3. [시뮬레이션 (가상 로봇)](#3-시뮬레이션-가상-로봇)
4. [비-하드웨어 점검](#4-비-하드웨어-점검)
5. [카메라 연결](#5-카메라-연결)
6. [드라이런 (Dry-run)](#6-드라이런-dry-run)
7. [실제 로봇 운용](#7-실제-로봇-운용)
8. [스모크 테스트](#8-스모크-테스트)
9. [TF · 토픽 디버그](#9-tf--토픽-디버그)
10. [그라스프 프레임 수집](#10-그라스프-프레임-수집)
11. [전원 차단 복구](#11-전원-차단-복구)

---

## 1. 환경 설정

```bash
# ROS 2 Humble 소싱 (매 터미널 시작 시)
source /opt/ros/humble/setup.bash

# Azas 워크스페이스 소싱 (빌드 후)
source /home/ssu/Azas/install/setup.bash

# ros2_ws 소싱 (Doosan 드라이버 필요 시)
source /home/ssu/ros2_ws/install/setup.bash
```

> **팁**: `~/.bashrc`에 `source /opt/ros/humble/setup.bash` 추가하면 편합니다.

---

## 2. 빌드 · 테스트

```bash
cd /home/ssu/Azas

# 전체 빌드
colcon build --symlink-install

# 특정 패키지만 빌드
colcon build --symlink-install --packages-select azas_perception azas_interfaces

# 전체 테스트
colcon test
colcon test-result --verbose

# 특정 패키지 테스트
colcon test --packages-select azas_voice
```

---

## 3. 시뮬레이션 (가상 로봇)

```bash
# 가상 Doosan M0609 시작
bash tools/run/run_doosan_virtual_m0609.sh

# MoveIt + RViz 수동 실행
source /home/ssu/ros2_ws/install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
  model:=m0609 mode:=virtual host:=127.0.0.1 port:=12345
```

---

## 4. 비-하드웨어 점검

```bash
# OSS 스택 전체 점검 (패키지·런치·의존성)
bash tools/checks/check_oss_stack.sh

# 제어 준비도 종합 점검
bash tools/checks/verify_control_readiness.sh

# 실제 모션 차단 요인 설명
bash tools/checks/explain_real_robot_blockers.sh

# 실제 모션 설정 점검
bash tools/checks/check_real_motion_config.sh

# 연결 단계 결정 (다음에 무엇을 연결해야 하는지)
bash tools/checks/check_connection_stage.sh

```

---

## 5. 카메라 연결

```bash
# RealSense D435i 시작
ros2 launch realsense2_camera rs_align_depth_launch.py

# 카메라 토픽 확인
ros2 topic list | grep camera
ros2 topic echo --once /camera/camera/color/camera_info

# 깊이 인코딩 확인
ros2 topic echo --once /camera/camera/aligned_depth_to_color/image_raw | grep encoding

# 카메라 TF 확인
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

# YOLO 탐지 확인
bash tools/checks/check_robot_detection.sh
```

모션으로 이어지는 컵 탐지는 `/azas/cup_detection` status가 `detected:upright`로 시작해야 합니다. `detected:lid`는 lid 확인용이며 `/jarvis/tumbler_dispenser/tumbler_pose`의 컵 pose 계약을 만족하지 않습니다.

---

## 6. 드라이런 (Dry-run)

드라이런은 실제 모션 없이 전체 파이프라인을 검증합니다.

```bash
# 카메라 기반 드라이런 전체 파이프라인
bash tools/run/run_robot_dryrun.sh

# 칵테일 드라이런 시퀀스
ros2 launch azas_bringup cocktail_dryrun.launch.py

# 현장 비-모션 종합 보고서
bash tools/run/field_no_motion_report.sh
```

---

## 7. 실제 로봇 운용

> **경고**: 실제 로봇 연결 전 반드시 아래 게이트를 통과해야 합니다.

### 7-0. 실로봇 테스트 사다리

한 번에 full cocktail을 실행하지 말고 아래 staged ladder로 실패 지점을 좁힙니다.

```bash
# 현재 차단 원인 설명
STAGE=status tools/run/run_real_robot_test_ladder.sh

# strict live gate까지 통과해야 실제 one-shot 테스트 가능
STAGE=live-gate RUN_LID_STABILITY=true RUN_CUP_STABILITY=true \
  tools/run/run_real_robot_test_ladder.sh

# 마지막 supervised one-shot 실제 pick
STAGE=pick-real CONFIRM=I_UNDERSTAND_THIS_WILL_MOVE_THE_ROBOT \
  tools/run/run_real_robot_test_ladder.sh
```

상세 절차 → `docs/real_robot_test_ladder.md`

### 7-1. 로봇 네트워크 연결

```bash
# 로봇 서브넷 IP 임시 추가 (기본 서브넷)
sudo ip addr add 192.168.137.50/24 dev enp128s31f6
ping 192.168.137.100

# 또는 대체 서브넷
sudo ip addr add 192.168.127.50/24 dev enp128s31f6
ping 192.168.127.100
```

### 7-2. 실제 로봇 MoveIt 연결

```bash
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash

ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
  mode:=real model:=m0609 host:=192.168.137.100 port:=12345
```

### 7-3. RG2 그리퍼 서비스 시작

```bash
source /home/ssu/ros2_ws/install/setup.bash
ros2 launch jarvis rg2_trigger.launch.py ip:=192.168.1.1
```

비-모션 check/smoke 명령은 RG2 서비스의 존재와 타입을 확인할 수 있지만, 실제 `/jarvis/rg2/open` 또는 `/jarvis/rg2/close` 동작 검증이 아닙니다. 실제 그리퍼 actuation 증거는 별도 현장 절차로 기록해야 합니다.

### 7-4. 하드웨어 게이트 통과 확인

```bash
# 비-모션 하드웨어 게이트 점검
bash tools/checks/check_live_hardware_gates.sh

# 엄격 모드 (모든 경고 포함) + 게이트 스탬프 발급
STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed \
  bash tools/checks/check_live_hardware_gates.sh
```

### 7-5. 실제 모션 실행

```bash
# 게이트 통과 후에만 실행 가능
bash tools/run/run_robot_real.sh

# 연결 로봇 제어 (통합)
bash tools/run/run_connected_robot_control.sh
```

`run_robot_real.sh`는 strict gate stamp와 측정 config를 다시 확인한 뒤에도, operator 확인 전 `detected:upright` cup pose와 실제 camera-derived tumbler pose를 요구합니다.

## 8. 스모크 테스트

하드웨어 없이 실행 가능한 자동화 테스트입니다.

```bash
# 픽앤얼라인 액션 비-모션 스모크
bash tools/smoke/smoke_pick_and_align_no_motion.sh

# 제어 경로 엔드투엔드 스모크
bash tools/smoke/smoke_control_path.sh

# 가짜 하드웨어 서비스 스모크
bash tools/smoke/smoke_fake_hardware_path.sh

# 칵테일 드라이런 시퀀스 스모크
bash tools/smoke/smoke_cocktail_dryrun_sequence.sh

# 실제 모션 진입점 게이트 스모크
bash tools/smoke/smoke_real_motion_entrypoint_gates.sh

# 가짜 하드웨어 서비스 수동 시작 (별도 터미널)
python3 tools/smoke/fake_hardware_services.py
```

---

## 9. TF · 토픽 디버그

```bash
# TF 파이프라인 점검
bash tools/checks/check_tf_pipeline.sh

# TF 트리 시각화
ros2 run tf2_tools view_frames

# TF 에코
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

# 탐지 포즈 확인
ros2 topic echo /jarvis/tumbler_dispenser/tumbler_pose

# 활성 토픽 목록
ros2 topic list | grep -E "tf|tumbler|camera|yolo"

# 깊이 투영 샘플 점검
bash tools/checks/check_depth_projection_sample.sh
```

---

## 10. 그라스프 프레임 수집

```bash
# 기본 수집 (rgb + depth + camera_info)
python3 tools/perception/export_grasp_frame.py \
  --output /tmp/azas_grasp_frame \
  --rgb-topic /camera/camera/color/image_raw \
  --depth-topic /camera/camera/aligned_depth_to_color/image_raw \
  --camera-info-topic /camera/camera/color/camera_info \
  --timeout-sec 10

# 탐지 bbox 대기 후 수집
python3 tools/perception/export_grasp_frame.py \
  --output /tmp/azas_grasp_frame \
  --rgb-topic /camera/camera/color/image_raw \
  --depth-topic /camera/camera/aligned_depth_to_color/image_raw \
  --camera-info-topic /camera/camera/color/camera_info \
  --wait-for-bbox \
  --timeout-sec 10
```

---

## 11. 전원 차단 복구

```bash
# 복구 절차 문서
cat docs/recovery_after_poweroff.md
```

---

## 현장 투입 순서 요약

```
① 환경 소싱
   source /opt/ros/humble/setup.bash && source install/setup.bash

② 가상 Doosan 시작
   bash tools/run/run_doosan_virtual_m0609.sh

③ 비-모션 전체 점검
   bash tools/checks/verify_control_readiness.sh

④ 엄격 게이트 스탬프 발급
   STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed \
     bash tools/checks/check_live_hardware_gates.sh

⑤ `detected:upright` 컵 pose 확인 후 실제 로봇 모션 실행
   bash tools/run/run_robot_real.sh
```

---

## 자주 쓰는 ros2 명령어

```bash
# 서비스 목록
ros2 service list | grep -E "gripper|motion|azas"

# 액션 목록
ros2 action list

# 노드 목록
ros2 node list

# 패키지 실행
ros2 run azas_perception yolo_tumbler_detector_node

# 런치
ros2 launch azas_bringup mvp_bringup.launch.py
ros2 launch azas_voice azas_voice.launch.py

# 파라미터 조회
ros2 param list /azas_pick_and_align_action_server
```

---

> 문서 최종 업데이트: 2026-05-13  
> 문의: GitHub Issues → `[docs]` 라벨로 등록
