# Azas Agent Guide

ROS 2 Humble 기반 칵테일 로봇 프로젝트입니다.

---

## 컵 좌표는 절대 직접 묻지 않습니다

**이것이 가장 중요한 규칙입니다.**

컵의 위치·자세 정보는 아래 비전 파이프라인이 자동으로 생성합니다.  
에이전트는 좌표를 사람에게 요구하거나, 하드코딩하거나, 임의로 생성해서는 안 됩니다.

### 컵 좌표 공급 파이프라인

```
카메라 (RealSense D435i)
  └─ /camera/camera/color/image_raw
  └─ /camera/camera/aligned_depth_to_color/image_raw
       │
       ▼
yolo_tumbler_detector_node          (src/azas_perception/)
  └─ 탐지: 직립 텀블러만 통과 (bbox 비율 height/width >= 1.2)
  └─ 발행: /azas/cup_detection  (azas_interfaces/msg/CupDetection)
       │
       ▼
cup_detection_pose_bridge_node      (src/azas_perception/)
  └─ 카메라 프레임 → base_link 변환 (TF2)
  └─ 발행: /jarvis/tumbler_dispenser/tumbler_pose  (geometry_msgs/PoseStamped)
       │
       ▼
PickAndAlignActionServer            (src/azas_task_manager/)
  └─ /jarvis/tumbler_dispenser/tumbler_pose 구독
  └─ PoseStamped.header.frame_id == "base_link" 검증 후 모션 계획
```

### 에이전트가 따라야 할 규칙

| 상황 | 올바른 행동 |
|------|------------|
| 컵 위치가 필요한 코드 작성 | `/jarvis/tumbler_dispenser/tumbler_pose` 토픽을 구독하는 코드 작성 |
| 테스트/디버그용 좌표 필요 | `tools/smoke/fake_hardware_services.py` 또는 시뮬레이션 사용 |
| 좌표가 NULL/미확정인 경우 | `calibration.yaml`의 해당 필드가 `null`임을 알리고 실측 전 진행 불가 표시 |
| 캘리브레이션 값 생성 요청 | 거부. 실측 데이터만 허용. |

### 절대 하지 말아야 할 것

```python
# 금지: 에이전트가 좌표를 직접 제공하거나 요구
cup_x = 0.42  # 하드코딩 금지
cup_pose = ask_user_for_coordinates()  # 사람에게 요구 금지
cup_pose = llm.generate_pose()  # LLM이 좌표 생성 금지
```

```python
# 올바른 방법: 파이프라인 토픽 구독
self.create_subscription(
    PoseStamped,
    "/jarvis/tumbler_dispenser/tumbler_pose",
    self._on_tumbler_pose,
    10,
)
```

---

## 프로젝트 규칙

- `build/`, `install/`, `log/`, `.omx/`, `.agents/`, `.codex/` 커밋 금지 (런타임 생성 파일)
- **LLM/VLA 역할 범위**: 사용자 의도 파악, 레시피 선택만 담당
  - 로봇 좌표, 궤적, 충돌 판단, 캘리브레이션 값 → **절대 생성 금지**
- 하드웨어 영향 코드 변경 시: 안전 가정, 속도 제한, 실패 동작, 검증 절차를 문서화
- PR은 모듈별로 분리: vision, calibration, motion, gripper, voice, bringup, integration

## 토픽 · 서비스 · 액션 참조

| 이름 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/azas/cup_detection` | `azas_interfaces/msg/CupDetection` | 발행 | YOLO 탐지 결과 |
| `/jarvis/tumbler_dispenser/tumbler_pose` | `geometry_msgs/PoseStamped` | 발행 | base_link 기준 컵 자세 |
| `/azas/pick_and_align` | `azas_interfaces/action/PickAndAlign` | 액션 서버 | 픽앤얼라인 실행 |
| `/jarvis/rg2/open` | `std_srvs/srv/Trigger` | 서비스 | 그리퍼 열기 |
| `/jarvis/rg2/close` | `std_srvs/srv/Trigger` | 서비스 | 그리퍼 닫기 |
| `/stt_result` | `std_msgs/msg/String` | 구독 | STT 음성 인식 결과 |

## 캘리브레이션 값 정책

`src/azas_bringup/config/calibration.yaml`에서 `null` 또는 `확인 필요`로 표시된 항목은  
**실측 완료 전 절대 수정하지 않습니다.**  
시뮬레이션용 임시 TF 값은 실제 로봇에서 사용하지 않습니다.

## 비-모션 점검 명령어

```bash
# 컵 탐지 확인
bash tools/checks/check_robot_detection.sh

# TF 파이프라인 점검
bash tools/checks/check_tf_pipeline.sh

# 전체 제어 준비도 점검
bash tools/checks/verify_control_readiness.sh
```

전체 명령어 → [COMMANDS.md](COMMANDS.md)
