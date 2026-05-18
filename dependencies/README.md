# dependencies/

Azas는 외부 소스 트리를 직접 포함하지 않습니다.  
외부 ROS 2 패키지는 `vcs` 매니페스트(`.repos` 파일)로 관리합니다.

---

## 파일 목록

| 파일 | 내용 |
|------|------|
| `ros2_sources.repos` | Doosan, MoveIt2, RealSense 등 ROS 2 소스 후보 |
| `dsr_deeptree_sources.repos` | DSR DeepTree 데모 소스 (리뷰 전용) |
| `experimental_sources.repos` | 실험적·고위험 소스 후보 |
| `python_optional_requirements.txt` | YOLO, STT 등 선택적 Python 의존성 |
| `system_apt_packages.txt` | 시스템 apt 패키지 후보 |

---

## 외부 소스를 벤더링하지 않는 이유

- Azas diff가 프로젝트 코드와 통합 로직에만 집중됩니다.
- 대형 외부 소스 트리를 실수로 커밋하는 것을 방지합니다.
- 업스트림 라이선스와 커밋 출처가 명확하게 유지됩니다.
- 메인 ROS 워크스페이스에 포함하기 전 `/tmp` 검토 워크스페이스에서 먼저 테스트할 수 있습니다.

---

## 리뷰 임포트 명령어

> **반드시 임시 검토 워크스페이스에서 먼저 실행하세요. `/home/ssu/Azas`에 직접 임포트하지 마세요.**

```bash
# ROS 2 소스 후보 검토
mkdir -p /tmp/azas_oss_review/src
cd /tmp/azas_oss_review
vcs import src < /home/ssu/Azas/dependencies/ros2_sources.repos
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install

# 실험적 소스 검토
mkdir -p /tmp/azas_oss_review_experimental/src
cd /tmp/azas_oss_review_experimental
vcs import src < /home/ssu/Azas/dependencies/experimental_sources.repos

# DSR DeepTree 데모 검토
mkdir -p /tmp/azas_demo_review/src
cd /tmp/azas_demo_review
vcs import src < /home/ssu/Azas/dependencies/dsr_deeptree_sources.repos
```

현재 데모 소스: `deeptree0819/DSR_DeepTree` 커밋 `22f5435086037a759e563047f535f2c3c418351e`

---

## Python 의존성 설치

```bash
python3 -m pip install -r /home/ssu/Azas/dependencies/python_optional_requirements.txt
```

`system_apt_packages.txt` 의 패키지는 로봇 PC 이미지(Ubuntu/ROS 버전)를 확인한 후 설치하세요.

---

## 검토 순서

1. **Doosan ROS 2** — Humble 브랜치, M0609 모델 문자열, 가상 에뮬레이터, `dsr_bringup2` MoveIt 런치, 라이선스 파일 확인
2. **MoveIt 2 / MoveItPy** — 일반 개발은 바이너리 패키지 우선, API/설정 검토 시에만 소스 임포트
3. **RealSense ROS 2 래퍼** — 카메라 모델, 스트림 프로파일, 토픽 이름, `CameraInfo`, 깊이 스케일, 프레임 ID 확인
4. **AprilTag ROS 2** — `image_rect`/`camera_info` 리맵과 마커 TF 네이밍 확인
5. **easy_handeye2** — Humble 빌드 확인 및 프리핸드 샘플링 워크플로우 검토
6. **OnRobot ROS2 드라이버** — 실험적. 가짜 하드웨어 우선 테스트 후 Modbus 연결, 속도/힘 제한, 정지 동작, 실패 처리 문서화 필수
7. **DSR_DeepTree 데모** — Task 1 액션 시퀀싱, Task 2 YOLO/핸드아이 변환, Task 3 STT 명령 매핑, Task 4 Isaac Sim 에셋 검토. 패턴만 포팅, 전체 트리 벤더링 금지

---

## 그라스프 탐지기 실험 현황 (2026-05-13)

Azas에 그라스프 탐지기 소스는 포함되지 않았습니다.  
RTX 5080 (16 GB VRAM) 환경 기준 추천 설치 순서:

1. `graspnet/graspnet-baseline` — 별도 conda 환경, RealSense 사전학습 체크포인트 사용 (라이선스 확인 후)
2. `NVlabs/contact_graspnet` — GraspNet 결과가 불량할 경우 (구버전 Python/TensorFlow/CUDA 환경 위험 감수)
3. `graspnet/anygrasp_sdk` — 라이선스 등록 승인 후에만 진행
4. `atenpas/gpd` / `atenpas/gpd_ros` — CPU/PCL 폴백 (`gpd_ros`는 ROS1 catkin, ROS2 Humble 패키지가 아님)

실험 결과물은 `/tmp` 또는 외부 검토 워크스페이스에만 보관하세요.

---

## 안전 게이트

하드웨어 모션에 영향을 줄 수 있는 의존성은 Azas 통합 노트에 다음을 반드시 문서화해야 합니다:

- 안전 가정 및 운영자 제어 방법
- 속도/가속도/힘 제한
- 예상 실패 동작
- 가상/가짜 하드웨어 검증 단계
- 실제 하드웨어 전환 기준

---

## 비-하드웨어 스택 점검

Azas와 `ros2_ws` 브릿지 패키지 빌드 후 실행하세요:

```bash
bash /home/ssu/Azas/tools/checks/check_oss_stack.sh
```

카메라, RG2, 실제 로봇 모션 없이 ROS 패키지 가용성, 런치 파일, 선택적 Python 임포트, YOLO 모델 경로를 점검합니다.
