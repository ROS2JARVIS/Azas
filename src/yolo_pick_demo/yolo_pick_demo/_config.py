"""공통 상수.

모든 MoveIt 기반 노드가 공유하는 로봇/그리퍼/카메라/YOLO 설정.
노드별 고유 상수(슬롯 위치, scan offset 등)는 각 노드 파일에 둔다.
"""

import os
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


# ── MoveIt ─────────────────────────────────────────
GROUP_NAME = "manipulator"
BASE_FRAME = "base_link"
EE_LINK    = "link_6"

HOME_JOINTS = {
    "joint_1": math.radians(3.0),
    "joint_2": math.radians(-12.7),
    "joint_3": math.radians(44.0),
    "joint_4": math.radians(-9.0),
    "joint_5": math.radians(133.0),
    "joint_6": math.radians(90.0),
}

# ── 안전 작업 영역 (m, base_link) ────────────────────
SAFE_X_MIN = -0.250
SAFE_X_MAX =  1.150
SAFE_Y_MIN = -0.600
SAFE_Y_MAX =  0.600
SAFE_Z_MIN =  0.070
SAFE_Z_MAX =  0.800

# ── 속도/가속도 스케일 (dry-run 안전 제한) ────────────
MAX_VEL_SCALE = 0.1
MAX_ACC_SCALE = 0.1

# ── Pick 파라미터 (m) ────────────────────────────────
Z_OFFSET = 0.20    # gripper tip ↔ link_6 (depth 측정 base z + 이 값 = pick_z)
SAFE_Z   = 0.40    # 안전 이동 높이

# ── Approach (재검출 직전 EE 미세 이동) ──────────────
APPROACH_OFFSET = (-0.05, -0.05)   # (dx, dy) m, Z 는 현재 유지
APPROACH_SETTLE = 0.5              # 이동 후 카메라 안정화 [s]

# ── 그리퍼 ──────────────────────────────────────────
GRIPPER_NAME     = "rg2"
TOOLCHARGER_IP   = "192.168.1.1"
TOOLCHARGER_PORT = 502

# ── YOLO ────────────────────────────────────────────
def _env_bool(name, default=False):
    value = os.getenv(name)
    if value is None:
        return default
    return value.lower() in {"1", "true", "yes", "on"}


def _default_yolo_model_path():
    candidates = [Path(__file__).resolve().parents[3] / "best.pt"]
    try:
        candidates.append(Path(get_package_share_directory("yolo_pick_demo")) / "best.pt")
    except Exception:
        pass

    for path in candidates:
        if path.exists():
            return str(path)
    return str(candidates[0])


YOLO_MODEL_PATH = os.getenv("YOLO_MODEL_PATH", _default_yolo_model_path())
YOLO_CONF_THRESH   = 0.5
AUTO_PICK_INTERVAL = 3.0    # 자동 모드 픽 간격 [s]
PREVIEW_ONLY = _env_bool("YOLO_PREVIEW_ONLY", False)

# ── 카메라 토픽 ──────────────────────────────────────
TOPIC_CAM_INFO  = os.getenv("YOLO_TOPIC_CAM_INFO", "/camera/camera/color/camera_info")
TOPIC_COLOR     = os.getenv("YOLO_TOPIC_COLOR", "/camera/camera/color/image_raw")
TOPIC_DEPTH     = os.getenv("YOLO_TOPIC_DEPTH", "/camera/camera/aligned_depth_to_color/image_raw")
TOPIC_DEBUG_IMAGE = os.getenv("YOLO_TOPIC_DEBUG_IMAGE", "/yolo_cup_uprighting/debug_image")
