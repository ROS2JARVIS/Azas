
import math
import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


PACKAGE_NAME = "azas_cup_uprighting"
PKG_SHARE = Path(get_package_share_directory(PACKAGE_NAME))
WORKSPACE_ROOT = Path(__file__).resolve().parents[3]


def load_yaml(file_name):
    file_path = PKG_SHARE / "config" / file_name
    with open(file_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


try:
    SAFETY_CFG = load_yaml('safety.yaml')
except Exception as e:
    print(f"[경고] safety.yaml을 불러오지 못했습니다: {e}")
    SAFETY_CFG = None


try:
    DISPENSER_CFG = load_yaml('measured_dispenser_collision.yaml')
except Exception as e:
    print(f"[경고] measured_dispenser_collision.yaml을 불러오지 못했습니다: {e}")
    DISPENSER_CFG = None


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


def _env_bool(name, default=False):
    value = os.getenv(name)
    if value is None:
        return default
    return value.lower() in {"1", "true", "yes", "on"}


def _workspace_bounds():
    if SAFETY_CFG and "motion" in SAFETY_CFG:
        bounds = SAFETY_CFG["motion"].get("workspace_bounds_m", {})
        return {
            "x_min": float(bounds.get("x_min", -0.250)),
            "x_max": float(bounds.get("x_max", 1.150)),
            "y_min": float(bounds.get("y_min", -0.600)),
            "y_max": float(bounds.get("y_max", 0.600)),
            "z_min": float(bounds.get("z_min", 0.070)),
            "z_max": float(bounds.get("z_max", 0.800)),
        }
    return {
        "x_min": -0.250,
        "x_max": 1.150,
        "y_min": -0.600,
        "y_max": 0.600,
        "z_min": 0.070,
        "z_max": 0.800,
    }


# ── 안전 작업 영역 (m, base_link) ────────────────────
_BOUNDS = _workspace_bounds()
SAFE_X_MIN = _BOUNDS["x_min"]
SAFE_X_MAX = _BOUNDS["x_max"]
SAFE_Y_MIN = _BOUNDS["y_min"]
SAFE_Y_MAX = _BOUNDS["y_max"]
SAFE_Z_MIN = _BOUNDS["z_min"]
SAFE_Z_MAX = _BOUNDS["z_max"]

# ── 속도/가속도 스케일 ───────────────────────────────
MAX_VEL_SCALE = float(os.getenv("YOLO_MAX_VEL_SCALE", "0.1"))
MAX_ACC_SCALE = float(os.getenv("YOLO_MAX_ACC_SCALE", "0.1"))

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
def _default_yolo_model_path():
    candidates = [
        Path("/home/ssu/Azas/local_models/best.pt"),
        WORKSPACE_ROOT / "local_models" / "best.pt",
        WORKSPACE_ROOT / "best.pt",
        PKG_SHARE / "config" / "best.pt",
        PKG_SHARE / "best.pt",
    ]
    for path in candidates:
        if path.exists():
            return str(path)
    return str(candidates[0])


YOLO_MODEL_PATH = os.getenv("YOLO_MODEL_PATH", _default_yolo_model_path())
YOLO_CONF_THRESH   = 0.5
AUTO_PICK_INTERVAL = 3.0    # 자동 모드 픽 간격 [s]
AUTO_PICK_ENABLED = _env_bool("YOLO_AUTO_PICK", False)
EXIT_AFTER_PICK = _env_bool("YOLO_EXIT_AFTER_PICK", False)
PREVIEW_ONLY = _env_bool("YOLO_PREVIEW_ONLY", True)
SHOW_WINDOW = _env_bool("YOLO_SHOW_WINDOW", False)

# ── 카메라 토픽 ──────────────────────────────────────
TOPIC_CAM_INFO  = os.getenv("YOLO_TOPIC_CAM_INFO", "/camera/camera/color/camera_info")
TOPIC_COLOR     = os.getenv("YOLO_TOPIC_COLOR", "/camera/camera/color/image_raw")
TOPIC_DEPTH     = os.getenv("YOLO_TOPIC_DEPTH", "/camera/camera/aligned_depth_to_color/image_raw")
TOPIC_DEBUG_IMAGE = os.getenv("YOLO_TOPIC_DEBUG_IMAGE", "/yolo_cup_uprighting/debug_image")
