
import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory


PKG_SHARE = get_package_share_directory('azas_cup_uprighting')


def load_yaml(file_name):
    file_path = os.path.join(PKG_SHARE, 'config', file_name)
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


# ── Pick 파라미터 (m) ────────────────────────────────
Z_OFFSET = 0.20    # gripper tip ↔ link_6 (depth 측정 base z + 이 값 = pick_z)


# ── Approach (재검출 직전 EE 미세 이동) ──────────────
APPROACH_OFFSET = (-0.05, -0.05)   # (dx, dy) m, Z 는 현재 유지
APPROACH_SETTLE = 0.5              # 이동 후 카메라 안정화 [s]

# ── 그리퍼 ──────────────────────────────────────────
GRIPPER_NAME     = "rg2"
TOOLCHARGER_IP   = "192.168.1.1"
TOOLCHARGER_PORT = 502

# ── YOLO ────────────────────────────────────────────
YOLO_MODEL_PATH = os.path.join(PKG_SHARE, 'config', 'best.pt')
YOLO_CONF_THRESH   = 0.5
AUTO_PICK_INTERVAL = 3.0    # 자동 모드 픽 간격 [s]

# ── 카메라 토픽 ──────────────────────────────────────
TOPIC_CAM_INFO  = "/camera/camera/color/camera_info"
TOPIC_COLOR     = "/camera/camera/color/image_raw"
TOPIC_DEPTH     = "/camera/camera/aligned_depth_to_color/image_raw"
