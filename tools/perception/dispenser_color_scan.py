#!/usr/bin/env python3
"""Scan dispenser positions to build a color→dispenser_id map.

Modes:
  --image-dir <dir>  : classify dispenser_1.png ~ dispenser_4.png from a directory
  --ros              : subscribe to camera + TF, project each dispenser's 3D position to pixel,
                       crop and classify. Requires robot connected with TF publishing.
  (default)          : fail with usage hint if neither flag is given

Output: {"1": "red", "2": "blue", ...} written to --output (default: outputs/dispenser_color_map.json)
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from tools.perception.color_discrimination import (  # noqa: E402
    bgr_patch_for_color,
    classify_bgr_crop,
    read_bgr,
)

try:
    import cv2  # type: ignore
except Exception:
    cv2 = None

CALIBRATION_PATH = ROOT / "src" / "azas_bringup" / "config" / "calibration.yaml"
HAND_EYE_PATH = ROOT / "src" / "dsr_practice" / "config" / "T_gripper2camera.npy"
DEFAULT_OUTPUT = ROOT / "outputs" / "dispenser_color_map.json"
DISPENSER_IDS = ("1", "2", "3", "4")
CAMERA_TOPIC = "/camera/camera/color/image_raw"
CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"
BASE_FRAME = "base_link"
EE_LINK = "link_6"
CROP_HALF_PX = 60  # half-side of crop box around projected pixel


def load_dispenser_ids() -> list[str]:
    """Return dispenser IDs from calibration.yaml, falling back to 1-4."""
    try:
        import yaml  # type: ignore
        with CALIBRATION_PATH.open() as f:
            data = yaml.safe_load(f)
        outlets = data.get("dispenser_outlets") or {}
        ids = sorted(str(k) for k in outlets.keys())
        return ids if ids else list(DISPENSER_IDS)
    except Exception:
        return list(DISPENSER_IDS)


def load_dispenser_positions() -> dict[str, list[float]]:
    """Return {dispenser_id: [x, y, z]} in base_link metres from calibration.yaml."""
    try:
        import yaml  # type: ignore
        with CALIBRATION_PATH.open() as f:
            data = yaml.safe_load(f)
        outlets = data.get("dispenser_outlets") or {}
        result = {}
        for k, v in outlets.items():
            xyz = v.get("outlet_pose_xyz_m")
            if xyz:
                result[str(k)] = list(xyz)
        return result
    except Exception:
        return {}


def load_hand_eye() -> "np.ndarray | None":
    """Load T_gripper2camera (4x4, translation in mm → convert to m)."""
    try:
        import numpy as np  # type: ignore
        T = np.load(str(HAND_EYE_PATH)).astype(float)
        T[:3, 3] /= 1000.0
        return T
    except Exception as exc:
        print(f"[dispenser_color_scan] WARNING: could not load hand-eye: {exc}", file=sys.stderr)
        return None


def project_base_point_to_pixel(
    xyz_base: list[float],
    T_base2ee: "np.ndarray",
    T_gripper2cam: "np.ndarray",
    fx: float, fy: float, cx: float, cy: float,
) -> tuple[int, int] | None:
    """Project a 3D point in base_link to a camera pixel.

    T_base2ee: 4x4 transform from base_link to EE (link_6), i.e. FK result.
    T_gripper2cam: 4x4 from gripper frame to camera frame (hand-eye).
    Returns (u, v) pixel or None if point is behind camera.
    """
    import numpy as np  # type: ignore
    p_base = np.array([xyz_base[0], xyz_base[1], xyz_base[2], 1.0])
    # base_link → link_6 frame
    T_ee2base = np.linalg.inv(T_base2ee)
    p_ee = T_ee2base @ p_base
    # link_6 frame → camera frame
    p_cam = T_gripper2cam @ p_ee
    if p_cam[2] <= 0.01:
        return None
    u = int(round(fx * p_cam[0] / p_cam[2] + cx))
    v = int(round(fy * p_cam[1] / p_cam[2] + cy))
    return u, v


def classify_image_file(path: Path) -> str:
    img = read_bgr(path)
    result = classify_bgr_crop(img)
    return result.color


def scan_from_image_dir(image_dir: Path) -> dict[str, str]:
    dispenser_ids = load_dispenser_ids()
    color_map: dict[str, str] = {}
    for did in dispenser_ids:
        img_path = image_dir / f"dispenser_{did}.png"
        if not img_path.exists():
            # try jpg fallback
            img_path = image_dir / f"dispenser_{did}.jpg"
        if not img_path.exists():
            print(f"[dispenser_color_scan] WARNING: image not found for dispenser {did}: {img_path}", file=sys.stderr)
            color_map[did] = "unknown"
            continue
        color = classify_image_file(img_path)
        color_map[did] = color
        print(f"[dispenser_color_scan] dispenser {did}: {color} (from {img_path.name})")
    return color_map


def scan_from_ros() -> dict[str, str]:
    try:
        import rclpy  # type: ignore
        from rclpy.qos import qos_profile_sensor_data  # type: ignore
        from sensor_msgs.msg import Image, CameraInfo  # type: ignore
        import tf2_ros  # type: ignore
        import numpy as np  # type: ignore
        from geometry_msgs.msg import TransformStamped  # type: ignore
    except ImportError as exc:
        print(f"[dispenser_color_scan] rclpy not available: {exc}", file=sys.stderr)
        print("[dispenser_color_scan] Source the ROS2 workspace before using --ros.", file=sys.stderr)
        sys.exit(1)

    import time

    T_gripper2cam = load_hand_eye()
    if T_gripper2cam is None:
        print("[dispenser_color_scan] ERROR: could not load T_gripper2camera.npy", file=sys.stderr)
        sys.exit(1)

    dispenser_positions = load_dispenser_positions()
    if not dispenser_positions:
        print("[dispenser_color_scan] ERROR: no dispenser positions in calibration.yaml", file=sys.stderr)
        sys.exit(1)

    frame_bgr = None
    cam_info = None

    def to_bgr(msg: "Image") -> "np.ndarray":
        enc = (msg.encoding or "").lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if enc in ("rgb8", "bgr8"):
            image = data.reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if enc == "rgb8" else image
        if enc in ("rgba8", "bgra8"):
            image = data.reshape((msg.height, msg.width, 4))
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR) if enc == "rgba8" else cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        if enc == "mono8":
            image = data.reshape((msg.height, msg.width))
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        raise RuntimeError(f"unsupported encoding: {msg.encoding}")

    def image_cb(msg: "Image") -> None:
        nonlocal frame_bgr
        if frame_bgr is None:
            frame_bgr = to_bgr(msg)

    def info_cb(msg: "CameraInfo") -> None:
        nonlocal cam_info
        cam_info = msg

    rclpy.init()
    node = rclpy.create_node("dispenser_color_scan_node")
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)
    node.create_subscription(Image, CAMERA_TOPIC, image_cb, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, info_cb, qos_profile_sensor_data)

    deadline = time.time() + 8.0
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if frame_bgr is not None and cam_info is not None:
                break
    finally:
        pass  # keep node alive for TF lookup below

    if frame_bgr is None:
        node.destroy_node(); rclpy.shutdown()
        print(f"[dispenser_color_scan] no frame from {CAMERA_TOPIC} within 8s", file=sys.stderr)
        sys.exit(1)
    if cam_info is None:
        node.destroy_node(); rclpy.shutdown()
        print(f"[dispenser_color_scan] no camera_info from {CAMERA_INFO_TOPIC} within 8s", file=sys.stderr)
        sys.exit(1)

    # Get TF: base_link → link_6 (EE)
    T_base2ee = None
    try:
        tf_msg: TransformStamped = tf_buffer.lookup_transform(
            BASE_FRAME, EE_LINK, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=3.0)
        )
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        import numpy as np  # type: ignore
        # quaternion → rotation matrix
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array([
            [1-2*(qy**2+qz**2),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
            [2*(qx*qy+qz*qw),   1-2*(qx**2+qz**2),   2*(qy*qz-qx*qw)],
            [2*(qx*qz-qy*qw),     2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)],
        ])
        T_base2ee = np.eye(4)
        T_base2ee[:3, :3] = R
        T_base2ee[:3, 3] = [t.x, t.y, t.z]
    except Exception as exc:
        print(f"[dispenser_color_scan] TF lookup {BASE_FRAME}→{EE_LINK} failed: {exc}", file=sys.stderr)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if T_base2ee is None:
        print("[dispenser_color_scan] ERROR: cannot get EE pose; is robot driver running?", file=sys.stderr)
        sys.exit(1)

    fx, fy = cam_info.k[0], cam_info.k[4]
    cx, cy = cam_info.k[2], cam_info.k[5]
    img_h, img_w = frame_bgr.shape[:2]
    color_map: dict[str, str] = {}

    for did, xyz in sorted(dispenser_positions.items()):
        uv = project_base_point_to_pixel(xyz, T_base2ee, T_gripper2cam, fx, fy, cx, cy)
        if uv is None:
            print(f"[dispenser_color_scan] dispenser {did}: projection behind camera, fallback unknown", file=sys.stderr)
            color_map[did] = "unknown"
            continue
        u, v = uv
        x1 = max(0, u - CROP_HALF_PX)
        x2 = min(img_w, u + CROP_HALF_PX)
        y1 = max(0, v - CROP_HALF_PX)
        y2 = min(img_h, v + CROP_HALF_PX)
        if x2 <= x1 or y2 <= y1:
            print(f"[dispenser_color_scan] dispenser {did}: projected pixel ({u},{v}) out of frame {img_w}x{img_h}", file=sys.stderr)
            color_map[did] = "unknown"
            continue
        crop = frame_bgr[y1:y2, x1:x2]
        result = classify_bgr_crop(crop)
        color_map[did] = result.color
        print(f"[dispenser_color_scan] dispenser {did}: {result.color} (pixel=({u},{v}) crop=[{x1}:{x2},{y1}:{y2}])")

    return color_map


def main() -> int:
    parser = argparse.ArgumentParser(description="Scan dispenser positions for color and output a color map JSON.")
    parser.add_argument("--image-dir", default="", help="Directory with dispenser_1.png ~ dispenser_4.png")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT), help="Output JSON path")
    parser.add_argument("--ros", action="store_true", help="Capture from ROS camera topic")
    args = parser.parse_args()

    if not args.image_dir and not args.ros:
        parser.print_help()
        print("\n[dispenser_color_scan] ERROR: specify --image-dir or --ros", file=sys.stderr)
        return 2

    if args.image_dir:
        color_map = scan_from_image_dir(Path(args.image_dir))
    else:
        color_map = scan_from_ros()

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(color_map, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"[dispenser_color_scan] saved: {out}")
    print(json.dumps(color_map, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
