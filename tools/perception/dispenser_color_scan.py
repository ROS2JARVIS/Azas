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
import itertools
import json
import math
import os
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
# Visible-handle detection can transiently miss a handle (operator arm in
# frame); keep retrying on fresh frames for this long before TF fallback.
VISIBLE_RETRY_SEC = 6.0
# TF projection this far outside the frame means stale extrinsics, not a
# handle "just off-screen"; edge-crop classification there is confidently
# wrong (e.g. everything "blue" from the chair/arm strip), so report unknown.
EDGE_CLAMP_MAX_PX = 40


# HSV ranges for the physical dispenser handle colors in the current booth.
# This is intentionally image-space only: it does not create robot poses or
# calibration values.  When the handles are visible, left-to-right order maps to
# dispenser IDs 1..4.
VISIBLE_HANDLE_HSV_RANGES = {
    "red": ((0, 80, 60, 10, 255, 255), (170, 80, 60, 179, 255, 255)),
    "yellow": ((20, 80, 60, 40, 255, 255),),
    "green": ((40, 60, 50, 85, 255, 255),),
    "blue": ((85, 80, 60, 130, 255, 255),),
}
HANDLE_CENTER_Y_MIN_FRACTION = 0.16
HANDLE_CENTER_Y_MAX_FRACTION = 0.38
MIN_HANDLE_HEIGHT_OVER_WIDTH = 0.45
MAX_HANDLE_ROW_STD_FRACTION = 0.045


def write_json_immediately(path: Path, payload: dict[str, str]) -> None:
    """Atomically write JSON and fsync it so the panel can read it immediately."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with tmp.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)
    dir_fd = os.open(str(path.parent), os.O_RDONLY)
    try:
        os.fsync(dir_fd)
    finally:
        os.close(dir_fd)


def unlink_immediately(path: Path) -> None:
    if not path.exists():
        return
    path.unlink()
    dir_fd = os.open(str(path.parent), os.O_RDONLY)
    try:
        os.fsync(dir_fd)
    finally:
        os.close(dir_fd)


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


def detect_visible_handle_color_map(
    frame_bgr: "np.ndarray",
    dispenser_ids: list[str],
    *,
    debug_image_path: Path | None = None,
) -> dict[str, str] | None:
    """Detect colored dispenser handles directly from the camera image.

    The earlier TF projection path can be wrong if hand-eye/camera extrinsics are
    stale, even when the handles are plainly visible.  This fallback uses only
    the visible colored handle blobs and assigns IDs by horizontal order.
    """
    if cv2 is None:
        return None
    import numpy as np  # type: ignore

    img_h, img_w = frame_bgr.shape[:2]
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    candidates_by_color: dict[str, list[tuple[float, str, int, int, int, int, float, float, float]]] = {}
    min_area = max(150.0, float(img_w * img_h) * 0.00035)
    min_w = max(8, int(round(img_w * 0.012)))
    min_h = max(20, int(round(img_h * 0.055)))
    max_w = max(80, int(round(img_w * 0.140)))
    max_h = max(90, int(round(img_h * 0.240)))

    for color, ranges in VISIBLE_HANDLE_HSV_RANGES.items():
        mask = np.zeros((img_h, img_w), dtype=np.uint8)
        for lo_h, lo_s, lo_v, hi_h, hi_s, hi_v in ranges:
            mask |= cv2.inRange(
                hsv,
                np.array([lo_h, lo_s, lo_v], dtype=np.uint8),
                np.array([hi_h, hi_s, hi_v], dtype=np.uint8),
            )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), dtype=np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        color_candidates: list[tuple[float, str, int, int, int, int, float, float, float]] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            x, y, w, h = cv2.boundingRect(contour)
            center_x = float(x) + float(w) * 0.5
            center_y = float(y) + float(h) * 0.5
            # Booth-specific visual gate: handles are vertical colored blobs in
            # the upper/middle image, not the operator clothes, chairs, or cup.
            if area < min_area or w < min_w or h < min_h or w > max_w or h > max_h:
                continue
            if float(h) / float(max(w, 1)) < MIN_HANDLE_HEIGHT_OVER_WIDTH:
                continue
            if not (
                HANDLE_CENTER_Y_MIN_FRACTION * img_h
                <= center_y
                <= HANDLE_CENTER_Y_MAX_FRACTION * img_h
            ):
                continue
            if not (0.25 * img_w <= center_x <= 0.90 * img_w):
                continue
            score = area + float(h) * 10.0
            color_candidates.append((score, color, x, y, w, h, area, center_x, center_y))
        color_candidates.sort(key=lambda item: item[0], reverse=True)
        if color_candidates:
            candidates_by_color[color] = color_candidates[:6]

    if len(candidates_by_color) != len(dispenser_ids):
        print(
            f"[dispenser_color_scan] visible-handle fallback found {len(candidates_by_color)}/{len(dispenser_ids)} "
            "colored handles; falling back to TF projection",
            file=sys.stderr,
        )
        return None

    # One large false-positive blob can beat the real handle by area (chairs,
    # clothes, or table reflection).  The four dispenser handles are physically
    # on one horizontal row, so choose the one-candidate-per-color combination
    # with the best row consistency instead of blindly taking max area per color.
    best_combo: tuple[float, tuple[tuple[float, str, int, int, int, int, float, float, float], ...]] | None = None
    for combo in itertools.product(*(candidates_by_color[color] for color in sorted(candidates_by_color))):
        centers_x = [item[7] for item in combo]
        if len(set(round(x) for x in centers_x)) != len(combo):
            continue
        centers_y = [item[8] for item in combo]
        mean_y = sum(centers_y) / float(len(centers_y))
        row_std = math.sqrt(sum((y - mean_y) ** 2 for y in centers_y) / float(len(centers_y)))
        if row_std > max(12.0, MAX_HANDLE_ROW_STD_FRACTION * img_h):
            continue
        area_score = sum(item[6] for item in combo)
        score = area_score - 200.0 * row_std
        if best_combo is None or score > best_combo[0]:
            best_combo = (score, combo)

    if best_combo is None:
        print("[dispenser_color_scan] visible-handle fallback could not choose a non-overlapping color row", file=sys.stderr)
        return None

    candidates = list(best_combo[1])
    candidates.sort(key=lambda item: item[2])
    color_map = {did: color for did, (_, color, *_rest) in zip(dispenser_ids, candidates)}
    if debug_image_path is not None:
        debug = frame_bgr.copy()
        palette = {
            "red": (0, 0, 255),
            "yellow": (0, 255, 255),
            "green": (0, 255, 0),
            "blue": (255, 0, 0),
        }
        for did, (_, color, x, y, w, h, area, *_centers) in zip(dispenser_ids, candidates):
            bgr = palette.get(color, (255, 255, 255))
            cv2.rectangle(debug, (x, y), (x + w, y + h), bgr, 2)
            cv2.putText(
                debug,
                f"{did}:{color} {int(area)}",
                (x, max(y - 8, 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                bgr,
                2,
                cv2.LINE_AA,
            )
        debug_image_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(debug_image_path), debug)
        print(f"[dispenser_color_scan] debug image saved: {debug_image_path}")
    debug = ", ".join(
        f"{did}={color}@box({x},{y},{w},{h})"
        for did, (_, color, x, y, w, h, _area, *_centers) in zip(dispenser_ids, candidates)
    )
    print(f"[dispenser_color_scan] visible-handle fallback: {debug}")
    return color_map


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


def scan_from_ros(
    *,
    clamp_out_of_frame: bool = True,
    visible_handle_fallback: bool = True,
    settle_sec: float = 1.5,
    sample_frames: int = 5,
    debug_image_path: Path | None = None,
) -> dict[str, str]:
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

    frame_bgr = None
    frame_count = 0
    first_frame_time: float | None = None
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
        nonlocal frame_bgr, frame_count, first_frame_time
        now = time.time()
        if first_frame_time is None:
            first_frame_time = now
        if now - first_frame_time < settle_sec:
            return
        frame_bgr = to_bgr(msg)
        frame_count += 1

    def info_cb(msg: "CameraInfo") -> None:
        nonlocal cam_info
        cam_info = msg

    rclpy.init()
    node = rclpy.create_node("dispenser_color_scan_node")
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)
    node.create_subscription(Image, CAMERA_TOPIC, image_cb, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, info_cb, qos_profile_sensor_data)

    deadline = time.time() + 8.0 + max(settle_sec, 0.0)
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if frame_bgr is not None and cam_info is not None and frame_count >= max(sample_frames, 1):
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

    dispenser_ids = load_dispenser_ids()
    print(
        f"[dispenser_color_scan] using stabilized frame: "
        f"settle_sec={settle_sec:.2f} sample_frames={frame_count} size={frame_bgr.shape[1]}x{frame_bgr.shape[0]}"
    )
    if visible_handle_fallback:
        # 한 프레임만 보면 일시적 가림(작업자 팔 등)으로 핸들 하나가 빠져
        # 4/4 검출 전체가 버려지고 TF 투영으로 떨어진다. 새 프레임을 받아
        # 잠시 재시도해서 일시적 가림을 흡수한다.
        retry_deadline = time.time() + VISIBLE_RETRY_SEC
        seen_count = frame_count
        while True:
            visible_map = detect_visible_handle_color_map(
                frame_bgr,
                dispenser_ids,
                debug_image_path=debug_image_path,
            )
            if visible_map is not None:
                node.destroy_node()
                rclpy.shutdown()
                return visible_map
            while rclpy.ok() and time.time() < retry_deadline and frame_count == seen_count:
                rclpy.spin_once(node, timeout_sec=0.1)
            if frame_count == seen_count:
                print(
                    f"[dispenser_color_scan] visible-handle detection failed for {VISIBLE_RETRY_SEC:.0f}s; "
                    "using TF projection",
                    file=sys.stderr,
                )
                break
            seen_count = frame_count

    T_gripper2cam = load_hand_eye()
    if T_gripper2cam is None:
        node.destroy_node(); rclpy.shutdown()
        print("[dispenser_color_scan] ERROR: could not load T_gripper2camera.npy", file=sys.stderr)
        sys.exit(1)

    dispenser_positions = load_dispenser_positions()
    if not dispenser_positions:
        node.destroy_node(); rclpy.shutdown()
        print("[dispenser_color_scan] ERROR: no dispenser positions in calibration.yaml", file=sys.stderr)
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
            if clamp_out_of_frame:
                clamped_u = min(max(u, 0), img_w - 1)
                clamped_v = min(max(v, 0), img_h - 1)
                overshoot = max(abs(u - clamped_u), abs(v - clamped_v))
                if overshoot > EDGE_CLAMP_MAX_PX:
                    print(
                        f"[dispenser_color_scan] dispenser {did}: projected pixel ({u},{v}) is "
                        f"{overshoot}px outside frame {img_w}x{img_h} (stale extrinsics?); fallback unknown",
                        file=sys.stderr,
                    )
                    color_map[did] = "unknown"
                    continue
                x1 = max(0, clamped_u - CROP_HALF_PX)
                x2 = min(img_w, clamped_u + CROP_HALF_PX)
                y1 = max(0, clamped_v - CROP_HALF_PX)
                y2 = min(img_h, clamped_v + CROP_HALF_PX)
                if x2 > x1 and y2 > y1:
                    print(
                        f"[dispenser_color_scan] dispenser {did}: projected pixel ({u},{v}) out of frame {img_w}x{img_h}; "
                        f"using edge crop around ({clamped_u},{clamped_v})",
                        file=sys.stderr,
                    )
                else:
                    print(f"[dispenser_color_scan] dispenser {did}: projected pixel ({u},{v}) out of frame {img_w}x{img_h}", file=sys.stderr)
                    color_map[did] = "unknown"
                    continue
            else:
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
    parser.add_argument("--no-clamp-out-of-frame", action="store_true", help="Do not classify edge crop when projected dispenser pixel is just outside the image")
    parser.add_argument("--no-visible-handle-fallback", action="store_true", help="Disable visible colored-handle detection and use only TF projection")
    parser.add_argument("--settle-sec", type=float, default=1.5, help="Seconds to ignore camera frames before color classification")
    parser.add_argument("--sample-frames", type=int, default=5, help="Number of stabilized frames to receive before classifying the latest one")
    parser.add_argument("--debug-image", default="", help="Optional path to save visible-handle debug overlay")
    args = parser.parse_args()

    if not args.image_dir and not args.ros:
        parser.print_help()
        print("\n[dispenser_color_scan] ERROR: specify --image-dir or --ros", file=sys.stderr)
        return 2

    if args.image_dir:
        color_map = scan_from_image_dir(Path(args.image_dir))
    else:
        color_map = scan_from_ros(
            clamp_out_of_frame=not args.no_clamp_out_of_frame,
            visible_handle_fallback=not args.no_visible_handle_fallback,
            settle_sec=max(args.settle_sec, 0.0),
            sample_frames=max(args.sample_frames, 1),
            debug_image_path=Path(args.debug_image) if args.debug_image else None,
        )

    unknown_ids = [did for did, color in color_map.items() if str(color).lower() == "unknown"]
    if unknown_ids:
        out = Path(args.output)
        failed_out = out.with_suffix(out.suffix + ".failed")
        write_json_immediately(failed_out, color_map)
        unlink_immediately(out)
        print(
            "[dispenser_color_scan] ERROR: unknown color result for dispenser(s): "
            + ", ".join(sorted(unknown_ids, key=lambda x: int(x) if str(x).isdigit() else str(x))),
            file=sys.stderr,
        )
        print(f"[dispenser_color_scan] failed result saved: {failed_out}", file=sys.stderr)
        print(json.dumps(color_map, ensure_ascii=False))
        return 1
    out = Path(args.output)
    write_json_immediately(out, color_map)
    failed_out = out.with_suffix(out.suffix + ".failed")
    unlink_immediately(failed_out)
    print(f"[dispenser_color_scan] saved: {out}")
    print(json.dumps(color_map, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
