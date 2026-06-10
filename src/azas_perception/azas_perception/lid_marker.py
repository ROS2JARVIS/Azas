from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class ImageRoi:
    x_min: int
    y_min: int
    x_max: int
    y_max: int

    @property
    def width(self) -> int:
        return max(self.x_max - self.x_min, 0)

    @property
    def height(self) -> int:
        return max(self.y_max - self.y_min, 0)


@dataclass(frozen=True)
class RedCircleConfig:
    min_area_px: float = 80.0
    min_radius_px: float = 4.0
    min_circularity: float = 0.65
    min_saturation: int = 80
    min_value: int = 40
    morph_kernel_px: int = 3


@dataclass(frozen=True)
class RedCircle:
    center_u: int
    center_v: int
    radius_px: float
    area_px: float
    circularity: float


@dataclass(frozen=True)
class ArucoMarker:
    center_u: int
    center_v: int
    marker_id: int
    side_px: float
    corners: np.ndarray


def padded_roi(roi: ImageRoi, image_width: int, image_height: int, padding_ratio: float) -> ImageRoi:
    padding_ratio = max(float(padding_ratio), 0.0)
    pad_x = int(round(roi.width * padding_ratio))
    pad_y = int(round(roi.height * padding_ratio))
    return ImageRoi(
        x_min=max(roi.x_min - pad_x, 0),
        y_min=max(roi.y_min - pad_y, 0),
        x_max=min(roi.x_max + pad_x, image_width),
        y_max=min(roi.y_max + pad_y, image_height),
    )


def detect_red_circle_marker(
    image_bgr: np.ndarray,
    roi: ImageRoi,
    config: RedCircleConfig,
) -> RedCircle | None:
    if image_bgr.size == 0 or roi.width <= 0 or roi.height <= 0:
        return None

    patch = image_bgr[roi.y_min : roi.y_max, roi.x_min : roi.x_max]
    if patch.size == 0:
        return None

    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    lower_red_a = np.array([0, config.min_saturation, config.min_value], dtype=np.uint8)
    upper_red_a = np.array([10, 255, 255], dtype=np.uint8)
    lower_red_b = np.array([170, config.min_saturation, config.min_value], dtype=np.uint8)
    upper_red_b = np.array([180, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_red_a, upper_red_a) | cv2.inRange(hsv, lower_red_b, upper_red_b)

    kernel_px = max(int(config.morph_kernel_px), 1)
    if kernel_px > 1:
        kernel = np.ones((kernel_px, kernel_px), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best: RedCircle | None = None
    best_score = -1.0
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < config.min_area_px:
            continue
        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 0.0:
            continue
        circularity = float(4.0 * math.pi * area / (perimeter * perimeter))
        if circularity < config.min_circularity:
            continue
        (local_x, local_y), radius = cv2.minEnclosingCircle(contour)
        if radius < config.min_radius_px:
            continue
        moments = cv2.moments(contour)
        if abs(moments["m00"]) > 1e-9:
            local_u = moments["m10"] / moments["m00"]
            local_v = moments["m01"] / moments["m00"]
        else:
            local_u = local_x
            local_v = local_y
        candidate = RedCircle(
            center_u=int(round(roi.x_min + local_u)),
            center_v=int(round(roi.y_min + local_v)),
            radius_px=float(radius),
            area_px=area,
            circularity=circularity,
        )
        score = area * circularity
        if score > best_score:
            best = candidate
            best_score = score
    return best


def detect_aruco_marker(
    image_bgr: np.ndarray,
    roi: ImageRoi,
    dictionary_name: str = "DICT_4X4_50",
    marker_id: int = -1,
) -> ArucoMarker | None:
    if image_bgr.size == 0 or roi.width <= 0 or roi.height <= 0:
        return None

    patch = image_bgr[roi.y_min : roi.y_max, roi.x_min : roi.x_max]
    if patch.size == 0:
        return None

    dictionary_id = _aruco_dictionary_id(dictionary_name)
    if dictionary_id is None:
        return None

    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    aruco = getattr(cv2, "aruco", None)
    if aruco is None:
        return None

    dictionary = aruco.getPredefinedDictionary(dictionary_id)
    parameters = _aruco_detector_parameters()
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary, parameters)
        corners_list, ids, _rejected = detector.detectMarkers(gray)
    elif hasattr(aruco, "detectMarkers"):
        kwargs = {"parameters": parameters} if parameters is not None else {}
        corners_list, ids, _rejected = aruco.detectMarkers(gray, dictionary, **kwargs)
    else:
        return None
    if ids is None or len(ids) == 0:
        return None

    desired_id = int(marker_id)
    best: ArucoMarker | None = None
    best_score = -1.0
    for corners, marker_id_array in zip(corners_list, ids):
        detected_id = int(marker_id_array[0])
        if desired_id >= 0 and detected_id != desired_id:
            continue
        local_corners = np.asarray(corners, dtype=float).reshape(4, 2)
        global_corners = local_corners + np.array([roi.x_min, roi.y_min], dtype=float)
        side_px = _aruco_side_px(global_corners)
        if side_px <= 0.0:
            continue
        center = np.mean(global_corners, axis=0)
        candidate = ArucoMarker(
            center_u=int(round(float(center[0]))),
            center_v=int(round(float(center[1]))),
            marker_id=detected_id,
            side_px=side_px,
            corners=global_corners,
        )
        if side_px > best_score:
            best = candidate
            best_score = side_px
    return best


def _aruco_dictionary_id(dictionary_name: str) -> int | None:
    aruco = getattr(cv2, "aruco", None)
    if aruco is None:
        return None
    name = str(dictionary_name).strip().upper()
    if not name:
        return None
    if not name.startswith("DICT_"):
        name = f"DICT_{name}"
    return getattr(aruco, name, None)


def _aruco_detector_parameters():
    aruco = getattr(cv2, "aruco", None)
    if aruco is None:
        return None
    if hasattr(aruco, "DetectorParameters"):
        return aruco.DetectorParameters()
    if hasattr(aruco, "DetectorParameters_create"):
        return aruco.DetectorParameters_create()
    return None


def _aruco_side_px(corners: np.ndarray) -> float:
    if corners.shape != (4, 2):
        return 0.0
    lengths = []
    for index in range(4):
        current = corners[index]
        next_corner = corners[(index + 1) % 4]
        lengths.append(float(np.linalg.norm(next_corner - current)))
    return float(np.mean(lengths))


def quaternion_from_lid_normal(normal: np.ndarray, finger_axis_hint: np.ndarray | None = None) -> tuple[float, float, float, float]:
    """Build a pose quaternion whose local +Z points away from the lid surface.

    The red sticker is circular, so it does not define in-plane yaw. The local
    +X axis uses a projected camera-axis hint as a deterministic finger-axis
    candidate; downstream motion must still verify actual gripper clearance.
    """
    z_axis = _unit_vector(np.asarray(normal, dtype=float))
    if z_axis is None:
        raise ValueError("lid normal must be non-zero")

    if finger_axis_hint is None:
        finger_axis_hint = np.array([1.0, 0.0, 0.0], dtype=float)
    x_hint = np.asarray(finger_axis_hint, dtype=float)
    x_axis = x_hint - float(np.dot(x_hint, z_axis)) * z_axis
    x_axis = _unit_vector(x_axis)
    if x_axis is None:
        fallback = np.array([0.0, 1.0, 0.0], dtype=float)
        x_axis = _unit_vector(fallback - float(np.dot(fallback, z_axis)) * z_axis)
    if x_axis is None:
        raise ValueError("could not construct lid finger axis")

    y_axis = _unit_vector(np.cross(z_axis, x_axis))
    if y_axis is None:
        raise ValueError("could not construct lid orientation basis")
    x_axis = _unit_vector(np.cross(y_axis, z_axis))
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    return quaternion_from_rotation_matrix(rotation)


def quaternion_from_rotation_matrix(rotation: np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError("rotation matrix must be 3x3")

    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        qw = (matrix[2, 1] - matrix[1, 2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0, 1] + matrix[1, 0]) / scale
        qz = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        qw = (matrix[0, 2] - matrix[2, 0]) / scale
        qx = (matrix[0, 1] + matrix[1, 0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        qw = (matrix[1, 0] - matrix[0, 1]) / scale
        qx = (matrix[0, 2] + matrix[2, 0]) / scale
        qy = (matrix[1, 2] + matrix[2, 1]) / scale
        qz = 0.25 * scale

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        raise ValueError("rotation produced a zero quaternion")
    return qx / norm, qy / norm, qz / norm, qw / norm


def _unit_vector(value: np.ndarray) -> np.ndarray | None:
    norm = float(np.linalg.norm(value))
    if not math.isfinite(norm) or norm <= 1e-12:
        return None
    return value / norm
