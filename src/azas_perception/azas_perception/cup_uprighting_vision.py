from __future__ import annotations

import math

import cv2
import numpy as np


def calculate_cup_major_axis_angle_rad(image_bgr: np.ndarray, bbox: tuple[int, int, int, int]) -> float:
    """Estimate the long-axis angle of a cup crop in image coordinates.

    This is the non-motion perception portion adapted from the
    yolo_cup_uprighting demo. It intentionally returns only an image-plane
    diagnostic angle; robot poses and trajectories must still come from the
    Azas depth/TF pipeline and motion stack.
    """

    if image_bgr is None or image_bgr.size == 0:
        return 0.0
    x1, y1, x2, y2 = _clamp_bbox(image_bgr, bbox)
    roi = image_bgr[y1:y2, x1:x2]
    if roi.size == 0:
        return 0.0

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0.0

    rect = cv2.minAreaRect(max(contours, key=cv2.contourArea))
    (_, _), (width, height), angle_deg = rect
    if width < height:
        angle_deg += 90.0
    return float(math.radians(angle_deg))


def is_red_marker_aligned_with_angle(
    image_bgr: np.ndarray,
    bbox: tuple[int, int, int, int],
    theta_rad: float,
) -> bool:
    """Return whether a red cup marker lies in the positive theta direction.

    If the marker is absent or the crop is invalid, the function returns True
    so callers do not invent a robot-side correction from missing evidence.
    """

    if image_bgr is None or image_bgr.size == 0:
        return True
    x1, y1, x2, y2 = _clamp_bbox(image_bgr, bbox)
    roi = image_bgr[y1:y2, x1:x2]
    if roi.size == 0:
        return True

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

    moments = cv2.moments(mask)
    if moments["m00"] == 0:
        return True

    marker = np.array(
        [
            moments["m10"] / moments["m00"] - (roi.shape[1] / 2.0),
            moments["m01"] / moments["m00"] - (roi.shape[0] / 2.0),
        ],
        dtype=float,
    )
    axis = np.array([math.cos(theta_rad), math.sin(theta_rad)], dtype=float)
    return bool(np.dot(marker, axis) > 0.0)


def _clamp_bbox(image_bgr: np.ndarray, bbox: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
    height, width = image_bgr.shape[:2]
    x1, y1, x2, y2 = map(int, bbox)
    return (
        max(0, min(x1, width)),
        max(0, min(y1, height)),
        max(0, min(x2, width)),
        max(0, min(y2, height)),
    )
