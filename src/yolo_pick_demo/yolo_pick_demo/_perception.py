"""YOLO 추론 + 카메라 좌표 변환.

Hand-Eye 행렬, pixel→base 변환, YOLO 검출을 함수로 제공.
"""

from pathlib import Path

import cv2  
import numpy as np

from ament_index_python.packages import get_package_share_directory

from . import _config as cfg
from ._motion import get_ee_matrix


def bbox_size(box) -> int:
    """bbox max(w, h) — 길이/지름 대표값."""
    x1, y1, x2, y2 = box
    return max(x2 - x1, y2 - y1)


def load_hand_eye():
    """T_gripper2camera.npy 로드 (mm → m)."""
    calib_file = (
        Path(get_package_share_directory("yolo_pick_demo"))
        / "config" / "T_gripper2camera.npy"
    )
    g2c = np.load(str(calib_file)).astype(float)
    g2c[:3, 3] /= 1000.0   # mm → m
    return g2c, calib_file


def transform_to_base(robot, gripper2cam, cam_xyz_m):
    """카메라 좌표 (m) → base 좌표 (m). 현재 EE 자세 기준."""
    coord = np.append(np.array(cam_xyz_m, dtype=float), 1.0)
    base2ee  = get_ee_matrix(robot)
    base2cam = base2ee @ gripper2cam
    return (base2cam @ coord)[:3]


def pixel_to_base(robot, gripper2cam, depth_image, intrinsics,
                  px: int, py: int, logger):
    """픽셀 + depth 이미지 → base 좌표 (m). 실패 시 None."""
    if depth_image is None or intrinsics is None:
        logger.warn("frame/intrinsics 아직 준비 안됨")
        return None

    h, w = depth_image.shape[:2]
    if not (0 <= px < w and 0 <= py < h):
        logger.warn("pixel 범위 초과")
        return None

    z_raw = depth_image[py, px]
    if z_raw == 0:
        logger.warn(f"depth=0 at ({px}, {py})")
        return None

    z_m = (float(z_raw) / 1000.0
           if depth_image.dtype == np.uint16 else float(z_raw))

    fx, fy   = intrinsics["fx"],  intrinsics["fy"]
    ppx, ppy = intrinsics["ppx"], intrinsics["ppy"]

    cam_x = (px - ppx) * z_m / fx
    cam_y = (py - ppy) * z_m / fy
    cam_z = z_m

    base = transform_to_base(robot, gripper2cam, (cam_x, cam_y, cam_z))
    logger.info(
        f"pixel({px},{py}) cam({cam_x:.3f},{cam_y:.3f},{cam_z:.3f}) "
        f"-> base({base[0]:.3f},{base[1]:.3f},{base[2]:.3f}) m"
    )
    return tuple(float(v) for v in base)


def _depth_at(depth_image, cx: int, cy: int) -> float:
    """픽셀의 depth (m). 없으면 inf."""
    if depth_image is None:
        return float("inf")
    h, w = depth_image.shape[:2]
    if not (0 <= cx < w and 0 <= cy < h):
        return float("inf")
    z_raw = depth_image[cy, cx]
    if z_raw == 0:
        return float("inf")
    return (float(z_raw) / 1000.0
            if depth_image.dtype == np.uint16 else float(z_raw))


def run_yolo(yolo, frame: np.ndarray, depth_image=None) -> list[dict]:
    """YOLO 추론. 각 detection 에 cx/cy/conf/cls/bbox/size/depth 포함."""
    results = yolo(frame, verbose=False)[0]
    detections = []

    for box in results.boxes:
        conf   = float(box.conf[0])
        cls_id = int(box.cls[0])
        if conf < cfg.YOLO_CONF_THRESH:
            continue

        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        cls_name = yolo.names.get(cls_id, str(cls_id))

        detections.append({
            "cx": cx, "cy": cy,
            "conf": conf,
            "cls_id": cls_id,
            "cls_name": cls_name,
            "box": (x1, y1, x2, y2),
            "size": bbox_size((x1, y1, x2, y2)),
            "depth": _depth_at(depth_image, cx, cy),
        })

    return detections


def _normalize_axis_angle(theta):
    """긴 축 각도를 [-pi/2, pi/2) 범위로 정규화합니다."""
    return (theta + np.pi / 2.0) % np.pi - np.pi / 2.0


def _dominant_line_theta(edges):
    """ROI edge 이미지에서 가장 지배적인 직선 방향을 추정합니다."""
    h, w = edges.shape[:2]
    min_len = max(25, int(min(h, w) * 0.28))
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180.0,
        threshold=30,
        minLineLength=min_len,
        maxLineGap=max(8, int(min(h, w) * 0.12)),
    )
    if lines is None:
        return None

    vectors = []
    for line in lines[:, 0, :]:
        x1, y1, x2, y2 = line
        dx = float(x2 - x1)
        dy = float(y2 - y1)
        length = np.hypot(dx, dy)
        if length < min_len:
            continue
        theta = _normalize_axis_angle(np.arctan2(dy, dx))
        vectors.append((theta, length))

    if not vectors:
        return None

    # theta와 theta + pi가 같은 축이므로 2*theta 공간에서 weighted mean.
    sin2 = sum(length * np.sin(2.0 * theta) for theta, length in vectors)
    cos2 = sum(length * np.cos(2.0 * theta) for theta, length in vectors)
    return _normalize_axis_angle(0.5 * np.arctan2(sin2, cos2))


def _pca_edge_theta(edges):
    """Edge 픽셀 분포의 PCA로 긴 축 방향을 추정합니다."""
    points = np.column_stack(np.where(edges > 0))
    if len(points) < 20:
        return None

    xy = points[:, ::-1].astype(np.float32)
    _, eigenvectors, eigenvalues = cv2.PCACompute2(xy, mean=None)
    if eigenvectors is None or len(eigenvectors) == 0:
        return None
    if eigenvalues is not None and len(eigenvalues) > 1 and eigenvalues[1][0] > 0:
        ratio = eigenvalues[0][0] / eigenvalues[1][0]
        if ratio < 1.2:
            return None

    vx, vy = eigenvectors[0]
    return _normalize_axis_angle(np.arctan2(float(vy), float(vx)))


def calculate_cup_orientation(depth_image, bbox, frame=None):
    """
    OpenCV를 이용해 Bounding Box 내부의 실제 컵 기울기(theta)를 정밀 추출합니다.
    """
    if frame is None:
        return 0.0

    x1, y1, x2, y2 = map(int, bbox)
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)
    roi = frame[y1:y2, x1:x2]

    if roi.size == 0:
        return 0.0

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    edges = cv2.Canny(gray, 45, 130)
    kernel = np.ones((3, 3), np.uint8)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=1)

    theta = _dominant_line_theta(edges)
    if theta is not None:
        return theta

    theta = _pca_edge_theta(edges)
    if theta is not None:
        return theta

    _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0.0

    c = max(contours, key=cv2.contourArea)
    rect = cv2.minAreaRect(c)
    (cx, cy), (w, h), angle = rect

    if w < h:
        angle += 90.0  

    return _normalize_axis_angle(np.deg2rad(angle))


def analyze_cup_mouth(frame, bbox, theta):
    """
    빨간 표시를 기준으로 컵 입구 방향을 분석합니다.

    반환값:
      - found: 빨간 표시 검출 여부
      - is_top: 빨간 표시가 theta 방향에 있으면 True
      - sticker_center: 이미지 전체 좌표계의 빨간 표시 중심 (x, y)
      - mouth_theta: 입구 쪽을 향하도록 보정된 theta
    """
    mouth_theta = theta
    result = {
        "found": False,
        "is_top": True,
        "sticker_center": None,
        "marker_theta": None,
        "mouth_theta": mouth_theta,
        "dot_product": 0.0,
    }

    if frame is None:
        return result

    x1, y1, x2, y2 = map(int, bbox)
    box_w = max(1, x2 - x1)
    box_h = max(1, y2 - y1)
    pad_x = int(box_w * 0.08)
    pad_y = int(box_h * 0.08)
    x1, y1 = max(0, x1 - pad_x), max(0, y1 - pad_y)
    x2, y2 = min(frame.shape[1], x2 + pad_x), min(frame.shape[0], y2 + pad_y)

    roi = frame[y1:y2, x1:x2]
    if roi.size == 0:
        return result

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 55, 55])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 55, 55])
    upper_red2 = np.array([180, 255, 255])

    mask = (
        cv2.inRange(hsv, lower_red1, upper_red1)
        + cv2.inRange(hsv, lower_red2, upper_red2)
    )
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    M = cv2.moments(mask)
    if M["m00"] == 0:
        return result

    cm_x = int(M["m10"] / M["m00"])
    cm_y = int(M["m01"] / M["m00"])
    center_x = roi.shape[1] / 2.0
    center_y = roi.shape[0] / 2.0

    vec_sticker = np.array([cm_x - center_x, cm_y - center_y])
    vec_theta = np.array([np.cos(theta), np.sin(theta)])
    dot_product = float(np.dot(vec_sticker, vec_theta))
    is_top = dot_product > 0
    marker_theta = float(np.arctan2(vec_sticker[1], vec_sticker[0]))

    if not is_top:
        mouth_theta = theta + np.pi

    result.update({
        "found": True,
        "is_top": is_top,
        "sticker_center": (x1 + cm_x, y1 + cm_y),
        "marker_theta": marker_theta,
        "mouth_theta": mouth_theta,
        "dot_product": dot_product,
    })
    return result


def is_top_pointing_towards_theta(frame, bbox, theta):
    """
    컵의 빨간 스티커(입구 부분)가 theta 방향에 있는지, 반대 방향인지 판별
    """
    return analyze_cup_mouth(frame, bbox, theta)["is_top"]
