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
        Path(get_package_share_directory("azas_cup_uprighting"))
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

    z_m = _depth_at(depth_image, px, py)
    if not np.isfinite(z_m):
        z_m = _median_depth_near(depth_image, px, py)
        if np.isfinite(z_m):
            logger.info(f"depth=0 at ({px}, {py}); using nearby median depth={z_m:.3f}m")
        else:
            logger.warn(f"depth=0 at ({px}, {py}) and no nearby valid depth")
            return None

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


def _median_depth_near(depth_image, cx: int, cy: int, radius: int = 4) -> float:
    """주변 patch의 유효 depth median (m). 없으면 inf."""
    if depth_image is None:
        return float("inf")
    h, w = depth_image.shape[:2]
    x1 = max(0, cx - radius)
    x2 = min(w, cx + radius + 1)
    y1 = max(0, cy - radius)
    y2 = min(h, cy + radius + 1)
    patch = depth_image[y1:y2, x1:x2]
    if patch.size == 0:
        return float("inf")

    if depth_image.dtype == np.uint16:
        valid = patch[patch > 0].astype(float) / 1000.0
    else:
        valid = patch[np.isfinite(patch) & (patch > 0)].astype(float)
    if valid.size == 0:
        return float("inf")
    return float(np.median(valid))


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


def calculate_cup_orientation(depth_image, bbox, frame=None):
    """
    OpenCV를 이용해 Bounding Box 내부의 실제 컵 기울기(theta)를 정밀 추출
    """
    if frame is None:
        return 0.0

    # Bounding Box 좌표를 정수로 변환하여 ROI(관심 영역) 자르기
    x1, y1, x2, y2 = map(int, bbox)
    roi = frame[y1:y2, x1:x2]
    
    if roi.size == 0:
        return 0.0

    # 이미지를 흑백으로 변환하고 이진화(Threshold)하여 컵과 배경 분리
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

    # 외곽선(Contours) 찾기
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0.0

    # 가장 넓은 외곽선을 컵의 본체로 간주
    c = max(contours, key=cv2.contourArea)

    # ── PCA: 컵 외곽선 점들의 주축 방향 ──
    pts = c.reshape(-1, 2).astype(np.float64)
    mean = pts.mean(axis=0)
    pts_centered = pts - mean
    cov = np.cov(pts_centered.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    principal = eigvecs[:, np.argmax(eigvals)]   # 가장 큰 분산 방향 = 컵 장축

    theta = np.arctan2(principal[1], principal[0])
    return theta



def is_top_pointing_towards_theta(frame, bbox, theta):
    """
    컵의 빨간 스티커(입구 부분)가 theta 방향에 있는지, 반대 방향인지 판별
    """
    if frame is None:
        return True

    x1, y1, x2, y2 = map(int, bbox)
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)
    
    roi = frame[y1:y2, x1:x2]
    if roi.size == 0:
        return True
        
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # 빨간색 마스크 추출
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
    
    # 빨간색 픽셀들의 무게중심(Center of Mass) 계산
    M = cv2.moments(mask)
    if M["m00"] == 0:
        return True 
        
    # ROI 내에서의 무게중심 좌표
    cm_x = int(M["m10"] / M["m00"])
    cm_y = int(M["m01"] / M["m00"])
    
    # ROI의 기하학적 중심 좌표
    center_x = roi.shape[1] / 2.0
    center_y = roi.shape[0] / 2.0
    
    # 컵 중심에서 스티커(무게중심)를 향하는 벡터 생성
    vec_sticker = np.array([cm_x - center_x, cm_y - center_y])
    
    # theta 각도가 가리키는 단위 벡터 생성
    vec_theta = np.array([np.cos(theta), np.sin(theta)])
    
    # 두 벡터의 내적(Dot Product) 계산

    dot_product = np.dot(vec_sticker, vec_theta)
    
    return dot_product > 0
