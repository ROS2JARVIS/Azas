#!/usr/bin/env python3
"""Depth-first cup candidate detector for the Azas MVP.

This is option D: do not wait for a labeled dataset. On the real robot, use an
aligned depth image plus CameraInfo to produce a conservative 3D cup candidate.
The detector intentionally returns no candidate when depth evidence is weak.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class CupCandidate:
    bbox_xyxy: tuple[int, int, int, int]
    center_uv: tuple[int, int]
    point_camera_m: tuple[float, float, float]
    confidence: float
    pixel_count: int


def project_pixel(u: int, v: int, depth_m: float, intrinsics: CameraIntrinsics) -> tuple[float, float, float]:
    if depth_m <= 0 or not np.isfinite(depth_m):
        raise ValueError("depth_m must be finite and positive")
    x = (u - intrinsics.cx) * depth_m / intrinsics.fx
    y = (v - intrinsics.cy) * depth_m / intrinsics.fy
    return float(x), float(y), float(depth_m)


def _neighbors(y: int, x: int, h: int, w: int) -> Iterable[tuple[int, int]]:
    if y > 0:
        yield y - 1, x
    if y + 1 < h:
        yield y + 1, x
    if x > 0:
        yield y, x - 1
    if x + 1 < w:
        yield y, x + 1


def _largest_component(mask: np.ndarray) -> tuple[np.ndarray, int]:
    h, w = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    best_pixels: list[tuple[int, int]] = []

    ys, xs = np.nonzero(mask)
    for start_y, start_x in zip(ys.tolist(), xs.tolist()):
        if seen[start_y, start_x]:
            continue
        stack = [(start_y, start_x)]
        seen[start_y, start_x] = True
        pixels: list[tuple[int, int]] = []
        while stack:
            y, x = stack.pop()
            pixels.append((y, x))
            for ny, nx in _neighbors(y, x, h, w):
                if mask[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    stack.append((ny, nx))
        if len(pixels) > len(best_pixels):
            best_pixels = pixels

    component = np.zeros_like(mask, dtype=bool)
    for y, x in best_pixels:
        component[y, x] = True
    return component, len(best_pixels)


def detect_cup_from_depth(
    depth_raw: np.ndarray,
    intrinsics: CameraIntrinsics,
    *,
    depth_scale: float = 0.001,
    min_depth_m: float = 0.15,
    max_depth_m: float = 1.20,
    min_pixels: int = 80,
    roi: tuple[int, int, int, int] | None = None,
) -> CupCandidate | None:
    """Return the nearest connected depth blob as a conservative cup candidate.

    Args:
        depth_raw: 2D aligned depth image. Integer millimeters are expected when
            depth_scale is 0.001.
        intrinsics: Camera intrinsics from sensor_msgs/CameraInfo.k.
        depth_scale: raw depth to meters multiplier.
        min_depth_m/max_depth_m: valid workspace range from the camera.
        min_pixels: reject tiny blobs/noise.
        roi: optional (x1, y1, x2, y2) image crop for the tabletop workspace.
    """
    if depth_raw.ndim != 2:
        raise ValueError("depth_raw must be a 2D array")

    depth_m = depth_raw.astype(np.float32) * float(depth_scale)
    valid = np.isfinite(depth_m) & (depth_m >= min_depth_m) & (depth_m <= max_depth_m)

    if roi is not None:
        x1, y1, x2, y2 = roi
        roi_mask = np.zeros_like(valid, dtype=bool)
        roi_mask[max(y1, 0):min(y2, valid.shape[0]), max(x1, 0):min(x2, valid.shape[1])] = True
        valid &= roi_mask

    if not np.any(valid):
        return None

    nearest = float(np.nanpercentile(depth_m[valid], 5))
    surface_band_m = 0.08
    mask = valid & (depth_m <= nearest + surface_band_m)
    component, pixel_count = _largest_component(mask)
    if pixel_count < min_pixels:
        return None

    ys, xs = np.nonzero(component)
    x1, x2 = int(xs.min()), int(xs.max())
    y1, y2 = int(ys.min()), int(ys.max())
    u = int(round(float(np.median(xs))))
    v = int(round(float(np.median(ys))))
    depth = float(np.median(depth_m[component]))
    point = project_pixel(u, v, depth, intrinsics)

    image_area = depth_raw.shape[0] * depth_raw.shape[1]
    confidence = min(1.0, pixel_count / max(float(image_area) * 0.03, 1.0))
    return CupCandidate((x1, y1, x2, y2), (u, v), point, confidence, pixel_count)
