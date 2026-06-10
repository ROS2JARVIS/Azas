#!/usr/bin/env python3
"""Offline HSV color discrimination utilities for dispenser/cocktail perception.

This module is intentionally perception-only: it classifies colors in image crops
or arrays and does not subscribe to cameras or command hardware.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import numpy as np

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover - handled by callers
    cv2 = None


COLOR_ORDER = ("red", "orange", "yellow", "green", "blue", "purple", "black", "white", "unknown")


@dataclass(frozen=True)
class HsvColorResult:
    color: str
    h_median: float
    s_median: float
    v_median: float
    confidence: float
    reason: str


def _require_cv2() -> None:
    if cv2 is None:
        raise RuntimeError("opencv-python is required for BGR/HSV color discrimination")


def center_crop_fraction(image: np.ndarray, fraction: float = 0.60) -> np.ndarray:
    """Return center crop for stable median color estimation.

    The center crop avoids box borders, text overlays, and specular edges.
    """
    if image.ndim < 2:
        raise ValueError("image must have at least HxW dimensions")
    fraction = float(fraction)
    if not (0.0 < fraction <= 1.0):
        raise ValueError("fraction must be in (0, 1]")
    h, w = image.shape[:2]
    ch, cw = max(1, int(round(h * fraction))), max(1, int(round(w * fraction)))
    y1 = max(0, (h - ch) // 2)
    x1 = max(0, (w - cw) // 2)
    return image[y1 : y1 + ch, x1 : x1 + cw]


def median_hsv_from_bgr(crop_bgr: np.ndarray, center_fraction: float = 0.60) -> tuple[float, float, float]:
    _require_cv2()
    if crop_bgr.size == 0:
        return 0.0, 0.0, 0.0
    crop = center_crop_fraction(crop_bgr, center_fraction)
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    pixels = hsv.reshape(-1, 3).astype(np.float32)
    return tuple(float(x) for x in np.median(pixels, axis=0))  # type: ignore[return-value]


def classify_hsv(h: float, s: float, v: float) -> HsvColorResult:
    """Classify OpenCV HSV median into robot-relevant color bins.

    OpenCV hue range is [0, 179]. The thresholds are deliberately conservative:
    low saturation/value becomes white/black before hue classification.
    """
    h = float(h) % 180.0
    s = float(s)
    v = float(v)

    if v < 45:
        return HsvColorResult("black", h, s, v, 0.95, "value below black threshold")
    if s < 35 and v >= 155:
        return HsvColorResult("white", h, s, v, 0.90, "low saturation and high value")
    if s < 28:
        return HsvColorResult("unknown", h, s, v, 0.30, "low saturation but not bright enough for white")

    # hue ranges in OpenCV units. Red wraps around 0/179.
    ranges: list[tuple[str, tuple[float, float] | tuple[tuple[float, float], tuple[float, float]], float]] = [
        ("red", ((0, 9), (170, 179)), 0.90),
        ("orange", (10, 22), 0.85),
        ("yellow", (23, 36), 0.85),
        ("green", (37, 84), 0.85),
        ("blue", (85, 124), 0.85),
        ("purple", (125, 160), 0.80),
    ]
    for name, rng, conf in ranges:
        if isinstance(rng[0], tuple):  # type: ignore[index]
            if any(lo <= h <= hi for lo, hi in rng):  # type: ignore[assignment]
                return HsvColorResult(name, h, s, v, conf, "hue inside wrapped range" if name == "red" else "hue inside range")
        else:
            lo, hi = rng  # type: ignore[misc]
            if lo <= h <= hi:
                return HsvColorResult(name, h, s, v, conf, "hue inside range")
    return HsvColorResult("unknown", h, s, v, 0.25, "hue outside configured ranges")


def classify_bgr_crop(crop_bgr: np.ndarray, center_fraction: float = 0.60) -> HsvColorResult:
    h, s, v = median_hsv_from_bgr(crop_bgr, center_fraction=center_fraction)
    return classify_hsv(h, s, v)


def classify_image_box(image_bgr: np.ndarray, xyxy: Sequence[float], center_fraction: float = 0.60) -> HsvColorResult:
    h, w = image_bgr.shape[:2]
    x1, y1, x2, y2 = [int(round(float(x))) for x in xyxy]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)
    if x2 <= x1 or y2 <= y1:
        return HsvColorResult("unknown", 0.0, 0.0, 0.0, 0.0, "empty crop")
    return classify_bgr_crop(image_bgr[y1:y2, x1:x2], center_fraction=center_fraction)


def bgr_patch_for_color(color: str, size: int = 96) -> np.ndarray:
    """Generate deterministic synthetic BGR patch for offline regression tests."""
    _require_cv2()
    hsv_values = {
        "red": (0, 220, 220),
        "orange": (16, 220, 230),
        "yellow": (30, 220, 235),
        "green": (60, 210, 210),
        "blue": (110, 210, 210),
        "purple": (142, 190, 200),
        "black": (0, 0, 25),
        "white": (0, 0, 230),
    }
    if color not in hsv_values:
        raise ValueError(f"unsupported synthetic color: {color}")
    hsv = np.zeros((size, size, 3), dtype=np.uint8)
    hsv[:, :] = hsv_values[color]
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    # Add mild deterministic brightness gradient to mimic real nonuniform lighting.
    grad = np.linspace(-12, 12, size, dtype=np.int16).reshape(1, size, 1)
    return np.clip(bgr.astype(np.int16) + grad, 0, 255).astype(np.uint8)


def read_bgr(path: Path) -> np.ndarray:
    _require_cv2()
    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError(f"failed to read image: {path}")
    return img
