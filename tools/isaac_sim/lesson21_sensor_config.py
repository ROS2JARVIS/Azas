#!/usr/bin/env python3
"""Shared configuration helpers for the lesson 21 Isaac Sim sensor examples.

The helpers in this file are intentionally Isaac-free so they can be unit
tested on a normal ROS workstation. Standalone scripts import Isaac Sim modules
inside ``main()`` only, because those modules are available through Isaac Sim's
``python.sh`` rather than the system Python.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class OpenCVCameraCalibration:
    width: int
    height: int
    camera_matrix: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]
    distortion_coefficients: tuple[float, ...]
    pixel_size_um: float = 3.0
    f_stop: float = 1.8
    focus_distance_m: float = 0.6
    clipping_range_m: tuple[float, float] = (0.05, 1.0e5)

    @property
    def fx(self) -> float:
        return float(self.camera_matrix[0][0])

    @property
    def fy(self) -> float:
        return float(self.camera_matrix[1][1])

    @property
    def cx(self) -> float:
        return float(self.camera_matrix[0][2])

    @property
    def cy(self) -> float:
        return float(self.camera_matrix[1][2])

    @property
    def horizontal_aperture_mm(self) -> float:
        return self.pixel_size_um * self.width * 1.0e-3

    @property
    def vertical_aperture_mm(self) -> float:
        return self.pixel_size_um * self.height * 1.0e-3

    @property
    def focal_length_mm(self) -> float:
        focal_x = self.fx * self.pixel_size_um * 1.0e-3
        focal_y = self.fy * self.pixel_size_um * 1.0e-3
        return (focal_x + focal_y) / 2.0


def validate_opencv_calibration(
    calibration: OpenCVCameraCalibration,
    *,
    distortion_count: int,
) -> None:
    if calibration.width <= 0 or calibration.height <= 0:
        raise ValueError("image width and height must be positive")
    if calibration.pixel_size_um <= 0:
        raise ValueError("pixel_size_um must be positive")
    if calibration.f_stop <= 0:
        raise ValueError("f_stop must be positive")
    near, far = calibration.clipping_range_m
    if near <= 0 or far <= near:
        raise ValueError("clipping_range_m must be positive and increasing")

    for row in calibration.camera_matrix:
        for value in row:
            if not math.isfinite(float(value)):
                raise ValueError("camera_matrix must contain finite values")
    if calibration.fx <= 0 or calibration.fy <= 0:
        raise ValueError("camera_matrix fx/fy must be positive")

    if len(calibration.distortion_coefficients) != distortion_count:
        raise ValueError(
            f"expected {distortion_count} distortion coefficients, "
            f"got {len(calibration.distortion_coefficients)}"
        )
    for value in calibration.distortion_coefficients:
        if not math.isfinite(float(value)):
            raise ValueError("distortion coefficients must be finite")


def opencv_pinhole_sample() -> OpenCVCameraCalibration:
    """Rational-polynomial sample matching the PDF's pinhole section."""
    return OpenCVCameraCalibration(
        width=1920,
        height=1200,
        camera_matrix=(
            (958.8, 0.0, 957.8),
            (0.0, 956.7, 589.5),
            (0.0, 0.0, 1.0),
        ),
        distortion_coefficients=(
            0.14,
            -0.03,
            -0.0002,
            -0.00003,
            0.009,
            0.5,
            -0.07,
            0.017,
        ),
    )


def opencv_fisheye_sample() -> OpenCVCameraCalibration:
    """Equidistant fisheye sample matching the PDF's fisheye section."""
    return OpenCVCameraCalibration(
        width=1920,
        height=1200,
        camera_matrix=(
            (455.8, 0.0, 943.8),
            (0.0, 454.7, 602.3),
            (0.0, 0.0, 1.0),
        ),
        distortion_coefficients=(0.05, 0.01, -0.003, -0.0005),
    )


def apply_common_camera_properties(
    camera,
    calibration: OpenCVCameraCalibration,
) -> None:
    """Apply calibration-derived optical parameters to an Isaac Sim Camera."""
    camera.set_focal_length(calibration.focal_length_mm)
    camera.set_focus_distance(calibration.focus_distance_m)
    camera.set_lens_aperture(calibration.f_stop)
    camera.set_horizontal_aperture(calibration.horizontal_aperture_mm)
    camera.set_vertical_aperture(calibration.vertical_aperture_mm)
    camera.set_clipping_range(*calibration.clipping_range_m)


def as_nested_float_list(
    values: Sequence[Sequence[float]],
) -> list[list[float]]:
    return [[float(item) for item in row] for row in values]
