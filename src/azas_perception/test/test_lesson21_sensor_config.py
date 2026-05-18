from pathlib import Path
import sys

import pytest


ISAAC_TOOLS = Path(__file__).resolve().parents[3] / "tools" / "isaac_sim"
sys.path.insert(0, str(ISAAC_TOOLS))

from lesson21_sensor_config import (  # noqa: E402
    OpenCVCameraCalibration,
    opencv_fisheye_sample,
    opencv_pinhole_sample,
    validate_opencv_calibration,
)


def test_opencv_fisheye_sample_matches_pdf_model_shape():
    calibration = opencv_fisheye_sample()

    validate_opencv_calibration(calibration, distortion_count=4)

    assert calibration.width == 1920
    assert calibration.height == 1200
    assert len(calibration.distortion_coefficients) == 4
    assert calibration.horizontal_aperture_mm == pytest.approx(5.76)
    assert calibration.vertical_aperture_mm == pytest.approx(3.6)
    assert calibration.focal_length_mm == pytest.approx(1.36575)


def test_opencv_pinhole_sample_uses_rational_polynomial_coefficients():
    calibration = opencv_pinhole_sample()

    validate_opencv_calibration(calibration, distortion_count=8)

    assert len(calibration.distortion_coefficients) == 8


def test_validate_opencv_calibration_rejects_wrong_distortion_count():
    calibration = OpenCVCameraCalibration(
        width=640,
        height=480,
        camera_matrix=(
            (500.0, 0.0, 320.0),
            (0.0, 500.0, 240.0),
            (0.0, 0.0, 1.0),
        ),
        distortion_coefficients=(0.1, 0.2),
    )

    with pytest.raises(ValueError, match="expected 4 distortion coefficients"):
        validate_opencv_calibration(calibration, distortion_count=4)
