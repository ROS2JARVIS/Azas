import pytest
import cv2
import numpy as np

from azas_perception.depth_projection import CameraIntrinsics, pixel_depth_to_camera_point
from azas_perception.lid_marker import (
    ImageRoi,
    RedCircleConfig,
    detect_aruco_marker,
    detect_red_circle_marker,
    quaternion_from_lid_normal,
)
from azas_perception.yolo_tumbler_detector_node import Detection2D, YoloTumblerDetectorNode


def test_pixel_depth_to_camera_point_projects_metric_coordinates():
    point = pixel_depth_to_camera_point(
        330,
        250,
        1000,
        CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0),
        depth_scale=0.001,
    )

    assert point == pytest.approx((0.02, 0.02, 1.0))


def test_pixel_depth_to_camera_point_rejects_invalid_intrinsics():
    with pytest.raises(ValueError, match="fx/fy must be positive"):
        pixel_depth_to_camera_point(
            320,
            240,
            1000,
            CameraIntrinsics(fx=0.0, fy=500.0, cx=320.0, cy=240.0),
        )


def test_bbox_orientation_thresholds_match_upright_policy():
    assert YoloTumblerDetectorNode._classify_cup_orientation(50, 61) == "upright"
    assert YoloTumblerDetectorNode._classify_cup_orientation(50, 39) == "lying"
    assert YoloTumblerDetectorNode._classify_cup_orientation(50, 50) == "unknown"


def test_largest_bbox_policy_uses_confidence_as_tie_breaker():
    first = Detection2D(0, 0, 10, 20, 5, 10, 10, 20, 200, 0.70, "cup")
    tied_area_higher_confidence = Detection2D(0, 0, 20, 10, 10, 5, 20, 10, 200, 0.80, "cup")

    assert YoloTumblerDetectorNode._is_better_detection(
        tied_area_higher_confidence,
        first,
        "largest_bbox",
    )


def test_detect_red_circle_marker_uses_lid_roi():
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    image[42:59, 42:59, 2] = 255
    roi = ImageRoi(20, 20, 80, 80)

    marker = detect_red_circle_marker(image, roi, RedCircleConfig(min_area_px=20.0))

    assert marker is not None
    assert marker.center_u == pytest.approx(50, abs=1)
    assert marker.center_v == pytest.approx(50, abs=1)
    assert marker.circularity > 0.7


def test_detect_aruco_marker_uses_configured_dictionary_and_roi():
    image = np.full((160, 160, 3), 255, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_image = cv2.aruco.generateImageMarker(dictionary, 7, 60)
    image[50:110, 50:110] = cv2.cvtColor(marker_image, cv2.COLOR_GRAY2BGR)
    roi = ImageRoi(30, 30, 130, 130)

    marker = detect_aruco_marker(image, roi, dictionary_name="DICT_4X4_50", marker_id=7)

    assert marker is not None
    assert marker.marker_id == 7
    assert marker.center_u == pytest.approx(80, abs=1)
    assert marker.center_v == pytest.approx(80, abs=1)
    assert marker.side_px == pytest.approx(59, abs=2)


def test_lid_normal_quaternion_points_local_z_to_normal():
    qx, qy, qz, qw = quaternion_from_lid_normal(np.array([0.0, 0.0, -1.0]))
    local_z = np.array([
        2.0 * (qx * qz + qy * qw),
        2.0 * (qy * qz - qx * qw),
        1.0 - 2.0 * (qx * qx + qy * qy),
    ])

    assert local_z == pytest.approx(np.array([0.0, 0.0, -1.0]))
