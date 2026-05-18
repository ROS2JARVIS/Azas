import pytest

from azas_perception.depth_projection import CameraIntrinsics, pixel_depth_to_camera_point
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

