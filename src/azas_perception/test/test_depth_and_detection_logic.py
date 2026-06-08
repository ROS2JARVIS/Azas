import pytest
import numpy as np

from azas_perception.depth_projection import CameraIntrinsics, pixel_depth_to_camera_point
from azas_perception.yolo_tumbler_detector_node import (
    BboxHeightStats,
    Detection2D,
    YoloTumblerDetectorNode,
)


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


def test_bbox_height_stats_use_empty_table_depth_difference():
    table = np.full((8, 8), 1.0, dtype=np.float32)
    cup = table.copy()
    cup[2:6, 2:6] = 0.92
    cup[2, 2] = 0.97
    detection = Detection2D(2, 2, 6, 6, 4, 4, 4, 4, 16, 0.9, "cup")

    stats = YoloTumblerDetectorNode._height_stats_from_depth_maps(table, cup, detection)

    assert stats is not None
    assert stats.median_m == pytest.approx(0.08)
    assert stats.p90_m == pytest.approx(0.08)
    assert stats.max_m == pytest.approx(0.08)
    assert stats.valid_ratio == pytest.approx(1.0)
    assert stats.center_median_m == pytest.approx(0.08)
    assert stats.edge_median_m == pytest.approx(0.08)


def test_height_orientation_thresholds_classify_depth_stats_when_configured():
    upright_stats = BboxHeightStats(0.070, 0.083, 0.090, 0.9, 90, 100)
    lying_stats = BboxHeightStats(0.045, 0.058, 0.060, 0.9, 90, 100)
    ambiguous_stats = BboxHeightStats(0.060, 0.070, 0.073, 0.9, 90, 100)

    assert YoloTumblerDetectorNode._classify_height_orientation(
        upright_stats,
        standing_threshold_m=0.075,
        lying_threshold_m=0.065,
        inverted_center_ratio_threshold=0.0,
        inverted_min_center_height_m=0.0,
        stat_name="p90",
        min_valid_ratio=0.1,
    ) == "upright"
    assert YoloTumblerDetectorNode._classify_height_orientation(
        lying_stats,
        standing_threshold_m=0.075,
        lying_threshold_m=0.065,
        inverted_center_ratio_threshold=0.0,
        inverted_min_center_height_m=0.0,
        stat_name="p90",
        min_valid_ratio=0.1,
    ) == "lying"
    assert YoloTumblerDetectorNode._classify_height_orientation(
        ambiguous_stats,
        standing_threshold_m=0.075,
        lying_threshold_m=0.065,
        inverted_center_ratio_threshold=0.0,
        inverted_min_center_height_m=0.0,
        stat_name="p90",
        min_valid_ratio=0.1,
    ) == "unknown"


def test_bbox_lying_is_not_promoted_by_tall_height_stat():
    assert (
        YoloTumblerDetectorNode._combine_bbox_and_height_orientation("lying", "upright")
        == "lying"
    )


def test_low_height_stat_rejects_bbox_upright_as_lying():
    assert (
        YoloTumblerDetectorNode._combine_bbox_and_height_orientation("upright", "lying")
        == "lying"
    )


def test_unknown_bbox_can_be_promoted_only_by_upright_height_stat():
    assert (
        YoloTumblerDetectorNode._combine_bbox_and_height_orientation("unknown", "upright")
        == "upright"
    )
    assert (
        YoloTumblerDetectorNode._combine_bbox_and_height_orientation("unknown", "lying")
        == "unknown"
    )
    assert (
        YoloTumblerDetectorNode._combine_bbox_and_height_orientation(
            "unknown",
            "lying",
            low_height_lie_candidate=True,
        )
        == "lying"
    )
    assert (
        YoloTumblerDetectorNode._combine_bbox_and_height_orientation("unknown", "unknown")
        == "unknown"
    )


def test_top_view_upright_can_promote_square_unknown_bbox():
    assert (
        YoloTumblerDetectorNode._combine_bbox_and_height_orientation(
            "unknown",
            "lying",
            top_view_upright=True,
        )
        == "upright"
    )
    assert (
        YoloTumblerDetectorNode._combine_bbox_and_height_orientation(
            "unknown",
            "unknown",
            top_view_upright=True,
        )
        == "upright"
    )


def test_top_view_upright_does_not_promote_bbox_lying():
    assert (
        YoloTumblerDetectorNode._combine_bbox_and_height_orientation(
            "lying",
            "upright",
            top_view_upright=True,
        )
        == "lying"
    )


def test_top_view_guard_band_requires_upright_height_stat():
    assert YoloTumblerDetectorNode._is_top_view_upright_candidate(
        0.72,
        "upright",
        aspect_min=0.85,
        aspect_max=1.15,
        guard_aspect_min=0.70,
        guard_aspect_max=1.35,
    )
    assert not YoloTumblerDetectorNode._is_top_view_upright_candidate(
        0.72,
        "lying",
        aspect_min=0.85,
        aspect_max=1.15,
        guard_aspect_min=0.70,
        guard_aspect_max=1.35,
    )


def test_height_orientation_ignores_sparse_samples():
    sparse_stats = BboxHeightStats(0.10, 0.11, 0.12, 0.05, 5, 100)

    assert YoloTumblerDetectorNode._classify_height_orientation(
        sparse_stats,
        standing_threshold_m=0.075,
        lying_threshold_m=0.065,
        inverted_center_ratio_threshold=0.0,
        inverted_min_center_height_m=0.0,
        stat_name="p90",
        min_valid_ratio=0.1,
    ) is None


def test_height_orientation_can_classify_inverted_when_thresholds_are_configured():
    inverted_stats = BboxHeightStats(
        0.080,
        0.086,
        0.091,
        0.9,
        90,
        100,
        center_median_m=0.081,
        edge_median_m=0.070,
        center_valid_ratio=0.9,
        edge_valid_ratio=0.9,
    )

    assert YoloTumblerDetectorNode._classify_height_orientation(
        inverted_stats,
        standing_threshold_m=0.075,
        lying_threshold_m=0.065,
        inverted_center_ratio_threshold=0.85,
        inverted_min_center_height_m=0.070,
        stat_name="p90",
        min_valid_ratio=0.1,
    ) == "inverted"
