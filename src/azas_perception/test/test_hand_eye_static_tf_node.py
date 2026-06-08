import numpy as np
import pytest

from azas_perception.hand_eye_static_tf_node import (
    compose_parent_to_published_child,
    load_hand_eye_matrix,
    matrix_from_quaternion,
    quaternion_from_matrix,
)


def test_load_hand_eye_matrix_scales_legacy_mm_translation(tmp_path):
    matrix = np.eye(4)
    matrix[:3, 3] = [34.0, 57.0, 11.0]
    path = tmp_path / "T_gripper2camera.npy"
    np.save(path, matrix)

    loaded = load_hand_eye_matrix(path, translation_scale=0.001)

    assert loaded[:3, 3] == pytest.approx([0.034, 0.057, 0.011])


def test_quaternion_matrix_roundtrip_for_180_degree_rotation():
    rotation = np.diag([-1.0, -1.0, 1.0])

    qx, qy, qz, qw = quaternion_from_matrix(rotation)
    restored = matrix_from_quaternion(qx, qy, qz, qw)

    assert restored == pytest.approx(rotation)


def test_compose_parent_to_published_child_preserves_matrix_child_pose():
    parent_from_optical = np.eye(4)
    parent_from_optical[:3, 3] = [0.10, 0.20, 0.30]
    camera_link_from_optical = np.eye(4)
    camera_link_from_optical[:3, 3] = [0.01, 0.02, 0.03]

    parent_from_camera_link = compose_parent_to_published_child(
        parent_from_optical,
        camera_link_from_optical,
    )

    assert parent_from_camera_link @ camera_link_from_optical == pytest.approx(parent_from_optical)
