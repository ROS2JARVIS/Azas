#!/usr/bin/env python3
"""OpenCV pinhole camera example based on the 21차시 교안코드."""
from __future__ import annotations

import argparse
from pathlib import Path

from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--no-headless", dest="headless", action="store_false")
    parser.add_argument("--test", action="store_true")
    parser.add_argument("--output-dir", default=".")
    return parser.parse_args()


args = parse_args()
simulation_app = SimulationApp({"headless": args.headless})

import cv2  # noqa: E402
import isaacsim.core.utils.numpy.rotations as rot_utils  # noqa: E402
import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import DynamicCuboid  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402


width, height = 1920, 1200
camera_matrix = [
    [958.8, 0.0, 957.8],
    [0.0, 956.7, 589.5],
    [0.0, 0.0, 1.0],
]
distortion_coefficients = [
    0.14,
    -0.03,
    -0.0002,
    -0.00003,
    0.009,
    0.5,
    -0.07,
    0.017,
]
pixel_size = 3
f_stop = 1.8
focus_distance = 3.0

output_dir = Path(args.output_dir)
output_dir.mkdir(parents=True, exist_ok=True)

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

cube_positions = [
    np.array([0, 0, 0.5]),
    np.array([2, 0, 0.5]),
    np.array([0, 4, 1]),
]
cube_scale = [1.0, 1.0, 2.0]
cube_colors = [
    np.array([255, 0, 0]),
    np.array([0, 255, 0]),
    np.array([0, 0, 255]),
]

for i, (position, scale, color) in enumerate(
    zip(cube_positions, cube_scale, cube_colors)
):
    world.scene.add(
        DynamicCuboid(
            prim_path=f"/new_cube_{i}",
            name=f"cube_{i}",
            position=position,
            scale=scale * np.ones((1, 3)),
            size=1.0,
            color=color,
        )
    )

camera_position = np.array([0.0, 0.0, 4.5])
camera_rotation_as_euler = np.array([0, 90, 0])
camera = Camera(
    prim_path="/World/camera",
    position=camera_position,
    frequency=30,
    resolution=(width, height),
    orientation=rot_utils.euler_angles_to_quats(
        camera_rotation_as_euler,
        degrees=True,
    ),
)

world.reset()
camera.initialize()

((fx, _, cx), (_, fy, cy), (_, _, _)) = camera_matrix
horizontal_aperture = pixel_size * width * 1e-6
vertical_aperture = pixel_size * height * 1e-6
focal_length_x = pixel_size * fx * 1e-6
focal_length_y = pixel_size * fy * 1e-6
focal_length = (focal_length_x + focal_length_y) / 2

camera.set_focal_length(focal_length)
camera.set_focus_distance(focus_distance)
camera.set_lens_aperture(f_stop)
camera.set_horizontal_aperture(horizontal_aperture)
camera.set_vertical_aperture(vertical_aperture)
camera.set_clipping_range(0.05, 1.0e5)
camera.set_opencv_pinhole_properties(
    cx=cx,
    cy=cy,
    fx=fx,
    fy=fy,
    pinhole=distortion_coefficients,
)

for _i in range(10):
    world.step(render=True)
img = cv2.cvtColor(camera.get_rgb().astype(np.uint8), cv2.COLOR_RGB2BGR)

cube_corners = np.array(
    [
        [0.5, 0.5, 0.5],
        [-0.5, 0.5, 0.5],
        [0.5, -0.5, 0.5],
        [-0.5, -0.5, 0.5],
        [0.5, 0.5, -0.5],
        [-0.5, 0.5, -0.5],
        [0.5, -0.5, -0.5],
        [-0.5, -0.5, -0.5],
    ],
    dtype=np.float64,
)

cube_corners_world = []
for position, scale in zip(cube_positions, cube_scale):
    cube_corners_world.append(cube_corners * scale + position)
object_points_world = np.vstack(cube_corners_world)

isaac_to_cv2_mat = np.array(
    [[0, -1, 0], [0, 0, -1], [1, 0, 0]],
    dtype=np.float64,
)
isaac_to_cv2 = Rotation.from_matrix(isaac_to_cv2_mat)
cam_rotation = Rotation.from_euler(
    "xyz",
    camera_rotation_as_euler,
    degrees=True,
)
cam_rotation_cv2 = isaac_to_cv2 * cam_rotation.inv() * isaac_to_cv2.inv()
object_points_cv2 = np.expand_dims(
    object_points_world @ isaac_to_cv2_mat.T,
    axis=1,
)
rvec_cv2 = cam_rotation_cv2.as_rotvec()
tvec_cv2 = -cam_rotation_cv2.apply(camera_position @ isaac_to_cv2_mat.T)
K = np.array(camera_matrix, dtype=np.float64)
D = np.array(distortion_coefficients, dtype=np.float64)
image_points, _ = cv2.projectPoints(
    object_points_cv2,
    rvec_cv2,
    tvec_cv2,
    K,
    D,
)

for i, pt in enumerate(image_points):
    cube_color = cube_colors[i // 8].astype(np.uint8)[::-1]
    color = tuple(cube_color.tolist())
    if np.any(pt[0] < 0):
        continue
    cv2.circle(img, tuple(pt[0].astype(int)), 5, (0, 255, 255), -1)
    cv2.circle(img, tuple(pt[0].astype(int)), 3, color, -1)

image_path = output_dir / "camera_opencv_pinhole.png"
usd_path = output_dir / "camera_opencv_pinhole.usd"
print(f"Saving the rendered image to: {image_path}", flush=True)
cv2.imwrite(str(image_path), img)
print(f"Saving the asset to: {usd_path}", flush=True)
world.scene.stage.Export(str(usd_path))

simulation_app.close()
