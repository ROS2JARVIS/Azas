#!/usr/bin/env python3
"""Lesson 21 Camera sensor standalone demo for Isaac Sim.

Run with Isaac Sim's Python, for example:

    /path/to/isaac-sim/python.sh tools/isaac_sim/camera_sensor_demo.py --test

The script mirrors the PDF exercise: create a camera prim, collect RGBA frames,
enable motion vectors, and demonstrate world/image coordinate conversion.
"""
from __future__ import annotations

import argparse
from pathlib import Path


def summarize_frame(frame: dict) -> dict:
    summary = {}
    for key, value in frame.items():
        shape = getattr(value, "shape", None)
        dtype = getattr(value, "dtype", None)
        if shape is not None:
            summary[key] = {
                "shape": tuple(int(item) for item in shape),
                "dtype": str(dtype),
            }
        else:
            summary[key] = type(value).__name__
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--test",
        action="store_true",
        help="run a bounded smoke",
    )
    parser.add_argument("--disable-output", action="store_true")
    parser.add_argument("--output-dir", default="isaac_camera_frames")
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--save-every", type=int, default=5)
    parser.add_argument("--warmup-frames", type=int, default=3)
    parser.add_argument("--verbose-frame-data", action="store_true")
    parser.add_argument("--show-plot", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})

    import matplotlib.image as mpimg
    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid
    import isaacsim.core.utils.numpy.rotations as rot_utils
    from isaacsim.sensors.camera import Camera

    output_dir = Path(args.output_dir)
    if not args.disable_output:
        output_dir.mkdir(parents=True, exist_ok=True)

    world = World(stage_units_in_meters=1.0)
    cube_1 = world.scene.add(
        DynamicCuboid(
            prim_path="/World/new_cube_1",
            name="cube_1",
            position=np.array([5.0, 3.0, 1.0]),
            scale=np.array([0.6, 0.5, 0.2]),
            size=1.0,
            color=np.array([255, 0, 0]),
        )
    )
    cube_2 = world.scene.add(
        DynamicCuboid(
            prim_path="/World/new_cube_2",
            name="cube_2",
            position=np.array([5.0, -1.0, 3.0]),
            scale=np.array([0.1, 0.1, 0.1]),
            size=1.0,
            color=np.array([0, 0, 255]),
            linear_velocity=np.array([0.0, 0.0, 0.4]),
        )
    )

    camera = Camera(
        prim_path="/World/camera",
        position=np.array([0.0, 0.0, 25.0]),
        frequency=20,
        resolution=(256, 256),
        orientation=rot_utils.euler_angles_to_quats(
            np.array([0, 90, 0]),
            degrees=True,
        ),
    )

    world.scene.add_default_ground_plane()
    world.reset()
    camera.initialize()
    camera.add_motion_vectors_to_frame()
    output_label = output_dir if not args.disable_output else "<disabled>"
    print(
        "[camera] initialized prim=/World/camera resolution=256x256 "
        f"output_dir={output_label}",
        flush=True,
    )

    reset_needed = False
    frame_index = 0
    if args.frames > 0:
        max_frames = args.frames
    else:
        max_frames = 15 if args.test else 600
    saved_count = 0
    pyplot = None
    if args.show_plot:
        import matplotlib.pyplot as pyplot

    while simulation_app.is_running() and frame_index < max_frames:
        world.step(render=True)
        should_sample = (
            frame_index >= args.warmup_frames
            and args.save_every > 0
            and (frame_index - args.warmup_frames) % args.save_every == 0
        )
        if should_sample:
            current_frame = camera.get_current_frame()
            points_2d = camera.get_image_coords_from_world_points(
                np.array([
                    cube_2.get_world_pose()[0],
                    cube_1.get_world_pose()[0],
                ])
            )
            points_3d = camera.get_world_points_from_image_coords(
                points_2d,
                np.array([24.94, 24.9]),
            )
            if not args.disable_output:
                if args.verbose_frame_data:
                    print(current_frame)
                else:
                    print(
                        f"[camera] frame={frame_index} "
                        f"{summarize_frame(current_frame)}",
                        flush=True,
                    )
                print(
                    f"[camera] world_to_image={points_2d.tolist()}",
                    flush=True,
                )
                print(
                    f"[camera] image_to_world={points_3d.tolist()}",
                    flush=True,
                )
                rgba = np.asarray(camera.get_rgba())
                if rgba.ndim == 1 and rgba.size == 256 * 256 * 4:
                    rgba = rgba.reshape((256, 256, 4))
                if rgba.ndim != 3 or rgba.shape[2] < 3:
                    print(
                        "[camera] skipped image save: "
                        f"unexpected rgba shape={rgba.shape}",
                        flush=True,
                    )
                    continue
                output_path = (
                    output_dir / f"camera_frame_{frame_index:03d}.png"
                )
                mpimg.imsave(output_path, rgba[:, :, :3])
                saved_count += 1
                print(f"[camera] saved {output_path}", flush=True)
                if pyplot is not None:
                    pyplot.imshow(rgba[:, :, :3])
                    pyplot.axis("off")
                    pyplot.show(block=False)
                    pyplot.pause(0.001)
                motion_vectors = camera.get_current_frame().get(
                    "motion_vectors"
                )
                if motion_vectors is not None:
                    print(
                        "[camera] motion_vectors "
                        f"shape={motion_vectors.shape} "
                        f"dtype={motion_vectors.dtype}",
                        flush=True,
                    )
        if world.is_stopped() and not reset_needed:
            reset_needed = True
        if world.is_playing() and reset_needed:
            world.reset()
            reset_needed = False
        frame_index += 1

    if not args.disable_output:
        print(
            f"[camera] done frames={frame_index} saved_images={saved_count}",
            flush=True,
        )
    simulation_app.close()


if __name__ == "__main__":
    main()
