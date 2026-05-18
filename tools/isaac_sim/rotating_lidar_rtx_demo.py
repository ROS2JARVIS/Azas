#!/usr/bin/env python3
"""Create an RTX rotary lidar and print scan data."""
from __future__ import annotations

import argparse


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--test", action="store_true")
    parser.add_argument("--config", default="Example_Rotary")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})

    import carb
    import numpy as np
    import omni.kit.commands
    import omni.replicator.core as rep
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid
    from isaacsim.sensors.rtx import LidarRtx

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    world.scene.add(
        DynamicCuboid(
            prim_path="/World/lidar_target",
            name="lidar_target",
            position=np.array([3.0, 0.0, 0.5]),
            scale=np.array([0.4, 0.4, 1.0]),
            color=np.array([0.1, 0.7, 0.2]),
        )
    )

    try:
        lidar = LidarRtx(
            prim_path="/World/rtx_lidar",
            name="rtx_lidar",
            translation=np.array([0.0, 0.0, 1.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            config_file_name=args.config,
        )
        render_product_path = lidar.get_render_product_path()
    except Exception as exc:
        carb.log_warn(
            "LidarRtx wrapper unavailable, falling back to command API: "
            f"{exc}"
        )
        _, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/rtx_lidar",
            parent="/World",
            config=args.config,
            translation=(0.0, 0.0, 1.0),
            orientation=(1.0, 0.0, 0.0, 0.0),
        )
        render_product_path = rep.create.render_product(
            sensor.GetPath(),
            [1, 1],
        )

    point_cloud = rep.AnnotatorRegistry.get_annotator(
        "IsaacExtractRTXSensorPointCloudNoAccumulator"
    )
    point_cloud.attach([render_product_path])
    flat_scan = rep.AnnotatorRegistry.get_annotator(
        "IsaacComputeRTXLidarFlatScan"
    )
    flat_scan.attach([render_product_path])

    world.reset()
    for _ in range(10 if args.test else 240):
        world.step(render=True)

    print({
        "point_cloud": point_cloud.get_data(),
        "flat_scan": flat_scan.get_data(),
    })
    simulation_app.close()


if __name__ == "__main__":
    main()
