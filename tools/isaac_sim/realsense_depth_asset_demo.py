#!/usr/bin/env python3
"""Load a RealSense D455 depth sensor asset and attach a depth annotator."""
from __future__ import annotations

import argparse


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--test", action="store_true")
    parser.add_argument("--prim-path", default="/World/realsense_d455")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})

    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.sensors.camera import SingleViewDepthSensorAsset
    from isaacsim.storage.native import get_assets_root_path

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("Isaac Sim assets root is unavailable")

    asset = SingleViewDepthSensorAsset(
        prim_path=args.prim_path,
        name="realsense_d455_depth_asset",
        usd_path=f"{assets_root}/Isaac/Sensors/Intel/RealSense/rsd455.usd",
        position=np.array([0.0, 0.0, 1.0]),
    )
    world.reset()
    asset.initialize()
    sensor_paths = asset.get_sensor_prim_paths()
    print({"sensor_prim_paths": sensor_paths})

    if sensor_paths:
        depth_sensor = asset.get_sensor(sensor_paths[0])
        depth_sensor.attach_annotator("DepthSensorDistance")
        for _ in range(5 if args.test else 120):
            world.step(render=True)
        print(depth_sensor.get_current_frame())

    simulation_app.close()


if __name__ == "__main__":
    main()
