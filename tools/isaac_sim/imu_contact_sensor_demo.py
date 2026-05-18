#!/usr/bin/env python3
"""Create IMU and Contact sensors on a falling cube and print readings."""
from __future__ import annotations

import argparse


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--test", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})

    import numpy as np
    from isaacsim.core.api.objects import DynamicCuboid
    from isaacsim.core.api.objects.ground_plane import GroundPlane
    from isaacsim.core.api.physics_context import PhysicsContext
    from isaacsim.sensors.physics import ContactSensor, IMUSensor, _sensor

    PhysicsContext()
    GroundPlane(
        prim_path="/World/groundPlane",
        size=10,
        color=np.array([0.5, 0.5, 0.5]),
    )
    DynamicCuboid(
        prim_path="/World/Cube",
        position=np.array([-0.5, -0.2, 1.0]),
        scale=np.array([0.5, 0.5, 0.5]),
        color=np.array([0.2, 0.3, 0.0]),
    )
    imu_sensor = IMUSensor(
        prim_path="/World/Cube/Imu_Sensor",
        name="Imu_Sensor",
        frequency=60,
        translation=np.array([0.0, 0.0, 0.0]),
    )
    contact_sensor = ContactSensor(
        prim_path="/World/Cube/Contact_Sensor",
        name="Contact_Sensor",
        frequency=60,
        translation=np.array([0.0, 0.0, 0.0]),
        min_threshold=0.0,
        max_threshold=10000000.0,
        radius=-1.0,
    )
    contact_interface = _sensor.acquire_contact_sensor_interface()

    for _ in range(20 if args.test else 240):
        simulation_app.update()
        imu_sensor.update()
        contact_sensor.update()

    print({"imu": imu_sensor.get_current_frame()})
    print({"contact_frame": contact_sensor.get_current_frame()})
    print(
        {
            "contact_reading": contact_interface.get_sensor_reading(
                "/World/Cube/Contact_Sensor",
                use_latest_data=True,
            )
        }
    )
    simulation_app.close()


if __name__ == "__main__":
    main()
