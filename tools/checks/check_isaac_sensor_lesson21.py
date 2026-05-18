#!/usr/bin/env python3
"""Static and config checks for the lesson 21 Isaac Sim sensor scripts."""
from __future__ import annotations

import ast
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
ISAAC_TOOLS = ROOT / "tools" / "isaac_sim"
sys.path.insert(0, str(ISAAC_TOOLS))

from lesson21_sensor_config import (  # noqa: E402
    opencv_fisheye_sample,
    opencv_pinhole_sample,
    validate_opencv_calibration,
)


REQUIRED_SCRIPTS = {
    "camera_sensor_demo.py": (
        "Camera",
        "get_current_frame",
        "get_rgba",
        "add_motion_vectors_to_frame",
    ),
    "camera_opencv_fisheye.py": ("set_opencv_fisheye_properties",),
    "camera_opencv_pinhole.py": ("set_opencv_pinhole_properties",),
    "realsense_depth_asset_demo.py": (
        "SingleViewDepthSensorAsset",
        "DepthSensorDistance",
    ),
    "rotating_lidar_rtx_demo.py": (
        "LidarRtx",
        "IsaacComputeRTXLidarFlatScan",
    ),
    "imu_contact_sensor_demo.py": (
        "IMUSensor",
        "ContactSensor",
        "get_sensor_reading",
    ),
}


def require(condition: bool, message: str, failures: list[str]) -> None:
    if condition:
        print(f"[PASS] {message}")
    else:
        failures.append(message)
        print(f"[FAIL] {message}")


def main() -> int:
    failures: list[str] = []
    validate_opencv_calibration(opencv_fisheye_sample(), distortion_count=4)
    validate_opencv_calibration(opencv_pinhole_sample(), distortion_count=8)
    print("[PASS] OpenCV fisheye/pinhole sample calibrations are valid")

    for script_name, required_tokens in REQUIRED_SCRIPTS.items():
        path = ISAAC_TOOLS / script_name
        require(path.exists(), f"{script_name} exists", failures)
        if not path.exists():
            continue
        source = path.read_text(encoding="utf-8")
        ast.parse(source, filename=str(path))
        print(f"[PASS] {script_name} parses")
        for token in required_tokens:
            require(
                token in source,
                f"{script_name} includes {token}",
                failures,
            )

    if failures:
        print(
            "[FAIL] lesson 21 Isaac sensor check failed: "
            f"{len(failures)} issue(s)"
        )
        return 1
    print(
        "[PASS] lesson 21 Isaac sensor scripts are present "
        "and statically valid"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
