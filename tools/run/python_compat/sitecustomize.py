"""Python import guard for ROS Humble binary extension compatibility.

The field machine has YOLO/Torch packages in the user site-packages, but ROS
Humble's cv_bridge and Ubuntu scipy are built against the system NumPy 1.x ABI.
Preload the ABI-sensitive modules from the system path before user packages can
pull in NumPy 2.x.
"""

from __future__ import annotations

import importlib
import sys


SYSTEM_DIST_PACKAGES = "/usr/lib/python3/dist-packages"
FORCE_SYSTEM_MODULES = ("numpy", "cv2", "scipy")


def _preload_from_system(module_name: str) -> None:
    if module_name in sys.modules:
        return

    original_path = list(sys.path)
    try:
        sys.path = [
            SYSTEM_DIST_PACKAGES,
            *[path for path in original_path if path != SYSTEM_DIST_PACKAGES],
        ]
        importlib.import_module(module_name)
    finally:
        sys.path = original_path


for _module_name in FORCE_SYSTEM_MODULES:
    try:
        _preload_from_system(_module_name)
    except Exception:
        # Let the real import site raise its normal traceback later.
        pass
