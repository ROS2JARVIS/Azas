#!/usr/bin/env python3
"""Compatibility wrapper for RGB-D grasp-frame export."""

from __future__ import annotations

import runpy
from pathlib import Path


TARGET = Path(__file__).resolve().parent / "perception" / "export_grasp_frame.py"


if __name__ == "__main__":
    runpy.run_path(str(TARGET), run_name="__main__")
