#!/usr/bin/env python3
"""Compatibility wrapper for the side-grasp planning candidate sweep."""

from __future__ import annotations

import runpy
from pathlib import Path


TARGET = Path(__file__).resolve().parent / "pick" / "sweep_side_grasp_planning_candidates.py"


if __name__ == "__main__":
    runpy.run_path(str(TARGET), run_name="__main__")
