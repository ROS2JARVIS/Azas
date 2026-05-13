#!/usr/bin/env python3
"""Compatibility wrapper for the supervised cup-pick entrypoint.

Implementation lives in tools/pick/run_supervised_real_single_cup_pick.py.
Keep this wrapper so existing docs, terminals, and teammates' commands do not
break while the tools directory is being organized.
"""

from __future__ import annotations

import runpy
from pathlib import Path


TARGET = Path(__file__).resolve().parent / "pick" / "run_supervised_real_single_cup_pick.py"


if __name__ == "__main__":
    runpy.run_path(str(TARGET), run_name="__main__")
