#!/usr/bin/env python3
"""Static/offline regression gate for dispenser color discrimination.

Runs without a camera and without robot hardware. It verifies synthetic color
patches classify into the expected HSV bins.
"""
from __future__ import annotations

import csv
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RESULT = ROOT / "outputs" / "color_discrimination" / "color_discrimination_results.csv"
SCRIPT = ROOT / "tools" / "perception" / "offline_color_discrimination_test.py"
EXPECTED = {"red", "orange", "yellow", "green", "blue", "purple", "black", "white"}


def fail(msg: str) -> int:
    print(f"[FAIL] {msg}")
    return 1


def main() -> int:
    proc = subprocess.run([sys.executable, str(SCRIPT)], cwd=str(ROOT), text=True, capture_output=True)
    print(proc.stdout, end="")
    if proc.stderr:
        print(proc.stderr, end="", file=sys.stderr)
    if proc.returncode != 0:
        return fail("offline_color_discrimination_test.py returned non-zero")
    if not RESULT.exists():
        return fail(f"missing result CSV: {RESULT}")
    rows = list(csv.DictReader(RESULT.open(encoding="utf-8")))
    got = {r["expected_color"] for r in rows if r.get("source") == "synthetic"}
    if got != EXPECTED:
        return fail(f"synthetic color set mismatch: got={sorted(got)} expected={sorted(EXPECTED)}")
    bad = [r for r in rows if str(r.get("pass")) != "True"]
    if bad:
        return fail(f"color classification failures: {bad}")
    print("[PASS] offline HSV color discrimination works without camera")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
