#!/usr/bin/env python3
"""Offline regression gate for dispenser_color_scan.py.

Creates synthetic per-dispenser images, runs dispenser_color_scan.py, and
verifies that all 4 dispenser IDs are present with valid color labels.
"""
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools" / "perception" / "dispenser_color_scan.py"
VALID_COLORS = {"red", "orange", "yellow", "green", "blue", "purple", "black", "white", "unknown"}
EXPECTED_IDS = {"1", "2", "3", "4"}
# One synthetic color per dispenser slot for the offline test
DISPENSER_COLORS = {"1": "red", "2": "green", "3": "yellow", "4": "blue"}


def fail(msg: str) -> int:
    print(f"[FAIL] {msg}")
    return 1


def create_synthetic_images(image_dir: Path) -> None:
    sys.path.insert(0, str(ROOT))
    from tools.perception.color_discrimination import bgr_patch_for_color  # noqa: E402
    try:
        import cv2  # type: ignore
    except ImportError as exc:
        raise RuntimeError(f"opencv-python required for synthetic image creation: {exc}") from exc

    image_dir.mkdir(parents=True, exist_ok=True)
    for did, color in DISPENSER_COLORS.items():
        patch = bgr_patch_for_color(color)
        out_path = image_dir / f"dispenser_{did}.png"
        cv2.imwrite(str(out_path), patch)


def main() -> int:
    with tempfile.TemporaryDirectory() as tmp_dir:
        image_dir = Path(tmp_dir) / "images"
        output_path = Path(tmp_dir) / "dispenser_color_map.json"

        try:
            create_synthetic_images(image_dir)
        except Exception as exc:
            return fail(f"synthetic image creation failed: {exc}")

        proc = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                "--image-dir", str(image_dir),
                "--output", str(output_path),
            ],
            cwd=str(ROOT),
            text=True,
            capture_output=True,
        )
        if proc.stdout:
            print(proc.stdout, end="")
        if proc.stderr:
            print(proc.stderr, end="", file=sys.stderr)

        if proc.returncode != 0:
            return fail(f"dispenser_color_scan.py exited with code {proc.returncode}")

        if not output_path.exists():
            return fail(f"output JSON not created: {output_path}")

        try:
            color_map: dict[str, str] = json.loads(output_path.read_text(encoding="utf-8"))
        except Exception as exc:
            return fail(f"output JSON is not valid: {exc}")

        missing_ids = EXPECTED_IDS - set(color_map.keys())
        if missing_ids:
            return fail(f"missing dispenser IDs in output: {sorted(missing_ids)}")

        invalid_colors = {did: c for did, c in color_map.items() if c not in VALID_COLORS}
        if invalid_colors:
            return fail(f"invalid color values in output: {invalid_colors}")

        print(f"[PASS] dispenser_color_scan produced valid map: {color_map}")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
