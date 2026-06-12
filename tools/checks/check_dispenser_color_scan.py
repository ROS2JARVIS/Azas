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
EXPECTED_FALSE_POSITIVE_MAP = {"1": "red", "2": "yellow", "3": "blue", "4": "green"}
EXPECTED_ARBITRARY_ORDER_MAP = {"1": "red", "2": "yellow", "3": "green", "4": "blue"}


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


def check_visible_handle_false_positive_filter() -> tuple[bool, str]:
    sys.path.insert(0, str(ROOT))
    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore
    except ImportError as exc:
        return False, f"opencv/numpy required for visible-handle test: {exc}"

    from tools.perception.color_discrimination import bgr_patch_for_color  # noqa: E402
    from tools.perception.dispenser_color_scan import detect_visible_handle_color_map  # noqa: E402

    def fill_box(frame: "np.ndarray", x: int, y: int, w: int, h: int, color: str) -> None:
        patch = bgr_patch_for_color(color, size=max(w, h))
        frame[y : y + h, x : x + w] = cv2.resize(patch, (w, h))

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:, :] = (45, 45, 45)
    # This upper, horizontal yellow blob reproduces the 2026-06-12 false
    # positive shape.  It must not become dispenser 1.
    fill_box(frame, 214, 38, 86, 31, "yellow")
    fill_box(frame, 254, 87, 36, 62, "red")
    fill_box(frame, 320, 89, 28, 63, "yellow")
    fill_box(frame, 380, 91, 32, 66, "blue")
    fill_box(frame, 445, 83, 33, 68, "green")

    color_map = detect_visible_handle_color_map(frame, ["1", "2", "3", "4"])
    if color_map != EXPECTED_FALSE_POSITIVE_MAP:
        return False, f"visible-handle map mismatch: got {color_map}, expected {EXPECTED_FALSE_POSITIVE_MAP}"

    frame_swapped = np.zeros((480, 640, 3), dtype=np.uint8)
    frame_swapped[:, :] = (45, 45, 45)
    fill_box(frame_swapped, 254, 87, 36, 62, "red")
    fill_box(frame_swapped, 320, 89, 28, 63, "yellow")
    fill_box(frame_swapped, 380, 91, 32, 66, "green")
    fill_box(frame_swapped, 445, 83, 33, 68, "blue")
    color_map = detect_visible_handle_color_map(frame_swapped, ["1", "2", "3", "4"])
    if color_map != EXPECTED_ARBITRARY_ORDER_MAP:
        return False, (
            "visible-handle arbitrary order mismatch: "
            f"got {color_map}, expected {EXPECTED_ARBITRARY_ORDER_MAP}"
        )
    return True, f"visible-handle false-positive filter map: {color_map}"


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

        ok, detail = check_visible_handle_false_positive_filter()
        if not ok:
            return fail(detail)
        print(f"[PASS] {detail}")
        print(f"[PASS] dispenser_color_scan produced valid map: {color_map}")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
