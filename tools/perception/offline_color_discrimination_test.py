#!/usr/bin/env python3
"""Run color discrimination tests without a camera.

Modes:
  1. Synthetic patches for red/orange/yellow/green/blue/purple/black/white.
  2. Optional image boxes from a CSV: image_path,expected_color,x1,y1,x2,y2.

Outputs a CSV and preview image directory under outputs/color_discrimination/.
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from tools.perception.color_discrimination import (  # noqa: E402
    COLOR_ORDER,
    bgr_patch_for_color,
    classify_bgr_crop,
    classify_image_box,
    read_bgr,
)

try:
    import cv2  # type: ignore
except Exception:
    cv2 = None

OUT_DIR = ROOT / "outputs" / "color_discrimination"
FIELDS = ["source", "expected_color", "predicted_color", "pass", "h_median", "s_median", "v_median", "confidence", "reason", "preview_path"]


def write_preview(path: Path, image: np.ndarray, label: str) -> None:
    if cv2 is None:
        return
    img = image.copy()
    cv2.putText(img, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(img, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 1, cv2.LINE_AA)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), img)


def run_synthetic(preview_dir: Path) -> list[dict]:
    rows = []
    for color in [c for c in COLOR_ORDER if c != "unknown"]:
        patch = bgr_patch_for_color(color)
        result = classify_bgr_crop(patch)
        preview = preview_dir / f"synthetic_{color}_pred_{result.color}.png"
        write_preview(preview, patch, f"gt={color} pred={result.color}")
        rows.append({
            "source": "synthetic",
            "expected_color": color,
            "predicted_color": result.color,
            "pass": result.color == color,
            "h_median": f"{result.h_median:.2f}",
            "s_median": f"{result.s_median:.2f}",
            "v_median": f"{result.v_median:.2f}",
            "confidence": f"{result.confidence:.2f}",
            "reason": result.reason,
            "preview_path": str(preview),
        })
    return rows


def run_box_csv(path: Path, preview_dir: Path) -> list[dict]:
    if not path.exists():
        return []
    rows = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader, start=1):
            image_path = Path(row["image_path"])
            expected = str(row["expected_color"]).strip().lower()
            xyxy = [float(row[c]) for c in ["x1", "y1", "x2", "y2"]]
            image = read_bgr(image_path)
            result = classify_image_box(image, xyxy)
            x1, y1, x2, y2 = [int(round(x)) for x in xyxy]
            crop = image[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
            preview = preview_dir / f"box_{i:03d}_{image_path.stem}_gt_{expected}_pred_{result.color}.png"
            write_preview(preview, crop if crop.size else image, f"gt={expected} pred={result.color}")
            rows.append({
                "source": str(image_path),
                "expected_color": expected,
                "predicted_color": result.color,
                "pass": result.color == expected,
                "h_median": f"{result.h_median:.2f}",
                "s_median": f"{result.s_median:.2f}",
                "v_median": f"{result.v_median:.2f}",
                "confidence": f"{result.confidence:.2f}",
                "reason": result.reason,
                "preview_path": str(preview),
            })
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--box-csv", default="", help="Optional CSV with image_path,expected_color,x1,y1,x2,y2")
    parser.add_argument("--output", default=str(OUT_DIR / "color_discrimination_results.csv"))
    args = parser.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    preview_dir = OUT_DIR / "preview"
    rows = run_synthetic(preview_dir)
    if args.box_csv:
        rows.extend(run_box_csv(Path(args.box_csv), preview_dir))

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    total = len(rows)
    passed = sum(str(r["pass"]) == "True" for r in rows)
    print(f"[Azas] offline color discrimination: {passed}/{total} passed")
    print(out)
    return 0 if passed == total else 1


if __name__ == "__main__":
    raise SystemExit(main())
