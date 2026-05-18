#!/usr/bin/env python3
"""Generate YOLO cup labels from a pretrained Ultralytics detector.

This is option C for the Azas cup dataset: use a generic COCO detector to
bootstrap labels, then review them manually before training a project model.

Example:
    python3 tools/perception/auto_label_yolo_cups.py \
        --dataset /home/ssu/Downloads/yolo_cup_dataset \
        --model yolo11n.pt \
        --conf 0.25 \
        --dry-run

Remove --dry-run to write labels/*.txt. The script maps any detected COCO
"cup" class to the local dataset class id 0.
"""
from __future__ import annotations

import argparse
from pathlib import Path

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=Path("/home/ssu/Downloads/yolo_cup_dataset"))
    parser.add_argument("--model", default="yolo11n.pt", help="Ultralytics model path/name")
    parser.add_argument("--conf", type=float, default=0.25, help="minimum detection confidence")
    parser.add_argument("--imgsz", type=int, default=640, help="inference image size")
    parser.add_argument("--splits", nargs="+", default=["train", "val", "test"])
    parser.add_argument("--dry-run", action="store_true", help="report counts without writing labels")
    parser.add_argument("--overwrite", action="store_true", help="replace existing label files")
    return parser.parse_args()


def load_yolo(model_name: str):
    try:
        from ultralytics import YOLO
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "ultralytics is not installed. Install in a disposable env first, e.g.\n"
            "  python3 -m venv /tmp/azas-yolo && source /tmp/azas-yolo/bin/activate\n"
            "  pip install ultralytics\n"
            "Then rerun this script."
        ) from exc
    return YOLO(model_name)


def cup_class_ids(model) -> set[int]:
    names = getattr(model, "names", {}) or {}
    return {int(idx) for idx, name in names.items() if str(name).lower() == "cup"}


def image_paths(dataset: Path, split: str) -> list[Path]:
    image_dir = dataset / "images" / split
    if not image_dir.exists():
        return []
    return sorted(p for p in image_dir.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)


def yolo_line(box) -> str:
    xywhn = box.xywhn[0].tolist()
    x, y, w, h = (float(v) for v in xywhn)
    return f"0 {x:.6f} {y:.6f} {w:.6f} {h:.6f}"


def main() -> None:
    args = parse_args()
    dataset = args.dataset.resolve()
    if not (dataset / "images").exists():
        raise SystemExit(f"dataset images directory not found: {dataset / 'images'}")

    model = load_yolo(args.model)
    cup_ids = cup_class_ids(model)
    if not cup_ids:
        raise SystemExit(f"model has no class named 'cup': {getattr(model, 'names', {})}")

    total_images = 0
    total_written = 0
    total_empty = 0

    for split in args.splits:
        labels_dir = dataset / "labels" / split
        if not args.dry_run:
            labels_dir.mkdir(parents=True, exist_ok=True)

        for image_path in image_paths(dataset, split):
            total_images += 1
            label_path = labels_dir / f"{image_path.stem}.txt"
            if label_path.exists() and not args.overwrite:
                continue

            result = model.predict(str(image_path), conf=args.conf, imgsz=args.imgsz, verbose=False)[0]
            lines = []
            for box in result.boxes:
                cls_id = int(box.cls[0].item())
                if cls_id in cup_ids:
                    lines.append(yolo_line(box))

            if lines:
                total_written += 1
            else:
                total_empty += 1

            if not args.dry_run:
                label_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")

    mode = "dry-run" if args.dry_run else "wrote"
    print(f"{mode}: images={total_images} labeled_images={total_written} empty_images={total_empty}")
    print("Review generated labels before training; generic COCO cup detections may miss transparent cups.")


if __name__ == "__main__":
    main()
