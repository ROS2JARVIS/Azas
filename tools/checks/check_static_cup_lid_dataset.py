#!/usr/bin/env python3
"""Check the offline cup/lid photo dataset before live camera bringup.

This is intentionally a no-hardware gate. It reads static files under the
operator-provided dataset directory and reports whether the dataset can support
the cup/lid detection path. It does not infer robot/base-frame coordinates.
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path


DEFAULT_DATASET = Path("/home/ssu/Downloads/로봇 데이터")
EXPECTED_FOLDERS = (
    "cup_only",
    "lid",
    "cup_lid_together",
    "demo_environment",
    "cup_noise",
)
EXPECTED_CLASSES = {"cup", "lid"}


@dataclass(frozen=True)
class DetectionRow:
    image: str
    source: str
    klass: str
    confidence: float | None
    cx_px: float
    cy_px: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate static cup/lid dataset readiness without hardware."
    )
    parser.add_argument(
        "--dataset",
        type=Path,
        default=DEFAULT_DATASET,
        help=f"dataset root, default: {DEFAULT_DATASET}",
    )
    parser.add_argument(
        "--min-confidence",
        type=float,
        default=0.60,
        help="minimum confidence used for low-confidence warnings",
    )
    return parser.parse_args()


def read_rows(path: Path) -> list[DetectionRow]:
    rows: list[DetectionRow] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            confidence = raw.get("confidence") or ""
            rows.append(
                DetectionRow(
                    image=raw["image"],
                    source=raw["source"],
                    klass=raw["class"],
                    confidence=float(confidence) if confidence else None,
                    cx_px=float(raw["cx_px"]),
                    cy_px=float(raw["cy_px"]),
                )
            )
    return rows


def summarize_predictions(rows: list[DetectionRow], min_confidence: float) -> tuple[list[str], list[str]]:
    report: list[str] = []
    warnings: list[str] = []
    pred_rows = [row for row in rows if row.source == "pred"]
    gt_rows = [row for row in rows if row.source == "gt"]
    pred_by_image: dict[str, list[DetectionRow]] = defaultdict(list)
    gt_by_image: dict[str, list[DetectionRow]] = defaultdict(list)
    for row in pred_rows:
        pred_by_image[row.image].append(row)
    for row in gt_rows:
        gt_by_image[row.image].append(row)

    class_counts = Counter(row.klass for row in pred_rows)
    avg_conf = sum(row.confidence or 0.0 for row in pred_rows) / max(1, len(pred_rows))
    low_conf = [row for row in pred_rows if (row.confidence or 0.0) < min_confidence]

    missing_by_class = Counter()
    class_mismatch: list[str] = []
    duplicate_class_predictions: list[str] = []
    for image, gt_for_image in gt_by_image.items():
        pred_classes = Counter(row.klass for row in pred_by_image.get(image, []))
        gt_classes = Counter(row.klass for row in gt_for_image)
        for klass, count in gt_classes.items():
            if pred_classes[klass] < count:
                missing_by_class[klass] += count - pred_classes[klass]
        unexpected = set(pred_classes) - set(gt_classes)
        if unexpected:
            class_mismatch.append(f"{image}: unexpected predicted class {sorted(unexpected)}")
        duplicates = [klass for klass, count in pred_classes.items() if count > gt_classes.get(klass, 0)]
        if duplicates:
            duplicate_class_predictions.append(f"{image}: duplicate/extra {duplicates}")

    report.append(f"prediction_rows={len(pred_rows)} gt_rows={len(gt_rows)}")
    report.append(f"predicted_classes={dict(sorted(class_counts.items()))}")
    report.append(f"average_confidence={avg_conf:.3f}")
    report.append(f"low_confidence_lt_{min_confidence:.2f}={len(low_conf)}")
    report.append(f"missing_gt_matches={dict(sorted(missing_by_class.items()))}")
    report.append(f"unexpected_class_images={len(class_mismatch)}")
    report.append(f"duplicate_or_extra_class_images={len(duplicate_class_predictions)}")

    if low_conf:
        warnings.append(
            "low confidence predictions: "
            + ", ".join(
                f"{row.image}:{row.klass}:{row.confidence:.2f}" for row in low_conf[:5] if row.confidence is not None
            )
        )
    if class_mismatch:
        warnings.append("class mismatch samples: " + "; ".join(class_mismatch[:5]))
    if missing_by_class:
        warnings.append(f"missing class matches against GT: {dict(sorted(missing_by_class.items()))}")

    return report, warnings


def main() -> int:
    args = parse_args()
    root = args.dataset.expanduser()
    failures: list[str] = []
    warnings: list[str] = []

    print(f"[Azas] Static cup/lid dataset check")
    print(f"dataset={root}")

    if not root.exists():
        print(f"[FAIL] dataset root missing: {root}")
        return 1

    for folder in EXPECTED_FOLDERS:
        path = root / "cup_lid_dataset" / folder
        count = len(list(path.glob("*.jpg"))) if path.exists() else 0
        print(f"{folder}_jpg={count}")
        if count == 0:
            failures.append(f"missing images for {folder}")

    weights = sorted(root.glob("**/best.pt"))
    print(f"weights={len(weights)}")
    for weight in weights:
        print(f"weight={weight} size_bytes={weight.stat().st_size}")
    if not weights:
        failures.append("no best.pt model weights found")

    yaml_path = root / "train_val_test.seperate" / "data.yaml"
    if yaml_path.exists():
        yaml_text = yaml_path.read_text(encoding="utf-8")
        class_ok = all(name in yaml_text for name in EXPECTED_CLASSES)
        print(f"data_yaml={yaml_path}")
        print(f"data_yaml_classes_ok={class_ok}")
        if not class_ok:
            failures.append("data.yaml does not contain both cup and lid classes")
    else:
        failures.append(f"missing data.yaml: {yaml_path}")

    centers_path = root / "test_predictions" / "centers.csv"
    if centers_path.exists():
        report, pred_warnings = summarize_predictions(read_rows(centers_path), args.min_confidence)
        for line in report:
            print(line)
        warnings.extend(pred_warnings)
    else:
        warnings.append(f"no prediction summary found: {centers_path}")

    for warning in warnings:
        print(f"[WARN] {warning}")
    if failures:
        for failure in failures:
            print(f"[FAIL] {failure}")
        return 1

    print("[PASS] Static cup/lid dataset is usable for offline perception checks.")
    print("[NOTE] This does not validate depth, hand-eye calibration, or robot base coordinates.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
