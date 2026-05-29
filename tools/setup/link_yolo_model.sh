#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SOURCE_MODEL="${1:-${MODEL_PATH:-/home/ssu/Downloads/best.pt}}"
TARGET_DIR="$ROOT/local_models"
TARGET_MODEL="$TARGET_DIR/best.pt"

mkdir -p "$TARGET_DIR"

if [[ ! -f "$SOURCE_MODEL" ]]; then
  cat >&2 <<MSG
[Azas] YOLO model source not found: $SOURCE_MODEL

Put the current best.pt on this PC, then run one of:
  bash tools/setup/link_yolo_model.sh /path/to/best.pt
  MODEL_PATH=/path/to/best.pt bash tools/setup/link_yolo_model.sh

Expected stable panel path:
  $TARGET_MODEL
MSG
  exit 1
fi

ln -sfn "$(readlink -f "$SOURCE_MODEL")" "$TARGET_MODEL"

echo "[Azas] YOLO model linked:"
echo "  $TARGET_MODEL -> $(readlink -f "$TARGET_MODEL")"

python3 - <<PY
from pathlib import Path
path = Path("$TARGET_MODEL")
print(f"[Azas] model size: {path.stat().st_size} bytes")
PY
