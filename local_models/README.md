# Local Model Files

Place or symlink runtime model weights here on each robot PC.

Expected file:

```text
local_models/best.pt
```

The `.pt` file is intentionally not committed because model weights are large
local runtime assets. Use `tools/setup/link_yolo_model.sh` to link an existing
downloaded model into this stable repo-local path.
