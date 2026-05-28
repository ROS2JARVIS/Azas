#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path


JOINTS_DEFAULT = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def _try_import_yaml():
    try:
        import yaml  # type: ignore

        return yaml
    except Exception:
        return None


def _load_joint_state_snapshot(path: Path) -> dict:
    """Load a JointState snapshot.

    Accepts:
      - `ros2 topic echo /joint_states --once > snap.txt` output (YAML-ish)
      - a minimal YAML dict with `name` and `position`
    """
    raw = path.read_text(encoding="utf-8", errors="replace")
    yaml = _try_import_yaml()
    if yaml is not None:
        try:
            data = yaml.safe_load(raw)
            if isinstance(data, dict) and "name" in data and "position" in data:
                return data
        except Exception:
            pass

    # Fallback: regex parse `name:` and `position:` lists from the text.
    name_match = re.search(r"(?ms)^name\\s*:\\s*\\[(.*?)\\]\\s*$", raw)
    pos_match = re.search(r"(?ms)^position\\s*:\\s*\\[(.*?)\\]\\s*$", raw)
    if not name_match or not pos_match:
        raise ValueError(
            f"Could not parse JointState snapshot from {path}. "
            "Expected fields `name: [...]` and `position: [...]`."
        )
    names = [token.strip().strip("'\"") for token in name_match.group(1).split(",") if token.strip()]
    positions = [float(token.strip()) for token in pos_match.group(1).split(",") if token.strip()]
    return {"name": names, "position": positions}


def _joint_map_from_snapshot(snapshot: dict) -> dict[str, float]:
    names = snapshot.get("name")
    positions = snapshot.get("position")
    if not isinstance(names, list) or not isinstance(positions, list):
        raise ValueError("JointState snapshot must contain list fields `name` and `position`.")
    if len(names) != len(positions):
        raise ValueError(f"JointState has name/position length mismatch: {len(names)} vs {len(positions)}")

    out: dict[str, float] = {}
    for name, pos in zip(names, positions):
        if not isinstance(name, str):
            continue
        try:
            out[name] = float(pos)
        except Exception:
            continue
    return out


def _format_float(x: float) -> str:
    # Keep it stable/readable; avoid scientific notation.
    if abs(x) < 1e-12:
        return "0.0"
    return f"{x:.10f}".rstrip("0").rstrip(".")


def _write_side_prepose_yaml(
    yaml_path: Path,
    joint_order: list[str],
    low_joints: list[float],
    high_joints: list[float],
    enable: bool,
    selection_mode: str,
) -> None:
    yaml = _try_import_yaml()
    if yaml is not None:
        data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        if not isinstance(data, dict) or "/**" not in data:
            raise ValueError("Unexpected YAML structure: expected top-level key `/**`.")
        params = data["/**"].get("ros__parameters")
        if not isinstance(params, dict):
            raise ValueError("Unexpected YAML structure: expected `/**: ros__parameters:` mapping.")

        params["side_prepose_joint_order"] = joint_order
        if selection_mode == "y":
            params["side_prepose_selection_mode"] = "y"
            params["side_prepose_joints_cup_left_rad"] = [float(v) for v in low_joints]
            params["side_prepose_joints_cup_right_rad"] = [float(v) for v in high_joints]
        else:
            params["side_prepose_selection_mode"] = "z"
            params["side_prepose_joints_low_rad"] = [float(v) for v in low_joints]
            params["side_prepose_joints_high_rad"] = [float(v) for v in high_joints]
        if enable:
            params["side_prepose_enabled"] = True

        yaml_path.write_text(
            yaml.safe_dump(data, sort_keys=False, default_flow_style=False),
            encoding="utf-8",
        )
        return

    # No PyYAML: do a minimal targeted rewrite preserving the existing file otherwise.
    text = yaml_path.read_text(encoding="utf-8", errors="replace")

    def replace_list(key: str, values: list[float]) -> str:
        pattern = rf"(?m)^(\\s*{re.escape(key)}\\s*:\\s*)\\[.*?\\]\\s*$"
        repl = lambda m: m.group(1) + "[" + ", ".join(_format_float(v) for v in values) + "]"
        new_text, n = re.subn(pattern, repl, text)
        if n == 0:
            raise ValueError(f"Could not find YAML key line for {key!r} to update.")
        return new_text

    text2 = text
    if selection_mode == "y":
        text2, _ = re.subn(
            r"(?m)^(\\s*side_prepose_selection_mode\\s*:\\s*).*$",
            r'\\1"y"',
            text2,
        )
        text2 = replace_list("side_prepose_joints_cup_left_rad", low_joints)
        text2 = replace_list("side_prepose_joints_cup_right_rad", high_joints)
    else:
        text2, _ = re.subn(
            r"(?m)^(\\s*side_prepose_selection_mode\\s*:\\s*).*$",
            r'\\1"z"',
            text2,
        )
        text2 = replace_list("side_prepose_joints_low_rad", low_joints)
        text2 = replace_list("side_prepose_joints_high_rad", high_joints)
    if enable:
        text2, _ = re.subn(
            r"(?m)^(\\s*side_prepose_enabled\\s*:\\s*).*$",
            r"\\1true",
            text2,
        )
    yaml_path.write_text(text2, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Fill dsr_practice side_prepose.yaml from JointState snapshots.",
    )
    parser.add_argument("--low", required=True, type=Path, help="LOW prepose JointState snapshot text file.")
    parser.add_argument("--high", required=True, type=Path, help="HIGH prepose JointState snapshot text file.")
    parser.add_argument(
        "--yaml",
        dest="yaml_path",
        default=Path("src/dsr_practice/config/side_prepose.yaml"),
        type=Path,
        help="Path to side_prepose.yaml",
    )
    parser.add_argument(
        "--joint-order",
        nargs="+",
        default=JOINTS_DEFAULT,
        help="Joint order to write (must match node's joint naming).",
    )
    parser.add_argument(
        "--enable",
        action="store_true",
        help="Also set side_prepose_enabled: true in the YAML.",
    )
    parser.add_argument(
        "--mode",
        choices=["y", "z"],
        default="y",
        help="Which prepose mapping to fill: y (cup_left/right) or z (low/high legacy).",
    )
    parser.add_argument(
        "--degrees",
        action="store_true",
        help="Treat snapshot positions as degrees (will convert to radians).",
    )

    args = parser.parse_args()
    yaml_path = Path(args.yaml_path)
    if not yaml_path.exists():
        raise FileNotFoundError(f"YAML not found: {yaml_path}")

    low_map = _joint_map_from_snapshot(_load_joint_state_snapshot(Path(args.low)))
    high_map = _joint_map_from_snapshot(_load_joint_state_snapshot(Path(args.high)))
    joint_order = [str(j) for j in args.joint_order]

    missing_low = [j for j in joint_order if j not in low_map]
    missing_high = [j for j in joint_order if j not in high_map]
    if missing_low:
        raise ValueError(f"LOW snapshot missing joints: {missing_low}")
    if missing_high:
        raise ValueError(f"HIGH snapshot missing joints: {missing_high}")

    low = [float(low_map[j]) for j in joint_order]
    high = [float(high_map[j]) for j in joint_order]
    if args.degrees:
        low = [math.radians(v) for v in low]
        high = [math.radians(v) for v in high]

    _write_side_prepose_yaml(
        yaml_path,
        joint_order,
        low,
        high,
        enable=bool(args.enable),
        selection_mode=str(args.mode),
    )

    print(f"Updated: {yaml_path}")
    print(f"- side_prepose_joint_order: {joint_order}")
    if str(args.mode) == "y":
        print(f"- side_prepose_selection_mode: 'y'")
        print(f"- side_prepose_joints_cup_left_rad: {len(low)} values")
        print(f"- side_prepose_joints_cup_right_rad: {len(high)} values")
    else:
        print(f"- side_prepose_selection_mode: 'z'")
        print(f"- side_prepose_joints_low_rad: {len(low)} values")
        print(f"- side_prepose_joints_high_rad: {len(high)} values")
    if args.enable:
        print("- side_prepose_enabled: true")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise
