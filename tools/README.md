# Azas Tools Layout

Use this directory as operator tooling, not as a dumping ground for one-off
experiments.

## Folders

- `pick/`: supervised cup-pick and side-grasp planning tools.
- `perception/`: frame export and perception-data capture tools.
- `checks/`: read-only readiness checks and contract checks.
- `smoke/`: automated smoke tests that should not require real robot motion.
- `legacy/`: old tools kept temporarily while references are migrated.

Root-level scripts are allowed only as compatibility wrappers or stable public
entrypoints. New implementation files should go into a folder above.

## Real Motion Rules

- Real robot motion must require `--enable-real-motion` plus the exact confirm
  phrase used by the entrypoint.
- A real-motion script must be one-shot by default. No automatic repeat loops.
- Plan first, execute only the successful trajectory, and report the action or
  service name used.
- Never silently fall back from failed MoveIt planning to a raw Doosan command.
- Gripper commands must stay explicit and separate from observe-only motion.

## Current Cup Pick Path

Use the compatibility command while docs are being migrated:

```bash
python3 tools/run_supervised_real_single_cup_pick.py --help
```

The implementation is:

```bash
python3 tools/pick/run_supervised_real_single_cup_pick.py --help
```

Default observe joint target starts from HOME, then moves to:

```text
joint_1=0, joint_2=25, joint_3=65, joint_4=0, joint_5=135, joint_6=0
```

This is an operator-tunable camera observation pose. It is not a calibrated
camera pose guarantee.
