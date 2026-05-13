# Smoke Tests

Place automated no-motion or fake-hardware regression tests here.

Rules:
- Smoke tests must be repeatable.
- Real robot motion is not allowed in this folder.
- Fake RG2/Doosan services must be named clearly as fake/no-motion.
- Tests that need live hardware belong in `checks/` with explicit wording, not
  in smoke tests.

Root-level `smoke_*.sh` files are being kept temporarily for compatibility.
