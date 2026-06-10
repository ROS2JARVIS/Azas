# Robot Operation Rules

## 2026-06-10 Real Motion Rule: Dry-Run Banned

User directive: 드라이런은 앞으로 금지.

For dispenser, press, cup placement, cup re-grasp, and other real robot motion checks, do not suggest or run dry-run / plan-only / no-execute commands as a substitute for motion validation.

Required behavior:

- When the user asks to see robot movement, use real execution commands only, with explicit `--execute` and the required `--confirm` phrase.
- Prefer low-speed, bounded, single-stage checks first, especially for dispenser press/contact verification.
- If actual execution is unsafe or blocked, state the concrete blocker and stop. Do not replace the requested real-motion check with a dry-run.
- RViz preview is only a visual aid when explicitly requested. It does not replace real-motion validation.
- Do not invent cup, dispenser, press, or collision coordinates. Use measured calibration values, robot services, or the vision pipeline.

Stop condition:

- Stop before motion if the command would use missing/ambiguous calibration, unknown TCP state, inconsistent FK, unavailable robot services, or an identified collision risk.

## 2026-06-10 Invalid Calibration: PRESS1_CONTACT

Dispenser 1 `press_contact_joints_deg` is operator-confirmed wrong.

Current status:

- `dispenser_outlets."1".press_contact_status: invalid_reteach_required`
- Real press motion for dispenser 1 must be blocked before cup placement or press execution.
- Do not infer, swap, or regenerate the replacement PRESS1 contact pose. Replace it only with a newly measured robot teaching value.
