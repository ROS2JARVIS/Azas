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

## 2026-06-10 Dispenser Press Contacts Confirmed

Operator note at 2026-06-10T16:48:12+09:00: all dispenser `PRESS_CONTACT` values have been saved.

Confirmed state:

- D1 press-only real-motion check completed and `press_contact_status: measured_confirmed`.
- D2 press-only real-motion check completed and `press_contact_status: measured_confirmed`.
- D3 press-only real-motion check completed and `press_contact_status: measured_confirmed`.
- D4 press-only real-motion check also works normally. The earlier observed `MoveLine returned success=false` was caused by a cable disconnect, not by the saved D4 contact. D4 status is `press_contact_status: measured_confirmed`.

Do not resurrect the stale `PRESS1_CONTACT invalid_reteach_required` note unless the operator newly invalidates the saved D1 contact.
