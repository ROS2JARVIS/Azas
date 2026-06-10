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

## 2026-06-10 Dispenser Cup Place Mapping

Operator-confirmed correction:

- Previous DISP2 cup place/re-grasp pose was actually the physical DISP4 cup position.
- Previous DISP3 cup place/re-grasp pose was actually the physical DISP2 cup position.
- Physical DISP3 cup place/re-grasp pose still needs re-teaching.

Current intended calibration state:

- D1 cup place: measured confirmed.
- D2 cup place: uses the previous D3 measured cup place values.
- D3 cup place: re-taught and measured confirmed at 2026-06-10T18:41-18:42+09:00.
- D4 cup place: uses the previous D2 measured cup place values.

Re-grasp path correction:

- Do not lower to measured `DISP_PRE` before the cup re-grasp intermediate point.
- After press, use `HOME joint -> high rear-entry -> lowered rear-entry -> final cup grasp`.
- For cup placement/re-grasp offset, `--move-release-offset-x-m -0.030` is 10 mm closer to the dispenser than `-0.040`.
