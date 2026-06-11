# Post-shake human hand handover plan

This document defines a safe, staged plan for ending the cocktail workflow by
tracking a person's hand and preparing a cup handover after shaking/serving.

## Summary

The feature is useful for the final user experience, but it is an HRI
(human-robot interaction) step: the robot would move near a person. Therefore the
current implementation is deliberately a dry-run plan only.

Implemented in this branch:

1. Add post-shake hand tracking and handover planning phases to the cocktail task plan.
2. Keep hand tracking as perception-only.
3. Compute a handover pose candidate only as data.
4. Require explicit operator approval.
5. Keep the actual cup-to-human handover motion disabled until a separate safety review.

No live robot command is added by this branch.

## Workflow phases added

The new final phases are appended after `POUR`:

1. `VERIFY_HUMAN_HAND_TRACKING`
   - input: `/azas/human_hand_detection`, `handover_safety.yaml`
   - purpose: require a stable open hand target
   - gate: `no_motion_hri_perception_only`

2. `COMPUTE_HANDOVER_POSE`
   - input: stable hand target and camera/base TF
   - purpose: compute a conservative pose candidate with approach offset
   - gate: `tf_required_no_motion`

3. `WAIT_FOR_HANDOVER_APPROVAL`
   - input: pose candidate, operator confirmation, still-open hand target
   - purpose: prevent accidental handover execution
   - gate: `operator_approval_required`

4. `HANDOVER_CUP_TO_HUMAN_DISABLED`
   - purpose: placeholder final handover step
   - gate: `disabled_until_hri_safety_review`
   - command: `disabled_handover_motion_placeholder`

## Why this is staged

A handover near a human should not be triggered by vision alone. Before any live
execution, the project needs at least:

- hand target stability check
- depth validity check
- person distance monitor
- emergency stop observer
- low force/speed limits
- retreat path
- operator confirmation
- real-robot dry-run with no cup
- real-robot dry-run with empty cup

## Regression check

```bash
cd /home/ssu/Azas
python3 tools/checks/check_cocktail_workflow_plan.py
```

Expected:

```text
[PASS] full cocktail workflow plan includes calibration, dispenser press, shake gates, and disabled post-shake handover planning
```

## Current limitation

There is no live hand detector in this branch. The workflow expects a future
perception source such as `/azas/human_hand_detection`. That source should be
validated offline first, similar to the camera-free color discrimination test.
