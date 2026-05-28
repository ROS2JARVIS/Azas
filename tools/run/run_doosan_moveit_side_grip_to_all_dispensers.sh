#!/usr/bin/env bash
set -euo pipefail

# Assumed cup-in-hand RViz validation:
# side-grip hold -> dispenser 1 -> dispenser 2 -> dispenser 3 -> dispenser 4.

export TASK_MODE="${TASK_MODE:-dispenser_sequence}"
export DISPENSER_SEQUENCE_IDS="${DISPENSER_SEQUENCE_IDS:-[1,2,3,4]}"
exec /home/ssu/Azas/tools/run/run_doosan_moveit_side_grip_to_dispenser.sh
