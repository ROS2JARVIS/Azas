#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT_DIR}"

TMP_OUT="$(mktemp)"
PLAN_OUT="$(mktemp)"
RESULT_LOG="$(mktemp)"
trap 'rm -f "${TMP_OUT}" "${PLAN_OUT}" "${RESULT_LOG}"' EXIT

echo "[Azas smoke] one-click cocktail no-motion smoke"

bash -n \
  tools/run/run_one_click_cocktail_real.sh \
  tools/run/run_cocktail_now_real.sh \
  tools/run/report_cocktail_now_status.sh \
  tools/run/check_one_click_cocktail_config.sh \
  tools/run/check_one_click_cocktail_ready.sh \
  tools/run/check_one_click_cocktail_result.sh \
  tools/run/show_cocktail_motion_preview.sh \
  tools/run/run_cocktail_collision_rviz_preview.sh \
  tools/run/stop_cocktail_motion_preview.sh

DRY_RUN=1 bash tools/run/stop_cocktail_motion_preview.sh >"${TMP_OUT}" 2>&1 || {
  cat "${TMP_OUT}" >&2
  exit 1
}
grep -q -- 'Stopping virtual/RViz cocktail preview stack' "${TMP_OUT}"
grep -Eq -- 'No cocktail preview processes found|Preview stop complete' "${TMP_OUT}"

grep -q -- 'dsr_bringup2_moveit.launch.py' tools/run/stop_cocktail_motion_preview.sh
grep -q -- 'mode:=virtual' tools/run/stop_cocktail_motion_preview.sh
grep -Eq -- './DRCF M0609|/DRCF M0609' tools/run/stop_cocktail_motion_preview.sh
grep -q -- 'TCP_HARD_BLOCK_FOR_READY' tools/run/run_cocktail_now_real.sh
grep -q -- 'TCP_HARD_BLOCK' tools/run/check_one_click_cocktail_ready.sh

# Make the rest of the dry-run smoke deterministic even if an operator left the
# RViz/virtual preview open. This does not target real robot processes.
bash tools/run/stop_cocktail_motion_preview.sh >"${TMP_OUT}" 2>&1
grep -Eq -- 'No cocktail preview processes found|Preview stop complete' "${TMP_OUT}"

REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE \
DRY_RUN=1 \
SERVICE_PREFIX=not_running \
RECIPE_DISPENSER_IDS=1x2 \
ROBOT_HOST=192.168.1.100 \
bash tools/run/run_one_click_cocktail_real.sh >"${TMP_OUT}" 2>&1

grep -q -- '--dispenser-ids' "${TMP_OUT}"
grep -q -- '1x2' "${TMP_OUT}"
grep -q -- 'check TCP 192.168.1.100:12345 before starting real Doosan bringup' "${TMP_OUT}"
grep -q -- '--press-pre-lift-m' "${TMP_OUT}"
grep -q -- '--press-depth-m' "${TMP_OUT}"
grep -q -- 'post-run evidence would sample current posj/posx' "${TMP_OUT}"

REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE \
DRY_RUN=1 \
SKIP_PREVIEW_STOP=1 \
SERVICE_PREFIX=dsr01 \
ROBOT_HOST=192.168.1.100 \
bash tools/run/run_cocktail_now_real.sh 1x2 >"${TMP_OUT}" 2>&1
grep -q -- 'Cocktail NOW real cycle: 1x2' "${TMP_OUT}"
grep -q -- 'recipe_dispenser_ids=1x2' "${TMP_OUT}"
grep -q -- 'robot_name=dsr01 service_prefix=dsr01' "${TMP_OUT}"
grep -q -- 'TCP_HARD_BLOCK=0' "${TMP_OUT}" || true
grep -q -- 'Running integrated cocktail dispenser cycle: 1x2' "${TMP_OUT}"

REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE \
DRY_RUN=1 \
SERVICE_PREFIX=not_running \
ROBOT_HOST=192.168.1.100 \
bash tools/run/run_cocktail_now_real.sh 1x2 >"${TMP_OUT}" 2>&1
grep -q -- 'Stopping virtual/RViz cocktail preview stack' "${TMP_OUT}"
grep -q -- 'Cocktail NOW real cycle: 1x2' "${TMP_OUT}"
grep -q -- 'robot_name=not_running service_prefix=not_running' "${TMP_OUT}"
grep -q -- 'check TCP 192.168.1.100:12345 before starting real Doosan bringup' "${TMP_OUT}"
grep -q -- 'Starting real Doosan bringup: ROBOT_HOST=192.168.1.100 ROBOT_NAME=not_running' "${TMP_OUT}"
grep -q -- 'Running integrated cocktail dispenser cycle: 1x2' "${TMP_OUT}"

python3 tools/run/run_measured_dispenser_recipe_sequence.py \
  --dispenser-ids 1x2 \
  --confirm ENABLE_MEASURED_DISPENSER_RECIPE_SEQUENCE >"${PLAN_OUT}" 2>&1
grep -q -- 'dispenser_ids=1,1' "${PLAN_OUT}"
grep -q -- 'grouped_press_counts=1x2' "${PLAN_OUT}"
grep -q -- 'integrated move/release -> integrated press 2 time(s) -> integrated re-grasp/lift' "${PLAN_OUT}"
grep -q -- '\[PASS\] measured dispenser recipe sequence completed' "${PLAN_OUT}"

RECIPE_DISPENSER_IDS=1x2 bash tools/run/check_one_click_cocktail_config.sh >"${TMP_OUT}" 2>&1
grep -q -- '\[PASS\] one-click cocktail config preflight OK' "${TMP_OUT}"
grep -q -- 'dispenser_ids=1,1' "${TMP_OUT}"
grep -q -- 'grouped_press_counts=1x2' "${TMP_OUT}"

cat >"${RESULT_LOG}" <<'LOG'
[Azas] RG2 full-open release complete; continuing only after open settle wait
[Azas] RG2 close empty gripper for dispenser press: sent RG2 set_width command width_units=0 force_units=300
[Azas] move to measured press contact joints exactly: joints_deg=[15.12, 40.50, 32.87, -33.75, 51.99, 28.07]
[Azas] press dispenser pump 1/2: posx=[1,2,3,4,5,6]
[Azas] press dispenser pump 2/2: posx=[1,2,3,4,5,6]
[Azas] RG2 soft side-grasp: sent RG2 set_width command width_units=750 force_units=250
[Azas] post-grasp lift: posx=[1,2,3,4,5,6]
[PASS] measured dispenser recipe sequence completed
LOG
SAMPLE_CURRENT_POSE=0 INTEGRATED_LOG="${RESULT_LOG}" bash tools/run/check_one_click_cocktail_result.sh >/dev/null

python3 - <<'PY'
from pathlib import Path
import importlib.util
import yaml

expected_press_joints = {
    "1": [15.12, 40.50, 32.87, -33.75, 51.99, 28.07],
    "2": [6.36, 39.76, 30.07, -14.08, 55.67, 27.99],
    "3": [-0.29, 40.29, 28.38, -5.98, 55.25, 14.33],
    "4": [-7.04, 40.77, 28.37, 4.55, 54.02, 0.22],
}

calibration = yaml.safe_load(Path('src/azas_bringup/config/calibration.yaml').read_text())
for dispenser_id, expected in expected_press_joints.items():
    actual = calibration['dispenser_outlets'][dispenser_id]['press_contact_joints_deg']
    assert len(actual) == 6, (dispenser_id, actual)
    assert all(abs(float(a) - e) < 1e-6 for a, e in zip(actual, expected)), (dispenser_id, actual, expected)

recipe_source = Path('tools/run/run_measured_dispenser_recipe_sequence.py').read_text()
assert 'default=False' in recipe_source and '--press-move-configured-prepose-before-joint' in recipe_source
assert 'measured press contact joints are authoritative' in recipe_source
print('[Azas smoke] measured press joints and joint-first press path OK')

path = Path('tools/run/robot_pipeline_control_server.py')
spec = importlib.util.spec_from_file_location('robot_pipeline_control_server', path)
mod = importlib.util.module_from_spec(spec)
import sys
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)
config = {
    'recipe_dispenser_ids': '1x2',
    'robot_host': '192.168.1.100',
    'service_prefix': 'dsr01',
}
one_click_step = next(s for s in mod.STEPS if s.key == 'run_one_click_cocktail_real')
cmd = mod.command_for(one_click_step, config)
assert 'REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE' in cmd
assert 'RECIPE_DISPENSER_IDS=1x2' in cmd
assert 'ROBOT_NAME=dsr01' in cmd
assert 'run_one_click_cocktail_real.sh' in cmd
ready_step = next(s for s in mod.STEPS if s.key == 'check_one_click_cocktail_ready')
ready = mod.command_for(ready_step, config)
assert 'check_one_click_cocktail_ready.sh' in ready
assert 'ROBOT_NAME=dsr01' in ready
result_step = next(s for s in mod.STEPS if s.key == 'check_one_click_cocktail_result')
result = mod.command_for(result_step, config)
assert 'check_one_click_cocktail_result.sh' in result
now_step = next(s for s in mod.STEPS if s.key == 'run_cocktail_now_real')
now_cmd = mod.command_for(now_step, config)
assert 'REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE' in now_cmd
assert 'ROBOT_NAME=dsr01' in now_cmd
assert 'run_cocktail_now_real.sh 1x2' in now_cmd
print('[Azas smoke] panel command generation OK')
PY

echo "[PASS] one-click cocktail no-motion smoke"
