#!/usr/bin/env bash
set -uo pipefail
repo="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
cd "$repo" || exit 90
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface:$repo/scripts:$repo/tests"
test -f artifacts/validation/2026-10-26/m3_return_command_guard_v1/verify_topology_transfer.py || exit 92
test -f artifacts/validation/2026-10-26/m3_rear_residual_v1/cachefix_runs/run11_visibility_strict/rows.json || exit 92
test -f artifacts/validation/2026-10-26/m3_rear_residual_v1/cachefix_runs/run14_scan_seam_fixturefix/rows.json || exit 92
python3 -c 'import numpy,pytest,arena_autonomy; print("numpy",numpy.__version__); print("pytest",pytest.__version__); print("arena_autonomy.__path__",list(arena_autonomy.__path__))'
python3 -I artifacts/validation/2026-10-26/m3_return_command_guard_v1/verify_topology_transfer.py --output artifacts/validation/2026-10-26/m3_return_command_guard_v1/run5_topology_transfer/data --baseline artifacts/validation/2026-10-26/m3_rear_residual_v1/cachefix_runs/run11_visibility_strict/rows.json --seam-baseline artifacts/validation/2026-10-26/m3_rear_residual_v1/cachefix_runs/run14_scan_seam_fixturefix/rows.json
