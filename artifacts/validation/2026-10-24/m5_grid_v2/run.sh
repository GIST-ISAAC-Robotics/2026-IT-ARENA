#!/usr/bin/env bash
set -eo pipefail
ROOT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
OUT="$ROOT/artifacts/validation/2026-10-24/m5_grid_v2"
cd "$ROOT"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$ROOT/src/arena_autonomy:$ROOT/src/arena_vehicle_interface:$ROOT/scripts:$ROOT/tests:$PYTHONPATH"
export PYTHONDONTWRITEBYTECODE=1
test ! -e "$OUT/exit_status.txt"
test ! -e "$OUT/preflight"
trap 'echo EXIT_STATUS=$? > "$OUT/exit_status.txt"' EXIT
python3 -B -m pytest -q -p no:cacheprovider tests/test_grid_start.py tests/test_start_signal_geometry.py tests/test_start_signal_render_fixture.py --junitxml="$OUT/junit.xml" > "$OUT/tests.log" 2>&1
python3 -B scripts/audit_grid_start.py --output "$OUT/preflight" > "$OUT/preflight.log" 2>&1
gz sdf -k "$OUT/preflight/fixture.sdf" > "$OUT/sdf_check.log" 2>&1
python3 -B scripts/capture_grid_start.py "$OUT/preflight" > "$OUT/capture.log" 2>&1
