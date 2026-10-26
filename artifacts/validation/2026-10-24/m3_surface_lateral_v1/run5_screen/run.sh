#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out='artifacts/validation/2026-10-24/m3_surface_lateral_v1/run5_screen'
set +e
PYTHONPATH=scripts:src/arena_autonomy:src/arena_vehicle_interface python3 -B -m pytest -q -p no:cacheprovider --junitxml="$out/regression.xml" tests/test_lateral_intent.py tests/test_surface_contact_eval.py 2>&1 | tee "$out/regression.log"
rc=${PIPESTATUS[0]}
printf 'regression_exit_code=%s\n' "$rc" | tee -a "$out/regression.log"
if [ "$rc" -ne 0 ]; then exit "$rc"; fi
PYTHONPATH=scripts:src/arena_autonomy:src/arena_vehicle_interface python3 -B 'artifacts/validation/2026-10-24/m3_surface_lateral_v1/screen_lateral.py' --output "$out/screen.json" --records e3_onroad_g1 e3_onroad_r0plus solo_g1_20261016 --no-sensitivity --no-bypass 2>&1 | tee "$out/screen.log"
rc=${PIPESTATUS[0]}
printf 'screen_exit_code=%s\n' "$rc" | tee -a "$out/screen.log"
exit "$rc"