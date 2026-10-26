#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out='artifacts/validation/2026-10-24/m3_surface_lateral_v1/run9_shape_regression'
set +e
PYTHONPATH=scripts:src/arena_autonomy:src/arena_vehicle_interface python3 -B -m pytest -q -p no:cacheprovider --junitxml="$out/regression.xml" tests/test_lateral_intent.py tests/test_surface_contact_eval.py >"$out/regression.log" 2>&1
trc=$?
printf 'pytest_exit_code=%s\n' "$trc" >>"$out/regression.log"
cat "$out/regression.log"
PYTHONPATH=scripts:src/arena_autonomy:src/arena_vehicle_interface python3 -B 'artifacts/validation/2026-10-24/m3_surface_lateral_v1/audit_side_surfaces.py' "$out/result.json" >"$out/audit.log" 2>&1
arc=$?
printf 'audit_exit_code=%s\n' "$arc" >>"$out/audit.log"
cat "$out/audit.log"
printf 'pytest_exit=%s audit_exit=%s\n' "$trc" "$arc"
exit 0