#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out='artifacts/validation/2026-10-24/m3_surface_lateral_v1/run14_query_optimization'
set +e
PYTHONPATH=scripts:src/arena_autonomy:src/arena_vehicle_interface python3 -B 'artifacts/validation/2026-10-24/m3_surface_lateral_v1/compare_query_optimization.py' 'artifacts/validation/2026-10-24/m3_surface_lateral_v1/run13_observed_trial/observed_lateral_trial.py' "$out/comparison.json" >"$out/comparison.log" 2>&1
rc=$?
printf 'comparison_exit_code=%s\n' "$rc" >>"$out/comparison.log"
cat "$out/comparison.log"
exit "$rc"