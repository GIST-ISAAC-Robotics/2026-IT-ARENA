#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out='artifacts/validation/2026-10-24/m3_surface_lateral_v1/run14_query_optimization'
set +e
PYTHONPATH=scripts:src/arena_autonomy:src/arena_vehicle_interface python3 -B 'artifacts/validation/2026-10-24/m3_surface_lateral_v1/measure_temporal_marginal.py' "$out/marginal.json" >"$out/marginal.log" 2>&1
rc=$?
printf 'marginal_exit_code=%s\n' "$rc" >>"$out/marginal.log"
cat "$out/marginal.log"
exit "$rc"