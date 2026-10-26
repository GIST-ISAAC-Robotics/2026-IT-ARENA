#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out='artifacts/validation/2026-10-24/m3_surface_lateral_v1/run8_side_surface_audit'
set +e
PYTHONPATH=scripts:src/arena_autonomy:src/arena_vehicle_interface python3 -B 'artifacts/validation/2026-10-24/m3_surface_lateral_v1/audit_side_surfaces.py' "$out/result.json" >"$out/stdout_stderr.log" 2>&1
rc=$?
printf 'audit_exit_code=%s\n' "$rc" >>"$out/stdout_stderr.log"
cat "$out/stdout_stderr.log"
exit "$rc"