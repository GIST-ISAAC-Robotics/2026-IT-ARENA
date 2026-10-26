#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out='artifacts/validation/2026-10-24/m3_surface_lateral_v1/run7_frame_support_probe'
set +e
PYTHONPATH=scripts:src/arena_autonomy:src/arena_vehicle_interface python3 -B 'artifacts/validation/2026-10-24/m3_surface_lateral_v1/compare_sweep_frames.py' "$out/frames.json" >"$out/stdout.log" 2>&1
rc=$?
printf 'probe_exit_code=%s\n' "$rc" >>"$out/stdout.log"
cat "$out/stdout.log"
exit "$rc"