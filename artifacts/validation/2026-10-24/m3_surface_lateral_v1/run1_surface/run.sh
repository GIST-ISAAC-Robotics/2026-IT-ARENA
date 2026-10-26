#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out='artifacts/validation/2026-10-24/m3_surface_lateral_v1/run1_surface'
set +e
python3 -B 'artifacts/validation/2026-10-24/m3_surface_lateral_v1/run_surface.py' --output "$out/surface.json" 2>&1 | tee "$out/surface.log"
rc=${PIPESTATUS[0]}
printf 'surface_exit_code=%s\n' "$rc" | tee -a "$out/surface.log"
exit "$rc"