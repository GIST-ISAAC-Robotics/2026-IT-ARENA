#!/usr/bin/env bash
set -euo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/solo_current_mask_r463_v1'
export PYTHONDONTWRITEBYTECODE=1
set +e
python3 -B analyze.py > analysis_v1.log 2>&1
code=$?
printf '%s\n' "$code" > analysis_v1.exit
cat analysis_v1.log
exit "$code"
