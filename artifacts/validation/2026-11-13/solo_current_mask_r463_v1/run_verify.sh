#!/usr/bin/env bash
set -euo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/solo_current_mask_r463_v1'
export PYTHONDONTWRITEBYTECODE=1
set +e
python3 -B verify_current.py > verification_v1.log 2>&1
code=$?
printf '%s\n' "$code" > verification_v1.exit
tail -c 8000 verification_v1.log
exit "$code"
