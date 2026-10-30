#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-30/g2_center_width_v1
test ! -e "$base/analysis1.log"
set +e
/usr/bin/python3 -B "$base/analyze.py" > "$base/analysis1.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$base/analysis1.exit"
exit "$rc"
