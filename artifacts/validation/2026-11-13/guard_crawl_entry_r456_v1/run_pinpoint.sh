#!/usr/bin/env bash
set -euo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/guard_crawl_entry_r456_v1'
export PYTHONDONTWRITEBYTECODE=1
set +e
python3 -B pinpoint.py > pinpoint_v1.log 2>&1 &
task_pid=$!
printf '%s\n' "$task_pid" > pinpoint_v1.pid
wait "$task_pid"
code=$?
printf '%s\n' "$code" > pinpoint_v1.exit
cat pinpoint_v1.log
exit "$code"
