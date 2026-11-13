#!/usr/bin/env bash
set -euo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/bounded_guard_approach_r459_v1'
export PYTHONDONTWRITEBYTECODE=1
test ! -e native_v1
set +e
python3 -B replay_native.py > native_v1.log 2>&1 &
task_pid=$!
printf '%s\n' "$task_pid" > native_v1.pid
wait "$task_pid"
code=$?
printf '%s\n' "$code" > native_v1.exit
cat native_v1.log
exit "$code"
