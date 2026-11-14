#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
artifact='artifacts/validation/2026-11-14/continuous_pass_preview_drive_r513_v1'
receipt="$artifact/execution_v1"
test ! -e "$artifact/registered.json"
test ! -e "$artifact/run1"
test ! -e "$receipt/run.pid"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
python3 -B "$artifact/prepare.py" --check > "$receipt/check_before_run.json"
date --utc --iso-8601=seconds > "$receipt/run_started_utc.txt"
python3 "$artifact/run.py" > "$receipt/runner.log" 2>&1 &
run_pid=$!
printf '%s\n' "$run_pid" > "$receipt/run.pid"
printf 'r513 one owned run started, pid=%s\n' "$run_pid"
set +e
wait "$run_pid"
run_code=$?
set -e
printf '%s\n' "$run_code" > "$receipt/run.exit"
date --utc --iso-8601=seconds > "$receipt/run_finished_utc.txt"
python3 -B "$artifact/prepare.py" --check > "$receipt/check_after_run.json"
printf 'r513 raw run exited: %s\n' "$run_code"
exit "$run_code"
