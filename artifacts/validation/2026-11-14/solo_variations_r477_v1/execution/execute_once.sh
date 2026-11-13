#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
artifact='artifacts/validation/2026-11-14/solo_variations_r477_v1'
case_name="${1:?one registered variation required}"
case "$case_name" in start_left20_yaw_plus3|wall_surface_inward10) ;; *) exit 2;; esac
case_dir="$artifact/cases/$case_name"
receipt="$case_dir/execution_v1"
test ! -e "$case_dir/registered.json"
test ! -e "$case_dir/run1"
test ! -e "$receipt/run.pid"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$receipt"
python3 -B "$artifact/prepare_execution.py" --check > "$receipt/check_before_run.json"
date --utc --iso-8601=seconds > "$receipt/run_started_utc.txt"
python3 "$artifact/run.py" "$case_name" > "$receipt/runner.log" 2>&1 &
run_pid=$!
printf '%s\n' "$run_pid" > "$receipt/run.pid"
printf 'r477 one owned variation started: %s pid=%s\n' "$case_name" "$run_pid"
set +e
wait "$run_pid"
run_code=$?
set -e
printf '%s\n' "$run_code" > "$receipt/run.exit"
date --utc --iso-8601=seconds > "$receipt/run_finished_utc.txt"
python3 -B "$artifact/prepare_execution.py" --check > "$receipt/check_after_run.json"
printf 'r477 variation exited: %s %s\n' "$case_name" "$run_code"
exit "$run_code"
