#!/usr/bin/env bash
set -u
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONDONTWRITEBYTECODE=1
a='artifacts/validation/2026-11-14/measured_steering_reacquire_lap_r550_v1'
python3 -B "$a/execution_v1/check_owned_processes.py" > "$a/execution_v1/process_collection.log" 2>&1
process_code=$?
python3 -B "$a/prepare.py" --check > "$a/execution_v1/final_integrity.json" 2> "$a/execution_v1/final_integrity_error.log"
integrity_code=$?
python3 -B "$a/collect_result.py" > "$a/execution_v1/collection.log" 2>&1
collection_code=$?
printf '{"process_check_exit":%s,"integrity_check_exit":%s,"collection_exit":%s}\n' "$process_code" "$integrity_code" "$collection_code" > "$a/execution_v1/collection_exit.json"
test "$process_code" -eq 0 && test "$integrity_code" -eq 0 && test "$collection_code" -eq 0
