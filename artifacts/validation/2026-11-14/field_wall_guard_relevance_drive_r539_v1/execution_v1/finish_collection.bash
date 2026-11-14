#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
a='artifacts/validation/2026-11-14/field_wall_guard_relevance_drive_r539_v1'
set +e
python3 -B "$a/execution_v1/check_owned_processes.py"
process_code=$?
python3 -B "$a/prepare.py" --check
integrity_code=$?
python3 -B "$a/collect_result.py"
summary_code=$?
python3 -B "$a/assess.py"
assessment_code=$?
python3 -B "$a/collect_evidence.py" --kind traffic
collection_code=$?
printf '{"process":%s,"integrity":%s,"summary":%s,"assessment":%s,"collection":%s}\n' "$process_code" "$integrity_code" "$summary_code" "$assessment_code" "$collection_code" > "$a/execution_v1/collection_exit.json"
test "$process_code" -eq 0 && test "$integrity_code" -eq 0 && test "$summary_code" -eq 0 && test "$assessment_code" -eq 0 && test "$collection_code" -eq 0
