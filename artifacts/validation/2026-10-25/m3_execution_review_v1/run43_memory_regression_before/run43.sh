#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONPATH='src/arena_autonomy:src/arena_vehicle_interface:scripts:tests'
D=artifacts/validation/2026-10-25/m3_execution_review_v1/run43_memory_regression_before
[ ! -e "$D/exit_code.txt" ] || exit 91
tar -cf "$D/source_before.tar" src/arena_autonomy/arena_autonomy/m3_execution_trial.py tests/test_m3_retained_threats.py
python3 -B -m pytest -p no:cacheprovider tests/test_m3_retained_threats.py -q --junitxml="$D/pytest.xml" > "$D/stdout.txt" 2> "$D/stderr.txt"
result_code=$?
printf '%s\n' "$result_code" > "$D/exit_code.txt"
exit "$result_code"
