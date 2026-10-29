#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-28/multilap_preparation_v1
export PYTHONPATH="$PWD/$base:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
test ! -e "$base/adapter_contracts_v1.log"
set +e
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider "$base/test_adapter.py" "$base/test_candidate.py" tests/test_g1_course_eval.py > "$base/adapter_contracts_v1.log" 2>&1
result=$?
printf '%s\n' "$result" > "$base/adapter_contracts_v1.exit"
exit "$result"
