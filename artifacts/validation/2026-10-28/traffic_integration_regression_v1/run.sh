#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/traffic_integration_regression_v1
test ! -e "$base/full_v1.log"
set +e
python3 -B -m pytest -q -rs -p no:cacheprovider tests > "$base/full_v1.log" 2>&1
result=$?
printf '%s\n' "$result" > "$base/full_v1.exit"
exit "$result"
