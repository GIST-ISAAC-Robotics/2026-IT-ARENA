#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
base=artifacts/validation/2026-10-29/m3_openframe_v1
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/$base:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
test ! -e "$base/contracts1.log" || exit 73
set +e
python3 -B -m pytest -q "$base/test_drive_contract.py" "$base/test_multipart_contract.py" > "$base/contracts1.log" 2>&1
result=$?
echo "$result" > "$base/contracts1.exit"
exit "$result"
