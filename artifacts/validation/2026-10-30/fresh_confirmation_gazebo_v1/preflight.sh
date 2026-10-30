#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-30/fresh_confirmation_gazebo_v1
test ! -e "$base/preflight.log"
set +e
/usr/bin/python3 -B "$base/preflight.py" > "$base/preflight.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$base/preflight.exit"
exit "$rc"
