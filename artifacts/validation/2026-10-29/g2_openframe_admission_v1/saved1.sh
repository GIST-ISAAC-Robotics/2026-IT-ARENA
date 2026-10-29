#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
set +u
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
set -u
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/g2_openframe_admission_v1
test ! -e "$base/saved1.log" || exit 73
python3 -B "$base/diagnose_saved.py" > "$base/saved1.log" 2>&1
result=$?
echo "$result" > "$base/saved1.exit"
exit "$result"
