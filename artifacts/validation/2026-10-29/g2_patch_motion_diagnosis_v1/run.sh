#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/g2_patch_motion_diagnosis_v1
name="$1"; shift
test ! -e "$base/$name.log" || exit 73
set +e
python3 -B "$base/analyze.py" --output "$name" "$@" > "$base/$name.log" 2>&1
result=$?
echo "$result" > "$base/$name.exit"
exit "$result"
