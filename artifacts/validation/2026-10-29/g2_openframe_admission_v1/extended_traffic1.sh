#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
set +u
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
set -u
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/g2_openframe_admission_v1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/$base${PYTHONPATH:+:$PYTHONPATH}"
test ! -e "$base/extended1.log" || exit 73
python3 -B "$base/probe_shape.py" --output extended1 --cases right_combined --duration 16 --policies observed_entry_space_v1 --trace-policy observed_entry_space_v1 --window trim_before_compare_v1 --reserve --reacquire known_hold_v1 --patch-policy sparse_returns_v1 > "$base/extended1.log" 2>&1
result=$?
echo "$result" > "$base/extended1.exit"
if [ "$result" -ne 0 ]; then exit "$result"; fi
test ! -e "$base/traffic1.log" || exit 73
python3 -B "$base/probe_shape.py" --output traffic1 --cases moving_combined stopping_combined --policies observed_entry_space_v1 --trace-policy observed_entry_space_v1 --window trim_before_compare_v1 --reserve --reacquire known_hold_v1 --patch-policy sparse_returns_v1 > "$base/traffic1.log" 2>&1
result=$?
echo "$result" > "$base/traffic1.exit"
exit "$result"
