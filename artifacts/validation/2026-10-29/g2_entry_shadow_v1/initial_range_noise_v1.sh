#!/usr/bin/env bash
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/g2_entry_shadow_v1
test ! -e "$base/initial_range_noise_v1.log" || exit 90
/usr/bin/python3 -B "$base/probe_noise_entry.py" > "$base/initial_range_noise_v1.log" 2>&1
probe=$?
printf '{"probe":%s}\n' "$probe" > "$base/initial_range_noise_v1_exit.json"
exit "$probe"
