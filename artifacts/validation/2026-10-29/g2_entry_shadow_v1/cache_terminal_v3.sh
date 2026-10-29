#!/usr/bin/env bash
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/g2_entry_shadow_v1
test ! -e "$base/cache_terminal_v3_probe.log" || exit 90
/usr/bin/python3 -B "$base/compare_cache_and_terminal.py" > "$base/cache_terminal_v3_probe.log" 2>&1
probe=$?
printf '{"tests":"unchanged_49_passed_in_v2","probe":%s}\n' "$probe" > "$base/cache_terminal_v3_exit.json"
exit "$probe"
