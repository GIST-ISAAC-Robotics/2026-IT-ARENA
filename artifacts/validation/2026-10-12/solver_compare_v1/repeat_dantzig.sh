#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-12/solver_compare_v1
test ! -e "$base/d2_dantzig.log"
rc=0
python3 -B "$base/repeat_dantzig.py" > "$base/d2_dantzig.log" 2>&1 || rc=$?
tail -20 "$base/d2_dantzig.log"
exit "$rc"
