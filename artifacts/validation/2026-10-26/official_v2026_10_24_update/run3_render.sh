#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
O=artifacts/validation/2026-10-26/official_v2026_10_24_update
D="$O/run3_render"
mkdir "$D"
trap 'rc=$?; echo "$rc" > "$D/exit_code.txt"; date -Is > "$D/end.txt"' EXIT
date -Is > "$D/start.txt"
ps -eo pid,args | grep -E 'gz sim|gz-sim|arena_bringup|parameter_bridge' | grep -v grep > "$D/processes_before.txt" || true
colcon build --packages-select arena_gazebo arena_bringup arena_autonomy --symlink-install > "$D/build_stdout.txt" 2> "$D/build_stderr.txt"
source install/setup.bash
python3 -B "$O/capture_update.py" --output "$D/views" --timeout 120 > "$D/stdout.txt" 2> "$D/stderr.txt"
ps -eo pid,args | grep -E 'gz sim|gz-sim|arena_bringup|parameter_bridge' | grep -v grep > "$D/processes_after.txt" || true
