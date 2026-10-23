#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
out=artifacts/validation/2026-10-21/g1_depth_window_v2
if [ -e "$out/probe.log" ]; then exit 2; fi
set +e
python3 -B scripts/benchmark_depth_window.py --output "$out/run" > "$out/probe.log" 2>&1
code=$?
printf '\nPROBE_EXIT_CODE=%s\n' "$code" >> "$out/probe.log"
tail -n 10 "$out/probe.log"
exit "$code"
