#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
base=artifacts/validation/2026-10-25/m3_execution_review_v1
out="$base/run21_right_encounter"
python3 -B "$base/analyze_selection_run.py" "$out/run" --output "$out/decomposition.json" > "$out/decomposition_stdout.txt" 2> "$out/decomposition_stderr.txt"
rc=$?
echo "$rc" > "$out/decomposition_exitcode.txt"
if [ "$rc" -ne 0 ]; then cat "$out/decomposition_stderr.txt"; exit "$rc"; fi
python3 -B scripts/traffic_front_target.py "$out/run" --output "$out/front_target.json" > "$out/front_stdout.txt" 2> "$out/front_stderr.txt"
rc=$?
echo "$rc" > "$out/front_exitcode.txt"
cat "$out/front_stdout.txt"
cat "$out/front_stderr.txt"
exit "$rc"
