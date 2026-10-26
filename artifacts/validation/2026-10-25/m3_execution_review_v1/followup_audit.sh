#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
base=artifacts/validation/2026-10-25/m3_execution_review_v1
out="$base/run20_followup_audit"
test ! -e "$out" || exit 91
mkdir -p "$out"
python3 -B "$base/analyze_selection_run.py" "$base/run19_stop_restart/run" --output "$out/run19_decomposition.json" > "$out/decomposition_stdout.txt" 2> "$out/decomposition_stderr.txt"
rc=$?
echo "$rc" > "$out/decomposition_exitcode.txt"
if [ "$rc" -ne 0 ]; then cat "$out/decomposition_stderr.txt"; exit "$rc"; fi
python3 -B scripts/traffic_front_target.py "$base/run19_stop_restart/run" --output "$out/run19_front.json" > "$out/front_stdout.txt" 2> "$out/front_stderr.txt"
rc=$?
echo "$rc" > "$out/front_exitcode.txt"
if [ "$rc" -ne 0 ]; then cat "$out/front_stderr.txt"; exit "$rc"; fi
python3 -B "$base/right_preflight.py" "$out/right_preflight.json" > "$out/right_stdout.txt" 2> "$out/right_stderr.txt"
rc=$?
echo "$rc" > "$out/right_exitcode.txt"
cat "$out/front_stdout.txt"
cat "$out/right_stdout.txt"
cat "$out/right_stderr.txt"
exit "$rc"
