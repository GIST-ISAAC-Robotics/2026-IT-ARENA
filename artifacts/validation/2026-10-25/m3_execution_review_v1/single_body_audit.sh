#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
base=artifacts/validation/2026-10-25/m3_execution_review_v1
out="$base/run23_body_hypothesis"
test ! -e "$out" || exit 91
mkdir -p "$out"
cp "$base/compare_single_body_hypothesis.py" "$out/"
python3 -B "$base/compare_single_body_hypothesis.py" "$base/run19_stop_restart/run/follow_decisions.jsonl" "$out/comparison.json" > "$out/stdout.txt" 2> "$out/stderr.txt"
rc=$?
echo "$rc" > "$out/exitcode.txt"
cat "$out/stdout.txt"
cat "$out/stderr.txt"
exit "$rc"
