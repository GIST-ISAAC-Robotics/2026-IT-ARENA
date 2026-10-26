#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
base=artifacts/validation/2026-10-25/m3_execution_review_v1
out="$base/run17_observed_release_closed_loop"
test ! -e "$out/follow_limits.json" || exit 91
python3 -B "$base/diagnose_follow_limits.py" "$out/run" --output "$out/follow_limits.json" > "$out/follow_limits_stdout.txt" 2> "$out/follow_limits_stderr.txt"
rc=$?
echo "$rc" > "$out/follow_limits_exitcode.txt"
cat "$out/follow_limits_stdout.txt"
cat "$out/follow_limits_stderr.txt"
exit "$rc"
