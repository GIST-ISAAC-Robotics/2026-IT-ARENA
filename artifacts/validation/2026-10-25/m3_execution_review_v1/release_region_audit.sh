#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
base=artifacts/validation/2026-10-25/m3_execution_review_v1
out="$base/run22_release_region_audit"
test ! -e "$out" || exit 91
mkdir -p "$out"
cp "$base/diagnose_release_region.py" "$out/"
python3 -B "$base/diagnose_release_region.py" "$base/run19_stop_restart/run/follow_decisions.jsonl" "$out/region.json" > "$out/stdout.txt" 2> "$out/stderr.txt"
rc=$?
echo "$rc" > "$out/exitcode.txt"
cat "$out/stdout.txt"
cat "$out/stderr.txt"
exit "$rc"
