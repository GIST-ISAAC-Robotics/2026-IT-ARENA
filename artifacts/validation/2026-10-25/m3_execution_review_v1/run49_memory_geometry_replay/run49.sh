#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
A=artifacts/validation/2026-10-25/m3_execution_review_v1
D="$A/run49_memory_geometry_replay"
[ ! -e "$D/exit_code.txt" ] || exit 91
cp "$A/compare_retained_threats.py" "$D/probe_source.py"
python3 -B "$D/probe_source.py" --before-source "$A/run47_memory_profile/m3_before.py" --expect-same --output "$D/replay" "$A/run13_selection_closed_loop/run/follow_decisions.jsonl" "$A/run17_observed_release_closed_loop/run/follow_decisions.jsonl" "$A/run21_right_encounter/run/follow_decisions.jsonl" "$A/run27_small_center_stop_restart/run/follow_decisions.jsonl" "$A/run35_committed_surface_corner/run/follow_decisions.jsonl" > "$D/stdout.txt" 2> "$D/stderr.txt"
result_code=$?
printf '%s\n' "$result_code" > "$D/exit_code.txt"
exit "$result_code"
