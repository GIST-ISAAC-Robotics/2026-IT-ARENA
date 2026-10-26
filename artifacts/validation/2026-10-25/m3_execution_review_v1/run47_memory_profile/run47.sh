#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
A=artifacts/validation/2026-10-25/m3_execution_review_v1
D="$A/run47_memory_profile"
[ ! -e "$D/exit_code.txt" ] || exit 91
cp "$A/profile_retained_threats.py" "$D/probe_source.py"
cp src/arena_autonomy/arena_autonomy/m3_execution_trial.py "$D/m3_before.py"
sha256sum src/arena_autonomy/arena_autonomy/m3_execution_trial.py > "$D/before.sha256"
python3 -B "$D/probe_source.py" --output "$D/profile" > "$D/stdout.txt" 2> "$D/stderr.txt"
result_code=$?
printf '%s\n' "$result_code" > "$D/exit_code.txt"
sha256sum -c "$D/before.sha256" > "$D/source_check.txt"
exit "$result_code"
