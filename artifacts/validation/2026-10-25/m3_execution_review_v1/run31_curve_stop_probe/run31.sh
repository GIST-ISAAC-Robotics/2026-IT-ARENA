#!/usr/bin/env bash
set -uo pipefail
cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local" || exit 90
A=artifacts/validation/2026-10-25/m3_execution_review_v1
D=$A/run31_curve_stop_probe
test ! -e "$D/probe.json" || exit 91
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
export PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
sha256sum src/arena_autonomy/arena_autonomy/*.py "$A/probe_curve_stop.py" > "$D/sha256_before.txt"
timeout 500 python3 -B "$A/probe_curve_stop.py" "$D/probe.json" > "$D/stdout.txt" 2> "$D/stderr.txt"
echo $? > "$D/exitcode.txt"
sha256sum src/arena_autonomy/arena_autonomy/*.py "$A/probe_curve_stop.py" > "$D/sha256_after.txt"
cmp -s "$D/sha256_before.txt" "$D/sha256_after.txt"; echo "sha_unchanged_rc=$?" >> "$D/exitcode.txt"
