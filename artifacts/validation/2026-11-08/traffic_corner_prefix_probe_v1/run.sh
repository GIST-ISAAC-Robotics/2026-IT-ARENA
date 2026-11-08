#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests"
A=artifacts/validation/2026-11-08/traffic_corner_prefix_probe_v1
python3 -B -m pytest -q "$A/test_candidate.py" > "$A/tests_v2.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$A/tests_v2.exit"
if [ "$rc" != 0 ]; then exit "$rc"; fi
python3 -B "$A/replay.py" > "$A/replay_v2.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$A/replay_v2.exit"
exit "$rc"
