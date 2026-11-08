#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A=artifacts/validation/2026-11-08/retained_corner_agreement_probe_v1
export PYTHONPATH="$PWD/$A:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests"
python3 -B -m pytest -q -p no:cacheprovider --tb=short "$A/test_candidate.py" > "$A/tests_v1.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$A/tests_v1.exit"
[ "$rc" -eq 0 ] || exit "$rc"
python3 -B "$A/replay.py" > "$A/replay_v1.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$A/replay_v1.exit"
exit "$rc"
