#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A=artifacts/validation/2026-11-08/corner_occlusion_witness_probe_v1
export PYTHONPATH="$PWD/$A:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests"
python3 -B -m pytest -q -p no:cacheprovider --tb=short "$A/test_candidate.py" > "$A/tests_v1.log" 2>&1
trc=$?
printf '%s\n' "$trc" > "$A/tests_v1.exit"
python3 -B "$A/probe.py" > "$A/probe_v1.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$A/probe_v1.exit"
exit "$rc"
