#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/g1_corner_evidence_v2
test ! -e "$base/tests.log"
set +e
python3 -B -m pytest -q tests/test_corner_evidence.py > "$base/tests.log" 2>&1
test_exit=$?
python3 -B "$base/probe.py" > "$base/probe.log" 2>&1
probe_exit=$?
set -e
printf 'tests=%s\nprobe=%s\n' "$test_exit" "$probe_exit" > "$base/exit.txt"
cat "$base/tests.log" "$base/probe.log"
if (( test_exit != 0 || probe_exit != 0 )); then exit 1; fi
