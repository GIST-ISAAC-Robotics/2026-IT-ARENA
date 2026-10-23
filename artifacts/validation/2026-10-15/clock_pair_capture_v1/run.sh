#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_vehicle_interface:$PWD/src/arena_autonomy:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/clock_pair_capture_v1
test ! -e "$base/tests.log"
python3 -B "$base/prepare.py"
python3 -B -m pytest -q tests/test_clock_pair_audit.py tests/test_clock_freshness.py tests/test_clock_bounds.py tests/test_clock_exchange_audit.py > "$base/tests.log" 2>&1
python3 -B scripts/capture_clock_pairs.py --samples 500 --period-ms 20 --output "$base/capture.jsonl" > "$base/capture.log" 2>&1
python3 -B scripts/audit_clock_pairs.py "$base/capture.jsonl" --output "$base/report.json" > "$base/audit.log" 2>&1
tail -3 "$base/tests.log"
tail -1 "$base/capture.log"
tail -1 "$base/audit.log"
