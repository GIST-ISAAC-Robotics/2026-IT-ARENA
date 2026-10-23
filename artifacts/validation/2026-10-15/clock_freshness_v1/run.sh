#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/clock_freshness_v1
test ! -e "$base/regression.log"
python3 -B -m pytest -q tests/test_clock_freshness.py tests/test_clock_bounds.py tests/test_clock_exchange_audit.py > "$base/regression.log" 2>&1
tail -4 "$base/regression.log"
python3 -B scripts/probe_clock_freshness.py --output "$base/probe" > "$base/probe.log" 2>&1
