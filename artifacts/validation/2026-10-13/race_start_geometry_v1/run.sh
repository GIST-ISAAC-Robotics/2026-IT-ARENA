#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONPATH="$PWD/scripts:$PWD/src/arena_autonomy${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/race_start_geometry_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_start_signal_geometry.py > "$base/tests.log" 2>&1
tail -10 "$base/tests.log"
python3 -B scripts/audit_start_signal_geometry.py --output "$base/result" > "$base/run.log" 2>&1
cat "$base/run.log"
