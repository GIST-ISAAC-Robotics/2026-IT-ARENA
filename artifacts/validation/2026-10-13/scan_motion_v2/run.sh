#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONPATH="$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/scan_motion_v2
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_scan_motion_diagnostic.py > "$base/tests.log" 2>&1
tail -4 "$base/tests.log"
test ! -e "$base/run.log"
python3 -B scripts/validate_scan_motion_diagnostic.py --output "$base/result" > "$base/run.log" 2>&1
tail -110 "$base/run.log"
