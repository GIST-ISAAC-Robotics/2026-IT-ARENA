#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/gapped_boundary_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_gapped_boundary_evidence.py tests/test_follow_trace.py tests/test_local_follow.py tests/test_follow_overlap.py > "$base/tests.log" 2>&1
tail -10 "$base/tests.log"
python3 -B scripts/probe_gapped_boundary.py artifacts/validation/2026-10-13/follow_overlap_v1/side_left_s0/follow_decisions.jsonl --output "$base/run" > "$base/run.log" 2>&1
tail -10 "$base/run.log"
