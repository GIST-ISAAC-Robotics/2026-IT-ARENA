#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/gapped_boundary_v3
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_fragmented_follow.py tests/test_gapped_boundary_evidence.py tests/test_follow_trace.py tests/test_local_follow.py tests/test_follow_overlap.py tests/test_g1_side_occlusion.py tests/test_physics_step_option.py tests/test_local_follow_validation.py > "$base/tests.log" 2>&1
tail -10 "$base/tests.log"
python3 -B scripts/probe_fragmented_follow.py artifacts/validation/2026-10-13/follow_overlap_v1/side_left_s0/follow_decisions.jsonl --output "$base/fixed_input" > "$base/fixed_input.log" 2>&1
tail -10 "$base/fixed_input.log"
