#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/g1_corner_integration_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_corner_evidence.py tests/test_corner_follow.py tests/test_local_follow.py tests/test_fragmented_follow.py tests/test_gapped_boundary_evidence.py tests/test_follow_trace.py tests/test_follow_profile.py tests/test_follow_profile_bundle.py tests/test_g1_solo_candidate_validation.py tests/test_occupancy_timing.py > "$base/tests.log" 2>&1
python3 -B scripts/probe_corner_follow.py artifacts/validation/2026-10-15/g1_solo_lap_v1/run/follow_decisions.jsonl --output "$base/probe" > "$base/probe.log" 2>&1
cat "$base/tests.log" "$base/probe.log"
