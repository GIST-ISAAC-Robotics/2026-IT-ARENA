#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/observed_fragments_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_observed_fragments.py tests/test_observed_fragment_audit.py tests/test_local_scene.py > "$base/tests.log" 2>&1
tail -12 "$base/tests.log"
python3 -B scripts/audit_observed_fragments.py \
  --trace artifacts/validation/2026-10-13/gapped_boundary_v3/side_left_s0/follow_decisions.jsonl \
  --trace artifacts/validation/2026-10-13/g1_right_candidate_v1/side_right_s0/follow_decisions.jsonl \
  --trace artifacts/validation/2026-10-13/g1_curve_candidate_v1/first_curve_s0/follow_decisions.jsonl \
  --output "$base/results" > "$base/audit.log" 2>&1
tail -1 "$base/audit.log"
