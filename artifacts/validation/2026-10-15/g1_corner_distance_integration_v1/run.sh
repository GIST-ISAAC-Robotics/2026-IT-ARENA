#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/g1_corner_distance_integration_v1
test ! -e "$base/tests.log"
tar -czf "$base/source.tar.gz" src/arena_autonomy/arena_autonomy src/arena_bringup/launch/g1_follow.launch.py config/tests/corner_distance_r0_sha256.json tests/test_corner_distance_probe.py tests/test_corner_distance_follow.py
python3 -B -m pytest -q tests/test_corner_distance_probe.py tests/test_corner_distance_follow.py tests/test_corner_follow.py tests/test_corner_evidence.py tests/test_corner_raycast.py tests/test_local_follow.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py tests/test_g1_solo_candidate_validation.py tests/test_pipeline_fault_validation.py > "$base/tests.log" 2>&1
python3 -B scripts/probe_corner_follow.py artifacts/validation/2026-10-15/g1_solo_lap_v3/run/follow_decisions.jsonl --mode corner_distance_candidate --output "$base/probe" > "$base/probe.log" 2>&1
tail -2 "$base/tests.log"
tail -1 "$base/probe.log"
