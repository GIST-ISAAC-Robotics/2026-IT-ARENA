#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/g1_per_ray_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_occupancy_timing.py tests/test_fragmented_follow.py tests/test_gapped_boundary_evidence.py tests/test_follow_trace.py tests/test_local_follow.py > "$base/tests.log" 2>&1
tail -3 "$base/tests.log"
python3 -B scripts/probe_per_ray_follow.py artifacts/validation/2026-10-13/g1_lidar_stress_v1/side_left_uniform_s0/follow_decisions.jsonl --output "$base/noisy_input_probe" > "$base/noisy_input_probe.log" 2>&1
tail -2 "$base/noisy_input_probe.log"
