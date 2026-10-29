#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/traffic_anchor_set_v1
python3 -B -m pytest -q -p no:cacheprovider tests/test_replay_anchor_cost.py tests/test_follow_trace.py tests/test_g1_partial_policy.py tests/test_recovery_threat_capture.py tests/test_traffic_recovery_contract.py tests/test_partial_observation.py tests/test_surface_anchor_probe.py tests/test_timed_policy_probe.py tests/test_timed_missing_sweep_probe.py tests/test_traffic_path_recovery.py tests/test_source_provenance_r0_plus_b3.py tests/test_anchor_set_provenance.py tests/test_corner_follow.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py > "$base/related_regression.log" 2>&1
colcon build --packages-select arena_autonomy arena_bringup --symlink-install > "$base/build.log" 2>&1
